// Offline VM only: no browser, camera, sockets, or credentials.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {test} = require('node:test');
const html = fs.readFileSync(path.join(__dirname, '../docs/session-archive-2026-10-05/phone_camera/camera.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function harness() {
  const callbacks = new Map(), events = [];
  let nextId = 1, delayedBlob = null, delayEncoding = false;
  const video = {
    videoWidth: 1280, videoHeight: 720,
    requestVideoFrameCallback(fn) { const id = nextId++; callbacks.set(id, fn); events.push('request-frame'); return id; },
    cancelVideoFrameCallback(id) { callbacks.delete(id); events.push('cancel-frame'); },
  };
  const elements = new Map([['preview', video]]);
  const canvas = {
    getContext: () => ({drawImage: () => events.push('draw')}),
    toBlob(fn) { if (delayEncoding) delayedBlob = fn; else fn({fakeJPEG: true}); },
  };
  const document = {
    hidden: false, visibilityState: 'visible',
    createElement: () => canvas,
    getElementById(id) { if (!elements.has(id)) elements.set(id, {}); return elements.get(id); },
    addEventListener() {},
  };
  const context = vm.createContext({document, navigator: {}, location: {hash: '#offline-test'},
    crypto: {randomUUID: () => 'test-client'}, window: {addEventListener() {}},
    performance: {now: () => 100}, Date, Math, console, DOMException, AbortController,
    setTimeout, clearTimeout, setInterval: () => 0,
    fetch: () => { throw Error('Unexpected fetch: this test has no network'); },
  });
  vm.runInContext(script + '\nactive=true;run=1;browserStream="test-stream";', context);
  return {context, video, events, document, callbacks,
    begin() {
      context.controller = new AbortController();
      return vm.runInContext("challengedFrame($('preview'),100,1,controller.signal)", context);
    },
    emit(now, extra = {}) {
      assert.equal(callbacks.size, 1);
      const [id, fn] = callbacks.entries().next().value;
      callbacks.delete(id);
      fn(now, {presentedFrames: 1, mediaTime: 1, presentationTime: 110, ...extra});
    },
    delayEncoding() { delayEncoding = true; },
    finishEncoding() { delayedBlob({fakeJPEG: true}); },
  };
}

test('waits for a frame presented after the challenge, then draws in that callback', async () => {
  const h = harness(), pending = h.begin();
  h.emit(120, {presentationTime: 99});
  assert(!h.events.includes('draw'));
  h.emit(125, {presentationTime: 115});
  const result = await pending;
  assert.deepEqual(h.events, ['request-frame', 'request-frame', 'draw']);
  assert.equal(result.timing.challenge_received_ms, 100);
  assert.equal(result.timing.presentation_time_ms, 115);
  assert.equal(result.timing.callback_now_ms, 125);
  assert.equal(result.timing.browser_stream_id, 'test-stream');
});

test('does not redraw repeated frame count or media identity', async () => {
  const h = harness();
  vm.runInContext('lastFrame={presented_frames:4,media_time_s:4}', h.context);
  const pending = h.begin();
  h.emit(120, {presentedFrames: 4, mediaTime: 5});
  h.emit(125, {presentedFrames: 5, mediaTime: 4});
  assert(!h.events.includes('draw'));
  h.emit(130, {presentedFrames: 6, mediaTime: 6});
  assert.equal((await pending).timing.presented_frames, 6);
  assert.equal(h.events.filter(e => e === 'draw').length, 1);
});

test('malformed browser timing fails without drawing', async () => {
  for (const extra of [{mediaTime: NaN}, {presentationTime: Infinity}, {presentedFrames: true}, {presentedFrames: 1.5}]) {
    const h = harness(), pending = h.begin();
    h.emit(120, extra);
    await assert.rejects(pending, /identity\/timing unavailable/);
    assert(!h.events.includes('draw'));
  }
});

test('hidden page or changed generation cannot provide timing proof', async () => {
  for (const expression of ['document.hidden=true', 'run++', 'active=false']) {
    const h = harness(), pending = h.begin();
    vm.runInContext(expression, h.context);
    h.emit(120);
    await assert.rejects(pending, /sharing paused/);
    assert(!h.events.includes('draw'));
  }
});

test('abort cancels pending callback; late encoder completion cannot resolve', async () => {
  const h = harness(), pending = h.begin();
  h.context.controller.abort();
  await assert.rejects(pending, {name: 'AbortError'});
  assert.equal(h.callbacks.size, 0);
  const encoded = harness();
  encoded.delayEncoding();
  const encoding = encoded.begin();
  encoded.emit(120);
  encoded.context.controller.abort();
  encoded.finishEncoding();
  await assert.rejects(encoding, {name: 'AbortError'});
});

test('fake upload orders challenge, new callback, draw, then same timing and image', async () => {
  const h = harness(), calls = [];
  h.context.fetch = async (url, options) => {
    calls.push({url, options});
    if (url === '/api/challenge') {
      assert.equal(h.callbacks.size, 0);
      return {ok: true, json: async () => ({protocol: 'server-challenge-video-frame-v1', challenge_id: 'one-use'})};
    }
    assert.equal(url, '/api/frame');
    assert.equal(h.events.filter(e => e === 'draw').length, 1);
    const timing = JSON.parse(options.headers['X-Camera-Timing']);
    assert.equal(timing.challenge_id, 'one-use');
    assert.equal(timing.presented_frames, 2);
    assert.equal(timing.presentation_time_ms, 115);
    assert.equal(options.body.fakeJPEG, true);
    // Stop recursive scheduling after this fake delivery.
    vm.runInContext('active=false', h.context);
    return {ok: true, json: async () => ({frame_timing: timing})};
  };
  const sending = vm.runInContext('sendFrame(1)', h.context);
  await new Promise(resolve => setImmediate(resolve));
  assert.equal(calls.length, 1);
  h.emit(120, {presentationTime: 115, presentedFrames: 2});
  await sending;
  assert.deepEqual(calls.map(c => c.url), ['/api/challenge', '/api/frame']);
});

test('missing video-frame callbacks fail closed without fetch', async () => {
  const h = harness();
  delete h.video.requestVideoFrameCallback;
  await vm.runInContext('sendFrame(1)', h.context);
  assert.equal(h.events.length, 0);
  assert.match(h.document.getElementById('error').textContent, /cannot confirm new video frames/);
});
