import { DurableObject } from 'cloudflare:workers';
import { timingSafeEqual } from 'node:crypto';

const SITE_ORIGINS = ['https://hoyabotto.com', 'https://www.hoyabotto.com', 'https://ronturetzky.github.io'];

async function boundedJson(request, limit) {
  const reader = request.body?.getReader();
  if (!reader) throw new Error('JSON body required');
  const chunks = [];
  let length = 0;
  while (true) {
    const {value, done} = await reader.read();
    if (done) break;
    length += value.byteLength;
    if (length > limit) { await reader.cancel(); throw new Error('Request too large'); }
    chunks.push(value);
  }
  const bytes = new Uint8Array(length);
  let offset = 0;
  for (const chunk of chunks) { bytes.set(chunk, offset); offset += chunk.byteLength; }
  const data = JSON.parse(new TextDecoder().decode(bytes));
  if (!data || typeof data !== 'object' || Array.isArray(data)) throw new Error('JSON object required');
  return data;
}

export class ShowConnection extends DurableObject {
  constructor(ctx, env) {
    super(ctx, env);
    this.ctx.storage.sql.exec('CREATE TABLE IF NOT EXISTS connection (id INTEGER PRIMARY KEY, origin TEXT NOT NULL)');
  }
  register(origin) {
    this.ctx.storage.sql.exec('INSERT INTO connection (id, origin) VALUES (1, ?) ON CONFLICT(id) DO UPDATE SET origin = excluded.origin', origin);
  }
  origin() {
    return this.ctx.storage.sql.exec('SELECT origin FROM connection WHERE id = 1').toArray()[0]?.origin || null;
  }
}

function authorized(request, secret) {
  if (!secret) return false;
  const given = new TextEncoder().encode(request.headers.get('Authorization') || '');
  const expected = new TextEncoder().encode('Bearer ' + secret);
  return given.byteLength === expected.byteLength && timingSafeEqual(given, expected);
}

function reply(body, status, origin) {
  const headers = new Headers({'Cache-Control':'no-store', 'X-Content-Type-Options':'nosniff', 'Referrer-Policy':'no-referrer'});
  if (SITE_ORIGINS.includes(origin)) {
    headers.set('Access-Control-Allow-Origin', origin);
    headers.set('Vary', 'Origin');
    headers.set('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    headers.set('Access-Control-Allow-Headers', 'Content-Type');
  }
  return Response.json(body, {status, headers});
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    const origin = request.headers.get('Origin');
    const connection = env.SHOW_CONNECTION.getByName('hoyabotto-single-robot');
    if (url.pathname === '/api/bridge/register' && request.method === 'POST') {
      if (!authorized(request, env.BRIDGE_TOKEN)) return reply({error:'Authentication required'}, 403, origin);
      try {
        if (Number(request.headers.get('Content-Length')) > 1024) throw new Error();
        const data = await boundedJson(request, 1024);
        const target = new URL(data.origin);
        if (target.protocol !== 'https:' || !/^[a-z0-9-]+\.trycloudflare\.com$/.test(target.hostname) ||
            target.username || target.password || target.port || target.pathname !== '/' || target.search || target.hash) throw new Error();
        await connection.register(target.origin);
        return reply({ok:true}, 200, origin);
      } catch (_) { return reply({error:'Expected a public HTTPS tunnel origin'}, 400, origin); }
    }
    // No operator controls, pages, arbitrary proxy paths, or bridge credentials reach visitors.
    const read = request.method === 'GET' && (url.pathname === '/api/state' || /^\/api\/requests\/[a-f0-9]{12}$/.test(url.pathname));
    const submit = request.method === 'POST' && url.pathname === '/api/requests';
    const preflight = request.method === 'OPTIONS' && url.pathname === '/api/requests';
    if (!(read || submit || preflight)) return reply({error:'Not a visitor endpoint'}, 403, origin);
    if (origin && !SITE_ORIGINS.includes(origin)) return reply({error:'Origin not allowed'}, 403, origin);
    if (preflight) return reply({ok:true}, 200, origin);
    if (submit && Number(request.headers.get('Content-Length')) > 4096) return reply({error:'Invalid request size'}, 400, origin);
    const target = await connection.origin();
    if (!target) return reply({error:'The robot show is reconnecting'}, 503, origin);
    try {
      const headers = new Headers();
      if (origin) headers.set('Origin', origin);
      headers.set('Content-Type', 'application/json');
      headers.set('User-Agent', 'HoyaBotto-Relay/1.0');
      headers.set('X-Hoya-Proxy-Token', env.BRIDGE_TOKEN || '');
      headers.set('X-Hoya-Visitor-IP', request.headers.get('CF-Connecting-IP') || 'unknown');
      let body;
      if (submit) {
        // Parse and reconstruct a bounded JSON envelope; never stream an unbounded visitor body upstream.
        let data;
        try { data = await boundedJson(request, 4096); }
        catch (_) { return reply({error:'Invalid request body'}, 400, origin); }
        body = JSON.stringify({name:data.name, gestures:data.gestures});
        if (body.length > 4096) return reply({error:'Invalid request size'}, 400, origin);
      }
      const upstream = await fetch(target + url.pathname, {
        method:request.method, headers, body, redirect:'manual', signal:AbortSignal.timeout(6000)
      });
      if (upstream.status >= 300 && upstream.status < 400) {
        await upstream.body?.cancel();
        throw new Error('Visitor upstream redirected; refusing to forward credentials');
      }
      const responseHeaders = reply({}, 200, origin).headers;
      responseHeaders.set('Content-Type', 'application/json');
      // Stream the response; neither robot certificates nor operator tokens live in this Worker.
      return new Response(upstream.body, {status:upstream.status, headers:responseHeaders});
    } catch (error) {
      console.error(JSON.stringify({event:'visitor_upstream_failed', type:error.name, message:error.message}));
      return reply({error:'The robot show is reconnecting. Please try again shortly.'}, 503, origin);
    }
  }
};
