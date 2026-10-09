// Phones (and any browser where 3D cannot start) watch the same journey as pre-rendered video:
// one square clip per chapter, captured from the 3D page at full quality, switched as you scroll.
(function () {
  var MAP = { hero: 0, explode: 1, sensors: 2, measure: 3, print: 4, assemble: 5, garden: 6, pack: 7, lift: 8, outro: 9 };
  var G4_STEPS = [0, 0.05, 0.13, 0.2, 0.27, 0.4, 0.42, 0.5, 0.57, 0.64, 0.69, 0.84, 0.92];
  var ease = function (t) { return t * t * (3 - 2 * t); };
  var E = function (t, a, b) { return ease(Math.min(1, Math.max(0, (t - a) / (b - a)))); };

  function start() {
    if (window.__vStarted) return; window.__vStarted = true;
    document.body.classList.add('vmode'); document.body.classList.remove('no3d');
    var l = document.getElementById('loading'); if (l) l.className = 'done';
    var chapters = [].slice.call(document.querySelectorAll('.chapter'));
    var cards = chapters.map(function (c) { return c.querySelector('.card'); });
    var stage = document.createElement('div'); stage.className = 'vstage'; stage.setAttribute('aria-hidden', 'true'); document.body.insertBefore(stage, document.body.firstChild);
    var vids = [0, 1].map(function () {
      var v = document.createElement('video'); v.muted = true; v.loop = true; v.playsInline = true; v.preload = 'auto';
      v.setAttribute('playsinline', ''); v.setAttribute('muted', ''); v.setAttribute('disablepictureinpicture', ''); v.style.opacity = 0; stage.appendChild(v); return v;
    });
    var front = 0, cur = -1, preloaded = {};
    var name = function (i) { return chapters[i].dataset.chapter; };
    var src = function (i) { return name(i) === 'learn' ? 'robot/training/training.mp4' : 'robot/journey/ch-' + MAP[name(i)] + '.mp4'; };
    var poster = function (i) { return name(i) === 'learn' ? 'robot/training/training-poster.jpg' : 'robot/journey/ch-' + MAP[name(i)] + '.jpg'; };
    function preload(i) {
      if (!chapters[i] || preloaded[i]) return; preloaded[i] = true;
      var p = document.createElement('link'); p.rel = 'preload'; p.as = 'video'; p.href = src(i); p.type = 'video/mp4'; document.head.appendChild(p);
    }
    function show(i) {
      if (i === cur) return; cur = i;
      var next = vids[1 - front], prev = vids[front];
      next.poster = poster(i); next.src = src(i); next.className = name(i) === 'learn' ? 'wide' : '';
      var p = next.play(); if (p && p.catch) p.catch(function () {});
      next.style.opacity = 1; prev.style.opacity = 0;
      setTimeout(function () { if (vids[front] !== prev) prev.pause(); }, 600);
      front = 1 - front; preload(i + 1);
    }
    var bar = document.getElementById('progress-bar');
    var sensorItems = [].slice.call(document.querySelectorAll('#sensor-list li'));
    var g4Steps = [].slice.call(document.querySelectorAll('#g4-steps li'));
    var g4Num = document.getElementById('g4-step-num'), g4Name = document.getElementById('g4-step-name');
    function tick() {
      var y = scrollY, max = document.documentElement.scrollHeight - innerHeight;
      if (bar) bar.style.width = (100 * y / Math.max(1, max)).toFixed(2) + '%';
      var s = 0;
      for (var i = chapters.length - 1; i >= 0; i--) {
        var el = chapters[i];
        if (y >= el.offsetTop || i === 0) { var span = Math.max(1, Math.min(el.offsetHeight, max - el.offsetTop)); s = i + Math.min(0.9999, Math.max(0, (y - el.offsetTop) / span)); break; }
      }
      var c = Math.floor(s), t = s - c; show(c);
      cards.forEach(function (el, j) {
        var k = j === c ? (j === 0 ? 1 : E(t, 0, 0.03)) * (j === cards.length - 1 ? 1 : 1 - E(t, 0.9, 0.985)) : 0;
        el.style.opacity = k.toFixed(3); el.style.visibility = k < 0.01 ? 'hidden' : 'visible';
      });
      // the clip's position is the chapter's progress: follow it in the sensor list and the G4 step counter
      var v = vids[front], vt = v.duration ? v.currentTime / v.duration : 0;
      if (name(c) === 'sensors') { var si = Math.max(0, Math.min(9, Math.floor((vt - 0.06) / 0.09))); sensorItems.forEach(function (li, j) { li.classList.toggle('on', j === si); }); }
      if (name(c) === 'assemble' && g4Num) {
        var st = 0; for (var k2 = 0; k2 < G4_STEPS.length; k2++) if (vt >= G4_STEPS[k2]) st = k2;
        g4Steps.forEach(function (li, j) { li.classList.toggle('on', j <= st); });
        g4Num.textContent = '手順 ' + st + ' / 12'; g4Name.textContent = g4Steps[st] ? g4Steps[st].dataset.name : '';
      }
      requestAnimationFrame(tick);
    }
    tick();
    // arriving from the wave page: glide into the exploded view, then hint to keep scrolling
    if (new URLSearchParams(location.search).has('tour')) {
      var cancelled = false, stop = function () { cancelled = true; };
      addEventListener('touchstart', stop, { passive: true, once: true }); addEventListener('wheel', stop, { passive: true, once: true });
      var hint = function () {
        var h = document.createElement('div'); h.className = 'scroll-hint'; h.innerHTML = '<span>↓</span>スクロールで続きを見る'; document.body.appendChild(h);
        var y0 = scrollY, off = function () { if (Math.abs(scrollY - y0) > 120) { h.classList.add('gone'); removeEventListener('scroll', off); } };
        addEventListener('scroll', off, { passive: true });
      };
      var moving = document.createElement('div'); moving.className = 'scrolling-badge'; moving.innerHTML = '<i>▼</i><i>▼</i><i>▼</i><b>下へスクロール中</b>';
      setTimeout(function () {
        document.body.appendChild(moving);
        var ch = chapters[1], to = ch.offsetTop + ch.offsetHeight * 0.3, from = scrollY, t0 = performance.now(), ms = 1100;
        var done = function () { moving.classList.add('gone'); setTimeout(function () { moving.remove(); }, 400); hint(); };
        var step = function (now) { if (cancelled) return done(); var u = Math.min(1, (now - t0) / ms); scrollTo(0, from + (to - from) * ease(u)); if (u < 1) requestAnimationFrame(step); else done(); };
        requestAnimationFrame(step);
      }, 250);
    }
  }
  window.__startVideoMode = start;
  if (window.__videoMode) { if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start); else start(); }
})();
