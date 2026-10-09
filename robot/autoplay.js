// ?tour (arriving from the wave page): play the whole page top to bottom, giving each chapter about as
// long as its animation. Touching, wheeling or a key pauses; the pill resumes from wherever you are.
(function () {
  if (!new URLSearchParams(location.search).has('tour')) return;
  var SEC = { hero: 2.5, explode: 7, sensors: 11, measure: 7, print: 9, assemble: 11, garden: 7, pack: 8, learn: 11, lift: 8, outro: 5 };
  var playing = false, last = 0, pill, badge;
  function chapters() { return [].slice.call(document.querySelectorAll('.chapter')); }
  function setPill() {
    pill.textContent = playing ? '❚❚' : '▶';
    pill.setAttribute('aria-label', playing ? '自動スクロールを一時停止' : '自動スクロールを再開');
    pill.classList.toggle('paused', !playing);
  }
  function pause() { if (!playing) return; playing = false; setPill(); }
  function play() {
    if (playing) return; playing = true; last = performance.now(); setPill(); requestAnimationFrame(step);
  }
  function step(now) {
    if (!playing) return;
    var dt = Math.min(0.05, (now - last) / 1000); last = now;
    var list = chapters(), y = scrollY, max = document.documentElement.scrollHeight - innerHeight, c = 0;
    for (var i = list.length - 1; i >= 0; i--) if (y >= list[i].offsetTop - 1) { c = i; break; }
    var el = list[c], span = Math.min(el.offsetHeight, max - el.offsetTop);
    var speed = Math.max(40, span / (SEC[el.dataset.chapter] || 7));   // px per second
    var ny = Math.min(max, y + speed * dt);
    scrollTo(0, ny);
    if (ny >= max - 1) { playing = false; pill.classList.add('gone'); return; }
    requestAnimationFrame(step);
  }
  function start() {
    pill = document.createElement('button'); pill.className = 'autoplay-pill'; pill.type = 'button';
    pill.addEventListener('click', function (e) { e.stopPropagation(); playing ? pause() : play(); });
    document.body.appendChild(pill);
    var stop = function (e) { if (e.target && e.target.closest && e.target.closest('.autoplay-pill')) return; pause(); };
    addEventListener('touchstart', stop, { passive: true }); addEventListener('wheel', stop, { passive: true });
    addEventListener('keydown', function (e) { if (['ArrowDown', 'ArrowUp', 'PageDown', 'PageUp', ' ', 'Home', 'End'].indexOf(e.key) >= 0) pause(); });
    badge = document.createElement('div'); badge.className = 'scrolling-badge'; badge.innerHTML = '<i>▼</i><i>▼</i><i>▼</i><b>下へスクロール中</b>';
    document.body.appendChild(badge);
    setTimeout(function () { badge.classList.add('gone'); setTimeout(function () { badge.remove(); }, 400); }, 1800);
    setTimeout(play, 250);
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', start); else start();
})();
