(()=>{'use strict';
const deck=document.getElementById('deck');if(!deck)return;
const slides=[...deck.querySelectorAll('[data-slide]')],pos=deck.querySelector('#deck-pos'),fill=deck.querySelector('#deck-fill'),toc=deck.querySelector('#deck-toc');
const outlineBtn=deck.querySelector('#deck-outline'),presentBtn=deck.querySelector('#deck-present');
const pad=n=>String(n).padStart(2,'0');let i=0;
function show(n,focus){n=Math.max(0,Math.min(slides.length-1,n));slides.forEach((s,k)=>{s.classList.toggle('active',k===n);s.setAttribute('aria-hidden',k===n?'false':'true')});i=n;
pos.textContent=`${pad(n+1)} / ${pad(slides.length)}`;fill.style.width=`${(n+1)/slides.length*100}%`;
toc.querySelectorAll('a').forEach(a=>a.setAttribute('aria-current',Number(a.dataset.go)===n?'true':'false'));
history.replaceState(null,'','#'+slides[n].id);if(focus)slides[n].querySelector('h2')?.focus({preventScroll:true});}
function fromHash(){const h=location.hash.slice(1);const k=slides.findIndex(s=>s.id===h);return k<0?0:k}
function setOutline(on){deck.classList.toggle('outline',on);toc.hidden=!on;outlineBtn.setAttribute('aria-pressed',String(on))}
function setPresent(on){document.body.classList.toggle('deck-present',on);presentBtn.setAttribute('aria-pressed',String(on));if(on){document.documentElement.requestFullscreen?.().catch(()=>{});setOutline(false)}else if(document.fullscreenElement)document.exitFullscreen?.()}
deck.querySelector('#deck-prev').addEventListener('click',()=>show(i-1,true));
deck.querySelector('#deck-next').addEventListener('click',()=>show(i+1,true));
outlineBtn.addEventListener('click',()=>setOutline(!deck.classList.contains('outline')));
presentBtn.addEventListener('click',()=>setPresent(!document.body.classList.contains('deck-present')));
toc.addEventListener('click',ev=>{const a=ev.target.closest('a[data-go]');if(!a)return;ev.preventDefault();setOutline(false);show(Number(a.dataset.go),true)});
addEventListener('keydown',ev=>{if(ev.target.matches('input,textarea,select'))return;const k=ev.key;
if(k==='ArrowRight'||k==='PageDown'||k===' '){ev.preventDefault();show(i+1,true)}
else if(k==='ArrowLeft'||k==='PageUp'){ev.preventDefault();show(i-1,true)}
else if(k==='Home')show(0,true);else if(k==='End')show(slides.length-1,true);
else if(k.toLowerCase()==='o')setOutline(!deck.classList.contains('outline'));
else if(k.toLowerCase()==='p')setPresent(!document.body.classList.contains('deck-present'));
else if(k==='Escape'){setPresent(false);setOutline(false)}});
document.addEventListener('fullscreenchange',()=>{if(!document.fullscreenElement&&document.body.classList.contains('deck-present'))setPresent(false)});
addEventListener('hashchange',()=>show(fromHash()));
let sx=0;addEventListener('touchstart',e=>sx=e.changedTouches[0].clientX,{passive:true});
addEventListener('touchend',e=>{const dx=e.changedTouches[0].clientX-sx;if(Math.abs(dx)>60)show(i+(dx<0?1:-1))},{passive:true});
addEventListener('beforeprint',()=>slides.forEach(s=>s.classList.add('active')));
addEventListener('afterprint',()=>show(i));
show(fromHash());
})();
