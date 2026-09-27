(()=>{'use strict';
const steps=[...document.querySelectorAll('.gstep')];if(!steps.length)return;
const fill=document.getElementById('gfill'),count=document.getElementById('gcount'),next=document.getElementById('gnext'),all=document.getElementById('gall');
function update(){let done=0,first=null;steps.forEach(s=>{const c=s.querySelector('input[type=checkbox]');const d=!!(c&&c.checked);s.classList.toggle('done',d);if(d)done++;else if(!first)first=s;s.classList.toggle('current',s===first)});
fill.style.width=`${done/steps.length*100}%`;count.textContent=`${done} / ${steps.length} done`;next.disabled=!first;return first}
steps.forEach(s=>s.querySelector('input[type=checkbox]')?.addEventListener('change',()=>setTimeout(update,0)));
next.addEventListener('click',()=>{const f=update();if(f)f.scrollIntoView({behavior:'smooth',block:'start'})});
all.addEventListener('click',()=>{document.body.classList.toggle('gfocus');all.textContent=document.body.classList.contains('gfocus')?'Show done steps':'Hide done steps'});
all.textContent='Hide done steps';
setTimeout(update,50);   // after app.js restores checkbox state
})();
