"""The exception viewer: a local web page on the Mac (reachable from a phone on the same Wi-Fi).

Shows the current state, the latest frames, why the robot paused, and only the
actions that are safe. Every button press becomes an intervention record with a
name. Not a safety device: the stop rules run in the controller regardless.
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse

HTML = """<!doctype html><html lang=en><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>farm viewer</title>
<style>
:root{--ink:#172a34;--blue:#154d77;--orange:#a84c0b;--red:#9b2c2c;--paper:#f7f5ee;--line:#c9d0cf}
body{margin:0;font:16px/1.5 system-ui,-apple-system,sans-serif;background:var(--paper);color:var(--ink)}
header{background:var(--ink);color:#fff;padding:12px 18px;display:flex;gap:16px;align-items:center;flex-wrap:wrap}
header b{font-size:18px} header code{background:#0008;padding:2px 6px;border-radius:4px}
main{padding:16px;display:grid;grid-template-columns:1fr 1fr;gap:16px;max-width:1400px}
section{background:#fffefa;border:1px solid var(--line);border-radius:10px;padding:14px}
h2{margin:0 0 8px;font-size:15px;letter-spacing:1px;text-transform:uppercase;color:var(--blue)}
.frames{display:grid;grid-template-columns:1fr 1fr;gap:8px}.frames img{width:100%;border-radius:6px;background:#dfe6e3}
.frames figcaption{font-size:12px;color:#556}
.tag{display:inline-block;padding:3px 10px;border-radius:12px;color:#fff;font-weight:700;font-size:13px}
.tag.PAUSED{background:var(--red)}.tag.DONE,.tag.IDLE{background:#2f6b3a}.tag.other{background:var(--blue)}
button{font:inherit;padding:10px 14px;border-radius:7px;border:2px solid var(--orange);background:#fbeee3;color:var(--orange);font-weight:700;cursor:pointer;margin:4px 6px 4px 0}
button.blue{border-color:var(--blue);background:#e8eff6;color:var(--blue)}
input{font:inherit;padding:8px;border:1px solid #899;border-radius:5px;width:100%;box-sizing:border-box}
.q{border-left:4px solid var(--orange);padding:8px 12px;margin:10px 0;background:#fff}
.feed{font-size:13px;max-height:260px;overflow:auto}.feed div{border-top:1px solid var(--line);padding:6px 0}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{text-align:left;padding:5px 6px;border-bottom:1px solid var(--line);vertical-align:top}
.muted{color:#667;font-size:13px} pre{white-space:pre-wrap;font-size:12px;background:#eef;padding:8px;border-radius:6px;max-height:220px;overflow:auto}
@media(max-width:900px){main{grid-template-columns:1fr}}
</style>
<header><b>farm viewer</b><span id=hdr class=muted></span><span style="flex:1"></span><button id=stopbtn style="background:#9b2c2c;border-color:#9b2c2c;color:#fff;font-size:18px;padding:10px 22px" onclick='fetch("/api/stop",{method:"POST",headers:{"content-type":"application/json"},body:JSON.stringify({who:who()})}).then(refresh)'>STOP</button><button class=blue id=resumebtn style="display:none" onclick='post("/api/resume",{})'>clear stop</button><label class=muted>your name <input id=who style="width:160px" placeholder="required to press anything"></label></header>
<main>
<section><h2>State</h2><div id=state></div><div class=frames id=frames></div></section>
<section><h2>Questions for a person</h2><div id=questions class=muted>none</div>
<h2 style="margin-top:16px">Authority</h2><div id=auth></div>
<h2 style="margin-top:16px">Open proposals (Astra)</h2><div id=props class=muted>none</div></section>
<section><h2>Unresolved actions</h2><div id=open class=muted>none</div><h2 style="margin-top:16px">Recent cycles</h2><table id=cycles></table></section>
<section><h2>Feed</h2><div id=feed class=feed></div></section>
</main>
<script>
const who=()=>document.getElementById('who').value.trim();
localStorage.who&&(document.getElementById('who').value=localStorage.who);
document.getElementById('who').addEventListener('input',e=>localStorage.who=e.target.value);
async function post(url,body){if(!who()){alert('Type your name first.');return}body.who=who();const r=await fetch(url,{method:'POST',headers:{'content-type':'application/json'},body:JSON.stringify(body)});if(!r.ok)alert(await r.text());refresh()}
function esc(s){return String(s??'').replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
async function refresh(){const d=await (await fetch('/api/state')).json();
document.getElementById('hdr').textContent=`${d.profile}${d.simulated?' · SIMULATED':''} · ${new Date().toLocaleTimeString()}`;
const st=d.state||'IDLE';const cls=st==='PAUSED'?'PAUSED':(st==='DONE'||st==='IDLE')?st:'other';
let s=`<span class="tag ${cls}">${esc(st)}</span> tray <b>${esc(d.tray||'-')}</b> · cycle <code>${esc(d.cycle_id||'-')}</code> · since ${d.since?Math.round(Date.now()/1000-d.since)+'s':'-'}`;
if(d.pause_reason)s+=`<p><b style="color:#9b2c2c">reason:</b> ${esc(d.pause_reason)}</p>`;
if(d.estop)s+=`<p><b style="color:#9b2c2c">STOP is engaged: motors hold. Type your name and press "clear stop" to continue.</b></p>`;document.getElementById('resumebtn').style.display=d.estop?'':'none';
if(d.problems&&d.problems.length)s+=`<p class=muted>startup problems: ${esc(d.problems.join(' · '))}</p>`;
if(d.judgement&&Object.keys(d.judgement).length)s+=`<pre>${esc(JSON.stringify(d.judgement,null,1))}</pre>`;
document.getElementById('state').innerHTML=s;
document.getElementById('frames').innerHTML=Object.entries(d.frames||{}).map(([n,h])=>`<figure style="margin:0"><img src="/images/${h}?t=${Date.now()}"><figcaption>${esc(n)} · ${h.slice(0,8)}</figcaption></figure>`).join('');
const qs=await (await fetch('/api/questions')).json();
document.getElementById('questions').innerHTML=qs.length?qs.map(q=>`<div class=q><b>${esc(q.text)}</b><div class=muted>asked ${Math.round(Date.now()/1000-q.asked_t)}s ago · ${esc(q.question_id)}</div>${q.options.map(o=>`<button onclick='post("/api/answer",{question_id:${JSON.stringify(q.question_id)},choice:${JSON.stringify(o)}})'>${esc(o)}</button>`).join('')}</div>`).join(''):'<span class=muted>none</span>';
document.getElementById('auth').innerHTML=`Jev level: <b>${esc(d.authority_level)}</b> (max ${esc(d.authority_max)}) ${['shadow','route','approve'].map(l=>`<button class=blue onclick='post("/api/authority",{level:"${l}"})'>${l}</button>`).join('')}`;
document.getElementById('props').innerHTML=(d.proposals||[]).length?d.proposals.map(p=>`<div class=q><b>${esc(p.title)}</b><div class=muted>${esc(p.change_json)}</div><div>${esc(p.expected)}</div><div class=muted>rollback: ${esc(p.rollback)}</div><button class=blue onclick='post("/api/proposal",{proposal_id:${JSON.stringify(p.proposal_id)},status:"ACCEPTED"})'>accept</button><button onclick='post("/api/proposal",{proposal_id:${JSON.stringify(p.proposal_id)},status:"REJECTED"})'>reject</button></div>`).join(''):'<span class=muted>none</span>';
document.getElementById('open').innerHTML=(d.open_actions||[]).length?d.open_actions.map(a=>`<div class=q><b>${esc(a.skill)}</b> ${esc(a.result)} · ${esc(a.params_json)}<div class=muted>${esc(a.note)}</div>${['I looked: water reached the tray','I looked: no water delivered','I looked: spill — cleaned'].map(o=>`<button onclick='post("/api/reconcile",{action_id:${JSON.stringify(a.action_id)},choice:${JSON.stringify(o)}})'>${esc(o)}</button>`).join('')}</div>`).join(''):'<span class=muted>none</span>';
document.getElementById('cycles').innerHTML='<tr><th>started</th><th>tray</th><th>result</th><th>note</th></tr>'+(d.cycles||[]).map(c=>`<tr><td>${new Date(c.started*1000).toLocaleTimeString()}</td><td>${esc(c.tray_id)}</td><td>${esc(c.result)}</td><td>${esc(c.note)}</td></tr>`).join('');
document.getElementById('feed').innerHTML=(d.feed||[]).slice().reverse().map(f=>`<div><span class=muted>${new Date(f.t*1000).toLocaleTimeString()}</span> ${esc(f.text)}</div>`).join('');
}
refresh();setInterval(refresh,2000);
</script></html>"""


def make_app(system) -> FastAPI:
    app = FastAPI(title="farm viewer")
    st = system.store

    @app.get("/", response_class=HTMLResponse)
    def index():
        return HTML

    @app.get("/api/state")
    def state():
        d = dict(system.state)
        d["authority_level"] = system.authority.level
        d["authority_max"] = system.authority.max_level
        d["open_actions"] = st.open_actions()
        d["cycles"] = st.recent_cycles(12)
        d["proposals"] = st.query("SELECT * FROM proposals WHERE status='OPEN' ORDER BY t DESC LIMIT 10")
        d["feed"] = getattr(system.human, "feed", [])[-60:]
        d["daily_cost_usd"] = round(st.daily_cost(), 4)
        return JSONResponse(json.loads(json.dumps(d, default=str)))

    @app.get("/api/questions")
    def questions():
        return JSONResponse(json.loads(json.dumps(system.human.pending(), default=str)))

    @app.post("/api/answer")
    async def answer(req: Request):
        b = await req.json()
        ok = system.human.answer(b.get("question_id", ""), b.get("choice", ""), b.get("who", ""), b.get("note", ""))
        return JSONResponse({"ok": ok}, status_code=200 if ok else 400)

    @app.post("/api/reconcile")
    async def reconcile(req: Request):
        b = await req.json()
        who, aid, choice = b.get("who", "").strip(), b.get("action_id", ""), b.get("choice", "")
        if not who or not aid:
            return JSONResponse({"ok": False, "error": "name and action_id required"}, status_code=400)
        result = "RECONCILED_OK" if "reached" in choice or "spill" in choice else "RECONCILED_FAIL"
        st.result(aid, result, note=f"{who}: {choice}")
        st.intervention(None, who, f"reconcile:{aid}", choice, result)
        if "spill" in choice:
            system.authority.demote_after_false_approval("spill reported at reconciliation")
        if not st.open_actions():
            system.state["needs_person"] = False
        return {"ok": True}

    @app.post("/api/authority")
    async def authority(req: Request):
        b = await req.json()
        who = b.get("who", "").strip()
        if not who:
            return JSONResponse({"ok": False}, status_code=400)
        system.authority.set_level(b.get("level", "shadow"), f"human:{who}", "set from viewer")
        return {"ok": True, "level": system.authority.level}

    @app.post("/api/proposal")
    async def proposal(req: Request):
        b = await req.json()
        who = b.get("who", "").strip()
        if not who:
            return JSONResponse({"ok": False}, status_code=400)
        st.decide_proposal(b.get("proposal_id", ""), b.get("status", "REJECTED"), f"human:{who}")
        return {"ok": True}

    @app.post("/api/stop")
    async def stop(req: Request):
        """Hold every motor now. Any request works: a stop must never wait for a name."""
        b = {}
        try:
            b = await req.json()
        except Exception:  # noqa: BLE001
            pass
        system.skills.estop.set()
        try:
            system.robot.stop()
        except Exception as e:  # noqa: BLE001
            return JSONResponse({"ok": False, "error": str(e)}, status_code=500)
        st.event(system.state.get("cycle_id"), "estop", {"who": b.get("who", "anonymous")})
        system.state["estop"] = True
        return {"ok": True}

    @app.post("/api/resume")
    async def resume(req: Request):
        b = await req.json()
        who = b.get("who", "").strip()
        if not who:
            return JSONResponse({"ok": False, "error": "name required to clear a stop"}, status_code=400)
        system.skills.estop.clear()
        system.state["estop"] = False
        st.intervention(system.state.get("cycle_id"), who, "estop", "cleared the stop", "resume")
        return {"ok": True}

    @app.get("/images/{h}")
    def image(h: str):
        p = st.image_path(h.split("?")[0])
        if not p.exists():
            return JSONResponse({"error": "no such image"}, status_code=404)
        return FileResponse(p, media_type="image/jpeg")

    @app.get("/api/cycle/{cycle_id}")
    def cycle(cycle_id: str):
        return JSONResponse(json.loads(json.dumps(st.cycle_summary(cycle_id), default=str)))

    return app


def serve_in_thread(system, port: int) -> threading.Thread:
    import uvicorn
    app = make_app(system)
    cfg = uvicorn.Config(app, host="0.0.0.0", port=port, log_level="warning")
    server = uvicorn.Server(cfg)
    t = threading.Thread(target=server.run, name="viewer", daemon=True)
    t.start()
    deadline = time.monotonic() + 3.0
    while not server.started:
        if not t.is_alive() or time.monotonic() >= deadline:
            server.should_exit = True
            t.join(timeout=1.0)
            raise RuntimeError(f"STOP viewer failed to start on port {port}")
        time.sleep(0.01)
    return t
