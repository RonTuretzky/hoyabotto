"""Apply quiet sensor display and robust DSML rejection to the external pilot.

Source-only, AST-validated, idempotent, with backups. No hardware or restart.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import json
from pathlib import Path


FILTER = """// Routine sensing stays in the model context and logs, not the conversation display.
const ROUTINE_SENSE_TOOLS=new Set(['robot_get_state','robot_get_motion','robot_list_motors','robot_get_claw_positions','robot_get_scene_points','robot_get_execution','robot_get_capabilities','robot_get_depth','robot_get_carton_tags']);
function routineSense(e){
 if(e.type==='state')return true;
 if(e.type==='supervisor'&&/^SENSE:\\s*/.test(String(e.text||'')))return true;
 if(e.type==='worker'&&(e.label==='Sensors'||e.model==='sensors'))return true;
 if((e.type==='tool_start'||e.type==='tool_result')&&ROUTINE_SENSE_TOOLS.has(e.name))return !(e.type==='tool_result'&&e.result&&e.result.ok===false);
 return false;
}
"""


def replace_once(source, old, new):
    if new in source:
        return source
    if source.count(old) != 1:
        raise ValueError(f"Expected one source hook; inspect before patching: {old[:100]!r}")
    return source.replace(old, new, 1)


def patch_html(source):
    source = replace_once(source, "function event(e){", FILTER + "function event(e){")
    source = replace_once(source,
                          "lastId=Math.max(lastId,e.id)}if(e.steer_id",
                          "lastId=Math.max(lastId,e.id)}if(routineSense(e))return;if(e.steer_id")
    source = replace_once(source,
                          "else{let d=elem('details','tool done');d.append(elem('summary','',e.name+' · result'),elem('pre','',JSON.stringify(e.result,null,2)));$('messages').append(d)}scroll()}",
                          "else{let failed=e.result&&e.result.ok===false;let d=elem('details','tool '+(failed?'refused':'done'));d.append(elem('summary','',e.name+(failed?' · refused':' · result')),elem('pre','',JSON.stringify(e.result,null,2)));$('messages').append(d)}scroll()}")
    return source


def patch_server(source):
    old = r"r'<\|DSML\|>|<invoke\b|<function_call|<tool_call|\binvoke name='"
    new = r"r'<[^>]*\bDSML\b|<invoke\b|<function_call|<tool_call|\binvoke name='"
    source = replace_once(source, old, new)
    ast.parse(source)
    return source


def install(pilot):
    originals = {name: (pilot / name).read_text() for name in ('chat.html', 'chat_server.py')}
    updated = {'chat.html': patch_html(originals['chat.html']),
               'chat_server.py': patch_server(originals['chat_server.py'])}
    receipt = {}
    for name, old in originals.items():
        new = updated[name]
        before = hashlib.sha256(old.encode()).hexdigest()
        if new != old:
            target = pilot / name
            backup = pilot / '.private/quiet-sensing-backups' / f'{target.stem}-{before}{target.suffix}'
            backup.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
            if not backup.exists():
                backup.write_text(old)
                backup.chmod(0o600)
            if target.read_text() != old:
                raise RuntimeError(f'{name} changed during install; inspect before retrying')
            temp = target.with_suffix('.quiet.tmp')
            temp.write_text(new)
            temp.chmod(target.stat().st_mode & 0o777)
            temp.replace(target)
        receipt[name] = {'changed': old != new, 'before_sha256': before,
                         'after_sha256': hashlib.sha256(new.encode()).hexdigest()}
    return receipt


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(install(args.pilot.resolve()), indent=2))
