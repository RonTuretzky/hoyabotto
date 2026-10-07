"""Render the full-goal progress ledger with direct links to saved evidence."""
from __future__ import annotations
import argparse
import html
import json
from pathlib import Path
from urllib.parse import quote


def render(root):
    root=Path(root).resolve();ledger=json.loads((root/'progress.json').read_text());links=[]
    def link(path,label):
        target=root/path
        if not target.is_file():
            raise ValueError(f'Missing declared evidence: {target}')
        links.append(path)
        return f'<a href="{quote(path,safe="/")}">{html.escape(label)}</a>'
    stages=[]
    for row in ledger['requirements']:
        stages.append('<tr>'+''.join(f'<td>{html.escape(row[key])}</td>' for key in ['requirement','status','evidence_or_next_action'])+'</tr>')
    evidence=[]
    for group in ledger['evidence_groups']:
        items=' · '.join(link(i['path'],i['label']) for i in group['links'])
        evidence.append(f'<h2>{html.escape(group["title"])}</h2><p>{html.escape(group["finding"])}</p><p>{items}</p>')
    document='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G4 full assembly — simulation progress</title>
<style>body{font:17px/1.55 system-ui;max-width:1120px;margin:36px auto;padding:0 24px;color:#17212b;background:#f5f3ee}h1{font-size:34px;line-height:1.15}h2{margin-top:32px}table{border-collapse:collapse;width:100%;font-size:15px}th,td{text-align:left;vertical-align:top;padding:10px;border-bottom:1px solid #999}th{background:#e2e5e7}a{color:#074f85}.status{border:2px solid;padding:10px 14px;display:inline-block}.scope{border-left:4px solid;padding-left:16px}</style>
<h1>G4 full assembly — simulation progress</h1>
<p class="status">FULL SEQUENCE INCOMPLETE · SIMULATION ONLY · NO HARDWARE COMMANDS</p>
'''
    document+=f'<p>{html.escape(ledger["objective"])}</p><p class="scope">{html.escape(ledger["scope"])}</p>'
    document+='<h2>Requirements for the complete goal</h2><table><tr><th>Stage</th><th>Current evidence</th><th>Result or next action</th></tr>'+''.join(stages)+'</table>'
    document+=''.join(evidence)
    document+='<h2>Evidence boundaries</h2><p>Software checks, a mechanism bench, a robot primitive, a complete simulated assembly and physical success are separate claims. Initializing a part in its assembled pose never earns placement credit. Hypothetical camera, fixture and material parameters remain assumptions. Failed runs are retained.</p></html>'
    (root/'index.html').write_text(document)
    (root/'report-links.json').write_text(json.dumps(dict(checked_links=links,missing=[],full_task_success=False),indent=2)+'\n')
    print(json.dumps(dict(links_checked=len(links),full_task_success=False)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    render(p.parse_args().root)
