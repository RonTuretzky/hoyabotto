"""Index immutable parallel G4 evidence without promoting it to task success.

The original episode scores remain visible with their scorer version. A newer
audit is linked separately; this tool never rewrites a saved physics outcome.
"""
from __future__ import annotations

import argparse
import html
import json
import os
from pathlib import Path
from urllib.parse import quote


def render(root):
    root = root.resolve()
    esc = html.escape
    links = []

    def link(path, label):
        path = Path(path).resolve()
        if not path.is_file():
            return ''
        relative = os.path.relpath(path, root)
        links.append(relative)
        return f'<a href="{quote(relative, safe="/")}">{esc(label)}</a>'

    pusher = []
    carrier = []
    for path in sorted(root.rglob('result.json')):
        if 'source' in path.relative_to(root).parts or 'source-snapshot' in path.relative_to(root).parts:
            continue
        row = json.loads(path.read_text())
        if 'target_source' in row and 'object_reset_offset_m' in row:
            pusher.append((path, row))
        elif 'geometry_audit' in row and 'trials' in row:
            carrier.append((path, row))

    pusher_rows = []
    for path, row in pusher:
        outcome = 'SAVED PASS' if row['success'] else 'SAVED FAIL'
        clean = row.get('clean_pickup_audit')
        clean_label = ('Clean audit: ' + ('PASS' if clean['passed'] else 'FAIL')) if clean else 'Clean audit absent from original score'
        reason = row.get('stop_reason') or ', '.join(row.get('failure_reasons', [])) or 'Passed the saved scorer; see independent review for qualification'
        evidence = [link(path, 'Saved score'), link(path.parent / 'timeline.gif', 'Recorded GIF'),
                    link(path.parent / 'physics.jsonl', 'Physics'), link(path.parent / 'commands.json', 'Commands'),
                    link(path.parent / 'observation-rgb.png', 'RGB'), link(path.parent / 'observation-depth.npz', 'Depth')]
        pusher_rows.append(f'<tr><td>{esc(str(path.parent.relative_to(root)))}</td>'
                           f'<td>{outcome}<br>scorer v{row.get("scorer_version", 1)}<br>{clean_label}</td>'
                           f'<td>{esc(str(row["object_reset_offset_m"]))}<br>seed {row["seed"]}<br>{esc(str(row.get("fault") or "normal"))}</td>'
                           f'<td>{esc(reason)}</td><td>{" · ".join(x for x in evidence if x)}</td></tr>')

    carrier_sections = []
    for path, row in carrier:
        # Root diagnostic summary is authoritative for its own explicit scope.
        # Do not relabel a passive carrier drop as robot insertion or assembly.
        evidence = [link(path, 'Diagnostic results')]
        for key, label in [('geometry_audit', 'Collision and clearance audit')]:
            value = row.get(key)
            if isinstance(value, str):
                target = Path(value)
                evidence.append(link(target if target.is_absolute() else path.parent / target, label))
        trial_rows = []
        for trial in row.get('trials', []):
            trial_links = []
            for key, label in [('gif', 'Recorded GIF'), ('observations', 'Raw observations'), ('scene', 'Scene')]:
                value = trial.get(key)
                if isinstance(value, str):
                    target = Path(value)
                    trial_links.append(link(target if target.is_absolute() else path.parent / target, label))
            trial_rows.append(f'<tr><td>{esc(str(trial.get("name", "unnamed")))}</td>'
                              f'<td>{esc(str(trial.get("status", "unknown")))}</td>'
                              f'<td>{esc(str(trial.get("reason", "")))}</td>'
                              f'<td>{" · ".join(x for x in trial_links if x)}</td></tr>')
        carrier_sections.append(f'<h3>{esc(str(path.parent.relative_to(root)))}</h3>'
                                f'<p>Status: {esc(str(row.get("status", "unknown")))}</p>'
                                f'<p>{" · ".join(x for x in evidence if x)}</p>'
                                '<table><tr><th>Trial</th><th>Diagnostic outcome</th><th>Reason</th><th>Evidence</th></tr>'
                                + ''.join(trial_rows) + '</table>')

    review_path = root / 'integration-review.json'
    review = json.loads(review_path.read_text()) if review_path.exists() else {}
    findings = ''.join(f'<li>{esc(text)}</li>' for text in review.get('findings', []))
    audits = []
    for item in review.get('evidence', []):
        target = Path(item['path'])
        audits.append(link(target if target.is_absolute() else root / target, item['label']))
    summary = dict(pusher_episodes=len(pusher),
                   pusher_saved_passes=sum(row['success'] for _, row in pusher),
                   pusher_saved_clean_passes=sum(row['success'] and row.get('clean_pickup_audit', {}).get('passed', False) for _, row in pusher),
                   pusher_scorer_versions=sorted({row.get('scorer_version', 1) for _, row in pusher}),
                   carrier_diagnostic_batches=len(carrier),
                   full_planter_success=False, neural_policy_trained=False,
                   physical_success=False, hardware_commands=0,
                   links=sorted(set(links)))
    title = 'G4 planter — parallel simulation audit'
    document = f'''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>{title}</title>
<style>body{{font:16px/1.6 system-ui;max-width:1220px;margin:40px auto;padding:0 22px;background:#f5f3ee;color:#17212b}}h1{{font-size:36px;line-height:1.15}}h2{{margin-top:40px}}p{{max-width:950px}}.badge{{display:inline-block;border:2px solid;padding:4px 10px;margin:5px 6px 5px 0;font-weight:700}}table{{border-collapse:collapse;width:100%;font-size:13px}}th,td{{padding:10px;text-align:left;vertical-align:top;border-bottom:1px solid #aaa;overflow-wrap:anywhere}}th{{background:#e1e4e6}}a{{color:#064f83}}.note{{border-left:5px solid #17212b;padding-left:18px}}</style>
<h1>{title}</h1><span class="badge">SIMULATION ONLY</span><span class="badge">FULL ASSEMBLY UNVALIDATED</span><span class="badge">NO HARDWARE COMMANDS</span>
<p>Three parallel streams improve the pusher staging fixture, audit independent scoring, and test a single empty carrier through the actual G4 guide and original holder. The carton chat supplied the process-isolation and frozen-source runner pattern.</p>
<p class="note">The pusher uses rendered production tags, noisy aligned depth, fitted registration, SO101 joint actuators and contact physics. The carrier experiment uses a privileged initial pose and fixed holder/guide fixture. Only the first funnel corridor has static collision geometry; the full background meshes are visual. It is a passive dynamics diagnostic, not a robot insertion controller or four-carrier assembly.</p>
<h2>Reviewed findings</h2><ul>{findings or '<li>Integration review is pending; saved episode scores below are provisional.</li>'}</ul>
<p>{' · '.join(x for x in audits if x)}</p>
<h2>Pusher evidence</h2><p>Every row links its original score. Scorer v1 is historical evidence; v2 additionally checks uninterrupted per-step evidence, loaded opposing contacts and actual free support after release. Version 3 also measures displacement of the whole rigid tool during hold and settled release. These versions do not establish retained grasp from the end of closing through the lift: the independent whole-sequence audit found late acquisition and unintended loaded fixture contact in otherwise passing traces. Saved passes remain visible as historical scorer outcomes, not clean pickup qualification. A newer audit or scorer is linked separately without rewriting saved results. Reused placements are regression cases; solver repeats are not new independent placements. No row alone qualifies a policy.</p>
<table><tr><th>Run / episode</th><th>Saved outcome</th><th>Offset (m), noise seed, fault</th><th>Reason</th><th>Evidence</th></tr>{''.join(pusher_rows)}</table>
<h2>Carrier geometry and gravity insertion</h2>{''.join(carrier_sections) or '<p>No completed diagnostic batch is available yet.</p>'}
<h2>Training boundary</h2><p>No neural policy or complete assembly is trained. Paper feeding, buckling, pusher withdrawal from real paper, guide removal, roller folding, sheet release and water transport still require separate evidence. Fixture dimensions, friction, masses and camera placement are assumptions until measured. Failed episodes remain visible and are excluded from successful demonstrations.</p>
</html>'''
    (root / 'index.html').write_text(document)
    (root / 'report-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    return summary


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    result = render(args.root)
    print(json.dumps({key: value for key, value in result.items() if key != 'links'}))
