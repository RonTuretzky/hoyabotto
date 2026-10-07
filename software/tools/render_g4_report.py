"""Render the saved G4 experiment results; never infer success from a GIF."""
import argparse
import html
import json
from pathlib import Path


def render(root):
    records=[]
    for path in sorted(root.glob('*/result.json')):
        report=json.loads(path.read_text())
        if 'trials' not in report:continue
        for category in ('trials','validation','controls'):
            for row in report.get(category,[]):
                records.append((path.parent.name,category,row))
    esc=html.escape
    table=[]
    for run,category,row in records:
        stop=row.get('stop_reason') or ', '.join(row['failure_reasons']) or 'Complete pickup/hold/release passed in simulation'
        link=f'{run}/{row["name"]}/timeline.gif'
        table.append(f'<tr><td>{esc(run)}<br>{esc(row["name"])}</td><td>{esc(category)}</td><td>{"SIM PASS" if row["success"] else "FAIL"}</td><td>{esc(stop)}</td><td><a href="{link}">Complete timeline</a> · <a href="{run}/{row["name"]}/result.json">Raw score</a></td></tr>')
    illustrative=next(((run,row) for run,category,row in reversed(records) if category=='trials' and row['success']),None)
    if illustrative is None:illustrative=next(((run,row) for run,category,row in reversed(records) if category=='trials' and row['completed']),None)
    figure=''
    if illustrative:
        run,row=illustrative
        figure=f'<figure><img src="{run}/{row["name"]}/timeline.gif"><figcaption>{esc(run)} / {esc(row["name"])} — {"complete simulated skill passed" if row["success"] else "command sequence completed but task FAILED"}. Full sequence, including the actual final state.</figcaption></figure>'
    result=f'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>G4 planter — first training experiments</title>
<style>body{{font:17px/1.6 system-ui;max-width:1150px;margin:45px auto;padding:0 22px;background:#f5f3ee;color:#182028}}h1{{font-size:38px;line-height:1.15}}h2{{margin-top:42px}}p{{max-width:850px}}.badge{{display:inline-block;border:2px solid;padding:4px 12px;margin:6px 8px 6px 0;font-weight:700}}table{{width:100%;border-collapse:collapse;font-size:14px}}td,th{{border-bottom:1px solid #b6babc;padding:12px;text-align:left;vertical-align:top}}th{{background:#e3e4df}}img{{width:100%;max-width:900px}}figure{{margin:25px 0}}figcaption{{font-size:14px}}a{{color:#064f83}}code{{background:#e5e5e0;padding:2px 5px}}.note{{border-left:5px solid #202c39;padding-left:18px}}</style>
<h1>G4 cress planter<br>First training experiments</h1>
<span class="badge">SIMULATION ONLY</span><span class="badge">FULL ASSEMBLY: NOT VALIDATED</span><span class="badge">PHYSICAL MOTION: 0</span>
<p>The first curriculum stage handles the actual G4 paper pusher: approach, close, lift, hold, lower, release and withdraw. These are parameter-search experiments on a joint-actuated visual controller. No neural policy or complete planter assembly is trained.</p>
<p class="note">The original trough and holder, shared guide, four gravity carriers and seam roller remain the intended G4 assembly. Paper feeding, buckling, guide removal, roller folding, sheet placement and water transport still require their own models and physical tests. The paper must wick, not the plastic.</p>
{figure}
<h2>What the experiment checks</h2>
<p>Real SO101 joint limits and meshes; CAD-matched pusher collision blocks; a free tool held only by contact; rendered production AprilTags; eight fitting and three independent calibration poses; aligned depth with 0.8mm noise and 25% dropout. Independent scores inspect every physics step. A commanded lift with the pusher left on its rest fails.</p>
<p>Station dimensions, camera placement, printed mass, friction, marker mount and the 20/40mm staging-rest prototypes are assumptions. Only one arm and its bench are simulated. Ground-truth contact stops are diagnostics, not deployable force sensing. The provisional 8N simulation stop is not a validated physical force threshold.</p>
<h2>Saved attempts</h2><table><thead><tr><th>Run / episode</th><th>Split</th><th>Task score</th><th>Measured outcome or stop reason</th><th>Evidence</th></tr></thead><tbody>{''.join(table)}</tbody></table>
<h2>Evidence boundaries</h2><p>Software tests validate the scoring and scene contracts. A simulated skill pass, if present above, concerns only pusher pickup in the stated model. It cannot establish physical pickup, paper manipulation, four-carrier assembly, hands-off flap retention or wicking.</p>
<p><a href="collision-audit.json">CAD collision audit</a> · <a href="summary.json">Experiment summary</a></p></html>'''
    (root/'index.html').write_text(result)
    return dict(episodes=len(records),training_episodes=sum(c=='trials' for _,c,_ in records),
                successful_training_episodes=sum(c=='trials' and r['success'] for _,c,r in records),
                heldout_episodes=sum(c=='validation' for _,c,_ in records),
                physical_motor_writes=0,full_planter_success=False)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);args=p.parse_args()
    summary=render(args.root);(args.root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary))
