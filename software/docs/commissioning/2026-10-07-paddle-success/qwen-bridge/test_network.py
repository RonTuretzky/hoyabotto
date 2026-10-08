"""After a Wi-Fi change: a dead quick-tunnel relay is replaced (never reused) and the LAN address is reported."""
import json,tempfile
from pathlib import Path
import redeploy_robot_server as R
tmp=Path(tempfile.mkdtemp());R.WORK=tmp;R.RELAY_LOG=tmp/'relay.log';R.RELAY_RECORD=tmp/'relay.json';R.NETWORK=tmp/'network.json'
R.CLOUDFLARED=tmp/'cloudflared';R.CLOUDFLARED.write_text('');R.say=lambda m:None
R.RELAY_LOG.write_text('INF |  https://old-dead-name.trycloudflare.com  |\n')
assert R.relay_hostname(R.RELAY_LOG)=='old-dead-name.trycloudflare.com'
killed=[];started=[]
class P:
 pid=4242
 def poll(self):return None
def popen(cmd,**kw):
 started.append(cmd);kw['stdout'].write('INF |  https://fresh-new-name.trycloudflare.com  |\n');kw['stdout'].flush();return P()
R.subprocess.Popen=popen;R.os.kill=lambda pid,sig:killed.append(pid)
# Alive process, hostname resolves: reused.
R.relay_processes=lambda:[111];R.resolves=lambda h:True
assert R.ensure_relay(False)=='old-dead-name.trycloudflare.com' and not started and not killed
# Alive process but the hostname no longer resolves (Mac changed network): replaced.
R.resolves=lambda h:False
assert R.ensure_relay(False)=='fresh-new-name.trycloudflare.com' and killed==[111]
assert started[0][-4:]==['tunnel','--url','tcp://127.0.0.1:1241','--no-autoupdate'] and json.loads(R.RELAY_RECORD.read_text())['pid']==4242
# Dry run never starts or kills anything.
started.clear();killed.clear();assert R.ensure_relay(True) is None and not started and not killed
# Network report: Bonjour LAN URL when the API listens beyond loopback.
R.lan_addresses=lambda:{'bonjour':'Neooooo.local','ips':['192.168.1.66']}
info=R.report_network('fresh-new-name.trycloudflare.com')
assert info['lan_url']=='https://Neooooo.local:1241' and json.loads(R.NETWORK.read_text())['relay_hostname']=='fresh-new-name.trycloudflare.com'
R.API_BIND='127.0.0.1';assert R.report_network(None)['lan_url'] is None
print('Network: dead relay replaced not reused, dry run inert, LAN URL reported; no network or hardware')

# OAK switch: the flag file keeps it off across deploys; ensure_oak stops a running stream instead of restarting it.
R.OAK_OFF=tmp/'oak-disabled';stopped=[]
R.oak_processes=lambda:['9 bash -c while true; do python -m farm.oak_camera stream']
R.stop_oak=lambda:stopped.append(1) or True
R.OAK_OFF.write_text('x');R.ensure_oak(False);assert stopped==[1]
R.ensure_oak(True);assert stopped==[1]  # dry run reports only
import remote_admin as A;assert A.MODES['oak-off']==['--cameras-only','--oak','off'] and A.MODES['oak-on']==['--cameras-only','--oak','on']
print('OAK switch: off persists across deploys, running stream stopped, admin modes oak-off/oak-on')

# Pinning camera IDs after cables moved: validated, head camera kept out of the wrists, saved for the next restart.
import wrist_cameras as W
root=tmp/'robot';(root/'work').mkdir(parents=True)
saved=A.set_wrist_ids(root,{'left_wrist':'0x12400005a39230','head_camera':'0x12130005a39230'})
assert saved['left_wrist']['camera_id']=='0x12400005a39230' and saved['left_wrist']['identity_verified'] is False and saved['head_camera_id']=='0x12130005a39230'
for bad in [{},{'left_wrist':'oak'},{'left_wrist':'0x12400005a39230','right_wrist':'0x12400005a39230'},{'left_wrist':'0x12130005a39230'},{'tag':'x'}]:
    try:A.set_wrist_ids(root,bad)
    except ValueError:pass
    else:raise AssertionError(f'accepted {bad}')
W.WRIST_CAMERA_IDS.update(right_wrist='0x12200005a39230',left_wrist='0x12140005a39230');W.configure(root)
assert W.WRIST_CAMERA_IDS['left_wrist']=='0x12400005a39230' and W.HEAD_CAMERA_ID=='0x12130005a39230'
# Auto-assignment never takes the saved head camera, and no stale hard-coded port is excluded.
listed=[{'camera_id':i,'name':'USB2.0_CAM1'} for i in ('0x12200005a39230','0x12130005a39230','0x12500005a39230')]
ids,ok,missing=W.resolve_ids(listed,current={'right_wrist':'0x12200005a39230','left_wrist':'0x12140005a39230'},verified={'right_wrist':True,'left_wrist':True})
assert ids=={'right_wrist':'0x12200005a39230','left_wrist':'0x12500005a39230'} and ok['left_wrist'] is False
print('Wrist IDs: pinned remotely, validated, head camera excluded by setting')
