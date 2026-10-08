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
