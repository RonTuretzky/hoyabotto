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
