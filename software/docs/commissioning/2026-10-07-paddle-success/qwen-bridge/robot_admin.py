"""Operator client for the robot's /admin API, run on the chat Mac (uses the chat's paired mTLS client certificate).

  python robot_admin.py status
  python robot_admin.py logs [owner,api,redeploy,camera_setup,last_job,...] [--lines 120]
  python robot_admin.py deploy [REF] [--mode restart|cameras-only|dry-run|checkout-only] [--no-wait]
  python robot_admin.py job JOB_ID
"""
import argparse,json,os,ssl,sys,time,urllib.error,urllib.request
CONFIG=os.environ.get('XLEROBOT_ADMIN_CONFIG','/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/robot.json')

def request(path,payload=None,timeout=20):
    c=json.load(open(CONFIG));ctx=ssl.create_default_context(cafile=c['server_certificate']);ctx.load_cert_chain(c['client_certificate'],c['client_key'])
    req=urllib.request.Request(c['url'].rstrip('/')+path,data=None if payload is None else json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
    try:
        with urllib.request.urlopen(req,context=ctx,timeout=timeout) as r:return json.loads(r.read())
    except urllib.error.HTTPError as e:return json.loads(e.read() or b'{}')

def show(value):print(json.dumps(value,indent=2))

def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    sub=ap.add_subparsers(dest='cmd',required=True)
    sub.add_parser('status')
    lg=sub.add_parser('logs');lg.add_argument('names',nargs='?',default='');lg.add_argument('--lines',type=int,default=80)
    dp=sub.add_parser('deploy');dp.add_argument('ref',nargs='?',default='main');dp.add_argument('--mode',default='restart');dp.add_argument('--no-wait',action='store_true')
    jb=sub.add_parser('job');jb.add_argument('id')
    a=ap.parse_args()
    if a.cmd=='status':return show(request('/admin/deploy'))
    if a.cmd=='logs':return show(request(f'/admin/logs?names={a.names}&lines={a.lines}'))
    if a.cmd=='job':return show(request(f'/admin/job?id={a.id}'))
    started=request('/admin/deploy',{'ref':a.ref,'mode':a.mode})
    if not started.get('ok') or a.no_wait:return show(started)
    job_id=started['job']['id'];print(f'job {job_id} started ({a.ref}, {a.mode}); waiting…',flush=True)
    deadline=time.time()+600
    while time.time()<deadline:
        time.sleep(3)
        try:job=request(f'/admin/job?id={job_id}&lines=400')
        except Exception:continue  # the API restarts during a deploy
        if job.get('ok') and job['job'].get('state')!='running':
            print('\n'.join(job['job'].get('log',{}).get('lines',[])));show({k:v for k,v in job['job'].items() if k!='log'})
            sys.exit(0 if job['job']['state']=='succeeded' else 1)
    sys.exit('Timed out waiting for the job; check with: python robot_admin.py job '+job_id)

if __name__=='__main__':main()
