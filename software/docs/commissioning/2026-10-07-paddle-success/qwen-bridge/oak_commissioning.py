"""Bounded OAK-only full-sensor trial. Never installs/restarts API, wrists or motor owner.
Runs from a child ref of the Joy-Con bootstrap. Failure restores the previous camera source.
"""
import argparse, hashlib, json, os, shlex, shutil, signal, subprocess, time
from pathlib import Path
import redeploy_robot_server as d

PROFILE = ['--full-sensor', '--isp-denominator', '4', '--fps', '5']

def manifest():
    try: return json.loads((Path(d.OAK_RAW_DIR)/'oak.json').read_text())
    except (OSError,ValueError): return {}

def start(python, software, flags, log):
    args=[python,'-m','farm.oak_camera','stream','--usb2','--wide',*flags,'--seconds','86400','--output',d.OAK_RAW_DIR]
    loop='while true; do '+shlex.join(args)+'; echo "oak stream exited ($?); restarting in 5 s"; sleep 5; done'
    with log.open('ab') as f:
        return subprocess.Popen(['bash','-c',loop],cwd=str(software),stdin=subprocess.DEVNULL,stdout=f,stderr=f,start_new_session=True)

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--dry-run',action='store_true')
    ap.add_argument('--cameras-only',action='store_true')
    ap.add_argument('--oak',choices=['on','off'])
    a=ap.parse_args()
    if a.oak: raise ValueError('This commissioning ref only accepts dry-run or cameras-only')
    owners=d.processes('gemma_hardware_owner.py'); apis=d.processes('gemma_robot_tools.py')
    before=d.read_status(); previous=manifest()
    if len(owners)!=1 or not previous or time.time()-previous['captured_at']>2:
        raise RuntimeError('Need one existing owner and a fresh baseline camera; changed nothing')
    checkout=d.SOFTWARE.parent
    dirty=subprocess.check_output(['git','status','--porcelain'],cwd=checkout,text=True)
    record={'profile':PROFILE,'owner_pids':owners,'api_pids':apis,'owner_status':before,
            'previous_manifest':previous,'dirty_paths':dirty,'camera_only':True,'motor_commands':0}
    print(json.dumps(record),flush=True)
    if a.dry_run:return
    if not a.cameras_only:raise ValueError('Explicit cameras-only required')
    if d.OAK_OFF.exists():raise RuntimeError('OAK explicitly disabled; changed nothing')
    stamp=time.strftime('%Y%m%d-%H%M%S'); backup=d.WORK/'oak-commissioning'/stamp
    backup.mkdir(parents=True)
    # Snapshot the parent's known baseline, without modifying any dirty remote files.
    baseline=subprocess.check_output(['git','show','8812ee4c3a9e67573ca92257ea06a3da7f07bcad:software/farm/oak_camera.py'],cwd=checkout)
    oldsoftware=backup/'baseline';(oldsoftware/'farm').mkdir(parents=True)
    (oldsoftware/'farm/__init__.py').write_text('')
    (oldsoftware/'farm/oak_camera.py').write_bytes(baseline)
    (backup/'before.json').write_text(json.dumps(record,indent=2))
    python=d.find_oak_python(d.SOFTWARE)
    if not python:raise RuntimeError('No OAK interpreter found; changed nothing')
    if not d.stop_oak():raise RuntimeError('Prior OAK process did not exit; will not open another')
    time.sleep(3)
    log=d.WORK/'oak-stream.log'
    proc=start(python,d.SOFTWARE,PROFILE,log)
    success=False
    try:
        deadline=time.monotonic()+100; samples=[]; streams=set(); first=None
        while time.monotonic()<deadline:
            time.sleep(1)
            m=manifest()
            if m.get('stream_id')==previous['stream_id'] or time.time()-m.get('captured_at',0)>1:continue
            if m.get('rgb_sensor_mode')!='13MP':raise RuntimeError('Unexpected camera configuration')
            streams.add(m['stream_id']); samples.append({k:m.get(k) for k in ('stream_id','seq','captured_at','rgb_captured_at','depth_captured_at','rgb_device_seq','depth_device_seq')})
            first=first or time.monotonic()
            if len(streams)>1:raise RuntimeError('OAK restarted during stability trial')
            if time.monotonic()-first>=60:
                if len(samples)<50:raise RuntimeError('Too few fresh observations')
                success=True;break
        if not success:raise RuntimeError('No stable 60-second full-sensor stream before deadline')
        if d.processes('gemma_hardware_owner.py')!=owners or d.processes('gemma_robot_tools.py')!=apis:
            raise RuntimeError('Owner/API process changed concurrently; trial invalid')
        final={'success':True,'profile':PROFILE,'samples':samples,'manifest':manifest(),
               'owner_pids_unchanged':True,'api_pids_unchanged':True,'after':d.read_status(),'backup':str(backup)}
        (backup/'result.json').write_text(json.dumps(final,indent=2));print(json.dumps(final),flush=True)
    except BaseException as e:
        if not d.stop_oak():raise RuntimeError('Cannot stop failed trial; refusing duplicate camera owner') from e
        time.sleep(3);start(python,oldsoftware,[],log)
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            m=manifest()
            if m.get('rgb_sensor_mode')!='13MP' and m.get('stream_id')!=previous['stream_id'] and time.time()-m.get('captured_at',0)<1:break
            time.sleep(.5)
        print(json.dumps({'success':False,'error':str(e),'rollback_manifest':manifest(),'backup':str(backup)}),flush=True)
        raise

if __name__=='__main__':main()
