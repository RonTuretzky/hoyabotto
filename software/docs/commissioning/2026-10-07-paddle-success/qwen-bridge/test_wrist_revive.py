import json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
import wrist_cameras as W
with tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp);work=root/'work';work.mkdir()
    fake=work/'capture-single'
    fake.write_text('#!'+sys.executable+'\nimport sys,json,time,hashlib\nfrom pathlib import Path\nd=Path(sys.argv[1]);name,cid=sys.argv[2].split("=")\n'
                    'for seq in range(1,8):\n data=b"x"*seq;img=f"{name}-{seq}.jpg";(d/img).write_bytes(data)\n'
                    ' (d/(name+".json")).write_text(json.dumps({"camera_id":cid,"stream_id":"s","seq":seq,"captured_at":time.time(),"image":img,"sha256":hashlib.sha256(data).hexdigest()}));time.sleep(.2)\n'
                    'time.sleep(60)  # stalls: alive, no frames\n')
    fake.chmod(0o755)
    os.environ.pop('XLEROBOT_WRIST_DIRS',None)
    try:W.select_wrist_manifest('left_wrist',W.wrist_dirs(root));raise SystemExit('expected no frames')
    except RuntimeError:pass
    t=[1000.0];clock=lambda:t[0]
    def sleep(s):time.sleep(s);t[0]+=s
    assert W.revive('left_wrist',root,clock=clock,sleep=sleep) is True
    W.select_wrist_manifest('left_wrist',W.wrist_dirs(root))  # fresh now
    assert W.revive('left_wrist',root,clock=clock,sleep=sleep) is False  # rate limited (<10 s)
    time.sleep(2.5)  # the fake stalls; frame goes stale
    try:W.select_wrist_manifest('left_wrist',W.wrist_dirs(root));raise SystemExit('expected stale')
    except RuntimeError as x:assert 'stale' in str(x)
    t[0]+=11;assert W.revive('left_wrist',root,clock=clock,sleep=sleep) is True
    out=subprocess.run(['ps','-axo','pid=,args='],capture_output=True,text=True).stdout
    for line in out.splitlines():
        if str(fake) in line:os.kill(int(line.split()[0]),signal.SIGTERM)
print('Wrist revive: stalled stream restarted on demand, fresh frame returned, rate limit honoured; no camera access')
