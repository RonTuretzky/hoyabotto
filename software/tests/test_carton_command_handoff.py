import json
import sys
import time
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts/carton_robot'))
import carton_command

def test_hold_waits_for_matching_owner_ack_not_an_old_healthy_state(tmp_path,monkeypatch,capsys):
    monkeypatch.setattr(carton_command,'folder',tmp_path)
    state={'phase':'holding','time':time.time(),'rows':{},'last_hold_accepted':1}
    (tmp_path/'status.json').write_text(json.dumps(state))
    seen=[]
    def consume(_):
        seen.append(True)
        command=json.loads((tmp_path/'command.json').read_text())
        state['last_hold_accepted']=command['id']
        state['time']=time.time()
        (tmp_path/'status.json').write_text(json.dumps(state))
    monkeypatch.setattr(carton_command.time,'sleep',consume)
    carton_command.send(op='hold')
    assert seen
    assert json.loads(capsys.readouterr().out)['phase']=='holding'

def test_stop_waits_for_verified_release_not_just_stopped_phase(tmp_path,monkeypatch,capsys):
    monkeypatch.setattr(carton_command,'folder',tmp_path)
    state={'phase':'holding','time':time.time(),'rows':{}}
    (tmp_path/'status.json').write_text(json.dumps(state))
    reads=[]
    def consume(_):
        reads.append(True)
        state.update(phase='stopped',released=True,release_errors=[])
        (tmp_path/'status.json').write_text(json.dumps(state))
    monkeypatch.setattr(carton_command.time,'sleep',consume)
    carton_command.send(op='stop')
    assert reads and json.loads(capsys.readouterr().out)['phase']=='stopped'
