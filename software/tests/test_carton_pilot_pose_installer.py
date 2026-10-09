"""Source-only carton integration: hook refusal, repeat install and read-only dispatch."""
import ast
import importlib.util
from pathlib import Path

import pytest

TOOLS = Path(__file__).resolve().parents[1] / 'tools'
def load(name):
    spec = importlib.util.spec_from_file_location(name, TOOLS / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

pose = load('install_carton_pilot_pose')
eyes = load('install_carton_pilot_eyes')

CHAT = '''from farm.perception.carton_tags import CartonTagRobot, carton_tag_lines
EYES_SYSTEM="old eyes"
SUPERVISOR_SYSTEM = """sense and/or "heights", "tags"
Carton AprilTags: sense with what ["tags"]"""
class Chat:
    def sense(self,what,send):
        out=[]
        if 'heights' in what:
            out.append('heights')
        return out
    def allowed(self):
        return ('state','motion','scene','heights','tags')
class Handler:
    def do_GET(self):
            if self.path=='/api/carton-tags':
                return self.reply(200, {})
def main(args):
    chat=Chat(CartonTagRobot(CalibrationRobot(TagRobot(TwinRobot(Robot(args.config))))))
'''
BACKEND = '''WHAT = ['state', 'motion', 'scene', 'heights', 'tags']
DESCRIPTION = 'sense only (tags: panel IDs)'
'''

def test_patches_compose_and_dispatch_without_motion():
    sources={'chat_server.py':CHAT,'model_backend.py':BACKEND}
    patched=pose.patch_sources(sources)
    assert pose.patch_sources(patched)==patched
    patched['chat_server.py']=eyes.replace_prompt(patched['chat_server.py'],'new factual eyes')
    assert pose.patch_sources(patched)==patched
    tree=ast.parse(patched['chat_server.py'])
    classes=[n for n in tree.body if isinstance(n,ast.ClassDef)]
    calls=[]
    class Robot:
        def call(self,name,args):
            calls.append((name,args));return {'ok':False,'error':'Head moved'}
    fake=type('Fake',(),{'robot':Robot()})()
    ns={'carton_pose_lines':lambda r:['refused: '+r['error']], 'chat':fake}
    exec(compile(ast.Module(body=classes,type_ignores=[]),'staged','exec'),ns)
    sensor=ns['Chat']()
    sensor.call_tool=lambda name,args,send:fake.robot.call(name,args)
    assert sensor.sense(['carton'],None)==['refused: Head moved']
    handler=ns['Handler']();handler.path='/api/carton-pose';handler.reply=lambda code,data:(code,data)
    assert handler.do_GET()==(502,{'ok':False,'error':'Head moved'})
    assert calls==[('robot_get_carton_pose',{'include_images':False}),('robot_get_carton_pose',{'include_images':True})]


def test_install_refuses_changed_backend_before_any_write(tmp_path):
    (tmp_path/'chat_server.py').write_text(CHAT)
    (tmp_path/'model_backend.py').write_text('changed layout')
    with pytest.raises(ValueError,match='layout changed'):
        pose.install(tmp_path)
    assert (tmp_path/'chat_server.py').read_text()==CHAT
    assert not (tmp_path/'.private').exists()


def test_install_backs_up_originals_and_repeat_is_noop(tmp_path):
    for name,source in [('chat_server.py',CHAT),('model_backend.py',BACKEND)]:
        (tmp_path/name).write_text(source)
    result=pose.install(tmp_path)
    assert result['restart_required'] and result['motor_writes']==0
    backups=list((tmp_path/'.private/carton-pose-backups').glob('*.py'))
    assert sorted(p.read_text() for p in backups)==sorted([CHAT,BACKEND])
    assert pose.install(tmp_path)['restart_required'] is False
    with pytest.raises(ValueError,match='Duplicate'):
        pose.replace_once('new new','old','new')


def test_eyes_only_replaces_literal_prompt_and_handles_unicode():
    source='label="日本語"; EYES_SYSTEM="old"\ndef action():\n    return "unchanged"\n'
    result=eyes.replace_prompt(source,'actual pads; no invented cm')
    ns={};exec(result,ns)
    assert ns['EYES_SYSTEM']=='actual pads; no invented cm' and ns['action']()=='unchanged'
    assert eyes.replace_prompt(result,ns['EYES_SYSTEM'])==result
    with pytest.raises(ValueError):eyes.replace_prompt('EYES_SYSTEM=compute_prompt()','x')


def test_supervisor_reconciliation_removes_conflicting_metric_and_pinch_claims():
    push = load('install_carton_pilot_push_policy')
    fixtures = Path(__file__).parent / 'fixtures/carton_pilot_push'
    sources = {n: (fixtures / n).read_text() for n in push.FILES}
    patched = push.patch_sources(sources)
    updated = eyes.reconcile_supervisor(patched['chat_server.py'])
    assert eyes.reconcile_supervisor(updated) == updated
    assert push.patch_sources(dict(patched, **{'chat_server.py': updated}))['chat_server.py'] == updated
    assert push.text_only_ast(updated) == push.text_only_ast(patched['chat_server.py'])
    for bad in ('Trust the verdict over the image', 'get the edge 1-2 cm INSIDE the zone',
                'ID 12 is the right short flap', 'image direction by about N cm'):
        assert bad not in updated
    for expected in ('11 is the robot-right short flap', 'unvalidated overlays',
                     'speed_profile "normal" or "demo"', '300 ticks/second', 'not a speed profile'):
        assert expected in updated
    with pytest.raises(ValueError, match='guidance changed'):
        eyes.reconcile_supervisor(patched['chat_server.py'].replace('1-2 cm INSIDE', '4-5 cm INSIDE'))
