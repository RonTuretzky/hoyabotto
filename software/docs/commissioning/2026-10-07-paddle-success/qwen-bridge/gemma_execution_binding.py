"""Local-only bridge binding; never imports drivers, starts an owner or arms one.

Trusted station integration injects a reviewed GemmaJointAdapter and a provider
for its fresh locally validated inference reference. No HTTP route can install,
replace or arm the binding. All actual dispatch remains ContinuousTransport.play.
"""
import importlib.util
import sys
import threading
from pathlib import Path
from carton.servo.continuous import ContinuousTransport

path=Path(__file__).resolve().parents[1]/'outputs/Gemma-Joint-Adapter.py'
spec=importlib.util.spec_from_file_location('gemma_joint_adapter',path)
module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
GemmaJointAdapter=module.GemmaJointAdapter

class TrustedExecutionBinding:
    def __init__(self,session):
        self.session=Path(session).resolve()
        self.adapter=None;self.reference_provider=None;self.source=None
        self.action_lock=threading.Lock()
    def bind(self,adapter,reference_provider,*,source):
        if self.adapter is not None:raise RuntimeError('Existing trusted binding cannot be replaced')
        if not isinstance(adapter,GemmaJointAdapter) or not isinstance(adapter.transport,ContinuousTransport):
            raise TypeError('Actual GemmaJointAdapter/ContinuousTransport required')
        if Path(adapter.transport.config['session_dir']).resolve()!=self.session:
            raise ValueError('Adapter session differs from independently bound owner')
        if not callable(reference_provider) or not isinstance(source,str) or not source.strip():
            raise ValueError('Trusted local reference provider/source required')
        self.adapter=adapter;self.reference_provider=reference_provider;self.source=source
    def readiness(self):
        result={'execution_adapter_bound':self.adapter is not None,'adapter_source':self.source,
                'controller_api':'GemmaJointAdapter.execute -> ContinuousTransport.play -> sole owner',
                'local_operator_gate':False,'owner_continuous_ready':False,'motion_ready':False,
                'remote_arming_supported':False}
        if self.adapter is None:
            result['blocker']='MISSING_TRUSTED_EXECUTION_ADAPTER'
            result['implementation_ready']=True
            result['binding_installed']=False
            result['construction_blocked_by']='ContinuousTransport constructor immediately calls validate_profile(profile, config); no real commissioned station profile has been supplied.'
            result['required_local_inputs']=['schema1 encoder_ticks profile for exact six selected arm joints', 'real commissioning_evidence, measured joint corridors and velocity/acceleration bounds', 'station config fingerprint/calibration hash and independently registered camera pair', 'fresh trusted visual-context and inference-reference providers', 'existing healthy sole owner with matching profile/protocol', 'local operator/watchdog gate (defaults false)']
            result['source_conditions']={'constructor':'carton/servo/continuous.py:ContinuousTransport.__init__ -> validate_profile(profile, config)', 'binding':'work/gemma_execution_binding.py:TrustedExecutionBinding.bind requires actual GemmaJointAdapter and ContinuousTransport for exact bound session', 'remote_install_or_arm_tool':False}
            return result
        try:
            result['local_operator_gate']=self.adapter.operator_gate() is True
            result['execute_enabled']=self.adapter.transport.execute is True
            status,_=self.adapter.transport.status()
            result['owner_continuous_ready']=status.get('phase')=='holding'
            result['owner_started']=status.get('started')
            result['profile_sha256']=self.adapter.transport.profile_hash
            result['arm']=self.adapter.transport.config['arm']
            result['motion_ready']=result['local_operator_gate'] and result['execute_enabled'] and result['owner_continuous_ready']
            if not result['motion_ready']:result['blocker']='LOCAL_GATE_OR_OWNER_NOT_READY'
        except Exception as exc:result['blocker']=str(exc)
        return result
    def execute_joint(self,args):
        ready=self.readiness()
        if not ready['motion_ready']:
            return {'accepted':False,'motor_writes':0,'reason':ready['blocker'],'execution_binding':ready}
        if args['arm']!=ready['arm']:raise ValueError('Request arm differs from commissioned owner')
        if not self.action_lock.acquire(blocking=False):raise RuntimeError('An adapter trajectory request is active')
        try:
            # Writer context uses the existing cross-client lock and aborts on
            # failure. It never creates or enables the physical motor owner.
            with self.adapter.transport:
                reference=self.reference_provider()
                result=self.adapter.execute({'units':'encoder_ticks','positions':args['positions'],
                                             'duration_s':args['duration_s']},reference)
            return {'accepted':True,'execution_result':result,
                    'controller_api':ready['controller_api'],'grasp_verified':False}
        finally:self.action_lock.release()
