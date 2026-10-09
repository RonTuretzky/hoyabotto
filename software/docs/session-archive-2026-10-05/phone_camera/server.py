import asyncio
import hashlib
import io
import json
import math
import os
import re
import secrets
import ssl
import time
from pathlib import Path
from aiohttp import web
from PIL import Image

BASE=Path(__file__).resolve().parent
# Load deployment secrets only in main(), never on offline import.
CONFIG={}
LATEST=BASE/'latest.jpg'
META=BASE/'latest.json'
STATE={'seq':0,'received_at':None,'received_monotonic_s':None,'width':None,'height':None,'image':None,'client':None,'frame_timing':None,'stream_id':None,'sha256':None}
TIMING_PROTOCOL='server-challenge-video-frame-v1'
CHALLENGE_TTL_S=1.0
CLOCK_TOLERANCE_S=0.05
SERVER_INSTANCE=secrets.token_hex(16)
CHALLENGES={}
# Bounded stream progression history; cleared on explicit camera release.
PROGRESS={}

def finite_number(value):
    try:return type(value) in (int,float) and math.isfinite(value) and value>=0
    except OverflowError:return False

def publisher(request):
    auth(request,'publisher')
    if not request.secure:
        raise web.HTTPForbidden(text='Camera uploads require HTTPS.')
    client=request.headers.get('X-Camera-Client','')
    if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,100}',client):
        raise web.HTTPBadRequest(text='Missing or invalid camera identity.')
    if STATE['client'] not in (None,client) and STATE['received_monotonic_s'] is not None and time.monotonic()-STATE['received_monotonic_s']<10:
        raise web.HTTPConflict(text='Another phone is currently sharing. Stop that camera first.')
    return client

async def challenge(request):
    client=publisher(request)
    stream=request.headers.get('X-Camera-Stream','')
    if not re.fullmatch(r'[A-Za-z0-9_.-]{1,100}',stream):
        raise web.HTTPBadRequest(text='Missing or invalid browser stream identity.')
    now=time.monotonic()
    for key,row in list(CHALLENGES.items()):
        if now-row['issued_monotonic_s']>CHALLENGE_TTL_S:
            del CHALLENGES[key]
    if client not in CHALLENGES and len(CHALLENGES)>=64:
        raise web.HTTPTooManyRequests(text='Too many pending capture challenges.')
    row={'id':secrets.token_urlsafe(24),'stream':stream,'issued_at':time.time(),'issued_monotonic_s':time.monotonic()}
    CHALLENGES[client]=row  # At most one outstanding challenge per publisher.
    return web.json_response({'protocol':TIMING_PROTOCOL,'challenge_id':row['id'],'expires_in_s':CHALLENGE_TTL_S})

def consume_timing(request,client,received_at,received_mono):
    raw=request.headers.get('X-Camera-Timing')
    if raw is None:
        if STATE['client']==client and STATE['frame_timing'] is not None:
            raise web.HTTPBadRequest(text='A timed stream cannot downgrade to receipt-only uploads.')
        return None,None
    row=CHALLENGES.pop(client,None)  # Even a rejected attempt consumes the challenge.
    try:
        if len(raw)>2048:raise ValueError()
        value=json.loads(raw)
        if not isinstance(value,dict) or row is None:raise ValueError()
        elapsed=received_mono-row['issued_monotonic_s']
        if (value.get('protocol')!=TIMING_PROTOCOL or value.get('challenge_id')!=row['id']
                or value.get('browser_stream_id')!=row['stream'] or not 0<=elapsed<=CHALLENGE_TTL_S
                or not 0<=received_at-row['issued_at']
                or abs((received_at-row['issued_at'])-elapsed)>CLOCK_TOLERANCE_S):raise ValueError()
        count=value.get('presented_frames')
        fields=('media_time_s','challenge_received_ms','callback_now_ms','presentation_time_ms')
        if type(count) is not int or not 1<=count<=2**53-1 or not all(finite_number(value.get(k)) for k in fields):raise ValueError()
        if not value['challenge_received_ms']<=value['presentation_time_ms']<=value['callback_now_ms']:raise ValueError()
        if (value['callback_now_ms']-value['challenge_received_ms'])/1000>elapsed+CLOCK_TOLERANCE_S:raise ValueError()
        stream_id=SERVER_INSTANCE+':'+client+':'+row['stream']
        previous=PROGRESS.get(stream_id)
        if previous and (count<=previous[0] or value['media_time_s']<=previous[1]):raise ValueError()
        if previous is None and len(PROGRESS)>=64:raise ValueError()
        timing={k:value[k] for k in fields}
        timing.update(protocol=TIMING_PROTOCOL,challenge_id=row['id'],browser_stream_id=row['stream'],
                      presented_frames=count,server_instance_id=SERVER_INSTANCE,clock_domain='phone_server_monotonic',
                      challenge_issued_at=row['issued_at'],challenge_issued_monotonic_s=row['issued_monotonic_s'],
                      received_monotonic_s=received_mono,browser_frame_age_upper_bound_s_at_receipt=elapsed,
                      sensor_exposure_at=None,sensor_exposure_age_s=None)
        return timing,stream_id
    except (ValueError,TypeError,KeyError):
        raise web.HTTPBadRequest(text='Invalid, expired, or replayed browser-frame timing.') from None

@web.middleware
async def headers(request,handler):
    response=await handler(request)
    response.headers.update({'Cache-Control':'no-store','Referrer-Policy':'no-referrer','X-Content-Type-Options':'nosniff','Permissions-Policy':'camera=(self), microphone=(), geolocation=()','X-Frame-Options':'DENY'})
    return response

def auth(request,role):
    value=request.headers.get('Authorization','')
    if value!='Bearer '+CONFIG[role+'_token']:
        raise web.HTTPUnauthorized(text='Camera link is missing or expired.')

def metadata():
    stamp=STATE['received_at']
    evaluated_at=time.time()
    age=None if stamp is None else time.monotonic()-STATE['received_monotonic_s']
    timing=STATE['frame_timing']
    clocks_ok=age is not None and 0<=age and 0<=evaluated_at-stamp and abs(evaluated_at-stamp-age)<=CLOCK_TOLERANCE_S
    bound=(timing['browser_frame_age_upper_bound_s_at_receipt']+max(age,evaluated_at-stamp)
           if timing is not None and clocks_ok else None)
    return {'seq':STATE['seq'],'received_at':stamp,'captured_at':None,
            'age_s':age,'live':clocks_ok and age<2,'width':STATE['width'],'height':STATE['height'],
            'stream_id':STATE['stream_id'],'sha256':STATE['sha256'],'frame_timing':timing,
            'freshness_basis':'browser_video_frame' if timing else 'server_receipt_only',
            'browser_frame_age_upper_bound_s':bound,'browser_frame_age_evaluated_at':evaluated_at,
            'browser_frame_fresh':bound is not None and 0<=bound<=1,
            'timestamp_semantics':('server challenge bounds browser frame presentation; physical sensor exposure unknown'
                                   if timing else 'server receipt; capture delay unknown')}

async def page(request):
    which={'/':'setup.html','/camera':'camera.html','/view':'viewer.html'}[request.path]
    content=(BASE/which).read_text().replace('__CAMERA_ORIGIN__',f'https://{CONFIG["ip"]}:{CONFIG["https_port"]}')
    return web.Response(text=content,content_type='text/html')

async def cert(request):
    name={'/camera.mobileconfig':'camera.mobileconfig','/camera-ca.cer':'camera-ca.cer'}[request.path]
    mime='application/x-apple-aspen-config' if name.endswith('mobileconfig') else 'application/x-x509-ca-cert'
    return web.Response(body=(BASE/name).read_bytes(),content_type=mime,headers={'Content-Disposition':f'attachment; filename="{name}"'})

async def frame(request):
    client=publisher(request)
    if request.content_type!='image/jpeg':raise web.HTTPUnsupportedMediaType()
    data=await request.read()
    # Recheck after the only await: an overlapping publisher may have committed.
    publisher(request)
    received_at,received_mono=time.time(),time.monotonic()
    timing,stream_id=consume_timing(request,client,received_at,received_mono)
    try:
        with Image.open(io.BytesIO(data)) as im:
            if im.format!='JPEG' or im.width>1920 or im.height>1920 or im.width<10 or im.height<10:
                raise ValueError('Invalid frame size')
            width,height=im.size
            im.verify()
        with Image.open(io.BytesIO(data)) as im:
            im.load()
    except Exception:
        raise web.HTTPBadRequest(text='Invalid JPEG frame.')
    if timing:
        PROGRESS[stream_id]=(timing['presented_frames'],timing['media_time_s'])
    STATE.update(seq=STATE['seq']+1,received_at=received_at,received_monotonic_s=received_mono,
                 width=width,height=height,image=data,client=client,frame_timing=timing,
                 stream_id=stream_id or SERVER_INSTANCE+':legacy:'+client,sha256=hashlib.sha256(data).hexdigest())
    temp=BASE/'latest.tmp';temp.write_bytes(data);os.replace(temp,LATEST)
    info=metadata();temp=BASE/'latest-meta.tmp';temp.write_text(json.dumps(info));os.replace(temp,META)
    return web.json_response(info)

async def status(request):
    auth(request,'viewer')
    return web.json_response(metadata())

async def image(request):
    auth(request,'viewer')
    if STATE['image'] is None:raise web.HTTPNotFound(text='Waiting for the phone camera.')
    return web.Response(body=STATE['image'],content_type='image/jpeg',headers={'X-Frame-Received':str(STATE['received_at'])})

async def stop(request):
    auth(request,'publisher')
    if not request.secure:
        raise web.HTTPForbidden(text='Camera control requires HTTPS.')
    client=request.headers.get('X-Camera-Client','')
    if STATE['client'] in (None,client):
        STATE.update(received_at=None,received_monotonic_s=None,image=None,client=None,frame_timing=None,stream_id=None,sha256=None)
        CHALLENGES.clear();PROGRESS.clear()
        LATEST.unlink(missing_ok=True)
        META.write_text(json.dumps(metadata()))
    return web.json_response({'stopped':True})

async def main():
    CONFIG.update(json.loads((BASE/'config.json').read_text()))
    app=web.Application(client_max_size=2*1024*1024,middlewares=[headers])
    app.add_routes([web.get('/',page),web.get('/camera',page),web.get('/view',page),web.get('/camera.mobileconfig',cert),web.get('/camera-ca.cer',cert),web.post('/api/challenge',challenge),web.post('/api/frame',frame),web.get('/api/status',status),web.get('/api/latest.jpg',image),web.post('/api/stop',stop)])
    runner=web.AppRunner(app,access_log=None);await runner.setup()
    context=ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER);context.minimum_version=ssl.TLSVersion.TLSv1_2
    context.load_cert_chain(BASE/'server.pem',BASE/'server.key')
    # Bind only loopback and this Mac's private LAN address, not every interface.
    sites=[]
    for host in ['127.0.0.1',CONFIG['ip']]:
        for port,ctx in [(CONFIG['http_port'],None),(CONFIG['https_port'],context)]:
            site=web.TCPSite(runner,host,port,ssl_context=ctx);await site.start();sites.append(site)
    (BASE/'server.pid').write_text(str(os.getpid()))
    LATEST.unlink(missing_ok=True);META.write_text(json.dumps(metadata()))
    print('Local phone camera ready. Waiting for the phone to start sharing.',flush=True)
    try: await asyncio.Event().wait()
    finally: await runner.cleanup()

if __name__=='__main__':asyncio.run(main())
