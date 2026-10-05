import asyncio
import io
import json
import os
import ssl
import time
from pathlib import Path
from aiohttp import web
from PIL import Image

BASE=Path(__file__).resolve().parent
CONFIG=json.loads((BASE/'config.json').read_text())
LATEST=BASE/'latest.jpg'
META=BASE/'latest.json'
STATE={'seq':0,'received_at':None,'width':None,'height':None,'image':None,'client':None}

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
    age=None if stamp is None else time.time()-stamp
    return {'seq':STATE['seq'],'received_at':stamp,'age_s':None if age is None else round(age,3),'live':age is not None and age<2,'width':STATE['width'],'height':STATE['height']}

async def page(request):
    which={'/':'setup.html','/camera':'camera.html','/view':'viewer.html'}[request.path]
    content=(BASE/which).read_text().replace('__CAMERA_ORIGIN__',f'https://{CONFIG["ip"]}:{CONFIG["https_port"]}')
    return web.Response(text=content,content_type='text/html')

async def cert(request):
    name={'/camera.mobileconfig':'camera.mobileconfig','/camera-ca.cer':'camera-ca.cer'}[request.path]
    mime='application/x-apple-aspen-config' if name.endswith('mobileconfig') else 'application/x-x509-ca-cert'
    return web.Response(body=(BASE/name).read_bytes(),content_type=mime,headers={'Content-Disposition':f'attachment; filename="{name}"'})

async def frame(request):
    auth(request,'publisher')
    if not request.secure:
        raise web.HTTPForbidden(text='Camera uploads require HTTPS.')
    client=request.headers.get('X-Camera-Client','')[:100]
    if not client: raise web.HTTPBadRequest(text='Missing camera identity.')
    if STATE['client'] not in (None,client) and STATE['received_at'] and time.time()-STATE['received_at']<10:
        raise web.HTTPConflict(text='Another phone is currently sharing. Stop that camera first.')
    if request.content_type!='image/jpeg':raise web.HTTPUnsupportedMediaType()
    data=await request.read()
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
    STATE.update(seq=STATE['seq']+1,received_at=time.time(),width=width,height=height,image=data,client=client)
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
        STATE.update(received_at=None,image=None,client=None)
        LATEST.unlink(missing_ok=True)
        META.write_text(json.dumps(metadata()))
    return web.json_response({'stopped':True})

async def main():
    app=web.Application(client_max_size=2*1024*1024,middlewares=[headers])
    app.add_routes([web.get('/',page),web.get('/camera',page),web.get('/view',page),web.get('/camera.mobileconfig',cert),web.get('/camera-ca.cer',cert),web.post('/api/frame',frame),web.get('/api/status',status),web.get('/api/latest.jpg',image),web.post('/api/stop',stop)])
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
