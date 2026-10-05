import datetime as dt
import ipaddress
import json
import os
import plistlib
import secrets
import sys
import uuid
from pathlib import Path

import cv2
from PIL import Image, ImageOps
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID, ExtendedKeyUsageOID

base=Path(__file__).resolve().parent
root=base.parents[1]
lan_ip=sys.argv[1]
config_path=base/'config.json'
config=json.loads(config_path.read_text()) if config_path.exists() else {'publisher_token':secrets.token_urlsafe(24),'viewer_token':secrets.token_urlsafe(24)}
config.update(ip=lan_ip,http_port=8876,https_port=8877)
config_path.write_text(json.dumps(config,indent=2));config_path.chmod(0o600)
now=dt.datetime.now(dt.timezone.utc)
ca_key=rsa.generate_private_key(public_exponent=65537,key_size=2048)
ca_name=x509.Name([x509.NameAttribute(NameOID.COMMON_NAME,'Robot Camera Local')])
ca=(x509.CertificateBuilder().subject_name(ca_name).issuer_name(ca_name).public_key(ca_key.public_key()).serial_number(x509.random_serial_number()).not_valid_before(now-dt.timedelta(minutes=5)).not_valid_after(now+dt.timedelta(days=7)).add_extension(x509.BasicConstraints(ca=True,path_length=0),critical=True).add_extension(x509.KeyUsage(digital_signature=True,content_commitment=False,key_encipherment=False,data_encipherment=False,key_agreement=False,key_cert_sign=True,crl_sign=True,encipher_only=False,decipher_only=False),critical=True).sign(ca_key,hashes.SHA256()))
key=rsa.generate_private_key(public_exponent=65537,key_size=2048)
name=x509.Name([x509.NameAttribute(NameOID.COMMON_NAME,'Robot camera on this Mac')])
cert=(x509.CertificateBuilder().subject_name(name).issuer_name(ca_name).public_key(key.public_key()).serial_number(x509.random_serial_number()).not_valid_before(now-dt.timedelta(minutes=5)).not_valid_after(now+dt.timedelta(days=7)).add_extension(x509.BasicConstraints(ca=False,path_length=None),critical=True).add_extension(x509.SubjectAlternativeName([x509.IPAddress(ipaddress.ip_address(lan_ip)),x509.IPAddress(ipaddress.ip_address('127.0.0.1')),x509.DNSName('Neooooo.local'),x509.DNSName('localhost')]),critical=False).add_extension(x509.ExtendedKeyUsage([ExtendedKeyUsageOID.SERVER_AUTH]),critical=False).add_extension(x509.KeyUsage(digital_signature=True,content_commitment=False,key_encipherment=True,data_encipherment=False,key_agreement=False,key_cert_sign=False,crl_sign=False,encipher_only=False,decipher_only=False),critical=True).sign(ca_key,hashes.SHA256()))
(base/'server.key').write_bytes(key.private_bytes(serialization.Encoding.PEM,serialization.PrivateFormat.PKCS8,serialization.NoEncryption()));(base/'server.key').chmod(0o600)
(base/'server.pem').write_bytes(cert.public_bytes(serialization.Encoding.PEM)+ca.public_bytes(serialization.Encoding.PEM))
(base/'camera-ca.pem').write_bytes(ca.public_bytes(serialization.Encoding.PEM))
der=ca.public_bytes(serialization.Encoding.DER)
(base/'camera-ca.cer').write_bytes(der)
# Only the certificate is distributed; this CA's signing key is never saved.
profile={'PayloadContent':[{'PayloadType':'com.apple.security.root','PayloadVersion':1,'PayloadIdentifier':'local.robotcamera.certificate','PayloadUUID':str(uuid.uuid4()),'PayloadDisplayName':'Robot Camera Local','PayloadDescription':'Trust the local camera connection to this Mac. Certificate only; no device management. Expires in seven days.','PayloadContent':der}],'PayloadType':'Configuration','PayloadVersion':1,'PayloadIdentifier':'local.robotcamera.profile','PayloadUUID':str(uuid.uuid4()),'PayloadDisplayName':'Robot Camera Local','PayloadDescription':'Certificate for the phone camera feed to this Mac. No device management.','PayloadOrganization':'Local robot camera','PayloadRemovalDisallowed':False}
(base/'camera.mobileconfig').write_bytes(plistlib.dumps(profile))
url=f'http://{lan_ip}:8876/#{config["publisher_token"]}'
viewer=f'http://127.0.0.1:8876/view#{config["viewer_token"]}'
config.update(phone_url=url,viewer_url=viewer,certificate_expires=(now+dt.timedelta(days=7)).isoformat())
config_path.write_text(json.dumps(config,indent=2))
qr=cv2.QRCodeEncoder_create().encode(url)
img=Image.fromarray(qr).convert('RGB')
img=ImageOps.expand(img,border=4,fill='white').resize(((qr.shape[1]+8)*12,(qr.shape[0]+8)*12),Image.Resampling.NEAREST)
out=root/'outputs';out.mkdir(exist_ok=True)
img.save(out/'Robot-Camera-QR.png')
(out/'Robot-Camera-Link.txt').write_text(f'Phone setup (same Wi-Fi as this Mac):\n{url}\n\nMac viewer:\n{viewer}\n\nThe phone page includes the one-time local certificate setup. Keep the camera page open after tapping Start camera. Video only, no audio. Latest frame is overwritten; no video archive.\n')
print(json.dumps({'phone_url':url,'viewer_url':viewer,'qr':str(out/'Robot-Camera-QR.png')}))
