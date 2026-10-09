"""Prepare matching NVIDIA user-space EGL libraries in a private temporary directory.

Some compute images expose CUDA but omit OpenGL libraries. This extracts only
the vendor's userspace libraries; it never installs or changes a kernel driver.
"""
from pathlib import Path
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import urllib.error
import urllib.request


def prepare(destination):
    root = Path(destination)
    root.mkdir(parents=True,exist_ok=False)
    version = subprocess.check_output(['nvidia-smi','--query-gpu=driver_version',
        '--format=csv,noheader'],text=True).splitlines()[0].strip()
    if not re.fullmatch(r'\d+\.\d+\.\d+',version):
        raise RuntimeError(f'Unexpected driver version: {version}')
    filename = f'NVIDIA-Linux-x86_64-{version}.run'
    installer = root/filename
    candidates = [f'https://us.download.nvidia.com/tesla/{version}/{filename}',
                  f'https://download.nvidia.com/XFree86/Linux-x86_64/{version}/{filename}']
    for url in candidates:
        try:
            with urllib.request.urlopen(url,timeout=60) as response, installer.open('wb') as out:
                shutil.copyfileobj(response,out)
            break
        except urllib.error.HTTPError as exc:
            if exc.code != 404:
                raise
    else:
        raise RuntimeError(f'Matching userspace graphics package unavailable for {version}')
    extracted = root/'extracted'
    subprocess.run(['sh',str(installer),'--extract-only','--target',str(extracted)],
                   check=True,timeout=120)
    libraries = root/'lib'
    libraries.mkdir()
    patterns = ['libEGL_nvidia.so.*','libGLX_nvidia.so.*','libnvidia-eglcore.so.*',
                'libnvidia-glcore.so.*','libnvidia-glsi.so.*','libnvidia-glvkspirv.so.*',
                'libnvidia-tls.so.*']
    for pattern in patterns:
        for p in extracted.glob(pattern):
            shutil.copy2(p,libraries/p.name)
    for name in ('libEGL_nvidia','libGLX_nvidia'):
        target = libraries/f'{name}.so.{version}'
        if not target.is_file():
            raise RuntimeError(f'Missing userspace library {name}')
        (libraries/f'{name}.so.0').symlink_to(target.name)
    icd = root/'10_nvidia.json'
    icd.write_text(json.dumps({'file_format_version':'1.0.0','ICD':{
        'library_path':str(libraries/'libEGL_nvidia.so.0')}}))
    with installer.open('rb') as f:
        digest = hashlib.file_digest(f,'sha256').hexdigest()
    report = dict(driver_version=version,source=url,sha256=digest,libraries=str(libraries),
                  egl_vendor_file=str(icd),kernel_driver_modified=False)
    (root/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    return report


if __name__ == '__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',required=True)
    a=ap.parse_args()
    prepare(a.out)
