"""Build a credential-free portable source/assets bundle for offline refit jobs."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import tarfile
import xml.etree.ElementTree as ET
from tools.run_claw_sweep import snapshot_sources


def package(software, simulation_root, destination):
    software, simulation_root, destination = map(Path, (software, simulation_root, destination))
    destination.mkdir(parents=True, exist_ok=False)
    snapshot_sources(software, destination/'software')
    sim = destination/'simulation'
    for relative in ('scene-assets', 'real-scene/assets', 'upstream/assets/robots/xlerobot'):
        shutil.copytree(simulation_root/relative, sim/relative)
    # Put all imported arm meshes on absolute paths at runtime: scene.xml is
    # written in a per-trial output directory, not beside arm-import.xml.
    xml = sim/'scene-assets/arm-import.xml'
    tree = ET.parse(xml)
    compiler = tree.getroot().find('compiler')
    compiler.set('meshdir', '__SIMULATION_ROOT__/real-scene/assets')
    tree.write(xml, encoding='unicode')
    manifest = sim/'scene-assets/jaw-collision/manifest.json'
    value = json.loads(manifest.read_text())
    for spec in value.values():
        spec['files'] = ['__SIMULATION_ROOT__/scene-assets/jaw-collision/'+Path(f).name for f in spec['files']]
    manifest.write_text(json.dumps(value, indent=2))
    hashes = {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in sorted(destination.rglob('*')) if p.is_file()}
    (destination/'manifest.json').write_text(json.dumps(dict(files=hashes, hardware_commands=False), indent=2))
    archive = destination.with_suffix('.tar.gz')
    with tarfile.open(archive, 'w:gz') as tar:
        tar.add(destination, arcname='refit')
    return archive


def relocate(simulation_root):
    root = Path(simulation_root).resolve()
    for name in ('scene-assets/arm-import.xml', 'scene-assets/jaw-collision/manifest.json'):
        path = root/name
        path.write_text(path.read_text().replace('__SIMULATION_ROOT__', str(root)))


if __name__ == '__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--simulation-root',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    print(package(Path(__file__).resolve().parents[1], a.simulation_root, a.out))
