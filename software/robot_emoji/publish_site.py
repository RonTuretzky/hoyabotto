"""Export the visitor pages into an existing GitHub Pages checkout (does not push)."""
import argparse
import json
import shutil
from pathlib import Path
from urllib.parse import urlsplit

from .server import STATIC

LINK = '''<!-- robot-emoji-link:start -->
<a href="wave.html" aria-label="Wave at Hoya Botto" style="position:fixed;top:16px;right:16px;z-index:9999;padding:12px 18px;border-radius:999px;background:#fff8f0;color:#1d1b2f;font:700 16px system-ui;text-decoration:none;border:2px solid #ff7a45;box-shadow:0 2px 16px #0004">👋 ロボットに手を振る · Wave</a>
<!-- robot-emoji-link:end -->
'''


def export(destination, api_url):
    url = urlsplit(api_url)
    if url.scheme != 'https' or not url.netloc or url.username or url.password or url.query or url.fragment or url.path not in ('', '/'):
        raise ValueError('The visitor API must be an HTTPS origin without credentials or a path')
    destination = Path(destination)
    homepage = destination / 'index.html'
    original = homepage.read_text()
    if '<!-- robot-emoji-link:start -->' not in original:
        if '</body>' not in original:
            raise ValueError('Existing homepage has no closing body tag')
        homepage.write_text(original.replace('</body>', LINK + '</body>', 1))
    for source, target in [('kiosk.html', 'wave.html'), ('screen.html', 'screen.html'), ('emoji-api.js', 'emoji-api.js')]:
        shutil.copyfile(STATIC / source, destination / target)
    (destination / 'emoji-config.js').write_text('window.ROBOT_EMOJI_API = ' + json.dumps(api_url.rstrip('/')) + ';\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--api-url', required=True)
    args = parser.parse_args()
    export(args.destination, args.api_url)


if __name__ == '__main__':
    main()
