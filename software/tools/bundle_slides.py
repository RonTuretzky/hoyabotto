"""Bundle an HTML slide deck and every local file it references (img/video src, poster) into one self-contained file.

    python tools/bundle_slides.py docs/carton-fold-policy-setup-slides.html [OUT.html]
Links to PDFs stay links (relative to the original); everything shown on the slides is embedded as data URIs.
"""
import base64, mimetypes, re, sys
from pathlib import Path

src = Path(sys.argv[1]).resolve()
out = Path(sys.argv[2]) if len(sys.argv) > 2 else src.with_name(src.stem + '-standalone.html')
html = src.read_text()
mimetypes.add_type('image/svg+xml', '.svg')


def embed(match):
    attr, path = match.group(1), match.group(2)
    f = (src.parent / path).resolve()
    if path.startswith(('http:', 'https:', 'data:')) or not f.is_file():
        return match.group(0)
    mime = mimetypes.guess_type(f.name)[0] or 'application/octet-stream'
    return f'{attr}="data:{mime};base64,{base64.b64encode(f.read_bytes()).decode()}"'


html, n = re.subn(r'\b(src|poster)="([^"]+)"', embed, html)
out.write_text(html)
print(f'{out} ({out.stat().st_size / 1e6:.1f} MB, {n} files embedded)')
