#!/usr/bin/env python3
"""Build the curated field guide and retain clearly marked historical pages."""
from pathlib import Path
import sys, json, shutil, html, re, runpy
from html.parser import HTMLParser
HERE=Path(__file__).resolve().parent
runpy.run_path(str(HERE/'build_xlerobot_deck.py'),run_name='__main__')
sys.path.insert(0,str(HERE/'handbook'))
from content import PAGES,page
out=HERE/'dist'
if out.exists():shutil.rmtree(out)
shutil.copytree(HERE/'handbook',out,ignore=shutil.ignore_patterns('*.py','__pycache__','*.blend1'))
(out/'.nojekyll').touch()
esc=html.escape
shop=json.loads((out/'data/shopping.json').read_text())
def rows(items):
 return ''.join(f'<tr><td><a href="https://www.amazon.com/dp/{i["asin"]}">{esc(i["item"])}</a></td><td>${i["usd"]:.2f}</td><td>{esc(i["recommendation"])}</td><td>{esc(i["why"])}</td></tr>' for i in items)
def table(items):return '<div class="scroll"><table><thead><tr><th>Item / Amazon</th><th>Pack price</th><th>Decision</th><th>Purpose / remaining work</th></tr></thead><tbody>'+rows(items)+'</tbody></table></div>'
page('shopping','Buy for the current build.','Item-by-item purpose, current cart presence and unresolved gaps. A cart is a dated shopping snapshot, not proof of delivery or a complete working robot.', '''
<div class="callout"><strong>Latest readback: September 22, ~11:05 ET.</strong> Eleven farm items are active, totaling <strong>$126.94</strong> before checkout tax/shipping. One cress packet was added for the NY trial; no purchase was made. Personal items are excluded from this public inventory.</div>
<h2>Seeds and growing supplies</h2><p>The $4.99 garden-cress packet was added on September 22; its purchase is unconfirmed and the New York trial it was meant for did not happen. Seeds for the Japan crop are bought locally. Household paper replaces special grow mats for the first batch. <a href="growing.html#timing">Trial status and setup →</a></p>
<figure class="blender-view"><a href="assets/blender/electronics-parts.png" target="_blank" rel="noopener"><img loading="lazy" src="assets/blender/electronics-parts.png" alt="Blender view of the sensor electronics and assembly supplies"></a><figcaption>Representative hardware shapes. Use the item-by-item inventory below for exact products and purchase status. Open the image for a larger view.</figcaption></figure><h2>In the active farm cart</h2>'''+table([i for i in shop['items'] if i['in_latest_cart']])+'''
<h2>Optional cuts still proposed</h2><p>For paper trays, defer the moisture probe ($14.45) and its relay pack ($8.99): <strong>$23.44</strong>, leaving <strong>$103.50</strong> at the last recorded prices. No cart removal was made. <a href="hardware.html#moisture-decision">Reasoning and water-check plan</a>.</p><p>The growing mats ($19.58, delivery Sep 30) and optional SHT31 air sensor ($9.99) could be deferred, saving <strong>$29.57</strong>. That would bring the current farm portion to <strong>$97.37</strong>. Together with the probe and relay, all four proposed deferrals save $53.01 and leave $73.93. These removals have not been made by this update.</p>
<h2>Previously reviewed, now absent from the active cart</h2><p>Absence does not mean “owned.” Reuse what you already have or arrange it locally. The functional USB data cable remains necessary for the ESP32 even though this separate listing is no longer active.</p>'''+table([i for i in shop['items'] if not i['in_latest_cart']])+'''
<h2>Already reported / separately arranged</h2><ul><li><strong>Confirmed bought:</strong> RSHTECH RSH-ST07 powered hub and Anker C300 DC. Verify actual cable/adapter inventory when assembling.</li><li><strong>Selected robot package:</strong> WowRobo XLeRobot Combo; use the delivered motor adapters/cables/cameras and check against its packing list.</li><li><strong>Computer and robot cart:</strong> Mac and the planned IKEA cart; the printable planter table is separate.</li><li><strong>Local/reused supplies:</strong> paper, clean water, rinse container, kitchen measure, a suitable pouring bottle or fitted spout, solder/stripping tools, small fasteners/retainers/insulation and a borrowed meter.</li></ul>
<h2>Do we need anything else?</h2><p>No new electronic category is needed to begin the NY grow trial. For the robot, resolve the missing data-cable availability, bottle outlet, dry enclosure and cable retention. These are real build dependencies; their absence from the cart must not be mistaken for completion.</p><p>A grow light depends on measured/observed light at the actual site. Start with a bright location and watch for stretching; don’t buy an arbitrary lamp simply to fill a checklist. The mobile battery harness is still uncommissioned. Use supplied wall power first after label checks.</p><p>No pump, external tank, scale, overhead camera, leak pads, labels, rubber sheets, silicone mat or measuring syringes are being reintroduced. The current seed packet is for New York; Japan seed remains a local-sourcing task.</p>
<p class="small">Prices/delivery can change. Check the actual checkout date before ordering. <a href="data/shopping.json">Download item data</a>.</p>
''')
NAV=[('index','Start here'),('visuals','Blender visual atlas'),('assembly','Robot assembly'),('prints','Printing & table'),('hardware','Hardware & wiring'),('shopping','Shopping list'),('software','Software roadmap'),('japan','Japan & carry-on'),('research','Research & files'),('growing','Growing trial & kits')]
class Plain(HTMLParser):
 def __init__(self):super().__init__();self.parts=[]
 def handle_data(self,d):self.parts.append(d)
def plain(text):p=Plain();p.feed(text);return re.sub(r'\s+',' ',' '.join(p.parts)).strip()
search=[]
for key,label in NAV:
 p=PAGES[key];nav=''.join(f'<a href="{k}.html"'+(' aria-current="page"' if k==key else '')+f'>{v}</a>' for k,v in NAV)
 body=f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="description" content="{esc(p['lead'],quote=True)}"><meta name="theme-color" content="#172a34"><title>{esc(label)} · XLeRobot Farm</title><link rel="stylesheet" href="style.css"><script src="app.js" defer></script></head><body><a class="skip" href="#main">Skip to content</a><aside><a class="brand" href="index.html">XLeRobot / Farm<small>THE WORKING FIELD GUIDE</small></a><div class="search"><label for="site-search">Find a topic</label><input type="search" id="site-search" placeholder="Try: wick, cable, recovery" autocomplete="off"><div class="search-results" id="search-results" aria-live="polite"></div></div><nav aria-label="Main navigation">{nav}</nav><p class="side-note">Current decisions · Sep 26, 2026<br>No parts printed; printer jammed.<br>Software Phase 0 on the Mac now.<br>Robot commissioning Sep 30.<br><br>Labels carry meaning; color is optional.</p></aside><main id="main"><header><p class="eyebrow">FIELD GUIDE / {esc(label.upper())}</p><h1>{p['title']}</h1><p class="lead">{p['lead']}</p></header>{p['body']}<footer>Consolidated September 26, 2026 · Plans are distinguished from verified results.<br><a href="index.html">Guide home</a> · <a href="research.html#decisions">Decision history &amp; sources</a> · <a href="https://github.com/RonTuretzky/xlerobot-farm">Project repository</a><p>Notebook data stays in your browser. This website has no connection to robot controls or checkout.</p></footer></main></body></html>'''
 (out/f'{key}.html').write_text(body)
 search.append({'title':label,'url':f'{key}.html','text':plain(p['title']+' '+p['lead']+' '+p['body'])})
 for match in re.finditer(r'<section id="([^"]+)"[^>]*>(.*?)(?=<section id=|$)',p['body'],re.S):
  title=re.search(r'<h2>(.*?)</h2>',match[2],re.S)
  if title:search.append({'title':plain(title[1]),'url':f'{key}.html#{match[1]}','text':plain(match[2])})
# Old bookmarked URLs keep their fragment and open the combined guide.
for old,anchor in [('ny-trial','timing'),('planters','wick')]:
 alias=(out/'growing.html').read_text().replace('<head>','<head><link rel="canonical" href="https://ronturetzky.github.io/xlerobot-farm/growing.html"><script>location.replace("growing.html"+(location.hash||"#'+anchor+'"))</script>',1)
 (out/f'{old}.html').write_text(alias)
(out/'data/search.json').write_text(json.dumps(search,indent=2))
# Keep the previous standalone companion URL and its section links working.
companion=(out/'software.html').read_text()
companion=re.sub(r'(href|src)="(?!https?:|#|/)([^"]+)"',lambda m:f'{m[1]}="../{m[2]}"',companion)
companion=companion.replace('<head>','<head><link rel="canonical" href="https://ronturetzky.github.io/xlerobot-farm/software.html">',1)
(out/'downloads/software-roadmap-detail.html').write_text(companion)

# Preserve existing public deep links, with an unavoidable historical label.
for name in ['xlerobot-deck.html','sensor-shopping.html']:
 original=(HERE/name).read_text()
 banner='<div style="position:fixed;top:0;left:0;right:0;z-index:100000;background:#172a34;color:white;padding:9px 18px;font:14px/1.4 system-ui;text-align:center">HISTORICAL — superseded hardware and shopping decisions. <a style="color:white;text-decoration:underline" href="index.html">Open current field guide →</a></div>'
 original=re.sub(r'(<body[^>]*>)',r'\1'+banner,original,count=1)
 (out/name).write_text(original)
shutil.copytree(HERE/'assets',out/'assets',dirs_exist_ok=True)
print(f'Built {len(NAV)} handbook pages + historical links at {out}')
