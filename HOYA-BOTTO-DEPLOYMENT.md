# Hoya Botto — Hackatsuon 2026

- Presentation: https://hoyabotto.com/
- Workflow illustration: https://hoyabotto.com/demo.html
- Existing PDF: https://hoyabotto.com/pitch-deck.pdf (earlier export)
- Existing field guide: field-guide.html
- Local labor-saving sources: sources/labor-saving-cases.html

This gh-pages branch contains the static site. Main retains the robot software and its history.
The homepage is the 20-slide Japanese HTML export supplied on October 10, 2026, with the visitor Wave link,
the name/emoji overlay and a Wave QR added. The original export SHA-256 is
c9b264bb68fa5091b02d38383921b23104eec551bbacc1e5469deb34ec29cf52 (the earlier 14-slide export was
8cbf2344f9d9b6301b2eddb9500c88f951bfbb3e444de39fbb15ee2a2a6e2654).

Edits to a fresh export, to repeat when the deck is re-exported:
- CSP `connect-src 'none'` becomes `connect-src https://hoya-botto-show.ronturetzky.workers.dev`.
- The `robot-emoji-link` and `robot-slide-overlay` blocks are appended before `</body>`, unchanged.
- A `wave-qr` style block moves the fixed Wave button to the top-left in landscape, so it does not cover the QR,
  and to the bottom-right in portrait, so it does not cover the cover title.
- Each slide canvas gets a clickable QR (inline SVG, `data-wave-qr`) to https://hoyabotto.com/wave.html:
  top-right at left 1606, top 16 (canvas px); slide 4 at left 1768, top 150, beside its full-width video.
  The cover gets a large QR card (380 px code) with a Japanese explanation of the wave show instead; its
  title, subtitle and byline move up (top 290→120, 470→280, 714→452) to make room.
Images and slide behavior are embedded in the HTML; fonts load from Google Fonts.
The supporting pages and earlier PDF are preserved.

Deploy by pushing this branch. GitHub Pages serves its root with .nojekyll.
Keep CNAME set to hoyabotto.com. Cloudflare provides DNS only: the apex A records
point to GitHub Pages, and www is a DNS-only CNAME to ronturetzky.github.io.
GitHub Pages handles TLS and redirects www to the apex; enforce HTTPS after issuance.
Keep private source recordings, chats, working notes, and credentials out of this branch.

## Visitor emoji show on the presentation

The current participant's name and emoji appear as a small right-side floating badge over every presentation
slide at https://hoyabotto.com/. Advancing slides preserves the overlay; it hides when idle or offline.
The slide contents remain unchanged. The Wave link opens the visitor form in another tab.

- Visitor form: https://hoyabotto.com/wave.html
- Optional dedicated display: https://hoyabotto.com/screen.html
- Local operator: http://127.0.0.1:8790/operator
- Fixed visitor API: https://hoya-botto-show.ronturetzky.workers.dev

GitHub Pages hosts the slides/static pages. The cloud API keeps the live connection origin in
a Durable Object. The installed Mac launch agent reconnects automatically after login/restart,
including when its internal Quick Tunnel changes address. No website edits are needed on restart.
Queue state is stored locally; interrupted performances fail instead of replaying. Startup stays paused.
The paired hardware client uses the internet relay with certificate verification, independent of LAN discovery.
The real robot Mac must remain online; its current relay still belongs to the existing hardware-server setup.
The hardware server was not redeployed or restarted for this work.

Sources and installation instructions: software/docs/robot-emoji.md on the robot-emoji-service branch.
Rollback the presentation integration by reverting its gh-pages commits. Domain/TLS settings were not changed.

The name badge now contains only name + emoji, uses small text on the right side, and drifts upward over 30 seconds.
The Full wave visitor choice has been retired; the short wave remains, targeting a roughly 30-second turn.

Visitor catalog: Quick wave 👋, Robot wiggle 🤖, Celebration 🎉, Look around 👀.
Each request accepts exactly one emoji. The floating name badge remains only on the presentation.
New motions are bounded, restore their initial pose and release, and require first supervised tests.

## 3D robot page and Japanese visitor flow · 10 October 2026

- `robot.html` + `robot/`: scroll-driven three.js page. The XLeRobot MuJoCo model (vendored in xlerobot-farm)
  is exploded into its printed/bought pieces, then acts out the planned work with honest status labels
  (verified / simulation / designed / idea). Assets are rebuilt by `export_assets.py` in the robot-page
  workspace; three.js 0.186 is vendored under `robot/vendor` (MIT).
- `wave.html` is Japanese and mobile-first. After sending, a card invites the visitor to `robot.html?ticket=<id>`
  and auto-opens it after 7 s; `robot.html` then shows the visitor's place in line from `/api/requests/<id>`.
- `screen.html` and `emoji-api.js` messages are Japanese. The homepage Wave button reads 「👋 ロボットに手を振る」.
- `emoji-config.js` is unchanged and must keep pointing at the workers.dev API (local previews use a mock).
