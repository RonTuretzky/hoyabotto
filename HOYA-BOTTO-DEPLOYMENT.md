# Hoya Botto — Hackatsuon 2026

- Presentation: https://hoyabotto.com/
- Workflow illustration: https://hoyabotto.com/demo.html
- Existing PDF: https://hoyabotto.com/pitch-deck.pdf (earlier export)
- Existing field guide: field-guide.html
- Local labor-saving sources: sources/labor-saving-cases.html

This gh-pages branch contains the static site. Main retains the robot software and its history.
The homepage retains the 14-slide HTML export supplied on October 9, 2026, with a small visitor Wave link appended.
The original export SHA-256 is 8cbf2344f9d9b6301b2eddb9500c88f951bfbb3e444de39fbb15ee2a2a6e2654.
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
