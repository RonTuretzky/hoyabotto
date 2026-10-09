# Hoya Botto — Hackatsuon 2026

- Presentation: https://hoyabotto.com/
- Workflow illustration: https://hoyabotto.com/demo.html
- Existing PDF: https://hoyabotto.com/pitch-deck.pdf (earlier export)
- Existing field guide: field-guide.html
- Local labor-saving sources: sources/labor-saving-cases.html

This gh-pages branch contains the static site. Main retains the robot software and its history.
The homepage is the exact 14-slide HTML export supplied on October 9, 2026.
Its SHA-256 is 8cbf2344f9d9b6301b2eddb9500c88f951bfbb3e444de39fbb15ee2a2a6e2654.
Images and slide behavior are embedded in the HTML; fonts load from Google Fonts.
The supporting pages and earlier PDF are preserved.

Deploy by pushing this branch. GitHub Pages serves its root with .nojekyll.
Keep CNAME set to hoyabotto.com. Cloudflare provides DNS only: the apex A records
point to GitHub Pages, and www is a DNS-only CNAME to ronturetzky.github.io.
GitHub Pages handles TLS and redirects www to the apex; enforce HTTPS after issuance.
Keep private source recordings, chats, working notes, and credentials out of this branch.
