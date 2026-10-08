# Hoya Botto

Affordable AI-assisted robotics and a proposed automation cooperative for Kesennuma manufacturing. The initial hardware experiment is one box flap under supervision.

- [Hackatsuon 2026 presentation](https://ronturetzky.github.io/hoyabotto/)
- [Workflow visualization](https://ronturetzky.github.io/hoyabotto/demo.html) — an illustration, not a recording of autonomous robot performance
- [Presentation PDF](https://ronturetzky.github.io/hoyabotto/pitch-deck.pdf)

This is the existing `xlerobot-farm` repository renamed to `hoyabotto`; its software, history, and field guides remain here. `main` contains robot source and guide sources, while `gh-pages` contains the deployed site and presentation assets. The presentation does not claim verified production savings.

> The guide builder below generates the older field guide. Preserve the Hoya Botto presentation at the Pages root when publishing guide updates; update `field-guide.html` separately.

## XLeRobot Farm — existing field guide

[Open the existing field guide](https://ronturetzky.github.io/hoyabotto/field-guide.html) · [Start the NY growing trial](https://ronturetzky.github.io/hoyabotto/ny-trial.html)

Nine connected guides consolidate current hardware, planter operation, print status, shopping, wiring, the software roadmap, travel and research. September 26 decisions supersede the archived original deck and sensor checklist. The home page is now the guide; old deck URLs remain accessible with historical banners.

Status September 26: no planter parts are printed (the K1 Max is jammed) and no growing trial has run. The software roadmap is an 18-slide architecture deck generated from `handbook/roadmap.py` (inline SVG diagrams; no code implemented yet). Robot commissioning remains September 30. The Blender file uses actual planter meshes but a clearly marked proxy for the selected table; original editable table files are still pending.

## Build and preview

```sh
python3 build_site.py
python3 -m http.server 8785 --bind 127.0.0.1 --directory dist
```

Edit `handbook/content.py` for guide content, `handbook/data/shopping.json` for the dated farm-only inventory, and `handbook/style.css` / `handbook/app.js` for navigation, search and local notebook. No runtime frameworks or external fonts/scripts. Local notebook data is not uploaded; export JSON before switching browsers. It is not a robot controller.

Run `python3 check_site.py` for static links, fragments, required files and privacy checks. Browser checks cover search, notebook persistence/export, navigation and mobile layout.

## Publish

Reuse this repository: `main` holds source, `gh-pages` holds built static files. GitHub Pages deploys the root of `gh-pages`; `.nojekyll` is included. Commit source and copy `dist` to a dedicated branch worktree, then push and verify the Pages build plus cache-busted live pages. Signing follows Git configuration. No custom domain is configured.

## Sources and licenses

Public technical summaries are curated; raw private transcripts, delivery addresses and personal cart items are excluded. Prices/delivery are explicitly dated snapshots, not order confirmations. Creators and model licenses are recorded in `handbook/downloads/MODEL-CREDITS.md`. Source planter STLs are unchanged; modified assembly/render arrangements retain CC BY-SA 4.0 attribution. All print files require inspection and physical validation.
