# Grasshopper Icon System for COMPAS FAB

- **`icon.png`**: true 24×24, transparent, anti-aliased. Each overwrites
  the placeholder icon in its component folder. Colors: ink `#2E333A`,
  accent `#008EA6` (teal).
- **`icons.js`**: the editable SVG source for every glyph (one 24-grid kit).
  Re-render the PNGs from this if you tweak a path.
- **[`icon_system.html`](icon_system.html)**: the catalog / spec sheet
  (grid, stroke, primitive kit, all icons grouped by subcategory, plus an
  "in Grasshopper" preview). Loads `icons.js` from the same folder.

## Design rules

- 24×24 artboard · 3px keyline · 18² live area
- 1.8px monoline stroke · round cap · round join
- Shared kit: cube = cell · square = body · cone = tool · triad = frame ·
  ring = goal · polyline = motion · brackets = library · chip = backend
- Accent (teal) marks the *semantic subject* of each icon only.

## Notes

- These PNGs were exported at native 24px. To regenerate at a different size,
  render `icons.js` glyphs onto an N×N canvas (viewBox stays `0 0 24 24`).
- `icon_system.html` is a static reference; if you add it to the mkdocs site,
  keep `icons.js` beside it (it is loaded with a relative `<script src>`).
