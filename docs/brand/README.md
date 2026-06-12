# CerberusCAN brand assets

Three angular Cerberus heads tapping a twisted pair — CAN‑H crimson, CAN‑L charcoal —
capped with bus terminators. The three heads guarding the CAN bus.

## Files

| File | Use |
|---|---|
| `cerberus-can-logo.svg` | Master logo — wordmark + grays auto-adapt to light/dark via `prefers-color-scheme` |
| `cerberus-can-logo-light.png` | Logo baked for **light** backgrounds (1360×880) |
| `cerberus-can-logo-dark.png` | Logo baked for **dark** backgrounds (1360×880) |
| `cerberus-can-avatar.svg` / `-512` / `-256` / `-64.png` | Hexagon avatar (heads + wire) — repo social preview / profile |
| `cerberus-can-favicon.svg` / `-64` / `-32.png` | Simplified mark (heads in hex, **no wire**) — stays crisp at favicon sizes |

## Palette

| Role | Hex |
|---|---|
| Crimson — heads, CAN‑H, hexagon | `#B5252C` |
| Amber — eyes | `#F3B229` |
| Charcoal — CAN‑L (light mode) | `#565C64` |
| Slate — drop line / terminators (light mode) | `#6E7681` |
| Light gray — wire / terminators (dark mode) | `#9DA7B0` |
| Wordmark — light / dark | `#24292F` / `#E6EDF3` |

Wordmark font: `ui-monospace` stack (JetBrains Mono → Cascadia Code → Consolas).

## Embedding (light/dark aware)

```html
<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/brand/cerberus-can-logo-dark.png">
  <img alt="CerberusCAN" src="docs/brand/cerberus-can-logo-light.png" width="460">
</picture>
```

For the GitHub repo's **social preview**, upload `cerberus-can-avatar-512.png` under
*Settings → General → Social preview*.
