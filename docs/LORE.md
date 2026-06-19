# The Lore

A small mythology for a small toolchain. Three tools, one story.

## The brood

In Greek myth, **Typhon** (father of monsters) and **Echidna** (mother of monsters) spawned a litter
of guardian hounds. Two of them matter here:

- **Orthrus**, the **two-headed** dog, who guarded the cattle of Geryon.
- **Cerberus**, his **three-headed** brother, who guards the gates of the underworld — and whose one
  rule is absolute: *souls may enter, but none may leave.*

They are watchers. They sit at thresholds, see everything that passes, and let nothing slip by
unnoticed. That is exactly what a bus interface that can *losslessly log the wire while you work* is.

## The gate

For us, **the gate** is VAG's gateway — the **J533** — and everything it hides behind **Component
Protection**. The guard dogs sit at that gate (the OBD-II port) and watch the traffic cross it.

## The hounds

- **Orthrus** — *the two-headed watch-dog.* Teensy 4.0, two heads: **K-line + CAN**. The compact
  guardian; two heads are enough for the older gates.
- **Cerberus** — *the three-headed guardian of the gate.* Teensy 4.1, three heads: **K-line + CAN +
  DoIP**. Watches every era of gate at once — drives an active diagnostic session on one head while
  losslessly logging the unmasked wire on another.

Both are **one codebase, two build targets** — see [PRODUCT-LINE.md](PRODUCT-LINE.md).

## The messenger

- **Hermes-CP** — *the one who walks past the guard.* In the myth, **Hermes** is the single figure
  who passes Cerberus freely: the **messenger**, the **boundary-crosser**, the **psychopomp** who
  escorts souls across the threshold and comes back out. A separate, single-purpose
  right-to-repair tool (private). It doesn't fight the guard dogs — it's named for the one being the
  guard dogs let pass.

## Why it fits

The naming encodes the architecture:

- **Heads = buses.** Two heads / three heads is literally the channel count.
- **Guardians watch; the messenger acts.** Orthrus and Cerberus are passive-capable diagnostic
  tools; Hermes is a single-action tool. The myth draws the same line we do — "watch the gate" vs
  "cross the gate."
- **Hermes passes Cerberus.** Build the guardians first, then the messenger that walks among them.

> *Typhon's hounds guard the gate. Hermes is the one who walks past them.*
