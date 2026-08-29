# Design notes

Two surfaces, deliberately inverted from one another.

**Asphalt** — the live screen. Near-black, because it is read through a windscreen in direct
sun with the phone mounted, and because a staging tree only reads as a staging tree against
dark.

**Paper** — the result screen. A timeslip is a printed receipt, so the result inverts to light:
it looks like the thing a track hands you, and it screenshots and shares legibly.

Amber, green and red are not decoration. They are the literal vocabulary of a drag-strip
staging tree, and they are used only for the states those lights mean.

## The signature element

The staging tree is the one memorable thing on the live screen, and it is structural rather
than ornamental: each bulb is one of the three ARMED preconditions from spec §4 — GNSS lock,
valid vehicle frame, detected stillness. Watching them fill answers the single most common
question in a car park, which is *why won't it arm*. The green bulb is the launch itself.

## The one typographic commitment

**The uncertainty is set co-equal with the number it qualifies.** Spec §9.4: "A 0–60 of
4.31 ± 0.04 s is a scientifically honest number; 4.31 s alone is not." Setting the ± small,
grey and optional would quietly undo the entire point of post-processing, so it is rendered at
the same weight and nearly the same size as the figure.

Numbers are monospaced throughout — a timeslip is machine-printed, and a proportional digit set
makes a speed readout jitter horizontally as it counts.

## What the live screen deliberately omits

An elapsed time. Spec Decision 1 says results never come from the live path, so a running clock
would invite the reader to trust a number that is not the answer. The time appears once, on the
timeslip, after the smoother has run.

## Two layout faults found by measurement

`docs/design-reference.html` shows both screens as built; `docs/layout-check.html` re-creates both screens at 1 pt = 1 px so the layout can be measured
without a device. Open it in any browser. Two faults came out of it:

**427 pt of dead space.** Two equal `Spacer()`s put 213 pt above and below the speed readout —
half an 852 pt screen — leaving a 92 pt number floating unanchored. The readout is now 148 pt
and shifted above the optical centre, with peak speed beneath it.

**The distance marks had no uncertainty at all.** As a four-column table they fit a 393 pt
phone only by giving the name column 83 pt and dropping the ± entirely — the one number §9.4
insists must never be dropped. Each mark is now a two-line row: the rollout ET with its ±
promoted, from-rest and trap demoted to a secondary line. Nothing was lost, and it reads at a
glance rather than requiring a column scan.
