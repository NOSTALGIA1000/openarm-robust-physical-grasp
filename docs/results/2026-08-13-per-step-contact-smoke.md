# OpenArm Per-Step Contact Smoke Gate

Date: 2026-08-13

## Purpose

Validate per-simulation-step contact instrumentation before rerunning the full 630-trial grid. This is a 27-trial engineering and scientific smoke gate, not a powered comparative study.

## Design

- Scene: experiment-owned `assets/cube_lift_scene.xml`
- Object: `orange_cube`
- Policies: `ee_base_contact_push`, `finger_center_contact_push`, `pre_grasp_finger_center_correction`
- Translation-noise standard deviations: 0, 10, and 30 mm
- Three paired seeds per condition
- 27 trials, nine summaries, and nine complete `(noise, seed)` policy groups
- Sampling windows: approach 80 frames, grasp motion plus close 120 frames, lift 80 frames

## Verification

- Full test suite: 40 passed in 35.38 s.
- All 27 result rows use sampled-frame totals `80/120/80`.
- Contact frames are within sampled-frame bounds.
- Two-finger frames never exceed any-contact frames.
- All frame rates lie in `[0, 1]`.
- Every pre-existing trial field exactly matches the frozen pre-instrumentation 27-trial reference: zero differing rows.

The compatibility result shows that the observer did not change the formal policies' trajectories, random draws, or existing metrics for this grid.

## Per-Step Summary

| policy | noise | approach contact-frame rate | grasp/close contact-frame rate | lift contact-frame rate | grasp two-finger success | lift two-finger success | sustained lift success |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `ee_base_contact_push` | 0mm | 0.000 | 0.450 | 0.237 | 1.000 | 1.000 | 0.000 |
| `ee_base_contact_push` | 10mm | 0.000 | 0.514 | 0.267 | 1.000 | 1.000 | 0.000 |
| `ee_base_contact_push` | 30mm | 0.000 | 0.556 | 0.188 | 0.667 | 0.333 | 0.000 |
| `finger_center_contact_push` | 0mm | 0.000 | 0.025 | 0.013 | 0.000 | 0.000 | 0.000 |
| `finger_center_contact_push` | 10mm | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 |
| `finger_center_contact_push` | 30mm | 0.183 | 0.700 | 0.229 | 0.333 | 0.333 | 0.000 |
| `pre_grasp_finger_center_correction` | 0mm | 0.000 | 0.592 | 0.263 | 1.000 | 1.000 | 0.000 |
| `pre_grasp_finger_center_correction` | 10mm | 0.000 | 0.600 | 0.213 | 0.667 | 0.667 | 0.000 |
| `pre_grasp_finger_center_correction` | 30mm | 0.183 | 0.725 | 0.196 | 0.333 | 0.000 | 0.000 |

## Interpretation

The new instrumentation detects contacts during motion that endpoint-only metrics could miss. Some trials contain simultaneous opposing-finger contact during lift, but none maintain opposing-finger contact for at least 50% of lift frames. Therefore the smoke grid provides no evidence of sustained lift grasp.

The nonzero approach contact-frame rate at 30 mm confirms that endpoint-only approach sampling understated transient approach interactions. This supports the value of per-step observation.

Because each condition has only three trials, differences between policies must not be treated as stable effect estimates. The full paired 630-trial rerun is required for comparative claims.

## Claim Boundary

Supported:

> Per-step instrumentation preserves existing formal-policy trajectories on the smoke grid and reveals transient stage contacts, including occasional simultaneous opposing-finger lift contact.

Not supported:

- Stable or force-closure grasping is solved.
- Two-finger contact in one frame proves object retention.
- The 27-trial rates establish a reliable policy ranking.
- Results transfer directly to real OpenArm hardware.
