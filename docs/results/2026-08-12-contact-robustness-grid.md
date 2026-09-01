# OpenArm Contact-Robustness Grid Result

Date: 2026-08-12

## Experiment

Scene:

```text
/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml
```

Original grid command (without stage-level columns):

```bash
cd /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp
python3 run_experiments.py \
  --scene /home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/assets/cube_lift_scene.xml \
  --object orange_cube \
  --policies ee_base_contact_push finger_center_contact_push pre_grasp_finger_center_correction \
  --translation-noise 0 0.002 0.005 0.01 0.015 0.02 0.03 \
  --trials 30 \
  --out outputs/contact_robustness_grid_paired
```

Design:

- 3 policies
- 7 translation-noise levels
- 30 paired-seed trials per condition
- 630 total trials
- Simulated perception: MuJoCo ground-truth object pose plus Gaussian translation noise

Output directory:

```text
/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/outputs/contact_robustness_grid_paired
```

## Integrity Check

- Result rows: 630
- Summary rows: 21
- Every `(policy, noise)` condition has 30 trials.
- `ik_success_rate` is `1.0` for all conditions except:
  - `ee_base_contact_push` at 30mm: `0.967`
  - `pre_grasp_finger_center_correction` at 30mm: `0.967`

## Contact Success Summary

Wilson 95% CI is shown for `contact_success_rate`.

| policy | noise | contact success | Wilson 95% CI | IK success | contact + 2cm lift | mean object motion | mean contacts |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `ee_base_contact_push` | 0mm | 1.000 | deterministic; CI not interpreted | 1.000 | 0.000 | 0.0405m | 82.0 |
| `ee_base_contact_push` | 2mm | 1.000 | 0.886–1.000 | 1.000 | 0.000 | 0.0262m | 82.0 |
| `ee_base_contact_push` | 5mm | 1.000 | 0.886–1.000 | 1.000 | 0.000 | 0.0258m | 78.7 |
| `ee_base_contact_push` | 10mm | 0.933 | 0.787–0.982 | 1.000 | 0.033 | 0.0323m | 66.1 |
| `ee_base_contact_push` | 15mm | 0.767 | 0.591–0.882 | 1.000 | 0.000 | 0.0266m | 49.8 |
| `ee_base_contact_push` | 20mm | 0.667 | 0.488–0.808 | 1.000 | 0.000 | 0.0281m | 30.1 |
| `ee_base_contact_push` | 30mm | 0.333 | 0.192–0.512 | 0.967 | 0.000 | 0.0257m | 20.4 |
| `finger_center_contact_push` | 0mm | 1.000 | deterministic; CI not interpreted | 1.000 | 0.000 | 0.0224m | 3.0 |
| `finger_center_contact_push` | 2mm | 0.533 | 0.361–0.698 | 1.000 | 0.000 | 0.0225m | 4.0 |
| `finger_center_contact_push` | 5mm | 0.500 | 0.332–0.668 | 1.000 | 0.000 | 0.0267m | 8.2 |
| `finger_center_contact_push` | 10mm | 0.300 | 0.167–0.479 | 1.000 | 0.000 | 0.0238m | 5.1 |
| `finger_center_contact_push` | 15mm | 0.400 | 0.246–0.577 | 1.000 | 0.000 | 0.0233m | 20.3 |
| `finger_center_contact_push` | 20mm | 0.433 | 0.274–0.608 | 1.000 | 0.000 | 0.0233m | 25.3 |
| `finger_center_contact_push` | 30mm | 0.233 | 0.118–0.409 | 1.000 | 0.000 | 0.0260m | 16.5 |
| `pre_grasp_finger_center_correction` | 0mm | 1.000 | deterministic; CI not interpreted | 1.000 | 0.000 | 0.0435m | 119.0 |
| `pre_grasp_finger_center_correction` | 2mm | 1.000 | 0.886–1.000 | 1.000 | 0.000 | 0.0355m | 118.6 |
| `pre_grasp_finger_center_correction` | 5mm | 1.000 | 0.886–1.000 | 1.000 | 0.033 | 0.0388m | 125.1 |
| `pre_grasp_finger_center_correction` | 10mm | 1.000 | 0.886–1.000 | 1.000 | 0.067 | 0.0446m | 115.5 |
| `pre_grasp_finger_center_correction` | 15mm | 1.000 | 0.886–1.000 | 1.000 | 0.067 | 0.0454m | 101.5 |
| `pre_grasp_finger_center_correction` | 20mm | 1.000 | 0.886–1.000 | 1.000 | 0.033 | 0.0467m | 98.2 |
| `pre_grasp_finger_center_correction` | 30mm | 0.667 | 0.488–0.808 | 0.967 | 0.000 | 0.0518m | 50.8 |

## Supported Claims

The result supports this conservative claim:

```text
In this MuJoCo simulated-perception-noise benchmark, pre-grasp finger-center correction improves finger-object contact robustness compared with single-shot finger-center contact planning.
```

The result also supports:

```text
The single-shot finger-center strategy is sensitive to small localization noise: contact success drops from 1.0 at 0mm to about 0.5 at 2–5mm and remains below 0.5 for most higher noise levels.
```

The result suggests:

```text
The direct EE-base baseline creates contact frequently at low and moderate noise, but this is not necessarily better grasping because it is a push/collision-style baseline and true lift success remains near zero.
```

## Unsupported Claims

Do not claim:

```text
The system implements robust visual grasping.
```

Do not claim:

```text
Pre-grasp correction improves grasp success.
```

The contact-plus-2cm-lift threshold rate is sparse and inconsistent:

- It is `0.0` for most conditions.
- The maximum observed `success_rate` is `0.067`, only 2/30 trials.
- This is insufficient evidence that lift-grasp is solved.

## Methodological Caveats Identified Before the Stage Rerun

- `pre_grasp_finger_center_correction` uses an extra simulated pre-grasp observation. The comparison evaluates the value of additional observation, not an equal-information controller advantage.
- The original `contact_success_rate` is cumulative over the whole trial. The stage rerun below adds sampling-window diagnostics.
- `mean_finger_object_contacts` is affected by execution duration and is not normalized by contact frames.
- The perception model is ground-truth pose plus Gaussian noise; it is not a real visual detector.
- The scene uses experiment-owned cube/contact parameters, so results should not be directly generalized to the real robot.

## Analysis Upgrades Proposed Before the Stage Rerun

Add stage-level contact metrics:

- `approach_contact_success`
- `post_grasp_contact_success`
- `lift_contact_success`
- `two_finger_contact_success`
- normalized `contact_frame_rate`

This would separate useful grasp-phase contact from accidental approach push/collision.

## Stage-Level Metrics Follow-Up

On 2026-08-13, the same 630-trial paired-seed grid was rerun with stage-level contact fields. The new output is:

```text
/home/nostalgia/openarm_ws/experiments/openarm_robust_grasp/outputs/contact_robustness_grid_stage_paired
```

Integrity checks:

- 630 trial rows and 21 summary rows.
- Every `(policy, noise)` condition has 30 trials.
- All 210 `(noise, seed)` groups contain the same three policies.
- Stage contact counts are non-negative and sum exactly to `finger_object_contacts`.
- All pre-existing fields match the earlier grid exactly for every `(policy, noise, seed)` row.
- 21 tests passed before the rerun and again after documentation synchronization; the copied test record is `outputs/contact_robustness_grid_stage_paired/verification.txt`.

The stage fields are:

- `approach_finger_object_contacts`
- `grasp_finger_object_contacts`
- `lift_finger_object_contacts`
- `post_grasp_contact_success`
- `lift_contact_success`

Observed stage-end/contact-window rates are shown below. `any`, `approach`, `grasp`, and `lift` are calculated directly from raw contact counts greater than zero and are not gated by the final `ik_success` flag. In contrast, the earlier `contact_success_rate` requires both IK success and at least one contact. `Approach` and `lift` are sampled at the ends of those motions; `grasp` includes the grasp endpoint and 40 gripper-close simulation steps.

| policy | noise | IK | any | approach endpoint | grasp/close | lift endpoint | object motion | contact + 2cm lift |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `ee_base_contact_push` | 0mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0405m | 0.000 |
| `ee_base_contact_push` | 2mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0262m | 0.000 |
| `ee_base_contact_push` | 5mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0258m | 0.000 |
| `ee_base_contact_push` | 10mm | 1.000 | 0.933 | 0.000 | 0.933 | 0.000 | 0.0323m | 0.033 |
| `ee_base_contact_push` | 15mm | 1.000 | 0.767 | 0.000 | 0.767 | 0.000 | 0.0266m | 0.000 |
| `ee_base_contact_push` | 20mm | 1.000 | 0.667 | 0.000 | 0.667 | 0.000 | 0.0281m | 0.000 |
| `ee_base_contact_push` | 30mm | 0.967 | 0.367 | 0.033 | 0.367 | 0.000 | 0.0257m | 0.000 |
| `finger_center_contact_push` | 0mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0224m | 0.000 |
| `finger_center_contact_push` | 2mm | 1.000 | 0.533 | 0.000 | 0.533 | 0.000 | 0.0225m | 0.000 |
| `finger_center_contact_push` | 5mm | 1.000 | 0.500 | 0.000 | 0.500 | 0.000 | 0.0267m | 0.000 |
| `finger_center_contact_push` | 10mm | 1.000 | 0.300 | 0.000 | 0.300 | 0.000 | 0.0238m | 0.000 |
| `finger_center_contact_push` | 15mm | 1.000 | 0.400 | 0.000 | 0.400 | 0.000 | 0.0233m | 0.000 |
| `finger_center_contact_push` | 20mm | 1.000 | 0.433 | 0.000 | 0.433 | 0.000 | 0.0233m | 0.000 |
| `finger_center_contact_push` | 30mm | 1.000 | 0.233 | 0.067 | 0.200 | 0.000 | 0.0260m | 0.000 |
| `pre_grasp_finger_center_correction` | 0mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0435m | 0.000 |
| `pre_grasp_finger_center_correction` | 2mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0355m | 0.000 |
| `pre_grasp_finger_center_correction` | 5mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.000 | 0.0388m | 0.033 |
| `pre_grasp_finger_center_correction` | 10mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.033 | 0.0446m | 0.067 |
| `pre_grasp_finger_center_correction` | 15mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.033 | 0.0454m | 0.067 |
| `pre_grasp_finger_center_correction` | 20mm | 1.000 | 1.000 | 0.000 | 1.000 | 0.033 | 0.0467m | 0.033 |
| `pre_grasp_finger_center_correction` | 30mm | 0.967 | 0.667 | 0.067 | 0.633 | 0.000 | 0.0518m | 0.000 |

Paired contact effects for correction minus single-shot finger-center planning:

| noise | rate difference | paired wins | paired losses | ties |
| ---: | ---: | ---: | ---: | ---: |
| 0mm | 0.000 | 0 | 0 | 30 |
| 2mm | +0.467 | 14 | 0 | 16 |
| 5mm | +0.500 | 15 | 0 | 15 |
| 10mm | +0.700 | 21 | 0 | 9 |
| 15mm | +0.600 | 18 | 0 | 12 |
| 20mm | +0.567 | 17 | 0 | 13 |
| 30mm | +0.433 | 15 | 2 | 13 |

Interpretation: under the current sampling protocol, observed finger-object contacts are concentrated in the grasp/close window. This does not prove that transient contacts never occur during approach or lift, because those 80-step motions are sampled only at their endpoints. Contact counts also must not be interpreted as force or independent collision counts because the grasp/close window has many more sampling opportunities.

The stage rerun strengthens the contact-robustness result but does not change the grasp conclusion: the `contact + 2cm lift` rate remains sparse (at most 2/30), and lift-end contact is at most 1/30. Physical lift grasp is still unsolved.

## Per-Step 630-Trial Rerun

On 2026-08-13, the same paired grid was rerun with contact observation after every MuJoCo simulation step:

```text
outputs/contact_robustness_grid_per_step_paired
```

Verification:

- 630 trials, 21 summaries, and 210 complete paired-seed groups.
- Every condition contains 30 trials.
- All pre-existing fields match the earlier 630-trial stage grid exactly: zero differing rows.
- All contact-frame and two-finger-frame bounds pass.
- Normal completed formal-policy trials use `80/120/80` approach/grasp/lift frames. The observed `80/40/0` rows are tested grasp-IK failure paths that preserve the old behavior of executing 40 close steps before stopping.

Key per-step results:

| policy | noise | approach contact-frame rate | grasp/close contact-frame rate | lift contact-frame rate | lift two-finger success | sustained lift success |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `ee_base_contact_push` | 0mm | 0.000 | 0.450 | 0.237 | 1.000 | 0.000 |
| `ee_base_contact_push` | 2mm | 0.000 | 0.450 | 0.245 | 1.000 | 0.000 |
| `ee_base_contact_push` | 5mm | 0.000 | 0.441 | 0.217 | 0.900 | 0.000 |
| `ee_base_contact_push` | 10mm | 0.000 | 0.417 | 0.179 | 0.633 | 0.000 |
| `ee_base_contact_push` | 15mm | 0.000 | 0.354 | 0.157 | 0.500 | 0.000 |
| `ee_base_contact_push` | 20mm | 0.000 | 0.263 | 0.072 | 0.200 | 0.000 |
| `ee_base_contact_push` | 30mm | 0.008 | 0.210 | 0.071 | 0.133 | 0.000 |
| `finger_center_contact_push` | 0mm | 0.000 | 0.025 | 0.013 | 0.000 | 0.000 |
| `finger_center_contact_push` | 2mm | 0.000 | 0.039 | 0.005 | 0.000 | 0.000 |
| `finger_center_contact_push` | 5mm | 0.000 | 0.102 | 0.022 | 0.000 | 0.000 |
| `finger_center_contact_push` | 10mm | 0.000 | 0.065 | 0.008 | 0.000 | 0.000 |
| `finger_center_contact_push` | 15mm | 0.000 | 0.168 | 0.054 | 0.033 | 0.000 |
| `finger_center_contact_push` | 20mm | 0.000 | 0.202 | 0.070 | 0.000 | 0.000 |
| `finger_center_contact_push` | 30mm | 0.022 | 0.109 | 0.040 | 0.033 | 0.000 |
| `pre_grasp_finger_center_correction` | 0mm | 0.000 | 0.592 | 0.263 | 1.000 | 0.000 |
| `pre_grasp_finger_center_correction` | 2mm | 0.000 | 0.594 | 0.243 | 1.000 | 0.000 |
| `pre_grasp_finger_center_correction` | 5mm | 0.000 | 0.595 | 0.260 | 1.000 | 0.000 |
| `pre_grasp_finger_center_correction` | 10mm | 0.000 | 0.593 | 0.259 | 0.767 | 0.033 |
| `pre_grasp_finger_center_correction` | 15mm | 0.000 | 0.593 | 0.270 | 0.433 | 0.033 |
| `pre_grasp_finger_center_correction` | 20mm | 0.000 | 0.595 | 0.254 | 0.567 | 0.033 |
| `pre_grasp_finger_center_correction` | 30mm | 0.022 | 0.319 | 0.110 | 0.300 | 0.000 |

Per-step observation confirms that endpoint-only sampling missed transient approach contacts at 30 mm. It also separates occasional simultaneous opposing-finger contact from sustained lift contact. The correction policy frequently reaches two-finger contact during lift at low and moderate noise, but sustained opposing-finger contact for at least 50% of lift frames appears in only 1/30 trials at 10, 15, and 20 mm and nowhere else. This is diagnostic evidence of intermittent contact, not a solved stable grasp.
