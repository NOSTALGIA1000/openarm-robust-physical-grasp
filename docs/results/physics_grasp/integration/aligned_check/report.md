# Physics-mode grasp trial aligned_check

- Result: `unilateral_contact`
- Success: `False`
- Samples: `924`
- Max elevation: `0.0000 m` (threshold `0.020 m`)
- Hold bilateral ratio: `0.000` (threshold `0.80`)
- Max relative drift: `0.0000 m` (threshold `0.010 m`)

## Stage times (simulation seconds)

- `settle`: `315.180`
- `hover`: `316.960`
- `open`: `318.540`
- `descend`: `319.680`
- `close`: `322.900`
- `retreat`: `329.560`

No software attachment and no object pose command exist in this mode; object motion is produced only by contact.

---

## CORRECTION (post-review, 2026-09-02)

The metrics above ("Max elevation: 0.0000 m", "Max relative drift: 0.0000 m")
are accurate values of their narrowly-scoped definitions but do **not** mean
the object stayed near the gripper/table. They cannot detect this trial's
actual outcome:

- `max_drift_m` only starts accumulating once bilateral contact is
  *confirmed* (`_confirm_bilateral_contact()` returns `True` in
  `physics_picker.py`), which never happened in this trial — so it reads
  `0.0` regardless of how far the object actually moved.
- `max_elevation_m` only tracks upward peaks, so it captured the momentary
  finger-squeeze bump (`4.322870694939418e-05` in `trial.json`) and is
  blind to the much larger downward/lateral displacement that followed.

**What `samples.csv` actually shows**: at t=324.14-324.46 s the object
accelerates laterally under sustained bilateral contact (8-17 N per
finger), then at t=324.48 s contact is lost (`False/False`) and the object
continues in free flight/slide at a constant ~0.456-0.462 m/s with no
measurable deceleration, falls off the table (`object_z`: 0.4499 m ->
0.029916538717898292 m, the floor, by ~t=325.6-325.9 s), and travels a
total of roughly 4.2-4.3 m in `object_x` by the end of the trial
(t=333.64 s). All of this happens inside the `close`/`retreat` stages, with
zero controller commands to the object at any point.

**Corrected finding**: this is a squeeze-ejection event — the gripper made
strong, genuine bilateral contact and then launched the object laterally,
rather than failing to contact it. The subsequent motion during
`any_contact: false` samples is the physically-expected continuation of
momentum imparted during real contact, not a software attachment (the
brief's "object motion with no contact" defect does not apply here — that
defect describes motion with no physical cause, not momentum carried
forward from a recorded contact event). The apparent near-zero friction
during the object's floor travel is a separate observation worth
recording for future friction tuning.

See `~/ros2_ws_course_repro/results/physics_grasp/integration/notes.md`
("Corrected aligned-trial finding" section) for the full trajectory data
and reasoning.
