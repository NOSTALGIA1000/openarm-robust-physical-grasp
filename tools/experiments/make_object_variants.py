#!/usr/bin/env python3
"""Generate scene variants holding cubes of a given side length.

The lab's real objects are cubes, so the simulation should hold cubes too. The
scene carried a 30x30x44mm cuboid, which is not what the hardware will present.

Side length moves three things at once, and all three have to move together:

  clearance   the pre-grasp gap is 60mm (open_position 0.030 per finger), so a
              cube of side s leaves (60 - s)/2 per side - the budget that
              absorbs residual error
  z bias      the detector measures the top face, the grasp needs the centre.
              The 44mm-tall object needed 27.5mm against a 22mm half-height,
              so the detector's own surface offset is 5.5mm and a cube of side
              s needs s/2 + 5.5mm
  placement   the body origin sits half a side above the table top at 0.425

Emits the matching detector_z_bias for each variant; the sweep passes it
through, since leaving it at the cuboid's value would put every descent off by
half the difference in height.

Usage: make_object_variants.py <scene.xml> <outdir> [sides_mm...]
"""
import pathlib
import re
import sys

TABLE_TOP_M = 0.425
DETECTOR_SURFACE_OFFSET_M = 0.0055


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    src = pathlib.Path(sys.argv[1])
    out = pathlib.Path(sys.argv[2])
    sides = [int(s) for s in (sys.argv[3:] or ["20", "30", "40"])]
    out.mkdir(parents=True, exist_ok=True)
    base = src.read_text(encoding="utf-8")

    size_pat = re.compile(
        r'(name="physics_object_geom".*?size=")([\d.]+ [\d.]+ [\d.]+)(")', re.S)
    pos_pat = re.compile(r'(<body name="physics_object" pos="[\d.]+ [\d.-]+ )([\d.]+)(")')
    if not size_pat.search(base) or not pos_pat.search(base):
        sys.exit("could not find the object geom or its body pos in %s" % src)

    for s in sides:
        half = s / 2000.0
        text = size_pat.sub(
            lambda m: "%s%.4f %.4f %.4f%s" % (m.group(1), half, half, half,
                                              m.group(3)), base, count=1)
        text = pos_pat.sub(
            lambda m: "%s%.4f%s" % (m.group(1), TABLE_TOP_M + half, m.group(3)),
            text, count=1)
        name = "table_scene_c%03d.xml" % s
        (out / name).write_text(text, encoding="utf-8")
        print("%s  cube %dmm  clearance %.1fmm/side  z_bias %.4f  origin_z %.4f" % (
            name, s, (60 - s) / 2.0, half + DETECTOR_SURFACE_OFFSET_M,
            TABLE_TOP_M + half))


if __name__ == "__main__":
    main()
