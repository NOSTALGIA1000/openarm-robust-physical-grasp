#!/usr/bin/env python3
"""Generate scene variants holding cubes of a given side length.

The lab's real objects are cubes, so the simulation should hold cubes too. The
scene carried a 30x30x44mm cuboid, which is not what the hardware will present.

Side length moves three things at once, and all three have to move together:

  clearance   the pre-grasp gap is 2*open_position, so a cube of side s
              leaves (2*open_position*1000 - s)/2 mm per side - the budget
              that absorbs residual error. This script is not told the
              opening unless --open is passed, so it prints the formula
              rather than a number; it used to print (60 - s)/2, which is
              right only at open_position 0.030 and was off by 5mm in every
              batch run at 0.025 - stage52_cube20, stage53_noise20 and
              stage51_offseterror all logged 20.0mm/side for a real 15.0
  z bias      the detector measures the top face, the grasp needs the centre.
              The 44mm-tall object needed 27.5mm against a 22mm half-height,
              so the detector's own surface offset is 5.5mm and a cube of side
              s needs s/2 + 5.5mm
  placement   the body origin sits half a side above the table top at 0.420

Emits the matching detector_z_bias for each variant; the sweep passes it
through, since leaving it at the cuboid's value would put every descent off by
half the difference in height.

Usage: make_object_variants.py <scene.xml> <outdir> [sides_mm...]
"""
import pathlib
import re
import sys

# Measured from the scene: table_top is a cylinder of half-height 0.02
# at z=0.40 inside a body at the origin, so its surface is 0.420. The
# historical object spawned 5mm above it and settled.
TABLE_TOP_M = 0.420
DETECTOR_SURFACE_OFFSET_M = 0.0055
CAMERA_Z_M = 1.2
# The object band must clear the object's own depth spread - the 44mm
# cuboid measured 0.736 to 0.746 - without reaching the table at 0.780.
# 20mm gives a 50mm cube exactly the 0.750 that was hardcoded, so cells
# already run under the old constant stay comparable.
#
# But a short object does not have 20mm to give: a 20mm cube's top sits
# exactly 20mm above the table, so a fixed 20mm margin puts the band edge on
# the table itself. The margin is therefore capped to leave at least 10mm of
# the object's own height below it, which changes nothing for 30mm and above
# and makes a 20mm cube workable.
OBJECT_BAND_MARGIN_M = 0.020
TABLE_MARGIN_M = 0.010
# Commanded closing overshoot, held constant across object sizes so grip force
# does not confound clearance. 30mm object closed to a 12mm gap.
SQUEEZE_M = 0.018


def band_margin(side_m):
    return min(OBJECT_BAND_MARGIN_M, side_m - TABLE_MARGIN_M)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    args = sys.argv[1:]
    # --open <metres per finger>, so the printed clearance can be the real one.
    # Optional because most sweeps set OPEN_POSITION after calling this script,
    # and reading it from the environment would be right for one sweep and
    # silently wrong for the rest.
    opening = None
    if "--open" in args:
        i = args.index("--open")
        opening = float(args[i + 1])
        del args[i:i + 2]
    # --object-y <metres> moves the object across the robot's centreline.
    # Needed because the two arms are not mirror images: their base poses are
    # mirrored but every link below is identical, so a pose one arm reaches
    # comfortably can force the other to sweep a link through the object on the
    # way in. The right-arm smoke test of 2026-09-18 launched the cube at 11 m/s
    # during descend with no finger contact recorded, which is what that looks
    # like. Omitted, the scene's own y is kept, so every existing batch is
    # unaffected.
    object_y = None
    if "--object-y" in args:
        i = args.index("--object-y")
        object_y = float(args[i + 1])
        del args[i:i + 2]
    src = pathlib.Path(args[0])
    out = pathlib.Path(args[1])
    sides = [int(s) for s in (args[2:] or ["20", "30", "40"])]
    out.mkdir(parents=True, exist_ok=True)
    base = src.read_text(encoding="utf-8")

    size_pat = re.compile(
        r'(name="physics_object_geom".*?size=")([\d.]+ [\d.]+ [\d.]+)(")', re.S)
    pos_pat = re.compile(r'(<body name="physics_object" pos="[\d.]+ )([\d.-]+)( )([\d.]+)(")')
    if not size_pat.search(base) or not pos_pat.search(base):
        sys.exit("could not find the object geom or its body pos in %s" % src)

    for s in sides:
        half = s / 2000.0
        text = size_pat.sub(
            lambda m: "%s%.4f %.4f %.4f%s" % (m.group(1), half, half, half,
                                              m.group(3)), base, count=1)
        text = pos_pat.sub(
            # The y is passed through VERBATIM when it is not being moved.
            # Reformatting it as %.4f turned "0" into "0.0000", which is the
            # same number and a different file, so every historical batch's
            # recorded model_sha256 would stop matching a freshly generated
            # scene that is physically identical to it.
            lambda m: "%s%s%s%.4f%s" % (
                m.group(1),
                m.group(2) if object_y is None else "%.4f" % object_y,
                m.group(3), TABLE_TOP_M + half, m.group(5)),
            text, count=1)
        # The y goes in the filename when it is moved, so a scene for one arm
        # can never be picked up by a batch meant for the other. mm, with the
        # sign spelled, because a minus in a filename does not survive every
        # tool this repository pipes names through.
        if object_y is None:
            name = "table_scene_c%03d.xml" % s
        else:
            name = "table_scene_c%03d_y%s%03d.xml" % (
                s, "m" if object_y < 0 else "p", round(abs(object_y) * 1000))
        top_depth = CAMERA_Z_M - (TABLE_TOP_M + s / 1000.0)
        (out / name).write_text(text, encoding="utf-8")
        # Sidecar with every derived value, so the rules live in one file
        # instead of being restated in each sweep. Three constants have gone
        # wrong in this project by being duplicated - the object band edge, the
        # reading-quality threshold, and the table height - each time because a
        # value tuned for one configuration was written into another.
        (out / name.replace(".xml", ".env")).write_text(
            "DETECTOR_Z_BIAS=%.4f\n"
            "DETECTOR_OBJECT_DEPTH_HI=%.4f\n"
            "CLOSE_POSITION=%.4f\n"
            "OBJECT_TOP_DEPTH=%.4f\n"
            % (half + DETECTOR_SURFACE_OFFSET_M,
               top_depth + band_margin(s / 1000.0),
               max(0.0, (s / 1000.0 - SQUEEZE_M) / 2.0),
               top_depth),
            encoding="utf-8")
        clearance = ("%.1fmm/side" % ((2.0 * opening * 1000.0 - s) / 2.0)
                     if opening is not None
                     else "(2*open_position*1000-%d)/2 mm/side" % s)
        print("%s  cube %dmm  clearance %s  z_bias %.4f  origin_z %.4f  "
              "top_depth %.4f  band_hi %.4f" % (
                  name, s, clearance, half + DETECTOR_SURFACE_OFFSET_M,
                  TABLE_TOP_M + half, top_depth,
                  top_depth + band_margin(s / 1000.0)))


if __name__ == "__main__":
    main()
