#!/usr/bin/env python3
"""Derive every configuration value for a real setup, and predict where it fails.

Nothing here is new physics - it is the same arithmetic the simulation batches
established, applied to numbers measured off the hardware instead of read out
of the scene file. Doing it on the day, by hand, next to a robot, is how the
wrong constant gets typed in.

Each output cites the batch it comes from, so a value that turns out wrong on
hardware can be traced to the measurement that produced it.

Usage:
  plan_hardware_params.py --fy 600 --cy 240 --standoff 0.75 --cube 0.050
                          [--fx FX] [--cx CX] [--table-to-camera 0.78]
"""
import argparse
import math

# Measured on the gripper used throughout: each finger travels this far, so the
# gap is twice it, and travel caps at 0.060 per finger.
FINGER_TRAVEL_MAX_M = 0.060
# The detector reads the top face; the grasp needs the centre. The 44mm object
# needed 27.5mm against a 22mm half-height, so this is the detector's own
# surface offset. (2026-09-09 clearance document.)
DETECTOR_SURFACE_OFFSET_M = 0.0055
# Squeeze held constant across object sizes so grip force does not confound
# clearance: 30mm object closed to a 12mm gap.
SQUEEZE_M = 0.018
# Band margin above the object's top face. 20mm covered the object's own depth
# spread (the 44mm object measured 0.736 to 0.746) without reaching the table.
OBJECT_BAND_MARGIN_M = 0.020
# Clearance that measured 12/12 at both biases on a 50mm cube (stage47_pairs).
TARGET_CLEARANCE_M = 0.025


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fy", type=float, required=True,
                    help="vertical focal length in pixels")
    ap.add_argument("--cy", type=float, required=True,
                    help="vertical principal point in pixels")
    ap.add_argument("--fx", type=float, help="defaults to fy")
    ap.add_argument("--cx", type=float, help="defaults to cy*4/3")
    ap.add_argument("--standoff", type=float, required=True,
                    help="camera to gripper distance at hover, metres")
    ap.add_argument("--cube", type=float, required=True,
                    help="cube edge length, metres")
    ap.add_argument("--table-to-camera", type=float,
                    help="camera to table surface, metres; "
                         "defaults to standoff + cube")
    args = ap.parse_args()

    fx = args.fx or args.fy
    cx = args.cx or args.cy * 4.0 / 3.0
    table_depth = args.table_to_camera or (args.standoff + args.cube)
    top_depth = table_depth - args.cube

    print("=== 输入 ===")
    print("  fy %.1f px, cy %.1f px, fx %.1f px, cx %.1f px" % (args.fy, args.cy, fx, cx))
    print("  相机→夹爪(hover) %.3f m, 相机→桌面 %.3f m, 方块 %.0f mm"
          % (args.standoff, table_depth, args.cube * 1000))

    print("\n=== 夹爪 ===")
    open_pos = (args.cube + 2 * TARGET_CLEARANCE_M) / 2.0
    if open_pos > FINGER_TRAVEL_MAX_M:
        open_pos = FINGER_TRAVEL_MAX_M
        note = "  (受限于行程上限 %.3f)" % FINGER_TRAVEL_MAX_M
    else:
        note = ""
    clearance = (2 * open_pos - args.cube) / 2.0
    close_pos = (args.cube - SQUEEZE_M) / 2.0
    print("  open_position   %.4f%s   -> 每侧余隙 %.1f mm" % (open_pos, note, clearance * 1000))
    print("  close_position  %.4f      -> 压缩量 %.1f mm" % (close_pos, args.cube * 1000 - close_pos * 2000))
    if clearance < 0.015:
        print("  !! 余隙 < 15 mm。stage47_pairs 在 15 mm 处于 50mm 方块见到滑脱")
    if close_pos < 0:
        print("  !! close_position 为负：方块小于压缩量，需减小 SQUEEZE_M")

    print("\n=== 探测器 ===")
    print("  detector_z_bias    %.4f   (= 边长/2 + %.4f)"
          % (args.cube / 2 + DETECTOR_SURFACE_OFFSET_M, DETECTOR_SURFACE_OFFSET_M))
    print("  object_depth_hi    %.4f   (= 顶面深度 %.4f + %.3f)"
          % (top_depth + OBJECT_BAND_MARGIN_M, top_depth, OBJECT_BAND_MARGIN_M))
    print("  桌面深度           %.4f   (须 > object_depth_hi，余量 %.1f mm)"
          % (table_depth, (table_depth - top_depth - OBJECT_BAND_MARGIN_M) * 1000))
    if table_depth <= top_depth + OBJECT_BAND_MARGIN_M:
        print("  !! 深度带会吃进桌面。减小 OBJECT_BAND_MARGIN_M 或抬高相机")

    print("\n=== 视场上限（可证伪的预测）===")
    y_max = args.cy * args.standoff / args.fy
    x_max = cx * args.standoff / fx
    print("  y 向: bias_max = cy·z/fy = %.1f mm" % (y_max * 1000))
    print("  x 向: bias_max = cx·z/fx = %.1f mm" % (x_max * 1000))
    print("  预测：y 向偏置超过 %.0f mm 后夹爪离开画面，失败形态转为 detector_timeout。" % (y_max * 1000))
    print("  仿真装置 cy=240 fy=579.4 z=0.684 得 283 mm，实测悬崖落在 288(9/12) 与 320(0/12) 之间。")
    print("  真机若在预测值附近出现同样的转变，视场解释即跨平台成立；若不在，则该解释只是本装置的巧合。")


if __name__ == "__main__":
    main()
