#!/usr/bin/env python3
"""
Offboard mode diagnostics: verifies that all topics required to
fly in offboard mode with PX4 + OptiTrack are active and consistent.

Usage:
    python3 check_offboard_readiness.py

Active prerequisites:
    - micro-XRCE-DDS agent:  sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
    - mocap + bridge:        ros2 launch mocap_px4_bridge run.launch.py
"""

import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    VehicleOdometry,
    VehicleLocalPosition,
    VehicleStatus,
    EstimatorStatusFlags,
)

MOCAP_RAW_TOPIC   = "/Drone_Whale/world"
MOCAP_CONV_TOPIC  = "/fmu/in/vehicle_visual_odometry"
PX4_ODOM_TOPIC    = "/fmu/out/vehicle_odometry"
PX4_LOCAL_TOPIC   = "/fmu/out/vehicle_local_position"
PX4_STATUS_TOPIC  = "/fmu/out/vehicle_status"
PX4_EKF_TOPIC     = "/fmu/out/estimator_status_flags"
TIMEOUT_S = 6.0

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


class DiagNode(Node):
    def __init__(self):
        super().__init__("offboard_diag")
        self._lock = threading.Lock()

        self.mocap_raw_msg:  PoseStamped | None         = None
        self.mocap_conv_msg: VehicleOdometry | None     = None
        self.px4_odom_msg:   VehicleOdometry | None     = None
        self.local_pos_msg:  VehicleLocalPosition | None = None
        self.status_msg:     VehicleStatus | None       = None
        self.ekf_msg:        EstimatorStatusFlags | None = None

        self.counts = {k: 0 for k in
                       ["raw", "conv", "odom", "local", "status", "ekf"]}

        self.create_subscription(PoseStamped,          MOCAP_RAW_TOPIC,  self._cb("raw",    "mocap_raw_msg"),  10)
        self.create_subscription(VehicleOdometry,      MOCAP_CONV_TOPIC, self._cb("conv",   "mocap_conv_msg"), PX4_QOS)
        self.create_subscription(VehicleOdometry,      PX4_ODOM_TOPIC,   self._cb("odom",   "px4_odom_msg"),   PX4_QOS)
        self.create_subscription(VehicleLocalPosition, PX4_LOCAL_TOPIC,  self._cb("local",  "local_pos_msg"),  PX4_QOS)
        self.create_subscription(VehicleStatus,        PX4_STATUS_TOPIC, self._cb("status", "status_msg"),     PX4_QOS)
        self.create_subscription(EstimatorStatusFlags, PX4_EKF_TOPIC,    self._cb("ekf",    "ekf_msg"),        PX4_QOS)

        self.create_timer(TIMEOUT_S, self._report)
        self.get_logger().info("Waiting for messages for %.0f seconds..." % TIMEOUT_S)

    def _cb(self, key, attr):
        def callback(msg):
            with self._lock:
                setattr(self, attr, msg)
                self.counts[key] += 1
        return callback

    def _report(self):
        with self._lock:
            raw    = self.mocap_raw_msg
            conv   = self.mocap_conv_msg
            odom   = self.px4_odom_msg
            lpos   = self.local_pos_msg
            status = self.status_msg
            ekf    = self.ekf_msg
            n = dict(self.counts)

        print()
        print("=" * 68)
        print("  OFFBOARD MODE DIAGNOSTICS")
        print("=" * 68)

        # ── 1. OptiTrack raw ──────────────────────────────────────────────
        ok1 = raw is not None
        _status(ok1, f"OptiTrack raw  ({MOCAP_RAW_TOPIC})", n["raw"], TIMEOUT_S)
        if ok1:
            p, q = raw.pose.position, raw.pose.orientation
            _pose("  OptiTrack", p.x, p.y, p.z, q.w, q.x, q.y, q.z)

        # ── 2. Bridge → PX4 ──────────────────────────────────────────────
        ok2 = conv is not None
        _status(ok2, f"Bridge output  ({MOCAP_CONV_TOPIC})", n["conv"], TIMEOUT_S)
        if ok2:
            p, q = conv.position, conv.q
            _pose("  Bridge→PX4", p[0], p[1], p[2], q[0], q[1], q[2], q[3])
            if ok1:
                _check_transform(raw.pose.position, raw.pose.orientation, conv)

        # ── 3. PX4 odometry ──────────────────────────────────────────────
        ok3 = odom is not None
        _status(ok3, f"PX4 odometry   ({PX4_ODOM_TOPIC})", n["odom"], TIMEOUT_S)
        if ok3:
            p, q = odom.position, odom.q
            _pose("  PX4 estimate", p[0], p[1], p[2], q[0], q[1], q[2], q[3])

        # ── 4. Local position + validity flags ────────────────────────────
        ok4 = lpos is not None
        _status(ok4, f"Local position ({PX4_LOCAL_TOPIC})", n["local"], TIMEOUT_S)
        if ok4:
            xy_ok  = lpos.xy_valid
            z_ok   = lpos.z_valid
            vxy_ok = lpos.v_xy_valid
            vz_ok  = lpos.v_z_valid
            hgt_ok = lpos.heading_good_for_control
            print(f"  pos=({lpos.x:+.3f}, {lpos.y:+.3f}, {lpos.z:+.3f})")
            print(f"  xy_valid={xy_ok}  z_valid={z_ok}  "
                  f"v_xy_valid={vxy_ok}  v_z_valid={vz_ok}  "
                  f"heading_ok={hgt_ok}")
            if not xy_ok or not z_ok:
                print("  [BLOCK]   xy_valid or z_valid = False → PX4 will reject offboard!")
                print("            Most likely cause: EKF2_EV_CTRL not configured")
            else:
                print("  [OK]      Local position valid → offboard should accept it")
        else:
            print("  [WARN]  local_position not received — possible QoS or EKF not initialized")

        # ── 5. Vehicle status ─────────────────────────────────────────────
        ok5 = status is not None
        _status(ok5, f"Vehicle status ({PX4_STATUS_TOPIC})", n["status"], TIMEOUT_S)
        if ok5:
            nav_state    = status.nav_state
            arming_state = status.arming_state
            arm_str = {0: "INIT", 1: "STANDBY", 2: "ARMED", 3: "STANDBY_ERROR",
                       4: "SHUTTINGDOWN", 5: "IN_AIR_RESTORE"}.get(arming_state, str(arming_state))
            nav_str = {
                0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
                14: "OFFBOARD", 17: "TAKEOFF", 18: "LAND",
            }.get(nav_state, str(nav_state))
            print(f"  arming_state={arm_str} ({arming_state})  "
                  f"nav_state={nav_str} ({nav_state})")

        # ── 6. EKF status flags ───────────────────────────────────────────
        ok6 = ekf is not None
        _status(ok6, f"EKF flags      ({PX4_EKF_TOPIC})", n["ekf"], TIMEOUT_S)
        if ok6:
            ev_pos  = getattr(ekf, "cs_ev_pos",  None)
            ev_yaw  = getattr(ekf, "cs_ev_yaw",  None)
            ev_hgt  = getattr(ekf, "cs_ev_hgt",  None)
            gps_pos = getattr(ekf, "cs_gps",     None)
            baro    = getattr(ekf, "cs_baro_hgt", None)
            if ev_pos is not None:
                yaw_align   = getattr(ekf, "cs_yaw_align",    None)
                ev_yaw_fault = getattr(ekf, "cs_ev_yaw_fault", None)
                reject_yaw  = getattr(ekf, "reject_yaw",      None)
                cs_mag_hdg  = getattr(ekf, "cs_mag_hdg",      None)
                cs_mag_3d   = getattr(ekf, "cs_mag_3d",       None)
                print(f"  EKF fusing: ev_pos={ev_pos}  ev_yaw={ev_yaw}  "
                      f"ev_hgt={ev_hgt}  gps={gps_pos}  baro={baro}")
                print(f"  Heading:   yaw_align={yaw_align}  ev_yaw_fault={ev_yaw_fault}  "
                      f"reject_yaw={reject_yaw}  mag_hdg={cs_mag_hdg}  mag_3d={cs_mag_3d}")
                if not ev_pos:
                    print("  [BLOCK]   EKF is not fusing external vision position!")
                    print("            Set EKF2_EV_CTRL=15 and EKF2_HGT_REF=3 in QGC")
                if yaw_align is False:
                    print("  [WARN]    yaw_align=False: EKF has not yet aligned yaw")
                    print("            Wait 10-15s from boot with mocap active and re-launch")
                if ev_yaw_fault:
                    print("  [BLOCK]   cs_ev_yaw_fault=True: EKF has discarded EV yaw!")
                    print("            → bridge quaternion may have wrong axis")
                if reject_yaw:
                    print("  [WARN]    reject_yaw=True: yaw innovation outside EKF gate")
                    print("            → try EKF2_EVA_NOISE=0.1 (more tolerant)")
            else:
                print("  (EV flags not available in this version of px4_msgs)")

        # ── 7. Parameter checklist ───────────────────────────────────────
        print()
        print("─" * 68)
        print("  PX4 PARAMETERS TO VERIFY (MAVLink Console in QGC)")
        print("─" * 68)
        _param("EKF2_EV_CTRL",   "15",
               "enable EV fusion: pos-XY(1)+pos-Z(2)+vel(4)+yaw(8)=15")
        _param("EKF2_HGT_REF",   "3",
               "height reference: 3=Vision (not baro/GPS)")
        _param("EKF2_EV_DELAY",  "0",
               "mocap pipeline latency in ms")
        _param("EKF2_EVP_NOISE", "0.05",
               "EV position noise (m)")
        _param("EKF2_EVA_NOISE", "0.05",
               "EV attitude noise (rad)")
        _param("COM_ARM_WO_GPS", "1",
               "arming without GPS")
        _param("SYS_HAS_GPS",    "0",
               "0 if no GPS → avoids EKF timeout waiting for GPS")
        print()
        print("  Quick commands in MAVLink Console:")
        print("    param set EKF2_EV_CTRL 15")
        print("    param set EKF2_HGT_REF 3")
        print("    param set COM_ARM_WO_GPS 1")
        print("    param set SYS_HAS_GPS 0")
        print("    param save")
        print("    reboot")

        # ── 8. Final summary ──────────────────────────────────────────────
        print()
        print("─" * 68)
        all_ok = ok1 and ok2 and ok3 and ok4
        if all_ok and lpos is not None and lpos.xy_valid and lpos.z_valid:
            print("  [PASS]  Full pipeline and valid position.")
            print("  If offboard still fails → check vehicle_status arming_state")
            print("  and verify EKF2_EV_CTRL with: param show EKF2_EV_CTRL")
        else:
            if not ok1:
                print("  [FAIL]  OptiTrack not publishing → rigid body 'Drone_Whale' in Motive?")
            if not ok2:
                print("  [FAIL]  Bridge silent → 'ros2 launch mocap_px4_bridge run.launch.py'")
            if not ok3:
                print("  [FAIL]  PX4 odometry absent → DDS agent connected?")
            if ok4 and lpos is not None and (not lpos.xy_valid or not lpos.z_valid):
                print("  [FAIL]  Local position NOT valid → configure EKF2_EV_CTRL=15")
        print("=" * 68)
        print()
        raise SystemExit(0 if (all_ok and ok4 and lpos and lpos.xy_valid and lpos.z_valid) else 1)


def _status(ok, label, count, timeout):
    hz  = count / timeout
    tag = "[OK]  " if ok else "[FAIL]"
    print(f"  {tag} {label}  →  {count} msg  ({hz:.1f} Hz)")


def _pose(label, px, py, pz, qw, qx, qy, qz):
    print(f"         {label}  pos=({px:+.3f}, {py:+.3f}, {pz:+.3f})  "
          f"q=({qw:+.3f}, {qx:+.3f}, {qy:+.3f}, {qz:+.3f})")


def _check_transform(raw_p, raw_q, conv):
    cp, cq = conv.position, conv.q
    ok = (abs(cp[0] -  raw_p.x)   < 0.01 and
          abs(cp[1] - (-raw_p.y)) < 0.01 and
          abs(cp[2] - (-raw_p.z)) < 0.01 and
          abs(cq[0] -  raw_q.w)   < 0.01 and
          abs(cq[1] -  raw_q.x)   < 0.01 and
          abs(cq[2] - (-raw_q.y)) < 0.01 and
          abs(cq[3] - (-raw_q.z)) < 0.01)
    if ok:
        print("         [OK]  ENU→FRD transform correct")
    else:
        print("         [WARN] Inconsistent transform — expected: "
              f"pos=({raw_p.x:+.3f}, {-raw_p.y:+.3f}, {-raw_p.z:+.3f})")


def _param(name, expected, desc):
    print(f"  {name:<20s} = {expected:<8s}  # {desc}")


def main():
    rclpy.init()
    node = DiagNode()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        rclpy.shutdown()
        sys.exit(e.code)


if __name__ == "__main__":
    main()
