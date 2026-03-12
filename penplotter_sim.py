"""
pantograph_kinematics.py
------------------------
Forward and inverse kinematics for the Pen-Pal scaled pantograph plotter.

SDF joint chain (XZ drawing plane, Y = out-of-plane rotation axis):

  Revolute_9  (phi1, bottom servo, axis=+Y) → cam2_1
                link3 merged into cam2_1, initial direction -Z
                Pen B (loop_closure) at z=-0.175 in cam2_1 local frame

  Revolute_10 (phi2, top servo, axis=+Y) → cam_1
                cam arm, initial direction +X, length 0.050 m
                  └─ Revolute_15 at (0.05, 0, 0) in cam_1 frame (phi15, axis=+Y) → link1_1
                       link1 arm, initial direction -Z, length 0.200 m
                         └─ Revolute_18 at (0, 0, -0.2) in link1_1 frame (phi18, axis=-Y) → link2_1
                              pen bar (arm3); also receives loop_closure_link2_link3

  loop_closure_link2_link3: cam2_1 → link2_1
    Passive joint. Satisfied automatically when phi15 and phi18 are set correctly.
    Pen B attachment in cam2_1 local frame: (0, 0, -0.175)
    Pen B attachment in link2_1 local frame: (0, 0, +PEN_A_TO_B) [+Z direction]

Dimensions from SDF:
  L_short    = 0.175 m   link3 arm (Revolute_9 → Pen B)
  L_long     = 0.050 m   cam arm   (Revolute_10 → Revolute_15)
  L_link     = 0.200 m   link1 arm (Revolute_15 → Pen A / Revolute_18)
  PEN_TO_A   = 0.240 m   pen tip → Pen A along arm3
  PEN_A_TO_B = 0.050 m   Pen A → Pen B along arm3

Four joints to command in Gazebo on every move:
  Revolute_9   ← phi1  (you command this)
  Revolute_10  ← phi2  (you command this)
  Revolute_15  ← phi15 (computed — prevents loop-closure pop)
  Revolute_18  ← phi18_gz = -phi18_raw  (axis is -Y in SDF, so negate)

FK coordinate convention (model XZ plane):
  A joint at angle θ with initial arm direction -Z:
    tip_x = L * sin(θ),   tip_z = -L * cos(θ)
  A joint at angle θ with initial arm direction +X (cam only):
    tip_x = L * cos(θ),   tip_z =  L * sin(θ)
"""

import math
from typing import Optional, Tuple, Dict, Any

# ── Dimensions (from SDF) ─────────────────────────────────────────────────────
L_short    = 0.175   # link3 arm    (Revolute_9  → Pen B)
L_long     = 0.050   # cam arm      (Revolute_10 → Revolute_15)
L_link     = 0.200   # link1 arm    (Revolute_15 → Pen A)
PEN_TO_A   = 0.240   # pen tip → Pen A along arm3
PEN_A_TO_B = 0.050   # Pen A → Pen B along arm3

# ── Gazebo joint names ────────────────────────────────────────────────────────
JOINT_PHI1  = "Revolute_9"    # bottom servo (commanded)
JOINT_PHI2  = "Revolute_10"   # top servo    (commanded)
JOINT_PHI15 = "Revolute_15"   # link1 rel. to cam (computed, axis=+Y)
JOINT_PHI18 = "Revolute_18"   # pen bar rel. to link1 (computed, axis=-Y → negate)


# ── Internal helpers ──────────────────────────────────────────────────────────

def _ci(c1x: float, c1z: float, r1: float,
        c2x: float, c2z: float, r2: float):
    """Circle-circle intersections in the XZ plane."""
    dx = c2x - c1x;  dz = c2z - c1z
    d  = math.sqrt(dx**2 + dz**2)
    if d < 1e-9 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return []
    a  = (r1**2 - r2**2 + d**2) / (2 * d)
    h  = math.sqrt(max(0.0, r1**2 - a**2))
    mx = c1x + a*dx/d;  mz = c1z + a*dz/d
    if h < 1e-9:
        return [(mx, mz)]
    return [
        (mx + h*(-dz/d),  mz + h*(dx/d)),
        (mx - h*(-dz/d),  mz - h*(dx/d)),
    ]


# ── Forward kinematics ────────────────────────────────────────────────────────

def forward_kinematics(
    phi1_deg: float,
    phi2_deg: float,
) -> Optional[Dict[str, Any]]:
    """
    Pantograph FK in the SDF model XZ plane.

    Parameters
    ----------
    phi1_deg : Revolute_9  angle (degrees) — bottom servo
    phi2_deg : Revolute_10 angle (degrees) — top servo

    Returns
    -------
    dict with:
      'pen_b'   : (x, z) Pen B world pos  (loop_closure, link3 tip)
      'r15'     : (x, z) Revolute_15 pos  (cam tip)
      'pen_a'   : (x, z) Pen A world pos  (Revolute_18, link1 tip)
      'pen'     : (x, z) pen tip world pos
      'phi15'   : Revolute_15 angle in degrees   → pass to gz joint as-is
      'phi18_gz': Revolute_18 angle for Gazebo   → already negated for -Y axis
    or None if unreachable.

    Loop-closure guarantee: setting all four Gazebo joints simultaneously
    to phi1, phi2, phi15, phi18_gz ensures the loop_closure_link2_link3
    constraint is satisfied exactly (0.000 mm error by construction).
    """
    phi1 = math.radians(phi1_deg)
    phi2 = math.radians(phi2_deg)

    # Pen B = link3 tip  (initial dir -Z, rotates by phi1 around +Y)
    #   x = L_short * sin(phi1),   z = -L_short * cos(phi1)
    pbx = L_short * math.sin(phi1)
    pbz = -L_short * math.cos(phi1)

    # Revolute_15 = cam tip  (initial dir +X, rotates by phi2 around +Y)
    #   x = L_long * cos(phi2),   z = L_long * sin(phi2)
    r15x = L_long * math.cos(phi2)
    r15z = L_long * math.sin(phi2)

    # Pen A: on circle(R15, L_link) AND circle(Pen_B, PEN_A_TO_B)
    pa_cands = _ci(r15x, r15z, L_link, pbx, pbz, PEN_A_TO_B)
    if not pa_cands:
        return None

    for ax, az in pa_cands:
        # ── phi15: angle of link1_1 in world, minus phi2 (parent cam angle) ──
        # link1_1 initial dir -Z. World dir = (sin(phi2+phi15), -cos(phi2+phi15))
        # sin(phi2+phi15) = (ax-r15x)/L_link   [x component]
        # cos(phi2+phi15) = -(az-r15z)/L_link  [z component: -cos → az-r15z]
        alpha    = math.atan2(ax - r15x, -(az - r15z))   # = phi2 + phi15 in radians
        phi15    = math.degrees(alpha) - phi2_deg

        # ── pen tip: extend arm3 240mm beyond Pen A, away from Pen B ──
        dx = ax - pbx;  dz = az - pbz
        d  = math.sqrt(dx**2 + dz**2)
        if d < 1e-9:
            continue
        pen_x = ax + PEN_TO_A * dx / d
        pen_z = az + PEN_TO_A * dz / d

        # ── phi18: angle of pen bar relative to link1 ──
        # pen bar world angle (same -Z initial convention):
        #   sin(penbar_angle) = dx/d,   cos(penbar_angle) = -(-dz/d) = ... 
        #   direction (dx/d, dz/d) → angle via atan2(dx, -dz)
        penbar_angle = math.atan2(dx, -dz)
        phi18_raw    = math.degrees(penbar_angle) - math.degrees(alpha)
        # Revolute_18 axis is xyz="0 -1 0" (negative Y), so negate for Gazebo
        phi18_gz     = -phi18_raw

        return {
            'pen_b':    (pbx,            pbz),
            'r15':      (r15x,           r15z),
            'pen_a':    (ax,             az),
            'pen':      (round(pen_x, 6), round(pen_z, 6)),
            'phi15':    round(phi15,   4),
            'phi18_gz': round(phi18_gz, 4),
        }
    return None


# ── Inverse kinematics ────────────────────────────────────────────────────────

def inverse_kinematics(
    x: float,
    z: float,
    flip: bool = False,
) -> Optional[Tuple[float, float]]:
    """
    Pantograph IK in the SDF model XZ plane.

    Parameters
    ----------
    x, z : float
        Target pen tip position (metres, model XZ plane).
    flip : bool
        Select the alternate Pen B / Pen A configuration when two exist.

    Returns
    -------
    (phi1_deg, phi2_deg) or None if unreachable.
    Call forward_kinematics(phi1, phi2) to obtain phi15 and phi18_gz.
    """
    # Pen B lies on circle(pen_tip, PEN_TO_A+PEN_A_TO_B) ∩ circle(origin, L_short)
    # (arm3 is rigid; Pen B is PEN_TO_A+PEN_A_TO_B beyond pen_tip along arm3)
    R = PEN_TO_A + PEN_A_TO_B
    pb_cands = _ci(0.0, 0.0, L_short, x, z, R)

    solutions = []
    for pbx, pbz in pb_cands:
        # phi1 from Pen B position  (sin(phi1)=pbx/L_short, cos(phi1)=-pbz/L_short)
        phi1 = math.degrees(math.atan2(pbx, -pbz))

        # Pen A: along arm3 from Pen B toward pen tip, PEN_A_TO_B from Pen B
        dx = x - pbx;  dz = z - pbz;  d = math.sqrt(dx**2 + dz**2)
        if d < 1e-9:
            continue
        ax = pbx + PEN_A_TO_B * dx / d
        az = pbz + PEN_A_TO_B * dz / d

        # Revolute_15 (R15): on circle(Pen_A, L_link) ∩ circle(origin, L_long)
        for r15x, r15z in _ci(ax, az, L_link, 0.0, 0.0, L_long):
            # phi2 from R15 position  (cos(phi2)=r15x/L_long, sin(phi2)=r15z/L_long)
            phi2 = math.degrees(math.atan2(r15z, r15x))

            # FK round-trip to verify
            fk = forward_kinematics(phi1, phi2)
            if fk is None:
                continue
            err = math.sqrt((fk['pen'][0] - x)**2 + (fk['pen'][1] - z)**2)
            if err < 0.001:
                solutions.append((phi1, phi2, err))

    if not solutions:
        return None

    solutions.sort(key=lambda s: s[2])
    unique = [solutions[0]]
    for s in solutions[1:]:
        if all(abs(s[0] - u[0]) > 0.5 or abs(s[1] - u[1]) > 0.5 for u in unique):
            unique.append(s)
    idx = 1 if (flip and len(unique) > 1) else 0
    return unique[idx][0], unique[idx][1]


# ── Self-test ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=== Forward kinematics + loop-closure verification ===")
    for p1, p2 in [(0,0),(0,45),(0,90),(-45,45),(45,45),(30,60),(10,80),(-30,30)]:
        r = forward_kinematics(p1, p2)
        if r:
            # Verify loop closure: Pen B from link2_1 side must equal pen_b
            link2_angle = math.radians(p2 + r['phi15'] - r['phi18_gz'])
            pb2x = r['pen_a'][0] - PEN_A_TO_B * math.sin(link2_angle)
            pb2z = r['pen_a'][1] + PEN_A_TO_B * math.cos(link2_angle)
            lc_err = math.sqrt((pb2x - r['pen_b'][0])**2 + (pb2z - r['pen_b'][1])**2)
            print(f"  φ1={p1:+4d}° φ2={p2:+3d}°  "
                  f"φ15={r['phi15']:+7.2f}°  φ18_gz={r['phi18_gz']:+7.2f}°  "
                  f"pen=({r['pen'][0]*1000:.1f},{r['pen'][1]*1000:.1f})mm  "
                  f"loop_err={lc_err*1000:.4f}mm")
        else:
            print(f"  φ1={p1:+4d}° φ2={p2:+3d}°  UNREACHABLE")

    print("\n=== IK round-trip ===")
    passed = failed = 0
    for p1, p2 in [(0,0),(0,45),(0,90),(-45,45),(45,45),(30,60),(10,80),(-30,30)]:
        r = forward_kinematics(p1, p2)
        if not r: print(f"  ({p1:+3d},{p2:+3d})  FK unreachable"); continue
        px, pz = r['pen']
        ik = inverse_kinematics(px, pz)
        if ik:
            vr  = forward_kinematics(*ik)
            err = math.sqrt((vr['pen'][0]-px)**2+(vr['pen'][1]-pz)**2) if vr else 999.
            mark = "✓" if err < 0.001 else "✗"
            print(f"  ({p1:+3d},{p2:+3d})  pen=({px*1000:.1f},{pz*1000:.1f})mm  "
                  f"→  φ1={ik[0]:+.1f}° φ2={ik[1]:+.1f}°  err={err*1000:.3f}mm {mark}")
            if err < 0.001: passed += 1
            else:           failed  += 1
        else:
            print(f"  ({p1:+3d},{p2:+3d})  pen=({px*1000:.1f},{pz*1000:.1f})mm  →  IK FAILED")
            failed += 1
    print(f"\n  {passed}/{passed+failed} passed")