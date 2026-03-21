import json, math, subprocess, sys, time, tempfile

# ── Link lengths ──────────────────────────────────────────────────────────────
# From Gazebo diagnostic zero pose:
#   cam1_1 (pivot2) and link1_1 share same XY — cam1 arm points along Z (out of plane)
#   So joint2 effectively drives hole B directly with combined arm = 200mm
L_ARM1      = 0.200   # joint1: pivot1 → hole A (cam2+link3 effective length)
L_ARM2      = 0.200   # joint2: pivot2 → hole B (cam1+link1 effective length)
L_AB        = 0.050   # hole A → hole B along link2 (fixed)
L_PEN       = 0.240   # hole A → pen tip along link2

# ── Ground pivot positions (world XY) ────────────────────────────────────────
# Confirmed from Gazebo diagnostic zero pose
PIVOT1 = (0.0,  0.0)   # joint1 (Revolute_9,  cam2)
PIVOT2 = (0.05, 0.0)   # joint2 (Revolute_10, cam1) — 50mm along X

# ── Zero-pose offsets ─────────────────────────────────────────────────────────
# At zero pose:
#   joint1: pivot1=(0,0) → hole A=(0, 0.2) → points along +Y → offset = π/2
#   joint2: pivot2=(0.05,0) → hole B=(0.05, 0.2) → points along +Y → offset = π/2
THETA1_ZERO_OFFSET = math.pi / 2
THETA2_ZERO_OFFSET = math.pi / 2

# ── Gazebo settings ───────────────────────────────────────────────────────────
WORLD      = "penplotter_world"
TOPIC      = f"/gazebo/{WORLD}/penplotter/joint_cmd"
STEP_DELAY = 0.05
LIFT_DELAY = 0.2

# ── Trail settings ────────────────────────────────────────────────────────────
TRAIL_SPHERE_RADIUS = 0.003
TRAIL_COLOUR        = "1 0 0 1"
TRAIL_HEIGHT        = 0.012
TRAIL_MODEL_NAME    = "pen_trail"


# ── Geometry helpers ──────────────────────────────────────────────────────────

def circle_intersect(c1x, c1y, r1, c2x, c2y, r2):
    """Return intersection points of two circles."""
    dx = c2x - c1x
    dy = c2y - c1y
    d = math.sqrt(dx**2 + dy**2)
    if d < 1e-9 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return []
    a = (r1**2 - r2**2 + d**2) / (2 * d)
    h = math.sqrt(max(0.0, r1**2 - a**2))
    mx = c1x + a * dx / d
    my = c1y + a * dy / d
    if h < 1e-9:
        return [(mx, my)]
    return [
        (mx + h * (-dy / d), my + h * (dx / d)),
        (mx - h * (-dy / d), my - h * (dx / d))
    ]


# ── Forward kinematics ────────────────────────────────────────────────────────

def forward_kinematics(theta1_rad, theta2_rad):
    """
    Given joint angles as reported by Gazebo (radians):
      theta1 = Revolute_9  (joint1, cam2+link3)
      theta2 = Revolute_10 (joint2, cam1+link1)

    Returns (holeA_x, holeA_y, holeB_x, holeB_y, pen_x, pen_y) or None.

    Zero pose sanity check (theta1=0, theta2=0):
      hole A = (0.0,   200.0) mm
      hole B = (50.0,  200.0) mm
      pen    = (-240.0, 200.0) mm
    """
    # ── Joint1: hole A ────────────────────────────────────────────────────────
    a1 = theta1_rad + THETA1_ZERO_OFFSET
    holeA_x = PIVOT1[0] + L_ARM1 * math.cos(a1)
    holeA_y = PIVOT1[1] + L_ARM1 * math.sin(a1)

    # ── Joint2: hole B ────────────────────────────────────────────────────────
    a2 = theta2_rad + THETA2_ZERO_OFFSET
    holeB_x = PIVOT2[0] + L_ARM2 * math.cos(a2)
    holeB_y = PIVOT2[1] + L_ARM2 * math.sin(a2)

    # ── Verify AB constraint ──────────────────────────────────────────────────
    dist_AB = math.sqrt((holeA_x - holeB_x)**2 + (holeA_y - holeB_y)**2)
    if abs(dist_AB - L_AB) > 0.005:  # 5mm tolerance
        return None

    # ── Pen tip: L_PEN beyond hole A, away from hole B ───────────────────────
    dx = holeA_x - holeB_x
    dy = holeA_y - holeB_y
    d  = math.sqrt(dx**2 + dy**2)
    if d < 1e-9:
        return None
    ux = dx / d
    uy = dy / d

    pen_x = holeA_x + L_PEN * ux
    pen_y = holeA_y + L_PEN * uy

    return holeA_x, holeA_y, holeB_x, holeB_y, pen_x, pen_y


# ── Inverse kinematics ────────────────────────────────────────────────────────

def inverse_kinematics(pen_x, pen_y, prev_t1=None, prev_t2=None):
    """
    Given desired pen tip position (metres),
    return (theta1_rad, theta2_rad) or None.

    Both angles are geometrically consistent with the closed-loop constraint.
    """
    solutions = []

    # Hole A lies on circle of radius L_PEN around pen tip
    # and circle of radius L_ARM1 around pivot1
    holeA_cands = circle_intersect(
        pen_x,     pen_y,     L_PEN,
        PIVOT1[0], PIVOT1[1], L_ARM1
    )

    for (hAx, hAy) in holeA_cands:
        # θ1 from hole A position
        t1 = math.atan2(hAy - PIVOT1[1], hAx - PIVOT1[0]) - THETA1_ZERO_OFFSET

        # Direction from hole A toward pen tip
        dx_pen = pen_x - hAx
        dy_pen = pen_y - hAy
        d_pen  = math.sqrt(dx_pen**2 + dy_pen**2)
        if d_pen < 1e-9:
            continue
        ux = dx_pen / d_pen
        uy = dy_pen / d_pen

        # Hole B is L_AB from hole A, opposite direction to pen
        hBx = hAx - L_AB * ux
        hBy = hAy - L_AB * uy

        # Hole B must be reachable by joint2 arm from pivot2
        dist_to_hB = math.sqrt((hBx - PIVOT2[0])**2 + (hBy - PIVOT2[1])**2)
        if abs(dist_to_hB - L_ARM2) > 0.005:
            continue

        # θ2 from hole B position
        t2 = math.atan2(hBy - PIVOT2[1], hBx - PIVOT2[0]) - THETA2_ZERO_OFFSET

        # Verify with FK
        fk = forward_kinematics(t1, t2)
        if fk is None:
            continue
        err = math.sqrt((fk[4] - pen_x)**2 + (fk[5] - pen_y)**2)
        if err < 0.001:
            solutions.append((t1, t2, err))

    if not solutions:
        return None

    if prev_t1 is not None and prev_t2 is not None:
        solutions.sort(key=lambda s: (s[0]-prev_t1)**2 + (s[1]-prev_t2)**2)
    else:
        solutions.sort(key=lambda s: s[2])

    return solutions[0][0], solutions[0][1]


# ── Publisher ─────────────────────────────────────────────────────────────────

def send_angles(t1_rad, t2_rad):
    """Send both joint angles (degrees) to the controller."""
    msg = f"{math.degrees(t1_rad):.4f},{math.degrees(t2_rad):.4f}"
    cmd = ["gz", "topic", "--publish", TOPIC, "--msg", f'data: "{msg}"']
    r   = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[WARN] {r.stderr.strip()}")


# ── Trail helpers ─────────────────────────────────────────────────────────────

def fk_to_world(fk):
    return fk[4], fk[5], TRAIL_HEIGHT


def build_trail_sdf(points):
    visuals = []
    for i, (wx, wy, wz) in enumerate(points):
        visuals.append(f"""
        <visual name='dot_{i}'>
          <pose>{wx:.6f} {wy:.6f} {wz:.6f} 0 0 0</pose>
          <geometry>
            <sphere><radius>{TRAIL_SPHERE_RADIUS}</radius></sphere>
          </geometry>
          <material>
            <ambient>{TRAIL_COLOUR}</ambient>
            <diffuse>{TRAIL_COLOUR}</diffuse>
          </material>
        </visual>""")

    sdf = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='{TRAIL_MODEL_NAME}'>
    <static>true</static>
    <link name='trail_link'>
      <pose>0 0 0 0 0 0</pose>
{"".join(visuals)}
    </link>
  </model>
</sdf>"""
    return sdf


def spawn_trail(points):
    if not points:
        print("[TRAIL] No points recorded — nothing to spawn.")
        return

    print(f"[TRAIL] Spawning {len(points)} marker(s) …")
    sdf_str = build_trail_sdf(points)

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf',
                                      delete=False, prefix='pen_trail_')
    tmp.write(sdf_str)
    tmp.close()

    cmd = ["gz", "model", "-w", WORLD, "-m", TRAIL_MODEL_NAME, "-f", tmp.name]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode == 0:
        print(f"[TRAIL] Spawned OK  ({tmp.name})")
    else:
        print(f"[TRAIL] Spawn failed: {r.stderr.strip()}")
        print(f"[TRAIL] SDF saved to {tmp.name} — spawn manually if needed.")


# ── Main drawing loop ─────────────────────────────────────────────────────────

def draw_segments(segments):
    print(f"Loaded {len(segments)} segments. Publishing to {TOPIC}")
    enable_trail = "--no-trail" not in sys.argv
    if enable_trail:
        print("[TRAIL] Pen trail enabled.")
    print("Starting in 2s — make sure Gazebo is running …")
    time.sleep(2)

    prev_t1 = 0.0
    prev_t2 = 0.0
    trail_points = []

    for i, seg in enumerate(segments):
        x0, y0 = seg[0][0] / 1000, seg[0][1] / 1000
        x1, y1 = seg[1][0] / 1000, seg[1][1] / 1000
        print(f"[{i+1}/{len(segments)}] "
              f"({seg[0][0]:.1f},{seg[0][1]:.1f})→({seg[1][0]:.1f},{seg[1][1]:.1f})mm")

        # ── Move to segment start ─────────────────────────────────────────────
        r = inverse_kinematics(x0, y0, prev_t1, prev_t2)
        if r is None:
            print("  [SKIP] unreachable")
            continue
        prev_t1, prev_t2 = r
        send_angles(prev_t1, prev_t2)
        time.sleep(LIFT_DELAY)

        # ── Draw segment ──────────────────────────────────────────────────────
        dx = x1 - x0
        dy = y1 - y0
        dist = math.sqrt(dx**2 + dy**2)
        steps = max(1, int(dist / 0.005))

        for k in range(1, steps + 1):
            frac = k / steps
            r = inverse_kinematics(x0 + frac * dx, y0 + frac * dy,
                                   prev_t1, prev_t2)
            if r is None:
                continue
            prev_t1, prev_t2 = r
            fk = forward_kinematics(prev_t1, prev_t2)
            if fk:
                print(f"  pen=({fk[4]*1000:.1f},{fk[5]*1000:.1f})mm  "
                      f"θ1={math.degrees(prev_t1):+.1f}°  "
                      f"θ2={math.degrees(prev_t2):+.1f}°")
                if enable_trail:
                    trail_points.append(fk_to_world(fk))
            send_angles(prev_t1, prev_t2)
            time.sleep(STEP_DELAY)

    print("Done.")
    if enable_trail:
        spawn_trail(trail_points)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_path.py segments.json [--no-trail]")
        sys.exit(1)

    with open(sys.argv[1]) as f:
        segments = json.load(f)

    draw_segments(segments)


# ── Sanity check ──────────────────────────────────────────────────────────────

print("Sanity check (θ1=0, θ2=0):")
fk = forward_kinematics(0.0, 0.0)
if fk:
    print(f"  hole A: ({fk[0]*1000:.1f}, {fk[1]*1000:.1f}) mm")
    print(f"  hole B: ({fk[2]*1000:.1f}, {fk[3]*1000:.1f}) mm")
    print(f"  pen:    ({fk[4]*1000:.1f}, {fk[5]*1000:.1f}) mm")
    print(f"  expected: hole A=(0, 200), hole B=(50, 200), pen=(-240, 200)")
else:
    print("  FK returned None — geometry issue")

print("\nIK round-trip test:")
if fk:
    ik = inverse_kinematics(fk[4], fk[5])
    if ik:
        print(f"  θ1={math.degrees(ik[0]):.2f}°  θ2={math.degrees(ik[1]):.2f}°")
        print(f"  expected: θ1=0.00°  θ2=0.00°")
    else:
        print("  IK returned None")

if __name__ == "__main__":
    main()