import json, math, subprocess, sys, time, tempfile

# ── Link lengths ──────────────────────────────────────────────────────────────
L_ARM1 = 0.050   # was 0.200
L_ARM2 = 0.050   # was 0.200
L_AB   = 0.050   # unchanged
L_PEN  = 0.150   # was 0.240

# ── Ground pivot positions (world XY) ────────────────────────────────────────
PIVOT1 = (0.0,  0.0)   # joint1 (Revolute_9,  cam2)
PIVOT2 = (0.05, 0.0)   # joint2 (Revolute_10, cam1)

# ── Zero-pose offsets ─────────────────────────────────────────────────────────
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


# ── Forward kinematics ────────────────────────────────────────────────────────

def forward_kinematics(theta1_rad, theta2_rad):
    """
    Returns (hA_x, hA_y, hB_x, hB_y, pen_x, pen_y) or None.
    Zero pose: hole A=(0,200)mm, hole B=(50,200)mm, pen=(-240,200)mm
    """
    a1 = theta1_rad + THETA1_ZERO_OFFSET
    hAx = PIVOT1[0] + L_ARM1 * math.cos(a1)
    hAy = PIVOT1[1] + L_ARM1 * math.sin(a1)

    a2 = theta2_rad + THETA2_ZERO_OFFSET
    hBx = PIVOT2[0] + L_ARM2 * math.cos(a2)
    hBy = PIVOT2[1] + L_ARM2 * math.sin(a2)

    dist_AB = math.sqrt((hAx - hBx)**2 + (hAy - hBy)**2)
    if abs(dist_AB - L_AB) > 0.010:
        return None

    dx = hAx - hBx
    dy = hAy - hBy
    d  = math.sqrt(dx**2 + dy**2)
    if d < 1e-9:
        return None

    pen_x = hAx + L_PEN * dx / d
    pen_y = hAy + L_PEN * dy / d
    return hAx, hAy, hBx, hBy, pen_x, pen_y


# ── Lookup table IK ───────────────────────────────────────────────────────────
# Built at startup: sweep ±60° at 0.1° resolution

print("Building IK lookup table (0.1° resolution, ±60° range)...")
_LOOKUP = []
for _t1 in range(-600, 601, 1):
    for _t2 in range(-600, 601, 1):
        _t1r = math.radians(_t1 / 10.0)
        _t2r = math.radians(_t2 / 10.0)
        _fk = forward_kinematics(_t1r, _t2r)
        if _fk:
            _LOOKUP.append((_fk[4], _fk[5], _t1r, _t2r))
print(f"Lookup table ready: {len(_LOOKUP)} valid configurations.")


def inverse_kinematics(pen_x, pen_y, prev_t1=None, prev_t2=None):
    """
    Returns (theta1_rad, theta2_rad) or None.
    Uses lookup table — accurate to ~0.5mm.
    """
    if not _LOOKUP:
        return None

    if prev_t1 is not None and prev_t2 is not None:
        # Weight by both pen position error and joint continuity
        best = min(_LOOKUP, key=lambda e:
                   (e[0]-pen_x)**2 + (e[1]-pen_y)**2 +
                   0.01 * ((e[2]-prev_t1)**2 + (e[3]-prev_t2)**2))
    else:
        best = min(_LOOKUP, key=lambda e: (e[0]-pen_x)**2 + (e[1]-pen_y)**2)

    err = math.sqrt((best[0]-pen_x)**2 + (best[1]-pen_y)**2)
    if err > 0.008:  # 8mm tolerance
        return None

    return best[2], best[3]


# ── Publisher ─────────────────────────────────────────────────────────────────

def send_angles(t1_rad, t2_rad):
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
        steps = max(1, int(dist / 0.002))  # 2mm steps for better accuracy

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
    print(f"  expected: hole A=(0,200), hole B=(50,200), pen=(-240,200)")

if __name__ == "__main__":
    main()