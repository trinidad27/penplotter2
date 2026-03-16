import json, math, subprocess, sys, time, os, tempfile

import websocket, base64

L_short    = 0.050
L_long     = 0.205
PEN_TO_A   = 0.240
PEN_A_TO_B = 0.050

WORLD      = "penplotter_world"
TOPIC      = f"/gazebo/{WORLD}/penplotter/joint_cmd"
STEP_DELAY = 0.05
LIFT_DELAY = 0.2

# ── Trail settings ──────────────────────────────────────────────────────────
# Gazebo pen plotter sits in the XZ plane (Y is up after the 90° world_to_base
# rotation).  The FK gives (x, y) in the robot's own 2-D plane; we map those
# to Gazebo world coordinates below.
TRAIL_SPHERE_RADIUS = 0.003          # 3 mm — visible but not intrusive
TRAIL_COLOUR        = "1 0 0 1"      # RGBA  — red
TRAIL_HEIGHT        = 0.012          # world Y of the marker plane (just above ground)
TRAIL_MODEL_NAME    = "pen_trail"    # name of the spawned model


# ── Kinematics ──────────────────────────────────────────────────────────────

def _ci(c1x, c1y, r1, c2x, c2y, r2):
    dx = c2x - c1x; dy = c2y - c1y
    d  = math.sqrt(dx**2 + dy**2)
    if d < 1e-9 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return []
    a  = (r1**2 - r2**2 + d**2) / (2 * d)
    h  = math.sqrt(max(0.0, r1**2 - a**2))
    mx = c1x + a * dx / d;  my = c1y + a * dy / d
    if h < 1e-9:
        return [(mx, my)]
    return [(mx + h * (-dy / d), my + h * (dx / d)),
            (mx - h * (-dy / d), my - h * (dx / d))]


def forward_kinematics(t1d, t2d):
    t1 = math.radians(t1d);  t2 = math.radians(t2d)
    ax = L_long  * math.cos(t2);  ay = L_long  * math.sin(t2)
    m1x = L_short * math.cos(t1); m1y = L_short * math.sin(t1)
    pbx = m1x + ax;               pby = m1y + ay
    dx = ax - pbx;                dy  = ay - pby
    d  = math.sqrt(dx**2 + dy**2)
    if d < 1e-9:
        return None
    return ax, ay, pbx, pby, ax + PEN_TO_A * dx / d, ay + PEN_TO_A * dy / d


def inverse_kinematics(x, y, prev_t1=None, prev_t2=None):
    solutions = []
    for ax, ay in _ci(x, y, PEN_TO_A, 0, 0, L_long):
        t2 = math.degrees(math.atan2(ay, ax))
        dxt = ax - x;  dyt = ay - y
        dt  = math.sqrt(dxt**2 + dyt**2)
        if dt < 1e-9:
            continue
        pbx = ax + PEN_A_TO_B * dxt / dt
        pby = ay + PEN_A_TO_B * dyt / dt
        m1x = pbx - ax;  m1y = pby - ay
        if abs(math.sqrt(m1x**2 + m1y**2) - L_short) > 1e-6:
            continue
        t1 = math.degrees(math.atan2(m1y, m1x))
        fk = forward_kinematics(t1, t2)
        if fk and math.sqrt((fk[4] - x)**2 + (fk[5] - y)**2) < 0.001:
            solutions.append((t1, t2))
    if not solutions:
        return None
    if prev_t1 is not None:
        solutions.sort(key=lambda s: (s[0] - prev_t1)**2 + (s[1] - prev_t2)**2)
    return solutions[0]


def send_angles(t1, t2):
    msg = f"{t1:.4f},{t2:.4f}"
    cmd = ["gz", "topic", "--publish", TOPIC, "--msg", f'data: "{msg}"']
    r   = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[WARN] {r.stderr.strip()}")


# ── Trail helpers ────────────────────────────────────────────────────────────

def fk_to_world(fk):
    """
    Convert FK pen position (robot 2-D x,y) → Gazebo world (wx, wy, wz).

    The SDF has:
      world_to_base: pose = "0 0 0.01  1.5708 0 0"
      Revolute_9 (cam2) at base_link y=+0.01
      Revolute_10 (cam) at base_link y=+0.05

    After the 90° pitch rotation, the robot's local Z axis maps to world −Y,
    and the robot's local X axis stays as world X.  The FK works in the plane
    of the disc (robot XZ), so:
        world_x  =  fk_x   (robot local X)
        world_y  =  TRAIL_HEIGHT  (constant — just above ground)
        world_z  = -fk_y   (robot local Y → world -Z ... actually +Z after
                             the world_to_base 90° pitch)

    Empirically the easiest check: the FK origin is roughly at the cam2 joint
    (0, 0.01 in base_link frame → world 0, 0.01, ~0).
    We keep it simple and map fk_x → world X, fk_y → world Z.
    """
    fx, fy = fk[4], fk[5]
    wx = fx
    wy = fy        # swap — let fy drive height
    wz = 0.012     # constant Z depth
    return wx, wy, wz


def build_trail_sdf(points):
    """
    Build an SDF string with one <visual> sphere per recorded pen position.
    We emit them as a single static model so Gazebo handles it efficiently.
    """
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

    visuals_str = "\n".join(visuals)

    sdf = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='{TRAIL_MODEL_NAME}'>
    <static>true</static>
    <link name='trail_link'>
      <pose>0 0 0 0 0 0</pose>
{visuals_str}
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

    # Write to a temp file so gz model -f can read it
    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf',
                                      delete=False, prefix='pen_trail_')
    tmp.write(sdf_str)
    tmp.close()

    cmd = ["gz", "model",
           "-w", WORLD,
           "-m", TRAIL_MODEL_NAME,
           "-f", tmp.name]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode == 0:
        print(f"[TRAIL] Spawned OK  ({tmp.name})")
    else:
        print(f"[TRAIL] Spawn failed: {r.stderr.strip()}")
        print(f"[TRAIL] SDF saved to {tmp.name} — you can spawn it manually.")

def draw_segments(segments):
    print(f"Loaded {len(segments)} segments. Publishing to {TOPIC}")
    enable_trail = "--no-trail" not in sys.argv
    if enable_trail:
        print(f"[TRAIL] Pen trail enabled — spheres will be spawned after path completes.")
    print("Starting in 2s — make sure Gazebo is running …")
    time.sleep(2)

    prev_t1, prev_t2 = 135.0, 90.0
    trail_points = []   # world-space (wx, wy, wz) recorded during pen-down moves

    for i, seg in enumerate(segments):
        x0, y0 = seg[0][0] / 1000, seg[0][1] / 1000
        x1, y1 = seg[1][0] / 1000, seg[1][1] / 1000
        print(f"[{i+1}/{len(segments)}] "
              f"({seg[0][0]:.1f},{seg[0][1]:.1f})→({seg[1][0]:.1f},{seg[1][1]:.1f})mm")

        # ── Move to segment start (pen up / lift) ──
        r = inverse_kinematics(x0, y0, prev_t1, prev_t2)
        if r and (abs(r[0] - prev_t1) > 90 or abs(r[1] - prev_t2) > 90):
            print(f"  [WARN] big angle jump! "
                  f"prev=({prev_t1:.1f},{prev_t2:.1f}) new=({r[0]:.1f},{r[1]:.1f})")
        if r is None:
            print("  [SKIP] unreachable"); continue
        prev_t1, prev_t2 = r
        send_angles(*r)
        time.sleep(LIFT_DELAY)

        # ── Draw segment (pen down) ──
        dx = x1 - x0;  dy = y1 - y0
        dist = math.sqrt(dx**2 + dy**2)
        steps = max(1, int(dist / 0.005))

        for k in range(1, steps + 1):
            frac = k / steps
            r = inverse_kinematics(x0 + frac * dx, y0 + frac * dy,
                                   prev_t1, prev_t2)
            if r is None:
                continue
            prev_t1, prev_t2 = r
            fk = forward_kinematics(*r)
            if fk:
                print(f"  pen=({fk[4]*1000:.1f},{fk[5]*1000:.1f})mm  "
                      f"θ1={r[0]:+.1f}° θ2={r[1]:.1f}°")
                if enable_trail:
                    trail_points.append(fk_to_world(fk))
            send_angles(*r)
            time.sleep(STEP_DELAY)

    print("Done.")

    if enable_trail:
        spawn_trail(trail_points)

def bridge_loop(sendback):
    def dummy_do_turn():
        input("Press enter to complete turn...")
        return sendback

    token = int.to_bytes(1 << 56, 8)

    ws = websocket.WebSocket()
    try:
        ws.connect("ws://localhost:3000/ws", header = {
            "Authorization": base64.urlsafe_b64encode(token).decode()
        })
    except Exception as e:
        print("Failed to connect to server", e)
        return

    print("Waiting for session to start...")
    msg = json.loads(ws.recv())
    if msg["type"] != "START_SESSION":
        print("Server error")
        return
    session_id = msg["session_id"]
    print("Session started")

    while True:
        msg = json.loads(ws.recv())
        assert msg["session_id"] == session_id
        match msg["type"]:
            case "TURN":
                lines = dummy_do_turn()
                submit_msg = {
                    "type": "SUBMIT",
                    "session_id": session_id,
                    "svg": lines 
                }
                ws.send(json.dumps(submit_msg))
            case "DRAW":
                draw_segments(msg["svg"])

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_path.py segments.json [--no-trail] [--bridge sendback.json]")
        sys.exit(1)

    bridge_idx = sys.argv.index("--bridge")

    if bridge_idx >= 0:
        if bridge_idx + 1 >= len(sys.argv):
            print("didn't specify the sendback file")
            sys.exit(1)

        with open(sys.argv[bridge_idx + 1]) as f:
            sendback = json.load(f)
        bridge_loop(sendback)
    else:
        with open(sys.argv[1]) as f:
            segments = json.load(f)

        draw_segments(segments)

if __name__ == "__main__":
    main()
