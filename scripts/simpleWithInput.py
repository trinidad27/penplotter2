#!/usr/bin/env python3
"""
pantograph_sim.py
-----------------
2D turtle simulation of the Pen-Pal pantograph plotter.

Servo ranges (physical):
  Bottom servo (θ1): [-180, 180]
  Top servo    (θ2): [-180, 180]

Run:
    python3 pantograph_sim.py
"""

import json
import math
import tkinter as tk
import tkinter.filedialog as filedialog
import turtle

import websocket
import threading
import base64
import queue

# ── Hardware dimensions (metres) ─────────────────────────────────────────────
L_short    = 0.050   # arm1: bottom servo motor arm
L_link     = 0.220   # arm2: link arm (arm1 tip → Pen B)
L_long     = 0.220   # arm4: top servo motor arm (→ Pen A)
PEN_TO_A   = 0.240   # pen tip → Pen A along arm3
PEN_A_TO_B = 0.050   # Pen A → Pen B along arm3

# ── Display ───────────────────────────────────────────────────────────────────
SCALE    = 700        # pixels per metre
ORIGIN_X = -180       # turtle-space x of motor pivot
ORIGIN_Y = -160       # turtle-space y of motor pivot


# ── Arm collision (capsule model) ────────────────────────────────────────────

ARM_WIDTH      = 0.010   # physical arm width, 10 mm
# Minimum angular separation: angle at which arm1's tip (L_short away from O)
# is exactly ARM_WIDTH clear of arm4's centre-line.
#   L_short * sin(MIN_ARM_ANGLE) = ARM_WIDTH  →  solve for angle
MIN_ARM_ANGLE  = math.degrees(math.asin(ARM_WIDTH / L_short))  # ≈ 11.5°


def arms_collide(theta1_deg, theta2_deg):
    """Return True if arm1 and arm4 would physically overlap.

    Both arms radiate from the same pivot O.  The binding constraint is the
    tip of arm1 (the shorter arm, L_short = 50 mm): when the angular gap
    between the two centre-lines is less than MIN_ARM_ANGLE the arms overlap.
    We use the shorter arm as the reference because it sets the tightest
    clearance — arm4 is longer so its tip sweeps a wider arc, but arm1's tip
    is the first thing that would physically clash.
    """
    diff = abs(theta1_deg - theta2_deg) % 360.0
    if diff > 180.0:
        diff = 360.0 - diff          # always in [0, 180]
    return diff < MIN_ARM_ANGLE


# ── Kinematics ────────────────────────────────────────────────────────────────

def _ci(c1x, c1y, r1, c2x, c2y, r2):
    dx = c2x-c1x; dy = c2y-c1y; d = math.sqrt(dx**2+dy**2)
    if d < 1e-9 or d > r1+r2+1e-9 or d < abs(r1-r2)-1e-9:
        return []
    a = (r1**2-r2**2+d**2)/(2*d)
    h = math.sqrt(max(0.0, r1**2-a**2))
    mx = c1x+a*dx/d; my = c1y+a*dy/d
    if h < 1e-9:
        return [(mx, my)]
    return [(mx+h*(-dy/d), my+h*(dx/d)), (mx-h*(-dy/d), my-h*(dx/d))]


def forward_kinematics(theta1_deg, theta2_deg):
    """Returns (ax,ay, pbx,pby, pen_x,pen_y) or None.

    Parallelogram constraint enforced directly:
      Pen_B = arm1_tip + arm4_vector  (arm2 ∥ arm4, same length)
    This is the unique valid configuration; no circle intersection needed.
    """
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    ax  = L_long  * math.cos(t2);  ay  = L_long  * math.sin(t2)   # Pen A
    m1x = L_short * math.cos(t1);  m1y = L_short * math.sin(t1)   # arm1 tip

    # Parallelogram: Pen_B = arm1_tip + (Pen_A - O) = arm1_tip + Pen_A
    pbx = m1x + ax
    pby = m1y + ay

    # Pen tip: extend arm3 beyond Pen_A, away from Pen_B, by PEN_TO_A
    dx = ax - pbx;  dy = ay - pby   # = -m1x, -m1y  (always well-defined)
    d  = math.sqrt(dx**2 + dy**2)
    if d < 1e-9:
        return None
    pen_x = ax + PEN_TO_A * dx / d
    pen_y = ay + PEN_TO_A * dy / d
    return ax, ay, pbx, pby, pen_x, pen_y


def inverse_kinematics(x, y, prev_t1=None, prev_t2=None):
    """Returns (theta1, theta2) or None.

    Parallelogram constraint: Pen_B = arm1_tip + Pen_A
    So: arm1_tip = Pen_B - Pen_A

    Given pen tip (x,y):
      Pen_A lies on circle(pen_tip, PEN_TO_A) ∩ circle(O, L_long)
      Pen_B = Pen_A + PEN_A_TO_B * unit(Pen_A - pen_tip)   [arm3 direction]
      arm1_tip = Pen_B - Pen_A                              [parallelogram]
      θ2 = atan2(Pen_A)
      θ1 = atan2(arm1_tip)   [arm1_tip must have length L_short]

    If prev_t1/prev_t2 given, picks closest solution to avoid flipping.
    Also filters out solutions where arms collide.
    """
    a_cands = _ci(x, y, PEN_TO_A, 0, 0, L_long)
    solutions = []
    for ax, ay in a_cands:
        t2 = math.degrees(math.atan2(ay, ax))

        # Pen_B along arm3, on far side of Pen_A from pen tip
        dxt = ax - x;  dyt = ay - y
        dt  = math.sqrt(dxt**2 + dyt**2)
        if dt < 1e-9:
            continue
        pbx = ax + PEN_A_TO_B * dxt / dt
        pby = ay + PEN_A_TO_B * dyt / dt

        # arm1 tip = Pen_B - Pen_A  (parallelogram constraint)
        m1x = pbx - ax
        m1y = pby - ay

        # Check arm1 length matches L_short
        d1 = math.sqrt(m1x**2 + m1y**2)
        if abs(d1 - L_short) > 1e-6:
            continue

        t1 = math.degrees(math.atan2(m1y, m1x))

        # Skip configurations where arms physically collide
        if arms_collide(t1, t2):
            continue

        fk = forward_kinematics(t1, t2)
        if fk is None:
            continue
        err = math.sqrt((fk[4] - x)**2 + (fk[5] - y)**2)
        if err < 0.001:
            solutions.append((t1, t2, err))

    if not solutions:
        return None
    if prev_t1 is not None and prev_t2 is not None:
        solutions.sort(key=lambda s: (s[0]-prev_t1)**2 + (s[1]-prev_t2)**2)
    else:
        solutions.sort(key=lambda s: s[2])
    return solutions[0][0], solutions[0][1]


# ── Coordinate conversion ─────────────────────────────────────────────────────

def to_screen(mx, my):
    return ORIGIN_X + mx * SCALE, ORIGIN_Y + my * SCALE


def from_screen(cx, cy):
    """Inverse of to_screen: canvas pixel → metres."""
    mx = (cx - 350 - ORIGIN_X) / SCALE
    my = (250 - cy - ORIGIN_Y) / SCALE
    return mx, my


# ── Drawing primitives ────────────────────────────────────────────────────────

def draw_line(t, x0, y0, x1, y1, color, width=2):
    t.penup(); t.goto(x0, y0)
    t.pendown(); t.pencolor(color); t.width(width)
    t.goto(x1, y1); t.penup()


def draw_dot(t, x, y, color, size=6):
    t.penup(); t.goto(x, y - size//2)
    t.pendown(); t.fillcolor(color); t.begin_fill()
    t.circle(size//2); t.end_fill(); t.penup()


# ── Main application ──────────────────────────────────────────────────────────

class PantographSim:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Pantograph Plotter — 2D Kinematics Sim")
        self.root.resizable(False, False)

        # Turtle canvas
        self.canvas = tk.Canvas(self.root, width=700, height=500, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4)
        self.screen = turtle.TurtleScreen(self.canvas)
        self.screen.tracer(0)

        self.t_grid  = turtle.RawTurtle(self.screen)
        self.t_arm   = turtle.RawTurtle(self.screen)
        self.t_path  = turtle.RawTurtle(self.screen)
        self.t_label = turtle.RawTurtle(self.screen)
        self.t_draw  = turtle.RawTurtle(self.screen)   # freehand drawing layer — on top
        for t in (self.t_grid, self.t_arm, self.t_path, self.t_label, self.t_draw):
            t.hideturtle(); t.speed(0); t.penup()
        self.t_path.pencolor("#2255cc")
        self.t_path.width(1)
        self.t_draw.pencolor("#cc2200")
        self.t_draw.width(2)

        # State
        self.theta1         = tk.DoubleVar(value=135.0)
        self.theta2         = tk.DoubleVar(value=90.0)
        self.last_valid_t1  = 135.0   # ← last collision-free θ1
        self.last_valid_t2  = 90.0    # ← last collision-free θ2
        self._slider_busy   = False   # re-entrancy guard for slider callbacks
        self.pen_down       = True
        self.last_pen_pos   = None
        self.status_var     = tk.StringVar(value="")
        assert not arms_collide(135.0, 90.0), "startup angles collide — fix defaults"

        # Freehand drawing state
        self.draw_mode      = False   # True when mouse-draw is active
        self.drawn_segments = []      # list of [[x0_mm,y0_mm],[x1_mm,y1_mm]]
        self._draw_last     = None    # last canvas (x,y) during a drag
        self.drawing_queue = queue.Queue()

        # Controls
        ctrl = tk.Frame(self.root, padx=8, pady=4)
        ctrl.grid(row=1, column=0, columnspan=4, sticky="ew")
        pad = dict(padx=6, pady=2)

        # θ1 — full range slider
        tk.Label(ctrl, text="Bottom servo θ1:", font=("Helvetica", 10)
                 ).grid(row=0, column=0, sticky="w", **pad)
        self.lbl1 = tk.Label(ctrl, text="+135.0°", width=8,
                              font=("Helvetica", 10, "bold"), fg="#2255aa")
        self.lbl1.grid(row=0, column=2, **pad)
        tk.Scale(ctrl, from_=-180, to=180,
                 orient=tk.HORIZONTAL, length=320, resolution=0.5,
                 variable=self.theta1, command=self._on_t1_slider,
                 showvalue=False, troughcolor="#b0c8f0"
                 ).grid(row=0, column=1, **pad)

        # θ2 — full range slider
        tk.Label(ctrl, text="Top servo θ2:", font=("Helvetica", 10)
                 ).grid(row=1, column=0, sticky="w", **pad)
        self.lbl2 = tk.Label(ctrl, text=" 90.0°", width=8,
                              font=("Helvetica", 10, "bold"), fg="#cc4400")
        self.lbl2.grid(row=1, column=2, **pad)
        tk.Scale(ctrl, from_=-180, to=180,
                 orient=tk.HORIZONTAL, length=320, resolution=0.5,
                 variable=self.theta2, command=self._on_t2_slider,
                 showvalue=False, troughcolor="#f0c0a0"
                 ).grid(row=1, column=1, **pad)

        # IK row
        ik = tk.Frame(ctrl)
        ik.grid(row=2, column=0, columnspan=4, sticky="w", **pad)
        tk.Label(ik, text="IK target  x (m):", font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.ik_x = tk.Entry(ik, width=7, font=("Helvetica", 10))
        self.ik_x.insert(0, "0.20"); self.ik_x.pack(side=tk.LEFT, padx=4)
        tk.Label(ik, text="y (m):", font=("Helvetica", 10)).pack(side=tk.LEFT)
        self.ik_y = tk.Entry(ik, width=7, font=("Helvetica", 10))
        self.ik_y.insert(0, "0.20"); self.ik_y.pack(side=tk.LEFT, padx=4)
        tk.Button(ik, text="Move", command=self._do_ik,
                  font=("Helvetica", 10), width=6).pack(side=tk.LEFT, padx=6)
        self.pen_btn = tk.Button(ik, text="Pen: DOWN", command=self._toggle_pen,
                                 font=("Helvetica", 10), width=10, fg="#225522")
        self.pen_btn.pack(side=tk.LEFT, padx=6)

        # Bottom row
        br = tk.Frame(ctrl)
        br.grid(row=3, column=0, columnspan=4, sticky="w", **pad)
        tk.Button(br, text="Clear path", command=self._clear_path,
                  font=("Helvetica", 10)).pack(side=tk.LEFT, padx=4)
        tk.Button(br, text="Load & Draw", command=self._load_and_draw,
                  font=("Helvetica", 10), fg="#225577").pack(side=tk.LEFT, padx=4)
        self.draw_btn = tk.Button(br, text="Draw Mode: OFF",
                  command=self._toggle_draw_mode,
                  font=("Helvetica", 10), fg="#888800", width=14)
        self.draw_btn.pack(side=tk.LEFT, padx=4)
        tk.Button(br, text="Clear Drawing", command=self._clear_drawing,
                  font=("Helvetica", 10), fg="#aa2222").pack(side=tk.LEFT, padx=4)
        tk.Button(br, text="Export Drawing", command=self._export_drawing,
                  font=("Helvetica", 10), fg="#226622").pack(side=tk.LEFT, padx=4)
        tk.Button(br, text="Quit", command=self.root.destroy,
                  font=("Helvetica", 10)).pack(side=tk.LEFT, padx=4)
        tk.Label(ctrl, textvariable=self.status_var,
                 font=("Helvetica", 9), fg="#555", anchor="w"
                 ).grid(row=3, column=2, columnspan=2, sticky="w", **pad)

        # Mouse bindings for freehand drawing
        self.canvas.bind("<ButtonPress-1>",   self._draw_start)
        self.canvas.bind("<B1-Motion>",       self._draw_move)
        self.canvas.bind("<ButtonRelease-1>", self._draw_end)

        self._draw_grid()
        self._redraw()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_t1_slider(self, _=None):
        if self._slider_busy:
            return
        t1 = self.theta1.get()
        t2 = self.theta2.get()
        if arms_collide(t1, t2):
            self._slider_busy = True
            self.theta1.set(self.last_valid_t1)
            self._slider_busy = False
            self.status_var.set("⚠  Arm collision — θ1 clamped.")
            return
        self.last_valid_t1 = t1
        self.lbl1.config(text=f"{t1:+7.1f}°")
        self._redraw()

    def _on_t2_slider(self, _=None):
        if self._slider_busy:
            return
        t1 = self.theta1.get()
        t2 = self.theta2.get()
        if arms_collide(t1, t2):
            self._slider_busy = True
            self.theta2.set(self.last_valid_t2)
            self._slider_busy = False
            self.status_var.set("⚠  Arm collision — θ2 clamped.")
            return
        self.last_valid_t2 = t2
        self.lbl2.config(text=f"{t2:6.1f}°")
        self._redraw()

    def _do_ik(self):
        try:
            x, y = float(self.ik_x.get()), float(self.ik_y.get())
        except ValueError:
            self.status_var.set("Invalid x/y."); return
        result = inverse_kinematics(x, y)
        if result is None:
            self.status_var.set(f"({x:.3f}, {y:.3f}) m — unreachable."); return
        self.theta1.set(round(result[0], 1))
        self.theta2.set(round(result[1], 1))
        self.last_valid_t1 = self.theta1.get()
        self.last_valid_t2 = self.theta2.get()
        self.lbl1.config(text=f"{self.theta1.get():+7.1f}°")
        self.lbl2.config(text=f"{self.theta2.get():6.1f}°")
        self._redraw()

    def _toggle_pen(self):
        self.pen_down = not self.pen_down
        self.pen_btn.config(
            text="Pen: DOWN" if self.pen_down else "Pen: UP  ",
            fg="#225522"    if self.pen_down else "#aa2222")
        self.last_pen_pos = None

    def _clear_path(self):
        self.t_path.clear(); self.last_pen_pos = None

    def _clear_drawing(self):
        self.t_draw.clear()
        self.drawn_segments = []
        self._draw_last = None
        self.status_var.set("Drawing cleared.")
        self.screen.update()

    def _toggle_draw_mode(self):
        self.draw_mode = not self.draw_mode
        if self.draw_mode:
            self.draw_btn.config(text="Draw Mode: ON ", fg="#cc4400")
            self.canvas.config(cursor="crosshair")
            self.status_var.set("Draw mode — click and drag on the canvas to draw.")
        else:
            self.draw_btn.config(text="Draw Mode: OFF", fg="#888800")
            self.canvas.config(cursor="")
            self.status_var.set(
                f"Draw mode off — {len(self.drawn_segments)} segment(s) recorded.")

    def _draw_start(self, event):
        if not self.draw_mode:
            return
        self._draw_last = (event.x, event.y)

    def _draw_move(self, event):
        if not self.draw_mode or self._draw_last is None:
            return
        x0c, y0c = self._draw_last
        x1c, y1c = event.x, event.y

        # Convert canvas pixels → metres → mm
        x0m, y0m = from_screen(x0c, y0c)
        x1m, y1m = from_screen(x1c, y1c)
        x0mm, y0mm = x0m * 1000, y0m * 1000
        x1mm, y1mm = x1m * 1000, y1m * 1000

        # Record segment
        self.drawn_segments.append([[x0mm, y0mm], [x1mm, y1mm]])

        # Draw on canvas via turtle (convert back to turtle coords)
        tx0, ty0 = to_screen(x0m, y0m)
        tx1, ty1 = to_screen(x1m, y1m)
        draw_line(self.t_draw, tx0, ty0, tx1, ty1, color="#cc2200", width=2)
        self.screen.update()

        self._draw_last = (x1c, y1c)

    def _draw_end(self, event):
        if not self.draw_mode:
            return
        self._draw_last = None

    def _export_drawing(self):
        if not self.drawn_segments:
            self.status_var.set("Nothing to export — draw something first.")
            return

        self.drawing_queue.put(json.dumps(self.drawn_segments))

    def _load_and_draw(self):
        """
        Load a JSON file containing line segments and draw them.

        Expected format — list of segments, each segment a pair of [x, y] points
        in millimetres (same unit as the IK entry boxes × 1000):
            [[[x0, y0], [x1, y1]], [[x1, y1], [x2, y2]], ...]

        The plotter lifts the pen when moving to the start of each segment
        and draws with it down along the segment.  Steps along each segment
        are 5 mm apart to keep motion smooth.
        """
        path = filedialog.askopenfilename(
            title="Open segment file",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if not path:
            return

        try:
            with open(path) as f:
                segments = json.load(f)
        except Exception as e:
            self.status_var.set(f"Error reading file: {e}")
            return

        try:
            for seg in segments:
                (seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])
        except Exception:
            self.status_var.set("Bad format — expected [[[x,y],[x,y]], ...]")
            return

        STEP_MM = 5.0

        def _set_angles(result, draw):
            """Commit a valid IK result to the sim state."""
            self.pen_down = draw
            self.pen_btn.config(
                text="Pen: DOWN" if draw else "Pen: UP  ",
                fg="#225522" if draw else "#aa2222")
            if not draw:
                self.last_pen_pos = None
            self.theta1.set(round(result[0], 1))
            self.theta2.set(round(result[1], 1))
            self.last_valid_t1 = self.theta1.get()
            self.last_valid_t2 = self.theta2.get()
            self.lbl1.config(text=f"{self.theta1.get():+7.1f}°")
            self.lbl2.config(text=f"{self.theta2.get():6.1f}°")
            self._redraw()
            self.root.update()

        def _draw_segment(x0_m, y0_m, x1_m, y1_m):
            """
            Interpolate from (x0,y0) to (x1,y1) with pen down.
            If a point is unreachable, lift the pen and wait.
            Re-lower the pen only once IK succeeds again, drawing
            from the re-entry point onward — no jumps.
            """
            dx   = x1_m - x0_m
            dy   = y1_m - y0_m
            dist = math.sqrt(dx**2 + dy**2)
            n_steps = max(1, int(dist / (STEP_MM / 1000)))

            pen_lifted_by_ik = False   # True when we lifted due to IK failure

            for k in range(n_steps + 1):
                t   = k / n_steps
                xpt = x0_m + t * dx
                ypt = y0_m + t * dy

                result = inverse_kinematics(
                    xpt, ypt,
                    prev_t1=self.theta1.get(),
                    prev_t2=self.theta2.get())

                if result is None:
                    # Point unreachable — lift pen silently, don't move
                    if not pen_lifted_by_ik:
                        pen_lifted_by_ik = True
                        self.pen_down = False
                        self.pen_btn.config(text="Pen: UP  ", fg="#aa2222")
                        self.last_pen_pos = None
                    continue

                if pen_lifted_by_ik:
                    # Re-entering reachable zone — move to this point with
                    # pen still up, then lower it here (no jump)
                    _set_angles(result, draw=False)
                    pen_lifted_by_ik = False

                _set_angles(result, draw=True)

        n_segs = len(segments)

        for i, seg in enumerate(segments):
            x0_mm, y0_mm = seg[0]
            x1_mm, y1_mm = seg[1]
            x0_m, y0_m = x0_mm / 1000, y0_mm / 1000
            x1_m, y1_m = x1_mm / 1000, y1_mm / 1000

            self.status_var.set(
                f"Segment {i+1}/{n_segs}  "
                f"({x0_mm:.0f},{y0_mm:.0f}) → ({x1_mm:.0f},{y1_mm:.0f}) mm")
            self.root.update()

            # Move to segment start with pen up
            result = inverse_kinematics(x0_m, y0_m,
                                        prev_t1=self.theta1.get(),
                                        prev_t2=self.theta2.get())
            if result is not None:
                _set_angles(result, draw=False)

            _draw_segment(x0_m, y0_m, x1_m, y1_m)

        self.status_var.set(f"Done — {n_segs} segments drawn.")

    def _draw(self, segments):
        try:
            for seg in segments:
                (seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])
        except Exception:
            self.status_var.set("Bad format — expected [[[x,y],[x,y]], ...]")
            return

        STEP_MM = 5.0

        def _set_angles(result, draw):
            """Commit a valid IK result to the sim state."""
            self.pen_down = draw
            self.pen_btn.config(
                text="Pen: DOWN" if draw else "Pen: UP  ",
                fg="#225522" if draw else "#aa2222")
            if not draw:
                self.last_pen_pos = None
            self.theta1.set(round(result[0], 1))
            self.theta2.set(round(result[1], 1))
            self.last_valid_t1 = self.theta1.get()
            self.last_valid_t2 = self.theta2.get()
            self.lbl1.config(text=f"{self.theta1.get():+7.1f}°")
            self.lbl2.config(text=f"{self.theta2.get():6.1f}°")
            self._redraw()
            self.root.update()

        def _draw_segment(x0_m, y0_m, x1_m, y1_m):
            """
            Interpolate from (x0,y0) to (x1,y1) with pen down.
            If a point is unreachable, lift the pen and wait.
            Re-lower the pen only once IK succeeds again, drawing
            from the re-entry point onward — no jumps.
            """
            dx   = x1_m - x0_m
            dy   = y1_m - y0_m
            dist = math.sqrt(dx**2 + dy**2)
            n_steps = max(1, int(dist / (STEP_MM / 1000)))

            pen_lifted_by_ik = False   # True when we lifted due to IK failure

            for k in range(n_steps + 1):
                t   = k / n_steps
                xpt = x0_m + t * dx
                ypt = y0_m + t * dy

                result = inverse_kinematics(
                    xpt, ypt,
                    prev_t1=self.theta1.get(),
                    prev_t2=self.theta2.get())

                if result is None:
                    # Point unreachable — lift pen silently, don't move
                    if not pen_lifted_by_ik:
                        pen_lifted_by_ik = True
                        self.pen_down = False
                        self.pen_btn.config(text="Pen: UP  ", fg="#aa2222")
                        self.last_pen_pos = None
                    continue

                if pen_lifted_by_ik:
                    # Re-entering reachable zone — move to this point with
                    # pen still up, then lower it here (no jump)
                    _set_angles(result, draw=False)
                    pen_lifted_by_ik = False

                _set_angles(result, draw=True)

        self._clear_path()
        n_segs = len(segments)

        for i, seg in enumerate(segments):
            x0_mm, y0_mm = seg[0]
            x1_mm, y1_mm = seg[1]
            x0_m, y0_m = x0_mm / 1000, y0_mm / 1000
            x1_m, y1_m = x1_mm / 1000, y1_mm / 1000

            self.status_var.set(
                f"Segment {i+1}/{n_segs}  "
                f"({x0_mm:.0f},{y0_mm:.0f}) → ({x1_mm:.0f},{y1_mm:.0f}) mm")
            self.root.update()

            # Move to segment start with pen up
            result = inverse_kinematics(x0_m, y0_m,
                                        prev_t1=self.theta1.get(),
                                        prev_t2=self.theta2.get())
            if result is not None:
                _set_angles(result, draw=False)

            _draw_segment(x0_m, y0_m, x1_m, y1_m)

        self.status_var.set(f"Done — {n_segs} segments drawn.")

    # ── Grid ──────────────────────────────────────────────────────────────────

    def _draw_grid(self):
        t = self.t_grid; t.clear()
        t.pencolor("#e0e0e0"); t.width(1)
        step = 0.050; n = 10
        for i in range(-n, n+1):
            sx, _  = to_screen(i*step, 0)
            _, sy0 = to_screen(0, -n*step); _, sy1 = to_screen(0, n*step)
            t.penup(); t.goto(sx, sy0); t.pendown(); t.goto(sx, sy1); t.penup()
            _, sy  = to_screen(0, i*step)
            sx0, _ = to_screen(-n*step, 0); sx1, _ = to_screen(n*step, 0)
            t.penup(); t.goto(sx0, sy); t.pendown(); t.goto(sx1, sy); t.penup()

        t.pencolor("#bbbbbb"); t.width(2)
        ox, oy = to_screen(0, 0)
        ax0, _ = to_screen(-n*step, 0); ax1, _ = to_screen(n*step, 0)
        _, ay0 = to_screen(0, -n*step); _, ay1 = to_screen(0, n*step)
        t.penup(); t.goto(ax0, oy); t.pendown(); t.goto(ax1, oy); t.penup()
        t.penup(); t.goto(ox, ay0); t.pendown(); t.goto(ox, ay1); t.penup()

        t.pencolor("#aaaaaa")
        for i in range(-n, n+1):
            if i == 0: continue
            if i % 2 == 0:
                sx, _ = to_screen(i*step, 0)
                t.goto(sx, oy-16)
                t.write(f"{i*50:.0f}", align="center", font=("Helvetica", 6, "normal"))
                _, sy = to_screen(0, i*step)
                t.goto(ox+4, sy-4)
                t.write(f"{i*50:.0f}", font=("Helvetica", 6, "normal"))

        draw_dot(t, ox, oy, "#666666", 5)
        self.screen.update()

    # ── Arm redraw ────────────────────────────────────────────────────────────

    def _redraw(self):
        t1  = self.theta1.get()
        t2  = self.theta2.get()
        fk  = forward_kinematics(t1, t2)
        arm = self.t_arm; arm.clear()
        ox, oy = to_screen(0, 0)

        if fk is None:
            self.status_var.set("⚠  Unreachable — adjust servos.")
            self.screen.update(); return

        ax, ay, pbx, pby, px, py = fk
        s_o  = (ox, oy)
        s_a  = to_screen(ax, ay)
        s_pb = to_screen(pbx, pby)
        s_p  = to_screen(px, py)
        s_m1 = to_screen(L_short * math.cos(math.radians(t1)),
                          L_short * math.sin(math.radians(t1)))

        # Arms
        draw_line(arm, *s_o,  *s_a,  color="#cc4400", width=5)  # arm4, top servo
        draw_line(arm, *s_o,  *s_m1, color="#003399", width=5)  # arm1, bottom servo
        draw_line(arm, *s_m1, *s_pb, color="#5599ee", width=3)  # arm2, link
        draw_line(arm, *s_pb, *s_a,  color="#228833", width=3)  # arm3 lower
        draw_line(arm, *s_a,  *s_p,  color="#228833", width=3)  # arm3 upper

        # Joints
        draw_dot(arm, *s_o,  "#333333", 8)
        draw_dot(arm, *s_m1, "#003399", 6)
        draw_dot(arm, *s_a,  "#cc4400", 7)
        draw_dot(arm, *s_pb, "#5599ee", 6)
        draw_dot(arm, *s_p,  "#cc0000", 9)

        # Labels
        lbl = self.t_label; lbl.clear(); lbl.pencolor("#333333")
        def wrt(sx, sy, txt, dx=7, dy=4):
            lbl.penup(); lbl.goto(sx+dx, sy+dy)
            lbl.write(txt, font=("Helvetica", 8, "normal"))
        wrt(*s_o,  "O",     dx=-16, dy=4)
        wrt(*s_m1, "arm1",  dx=5,   dy=-14)
        wrt(*s_a,  "Pen A", dx=5,   dy=-14)
        wrt(*s_pb, "Pen B", dx=5,   dy=4)
        wrt(*s_p,  "PEN",   dx=6,   dy=5)

        self.status_var.set(
            f"θ1 = {t1:+.1f}°   θ2 = {t2:.1f}°   "
            f"pen = ({px*1000:.1f}, {py*1000:.1f}) mm"
        )

        if self.pen_down:
            if self.last_pen_pos is not None:
                tp = self.t_path
                tp.penup(); tp.goto(*self.last_pen_pos)
                tp.pendown(); tp.goto(*s_p); tp.penup()
            self.last_pen_pos = s_p
        else:
            self.last_pen_pos = None

        self.screen.update()

    def run(self):
        bridge_thread = threading.Thread(target=bridge_loop, args=[self])
        bridge_thread.start()
        self.root.mainloop()

def bridge_loop(sim: PantographSim):
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
        print("got to here")
        assert msg["session_id"] == session_id
        if msg["type"] == "TURN":
            print("OUR TURN")
            lines = sim.drawing_queue.get(block=True)
            submit_msg = {
                "type": "SUBMIT",
                "session_id": session_id,
                "svg": lines 
            }
            ws.send(json.dumps(submit_msg))
        if msg["type"] == "DRAW":
            print("OUR DRAW")
            sim._draw(json.loads(msg["svg"]))

if __name__ == "__main__":
    sim = PantographSim()
    sim.run()

