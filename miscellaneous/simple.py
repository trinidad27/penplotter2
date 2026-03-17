#!/usr/bin/env python3
#2d simulation with no constraints on sliders
"""
pantograph_sim.py
-----------------
2D turtle simulation of the Pen-Pal pantograph plotter.

Shows the arm linkage as animated lines on a grid.
Use the sliders to move the servos, or type a target (x, y)
to invoke inverse kinematics.

Controls:
  Sliders   — move each servo manually
  IK entry  — type target x,y and press Move
  Clear     — clear the drawn path
  Quit      — close the window

Run:
    python3 pantograph_sim.py
"""

import math
import tkinter as tk
from tkinter import ttk
import turtle

# ── Hardware dimensions (metres) ─────────────────────────────────────────────
L_short    = 0.050   # arm1: bottom servo motor arm
L_link     = 0.220   # arm2: link arm (arm1 tip → Pen B)
L_long     = 0.220   # arm4: top servo motor arm (→ Pen A)
PEN_TO_A   = 0.240   # pen tip → Pen A along arm3
PEN_A_TO_B = 0.050   # Pen A → Pen B along arm3

# ── Display scaling ───────────────────────────────────────────────────────────
SCALE      = 800     # pixels per metre
ORIGIN_X   = -250    # turtle-space x of motor pivot
ORIGIN_Y   = -150    # turtle-space y of motor pivot

# Servo angle limits (degrees)
T1_MIN, T1_MAX = -180, 180
T2_MIN, T2_MAX = 0, 180


# ── Kinematics ────────────────────────────────────────────────────────────────

def _circle_intersections(c1x, c1y, r1, c2x, c2y, r2):
    dx = c2x - c1x;  dy = c2y - c1y
    d  = math.sqrt(dx**2 + dy**2)
    if d < 1e-9 or d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return []
    a  = (r1**2 - r2**2 + d**2) / (2 * d)
    h  = math.sqrt(max(0.0, r1**2 - a**2))
    mx = c1x + a * dx / d;  my = c1y + a * dy / d
    if h < 1e-9:
        return [(mx, my)]
    return [
        (mx + h * (-dy / d), my + h * ( dx / d)),
        (mx - h * (-dy / d), my - h * ( dx / d)),
    ]


def forward_kinematics(theta1_deg, theta2_deg):
    """Returns (ax,ay, pbx,pby, px,py) or None."""
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    ax  = L_long  * math.cos(t2);  ay  = L_long  * math.sin(t2)
    m1x = L_short * math.cos(t1);  m1y = L_short * math.sin(t1)
    pb_cands = _circle_intersections(m1x, m1y, L_link, ax, ay, PEN_A_TO_B)
    if not pb_cands:
        return None
    for pbx, pby in pb_cands:
        dx = ax - pbx;  dy = ay - pby
        d  = math.sqrt(dx**2 + dy**2)
        if d < 1e-9:
            continue
        return ax, ay, pbx, pby, ax + PEN_TO_A*dx/d, ay + PEN_TO_A*dy/d
    return None


def inverse_kinematics(x, y, flip=False):
    """Returns (theta1_deg, theta2_deg) or None."""
    a_cands = _circle_intersections(x, y, PEN_TO_A, 0, 0, L_long)
    solutions = []
    for ax, ay in a_cands:
        t2  = math.degrees(math.atan2(ay, ax))
        dxt = ax - x;  dyt = ay - y
        dt  = math.sqrt(dxt**2 + dyt**2)
        if dt < 1e-9:
            continue
        pbx = ax + PEN_A_TO_B * dxt / dt
        pby = ay + PEN_A_TO_B * dyt / dt
        m1_cands = _circle_intersections(pbx, pby, L_link, 0, 0, L_short)
        for m1x, m1y in m1_cands:
            t1 = math.degrees(math.atan2(m1y, m1x))
            fk = forward_kinematics(t1, t2)
            if fk is None:
                continue
            err = math.sqrt((fk[4] - x)**2 + (fk[5] - y)**2)
            if err < 0.001:
                solutions.append((t1, t2, err))
    if not solutions:
        return None
    solutions.sort(key=lambda s: s[2])
    unique = [solutions[0]]
    for s in solutions[1:]:
        if all(abs(s[0]-u[0]) > 0.5 or abs(s[1]-u[1]) > 0.5 for u in unique):
            unique.append(s)
    idx = 1 if (flip and len(unique) > 1) else 0
    return unique[idx][0], unique[idx][1]


# ── Coordinate helpers ────────────────────────────────────────────────────────

def to_screen(mx, my):
    """Convert metres (robot space) to turtle screen coordinates."""
    return ORIGIN_X + mx * SCALE, ORIGIN_Y + my * SCALE


# ── Drawing helpers ───────────────────────────────────────────────────────────

def draw_line(t, x0, y0, x1, y1, color, width=2):
    t.penup()
    t.goto(x0, y0)
    t.pendown()
    t.pencolor(color)
    t.width(width)
    t.goto(x1, y1)
    t.penup()


def draw_dot(t, x, y, color, size=6):
    t.penup()
    t.goto(x, y - size // 2)
    t.pendown()
    t.fillcolor(color)
    t.begin_fill()
    t.circle(size // 2)
    t.end_fill()
    t.penup()


# ── Main application ──────────────────────────────────────────────────────────

class PantographSim:
    def __init__(self):
        # ── Tkinter root ──────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("Pantograph Plotter — 2D Kinematics Sim")
        self.root.resizable(False, False)

        # ── Turtle canvas ─────────────────────────────────────────────────────
        self.canvas = tk.Canvas(self.root, width=700, height=500, bg="white")
        self.canvas.grid(row=0, column=0, columnspan=4)

        self.screen = turtle.TurtleScreen(self.canvas)
        self.screen.tracer(0)

        # Turtles: grid, arm, path, labels
        self.t_grid  = turtle.RawTurtle(self.screen)
        self.t_arm   = turtle.RawTurtle(self.screen)
        self.t_path  = turtle.RawTurtle(self.screen)
        self.t_label = turtle.RawTurtle(self.screen)

        for t in (self.t_grid, self.t_arm, self.t_path, self.t_label):
            t.hideturtle()
            t.speed(0)
            t.penup()

        self.t_path.pencolor("#2255cc")
        self.t_path.width(1)

        # ── State ─────────────────────────────────────────────────────────────
        self.theta1 = tk.DoubleVar(value=30.0)
        self.theta2 = tk.DoubleVar(value=90.0)
        self.pen_down = True
        self.last_pen_pos = None
        self.status_var = tk.StringVar(value="")

        # ── Controls ─────────────────────────────────────────────────────────
        ctrl = tk.Frame(self.root, padx=8, pady=6)
        ctrl.grid(row=1, column=0, columnspan=4, sticky="ew")

        pad = dict(padx=6, pady=3)

        # Servo 1
        tk.Label(ctrl, text="Bottom servo θ1:", font=("Helvetica", 10)
                 ).grid(row=0, column=0, sticky="w", **pad)
        self.lbl1 = tk.Label(ctrl, text=" 30.0°", width=7,
                              font=("Helvetica", 10, "bold"), fg="#2255aa")
        self.lbl1.grid(row=0, column=2, **pad)
        tk.Scale(ctrl, from_=T1_MIN, to=T1_MAX, orient=tk.HORIZONTAL,
                 length=320, resolution=0.5, variable=self.theta1,
                 command=self._on_slider, showvalue=False
                 ).grid(row=0, column=1, **pad)

        # Servo 2
        tk.Label(ctrl, text="Top servo θ2:", font=("Helvetica", 10)
                 ).grid(row=1, column=0, sticky="w", **pad)
        self.lbl2 = tk.Label(ctrl, text=" 90.0°", width=7,
                              font=("Helvetica", 10, "bold"), fg="#2255aa")
        self.lbl2.grid(row=1, column=2, **pad)
        tk.Scale(ctrl, from_=T2_MIN, to=T2_MAX, orient=tk.HORIZONTAL,
                 length=320, resolution=0.5, variable=self.theta2,
                 command=self._on_slider, showvalue=False
                 ).grid(row=1, column=1, **pad)

        # IK target entry
        ik_frame = tk.Frame(ctrl)
        ik_frame.grid(row=2, column=0, columnspan=4, sticky="w", **pad)
        tk.Label(ik_frame, text="IK target  x (m):", font=("Helvetica", 10)
                 ).pack(side=tk.LEFT)
        self.ik_x = tk.Entry(ik_frame, width=7, font=("Helvetica", 10))
        self.ik_x.insert(0, "0.20")
        self.ik_x.pack(side=tk.LEFT, padx=4)
        tk.Label(ik_frame, text="y (m):", font=("Helvetica", 10)
                 ).pack(side=tk.LEFT)
        self.ik_y = tk.Entry(ik_frame, width=7, font=("Helvetica", 10))
        self.ik_y.insert(0, "0.20")
        self.ik_y.pack(side=tk.LEFT, padx=4)
        tk.Button(ik_frame, text="Move", command=self._do_ik,
                  font=("Helvetica", 10), width=6
                  ).pack(side=tk.LEFT, padx=6)

        # Pen down / up toggle
        self.pen_btn = tk.Button(ik_frame, text="Pen: DOWN",
                                 command=self._toggle_pen,
                                 font=("Helvetica", 10), width=10, fg="#225522")
        self.pen_btn.pack(side=tk.LEFT, padx=6)

        # Clear / Quit
        btn_frame = tk.Frame(ctrl)
        btn_frame.grid(row=3, column=0, columnspan=4, sticky="w", **pad)
        tk.Button(btn_frame, text="Clear path", command=self._clear_path,
                  font=("Helvetica", 10)).pack(side=tk.LEFT, padx=4)
        tk.Button(btn_frame, text="Quit", command=self.root.destroy,
                  font=("Helvetica", 10)).pack(side=tk.LEFT, padx=4)

        # Status label
        tk.Label(ctrl, textvariable=self.status_var,
                 font=("Helvetica", 9), fg="#666", anchor="w"
                 ).grid(row=3, column=2, columnspan=2, sticky="w", **pad)

        # ── Initial draw ──────────────────────────────────────────────────────
        self._draw_grid()
        self._redraw()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_slider(self, _=None):
        self.lbl1.config(text=f"{self.theta1.get():+6.1f}°")
        self.lbl2.config(text=f"{self.theta2.get():+6.1f}°")
        self._redraw()

    def _do_ik(self):
        try:
            x = float(self.ik_x.get())
            y = float(self.ik_y.get())
        except ValueError:
            self.status_var.set("Invalid x/y values.")
            return
        result = inverse_kinematics(x, y)
        if result is None:
            self.status_var.set(f"({x:.3f}, {y:.3f}) m — unreachable.")
            return
        t1, t2 = result
        self.theta1.set(round(t1, 1))
        self.theta2.set(round(t2, 1))
        self._on_slider()

    def _toggle_pen(self):
        self.pen_down = not self.pen_down
        if self.pen_down:
            self.pen_btn.config(text="Pen: DOWN", fg="#225522")
            self.last_pen_pos = None   # start fresh stroke on next move
        else:
            self.pen_btn.config(text="Pen: UP  ", fg="#aa2222")
            self.last_pen_pos = None

    def _clear_path(self):
        self.t_path.clear()
        self.last_pen_pos = None

    # ── Grid ──────────────────────────────────────────────────────────────────

    def _draw_grid(self):
        t = self.t_grid
        t.clear()
        t.pencolor("#dddddd")
        t.width(1)

        # Grid lines every 50mm
        step_m = 0.050
        step_px = step_m * SCALE   # 40px

        # Range to cover: ±0.5m from origin
        n = 10
        for i in range(-n, n + 1):
            # vertical
            sx, _ = to_screen(i * step_m, -n * step_m)
            _, sy0 = to_screen(0, -n * step_m)
            _, sy1 = to_screen(0,  n * step_m)
            t.penup(); t.goto(sx, sy0); t.pendown(); t.goto(sx, sy1); t.penup()
            # horizontal
            _, sy = to_screen(0, i * step_m)
            sx0, _ = to_screen(-n * step_m, 0)
            sx1, _ = to_screen( n * step_m, 0)
            t.penup(); t.goto(sx0, sy); t.pendown(); t.goto(sx1, sy); t.penup()

        # Axes
        t.pencolor("#aaaaaa")
        t.width(2)
        ox, oy = to_screen(0, 0)
        ax1, _ = to_screen(-n * step_m, 0); ax2, _ = to_screen(n * step_m, 0)
        _, ay1  = to_screen(0, -n * step_m); _, ay2  = to_screen(0, n * step_m)
        t.penup(); t.goto(ax1, oy); t.pendown(); t.goto(ax2, oy); t.penup()
        t.penup(); t.goto(ox, ay1); t.pendown(); t.goto(ox, ay2); t.penup()

        # Origin dot
        draw_dot(t, ox, oy, "#888888", 5)

        # Scale label
        t.pencolor("#999999")
        t.goto(to_screen(0.05, -n * step_m + 0.01)[0],
               to_screen(0, -n * step_m + 0.01)[1])
        t.write("50mm", font=("Helvetica", 7, "normal"))

        self.screen.update()

    # ── Arm redraw ────────────────────────────────────────────────────────────

    def _redraw(self):
        t1 = self.theta1.get()
        t2 = self.theta2.get()
        fk = forward_kinematics(t1, t2)

        t = self.t_arm
        t.clear()

        ox, oy = to_screen(0, 0)

        if fk is None:
            self.status_var.set("Unreachable configuration.")
            self.screen.update()
            return

        ax, ay, pbx, pby, px, py = fk

        # Convert all points to screen coords
        s_o   = (ox, oy)
        s_a   = to_screen(ax,  ay)    # Pen A (arm4 tip)
        s_pb  = to_screen(pbx, pby)   # Pen B (arm2 far end)
        s_p   = to_screen(px,  py)    # Pen tip
        s_m1  = to_screen(L_short * math.cos(math.radians(t1)),
                          L_short * math.sin(math.radians(t1)))  # arm1 tip

        # arm4: origin → Pen A  (top servo, long arm)
        draw_line(t, *s_o, *s_a,  color="#cc4400", width=4)
        # arm1: origin → arm1 tip  (bottom servo, short arm)
        draw_line(t, *s_o, *s_m1, color="#0044cc", width=4)
        # arm2 (link): arm1 tip → Pen B
        draw_line(t, *s_m1, *s_pb, color="#4488ff", width=3)
        # arm3: Pen B → Pen A → Pen tip
        draw_line(t, *s_pb, *s_a,  color="#228822", width=3)
        draw_line(t, *s_a,  *s_p,  color="#228822", width=3)

        # Joint dots
        draw_dot(t, *s_o,  "#333333", 7)   # motor pivot
        draw_dot(t, *s_m1, "#0044cc", 5)   # arm1 tip
        draw_dot(t, *s_a,  "#cc4400", 6)   # Pen A
        draw_dot(t, *s_pb, "#4488ff", 5)   # Pen B
        draw_dot(t, *s_p,  "#cc0000", 7)   # pen tip

        # Labels
        lbl = self.t_label
        lbl.clear()
        lbl.pencolor("#444444")
        def label(sx, sy, text, dx=6, dy=6):
            lbl.penup(); lbl.goto(sx + dx, sy + dy)
            lbl.write(text, font=("Helvetica", 8, "normal"))

        label(*s_o,  "O (pivot)")
        label(*s_m1, "arm1 tip")
        label(*s_a,  "Pen A", dx=6, dy=-14)
        label(*s_pb, "Pen B", dx=6, dy=4)
        label(*s_p,  "PEN TIP", dx=6, dy=4)

        # Status
        self.status_var.set(
            f"θ1={t1:+.1f}°  θ2={t2:+.1f}°   "
            f"pen = ({px*1000:.1f}, {py*1000:.1f}) mm"
        )

        # Draw path if pen is down
        if self.pen_down:
            if self.last_pen_pos is not None:
                tp = self.t_path
                tp.penup()
                tp.goto(*self.last_pen_pos)
                tp.pendown()
                tp.goto(*s_p)
                tp.penup()
            self.last_pen_pos = s_p
        else:
            self.last_pen_pos = None

        self.screen.update()

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        self.root.mainloop()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    sim = PantographSim()
    sim.run()