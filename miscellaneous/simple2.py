#!/usr/bin/env python3
#2d with constraints
"""
pantograph_sim.py
-----------------
2D turtle simulation of the Pen-Pal pantograph plotter.

Servo ranges (physical):
  Bottom servo (θ1): [-180, -90] ∪ [90, 180]   (back half only)
  Top servo    (θ2): [0, 180]    slider left=180, right=0

Run:
    python3 pantograph_sim.py
"""

import math
import tkinter as tk
import turtle

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

T2_MIN, T2_MAX = 0, 180


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
    """Returns (ax,ay, pbx,pby, pen_x,pen_y) or None."""
    t1 = math.radians(theta1_deg)
    t2 = math.radians(theta2_deg)
    ax  = L_long  * math.cos(t2);  ay  = L_long  * math.sin(t2)
    m1x = L_short * math.cos(t1);  m1y = L_short * math.sin(t1)
    pb  = _ci(m1x, m1y, L_link, ax, ay, PEN_A_TO_B)
    if not pb:
        return None
    for pbx, pby in pb:
        dx = ax-pbx; dy = ay-pby; d = math.sqrt(dx**2+dy**2)
        if d < 1e-9:
            continue
        return ax, ay, pbx, pby, ax+PEN_TO_A*dx/d, ay+PEN_TO_A*dy/d
    return None


def _t1_valid(t1):
    return (90 <= t1 <= 180) or (-180 <= t1 <= -90)


def inverse_kinematics(x, y, prefer_positive=True):
    """Returns (theta1, theta2) in valid servo ranges, or None."""
    a_cands = _ci(x, y, PEN_TO_A, 0, 0, L_long)
    solutions = []
    for ax, ay in a_cands:
        t2 = math.degrees(math.atan2(ay, ax))
        if not (T2_MIN <= t2 <= T2_MAX):
            continue
        dxt = ax-x; dyt = ay-y; dt = math.sqrt(dxt**2+dyt**2)
        if dt < 1e-9:
            continue
        pbx = ax + PEN_A_TO_B*dxt/dt
        pby = ay + PEN_A_TO_B*dyt/dt
        m1_cands = _ci(pbx, pby, L_link, 0, 0, L_short)
        for m1x, m1y in m1_cands:
            t1 = math.degrees(math.atan2(m1y, m1x))
            if not _t1_valid(t1):
                continue
            fk = forward_kinematics(t1, t2)
            if fk is None:
                continue
            err = math.sqrt((fk[4]-x)**2 + (fk[5]-y)**2)
            if err < 0.001:
                solutions.append((t1, t2, err))
    if not solutions:
        return None
    solutions.sort(key=lambda s: (0 if (s[0] >= 0) == prefer_positive else 1, s[2]))
    return solutions[0][0], solutions[0][1]


# ── Coordinate conversion ─────────────────────────────────────────────────────

def to_screen(mx, my):
    return ORIGIN_X + mx * SCALE, ORIGIN_Y + my * SCALE


# ── Drawing primitives ────────────────────────────────────────────────────────

def draw_line(t, x0, y0, x1, y1, color, width=2):
    t.penup(); t.goto(x0, y0)
    t.pendown(); t.pencolor(color); t.width(width)
    t.goto(x1, y1); t.penup()


def draw_dot(t, x, y, color, size=6):
    t.penup(); t.goto(x, y - size//2)
    t.pendown(); t.fillcolor(color); t.begin_fill()
    t.circle(size//2); t.end_fill(); t.penup()


# ── Dual-range slider ─────────────────────────────────────────────────────────

class DualRangeSlider(tk.Canvas):
    """
    Custom slider spanning [-180,-90] ∪ [90,180].
    The forbidden middle zone is shown greyed out.
    """
    W, H, PAD, GAP = 320, 32, 10, 22

    def __init__(self, parent, variable, callback, **kwargs):
        super().__init__(parent, width=self.W, height=self.H,
                         bg=parent.cget("bg"), highlightthickness=0, **kwargs)
        self.var = variable
        self.callback = callback
        self._draw_track()
        self.bind("<Button-1>", self._on_click)
        self.bind("<B1-Motion>", self._on_drag)
        self.var.trace_add("write", lambda *_: self._redraw_thumb())

    # value ↔ pixel mapping
    def _half(self):
        return (self.W - 2*self.PAD - self.GAP) / 2

    def _val_to_x(self, val):
        P, half = self.PAD, self._half()
        if val <= -90:
            return P + (val + 180) / 90.0 * half
        else:
            return P + half + self.GAP + (val - 90) / 90.0 * half

    def _x_to_val(self, x):
        P, half, GAP = self.PAD, self._half(), self.GAP
        x = max(P, min(x, self.W - P))
        mid_lo = P + half
        mid_hi = P + half + GAP
        if x <= mid_lo:
            return round(-180 + (x - P) / half * 90, 1)
        elif x >= mid_hi:
            return round(90  + (x - mid_hi) / half * 90, 1)
        else:
            return -90.0 if x < (mid_lo + mid_hi) / 2 else 90.0

    def _draw_track(self):
        self.delete("track")
        P, H, half, GAP = self.PAD, self.H, self._half(), self.GAP
        cy = H // 2 - 4
        # Dead zone
        self.create_rectangle(P+half, cy-4, P+half+GAP, cy+4,
                               fill="#e0e0e0", outline="", tags="track")
        self.create_text((2*P + 2*half + GAP) / 2, cy,
                         text="✕", font=("Helvetica", 8), fill="#bbbbbb", tags="track")
        # Active tracks
        for x0, x1 in [(P, P+half), (P+half+GAP, P+2*half+GAP)]:
            self.create_rectangle(x0, cy-4, x1, cy+4,
                                   fill="#7ab0e8", outline="", tags="track")
        # Tick marks and labels
        for val, lbl in [(-180, "-180"), (-90, "-90"), (90, "90"), (180, "180")]:
            x = self._val_to_x(val)
            self.create_line(x, cy-5, x, cy+5, fill="#555", tags="track")
            self.create_text(x, cy+14, text=lbl,
                             font=("Helvetica", 7), fill="#555", tags="track")

    def _redraw_thumb(self):
        self.delete("thumb")
        x  = self._val_to_x(self.var.get())
        cy = self.H // 2 - 4
        r  = 9
        self.create_oval(x-r, cy-r, x+r, cy+r,
                         fill="#2255aa", outline="white", width=2, tags="thumb")

    def _on_click(self, e):
        self.var.set(self._x_to_val(e.x))
        self.callback()

    def _on_drag(self, e):
        self._on_click(e)


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
        for t in (self.t_grid, self.t_arm, self.t_path, self.t_label):
            t.hideturtle(); t.speed(0); t.penup()
        self.t_path.pencolor("#2255cc")
        self.t_path.width(1)

        # State
        self.theta1       = tk.DoubleVar(value=135.0)
        self.theta2       = tk.DoubleVar(value=90.0)
        self.pen_down     = True
        self.last_pen_pos = None
        self.status_var   = tk.StringVar(value="")

        # Controls
        ctrl = tk.Frame(self.root, padx=8, pady=4)
        ctrl.grid(row=1, column=0, columnspan=4, sticky="ew")
        pad = dict(padx=6, pady=2)

        # θ1 — dual-range slider
        tk.Label(ctrl, text="Bottom servo θ1:", font=("Helvetica", 10)
                 ).grid(row=0, column=0, sticky="w", **pad)
        self.lbl1 = tk.Label(ctrl, text="+135.0°", width=8,
                              font=("Helvetica", 10, "bold"), fg="#2255aa")
        self.lbl1.grid(row=0, column=2, **pad)
        self.slider1 = DualRangeSlider(ctrl, self.theta1, self._on_slider)
        self.slider1.grid(row=0, column=1, **pad)
        self.slider1._redraw_thumb()

        # θ2 — standard slider, from=180 (left) to=0 (right)
        tk.Label(ctrl, text="Top servo θ2:", font=("Helvetica", 10)
                 ).grid(row=1, column=0, sticky="w", **pad)
        self.lbl2 = tk.Label(ctrl, text=" 90.0°", width=8,
                              font=("Helvetica", 10, "bold"), fg="#cc4400")
        self.lbl2.grid(row=1, column=2, **pad)
        tk.Scale(ctrl, from_=T2_MAX, to=T2_MIN,
                 orient=tk.HORIZONTAL, length=320, resolution=0.5,
                 variable=self.theta2, command=self._on_slider,
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
        tk.Button(br, text="Quit", command=self.root.destroy,
                  font=("Helvetica", 10)).pack(side=tk.LEFT, padx=4)
        tk.Label(ctrl, textvariable=self.status_var,
                 font=("Helvetica", 9), fg="#555", anchor="w"
                 ).grid(row=3, column=2, columnspan=2, sticky="w", **pad)

        self._draw_grid()
        self._redraw()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_slider(self, _=None):
        self.lbl1.config(text=f"{self.theta1.get():+7.1f}°")
        self.lbl2.config(text=f"{self.theta2.get():6.1f}°")
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
        self.slider1._redraw_thumb()
        self._on_slider()

    def _toggle_pen(self):
        self.pen_down = not self.pen_down
        self.pen_btn.config(
            text="Pen: DOWN" if self.pen_down else "Pen: UP  ",
            fg="#225522"    if self.pen_down else "#aa2222")
        self.last_pen_pos = None

    def _clear_path(self):
        self.t_path.clear(); self.last_pen_pos = None

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
        self.root.mainloop()


if __name__ == "__main__":
    sim = PantographSim()
    sim.run()