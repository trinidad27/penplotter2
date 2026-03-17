#!/usr/bin/env python3
"""
penplottersliders_sim.py
------------------------
Gazebo controller for the Pen-Pal pantograph plotter.

Sets ALL FOUR joints on every move to prevent loop_closure_link2_link3 popping:

  Revolute_9   ← phi1  (slider, bottom servo)
  Revolute_10  ← phi2  (slider, top servo)
  Revolute_15  ← phi15 (computed from FK, axis=+Y)
  Revolute_18  ← phi18_gz (computed + negated for -Y axis)

Usage:
    # Terminal 1 — Gazebo:
    gazebo --verbose ~/Documents/penplotter/worlds/penplotter.world

    # Terminal 2 — spawn robot:
    gz model -w penplotter_world -m penplotter \\
        -f ~/Documents/penplotter/urdf/penplotter.sdf

    # Terminal 3 — this script:
    python3 ~/Documents/penplotter/penplottersliders_sim.py
"""

import json, subprocess, math, time, threading
import tkinter as tk
import tkinter.filedialog as filedialog
from tkinter import ttk, messagebox

# ── Dimensions (from SDF) ─────────────────────────────────────────────────────
L_short    = 0.175
L_long     = 0.050
L_link     = 0.200
PEN_TO_A   = 0.240
PEN_A_TO_B = 0.050

WORLD_NAME  = "penplotter_world"
ROBOT_NAME  = "penplotter"
JOINT_PHI1  = "Revolute_9"
JOINT_PHI2  = "Revolute_10"
JOINT_PHI15 = "Revolute_15"
JOINT_PHI18 = "Revolute_18"

trajectory = []
recording  = False
pen_down   = True    # True = pen drawing; False = pen lifted (travel move)


# ── Kinematics ────────────────────────────────────────────────────────────────

def _ci(c1x, c1z, r1, c2x, c2z, r2):
    dx=c2x-c1x; dz=c2z-c1z; d=math.sqrt(dx**2+dz**2)
    if d<1e-9 or d>r1+r2+1e-9 or d<abs(r1-r2)-1e-9: return []
    a=(r1**2-r2**2+d**2)/(2*d); h=math.sqrt(max(0.,r1**2-a**2))
    mx=c1x+a*dx/d; mz=c1z+a*dz/d
    if h<1e-9: return [(mx,mz)]
    return [(mx+h*(-dz/d),mz+h*(dx/d)),(mx-h*(-dz/d),mz-h*(dx/d))]


def forward_kinematics(phi1_deg, phi2_deg):
    """
    Returns dict(pen_b, r15, pen_a, pen, phi15, phi18_gz) or None.
    Setting Revolute_15=phi15 and Revolute_18=phi18_gz eliminates
    loop_closure_link2_link3 pop (guaranteed 0 mm closure error).
    """
    phi1 = math.radians(phi1_deg)
    phi2 = math.radians(phi2_deg)

    pbx  = L_short * math.sin(phi1)     # Pen B (link3 tip, initial dir -Z)
    pbz  = -L_short * math.cos(phi1)
    r15x = L_long  * math.cos(phi2)     # cam tip / R15 (initial dir +X)
    r15z = L_long  * math.sin(phi2)

    for ax, az in _ci(r15x, r15z, L_link, pbx, pbz, PEN_A_TO_B):
        # phi15: link1 world angle minus phi2
        alpha = math.atan2(ax - r15x, -(az - r15z))  # phi2 + phi15
        phi15 = math.degrees(alpha) - phi2_deg

        # pen tip
        dx=ax-pbx; dz=az-pbz; d=math.sqrt(dx**2+dz**2)
        if d < 1e-9: continue
        pen_x = ax + PEN_TO_A*dx/d
        pen_z = az + PEN_TO_A*dz/d

        # phi18_gz: pen bar angle rel. to link1, negated for axis=-Y
        penbar_angle = math.atan2(dx, -dz)
        phi18_gz     = -(math.degrees(penbar_angle) - math.degrees(alpha))

        return {'pen_b':(pbx,pbz), 'r15':(r15x,r15z), 'pen_a':(ax,az),
                'pen':(round(pen_x,6),round(pen_z,6)),
                'phi15':round(phi15,4), 'phi18_gz':round(phi18_gz,4)}
    return None


def inverse_kinematics(x, z, flip=False, prev_phi1=None, prev_phi2=None):
    """Returns (phi1_deg, phi2_deg) or None.

    If prev_phi1/prev_phi2 are given, picks the solution closest to the
    previous pose to prevent joint-angle flipping during continuous motion
    (e.g. JSON segment playback).  The flip flag selects the alternate
    configuration when two geometrically distinct solutions exist.
    """
    R = PEN_TO_A + PEN_A_TO_B
    solutions = []
    for pbx, pbz in _ci(0.,0.,L_short, x,z,R):
        phi1 = math.degrees(math.atan2(pbx, -pbz))
        dx=x-pbx; dz=z-pbz; d=math.sqrt(dx**2+dz**2)
        if d<1e-9: continue
        ax=pbx+PEN_A_TO_B*dx/d; az=pbz+PEN_A_TO_B*dz/d
        for r15x,r15z in _ci(ax,az,L_link,0.,0.,L_long):
            phi2 = math.degrees(math.atan2(r15z, r15x))
            fk = forward_kinematics(phi1, phi2)
            if fk is None: continue
            err = math.sqrt((fk['pen'][0]-x)**2+(fk['pen'][1]-z)**2)
            if err < 0.001:
                solutions.append((phi1, phi2, err))
    if not solutions: return None
    # Prefer solution closest to previous pose when available (continuity)
    if prev_phi1 is not None and prev_phi2 is not None:
        solutions.sort(key=lambda s: (s[0]-prev_phi1)**2 + (s[1]-prev_phi2)**2)
    else:
        solutions.sort(key=lambda s: s[2])
    unique=[solutions[0]]
    for s in solutions[1:]:
        if all(abs(s[0]-u[0])>0.5 or abs(s[1]-u[1])>0.5 for u in unique):
            unique.append(s)
    idx = 1 if (flip and len(unique)>1) else 0
    return unique[idx][0], unique[idx][1]


# ── Gazebo helpers ────────────────────────────────────────────────────────────

def _gz(args):
    subprocess.run(["gz"]+args, capture_output=True, text=True)

def _set(name, deg):
    _gz(["joint","-w",WORLD_NAME,"-m",ROBOT_NAME,
         "--joint-name",name,"--pos-t",str(math.radians(deg))])

def apply_joints(phi1, phi2):
    """Compute FK and fire all four joints in parallel. Returns FK dict or None."""
    fk = forward_kinematics(phi1, phi2)
    if fk:
        threads = [
            threading.Thread(target=_set, args=(JOINT_PHI1,  phi1),            daemon=True),
            threading.Thread(target=_set, args=(JOINT_PHI2,  phi2),            daemon=True),
            threading.Thread(target=_set, args=(JOINT_PHI15, fk['phi15']),     daemon=True),
            threading.Thread(target=_set, args=(JOINT_PHI18, fk['phi18_gz']),  daemon=True),
        ]
        for t in threads: t.start()
        for t in threads: t.join()
    return fk


# ── SVG export ────────────────────────────────────────────────────────────────

def save_svg(path, filename="drawing.svg", scale=1000):
    if not path: return None
    mr = L_long+L_link+PEN_TO_A; sz=int(mr*2*scale*1.1); c=sz/2
    pts=" ".join(f"{c+x*scale:.2f},{c-z*scale:.2f}" for x,z in path)
    with open(filename,"w") as f:
        f.write(f'<?xml version="1.0"?>\n<svg xmlns="http://www.w3.org/2000/svg" '
                f'width="{sz}" height="{sz}" viewBox="0 0 {sz} {sz}">\n'
                f'<rect width="100%" height="100%" fill="white"/>\n'
                f'<polyline points="{pts}" fill="none" stroke="black" '
                f'stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>\n'
                f'<circle cx="{c+path[0][0]*scale:.2f}" cy="{c-path[0][1]*scale:.2f}" '
                f'r="4" fill="green"/>\n'
                f'<circle cx="{c+path[-1][0]*scale:.2f}" cy="{c-path[-1][1]*scale:.2f}" '
                f'r="4" fill="red"/>\n</svg>')
    print(f"[INFO] SVG → {filename}  ({len(path)} pts)")
    return filename


# ── GUI ───────────────────────────────────────────────────────────────────────

def build_gui():
    global recording
    root = tk.Tk()
    root.title("Pen Plotter — Gazebo Controller")
    root.resizable(False, False)
    P = {"padx":16,"pady":6}

    # ── 2D preview canvas ─────────────────────────────────────────────────────
    _CW, _CH = 400, 400
    _WS      = 0.55                       # workspace half-span metres
    _SCALE   = _CW / (2 * _WS)

    def _to_cv(px, pz):
        return _CW/2 + px*_SCALE, _CH/2 - pz*_SCALE

    pf = tk.LabelFrame(root, text="Live pen path preview  (model XZ plane)",
                       font=("Helvetica", 10), padx=4, pady=4)
    pf.grid(row=0, column=0, columnspan=4, padx=16, pady=(12, 4))
    cv = tk.Canvas(pf, width=_CW, height=_CH, bg="#1a1a2e",
                   highlightthickness=1, highlightbackground="#444")
    cv.pack()

    # Grid lines every 100 mm
    ox, oy = _to_cv(0, 0)
    for mm in range(-500, 501, 100):
        m = mm / 1000
        x0, _ = _to_cv(m, -_WS); x1, _ = _to_cv(m, _WS)
        _, y0 = _to_cv(-_WS, m); _, y1 = _to_cv(_WS, m)
        col = "#2a2a4a" if mm % 200 else "#3a3a6a"
        cv.create_line(x0, 0, x0, _CH, fill=col)
        cv.create_line(0, y0, _CW, y0, fill=col)
        if mm and mm % 200 == 0:
            cx2, _ = _to_cv(m, 0); _, cy2 = _to_cv(0, m)
            cv.create_text(cx2, oy+10, text=f"{mm}", fill="#555577",
                           font=("Helvetica", 7))
            cv.create_text(ox+14, cy2, text=f"{mm}", fill="#555577",
                           font=("Helvetica", 7))
    cv.create_line(0, oy, _CW, oy, fill="#555588", width=1)
    cv.create_line(ox, 0, ox, _CH, fill="#555588", width=1)

    _prev_cv = [None]   # last canvas point while pen down

    def _cv_update(px, pz, down):
        cx, cy = _to_cv(px, pz)
        if down and _prev_cv[0]:
            cv.create_line(_prev_cv[0][0], _prev_cv[0][1], cx, cy,
                           fill="#3399ff", width=1.5, capstyle="round", tags="path")
        _prev_cv[0] = (cx, cy) if down else None

    def _cv_clear():
        cv.delete("path"); _prev_cv[0] = None


    # Header
    tk.Label(root,text="Pen Plotter — Gazebo Controller",
             font=("Helvetica",15,"bold")).grid(row=1,column=0,columnspan=4,pady=(16,2))
    tk.Label(root,
             text="All 4 joints (R9, R10, R15, R18) are set simultaneously — no loop-closure pop.",
             font=("Helvetica",9),fg="grey").grid(row=2,column=0,columnspan=4)
    tk.Label(root,
             text=f"link3={L_short*1e3:.0f}mm  cam={L_long*1e3:.0f}mm  "
                  f"link1={L_link*1e3:.0f}mm  pen→A={PEN_TO_A*1e3:.0f}mm  A→B={PEN_A_TO_B*1e3:.0f}mm",
             font=("Helvetica",9),fg="#888").grid(row=2,column=0,columnspan=4,pady=(0,6))
    ttk.Separator(root,orient="horizontal").grid(
        row=4,column=0,columnspan=4,sticky="ew",padx=16)

    # ── Sliders ───────────────────────────────────────────────────────────────
    tk.Label(root,text="Manual joint control",font=("Helvetica",11,"bold"),fg="#444").grid(
        row=5,column=0,columnspan=4,sticky="w",padx=16,pady=(8,2))

    phi1_var = tk.DoubleVar(value=0.0)
    phi2_var = tk.DoubleVar(value=45.0)

    def _slider_row(row, label, color, from_, to_, var):
        tk.Label(root,text=label,font=("Helvetica",11),width=30,anchor="w"
                 ).grid(row=row,column=0,sticky="w",**P)
        lbl = tk.Label(root,text=f"{var.get():+.1f}°",width=8,
                       font=("Helvetica",11,"bold"),fg=color)
        lbl.grid(row=row,column=2,**P)
        rad = tk.Label(root,text=f"({math.radians(var.get()):.3f} rad)",width=13,
                       font=("Helvetica",9),fg="grey")
        rad.grid(row=row,column=3,**P)
        tk.Scale(root,from_=from_,to=to_,orient=tk.HORIZONTAL,length=420,
                 resolution=0.5,variable=var,command=lambda v: _refresh(),
                 showvalue=False,
                 troughcolor=("#b0c8f0" if color=="#2255aa" else "#f0c0a0")
                 ).grid(row=row,column=1,**P)
        return lbl, rad

    phi1_lbl, phi1_rad = _slider_row(5,"φ1  Revolute_9  (bottom servo)",
                                     "#2255aa",-180,180, phi1_var)
    phi2_lbl, phi2_rad = _slider_row(6,"φ2  Revolute_10  (top servo)",
                                     "#cc4400", 180,-180, phi2_var)

    ttk.Separator(root,orient="horizontal").grid(
        row=8,column=0,columnspan=4,sticky="ew",padx=16,pady=(4,0))

    # ── Computed joint readout ────────────────────────────────────────────────
    cr = tk.Frame(root); cr.grid(row=8,column=0,columnspan=4,sticky="w",padx=16,pady=(6,2))
    tk.Label(cr,text="Computed:  φ15 (R15) =",font=("Helvetica",10),fg="grey").pack(side=tk.LEFT)
    phi15_lbl=tk.Label(cr,text="—",width=10,font=("Helvetica",10,"bold"),fg="#228833")
    phi15_lbl.pack(side=tk.LEFT)
    tk.Label(cr,text="   φ18_gz (R18) =",font=("Helvetica",10),fg="grey").pack(side=tk.LEFT,padx=(8,0))
    phi18_lbl=tk.Label(cr,text="—",width=10,font=("Helvetica",10,"bold"),fg="#228833")
    phi18_lbl.pack(side=tk.LEFT)
    tk.Label(cr,text="   (set automatically — prevents loop-closure pop)",
             font=("Helvetica",9),fg="#aaa").pack(side=tk.LEFT,padx=(8,0))

    # ── FK readout ────────────────────────────────────────────────────────────
    fr=tk.Frame(root); fr.grid(row=9,column=0,columnspan=4,sticky="w",padx=16,pady=(2,4))
    tk.Label(fr,text="Pen tip (FK):",font=("Helvetica",10),fg="grey").pack(side=tk.LEFT,padx=(0,8))
    fk_pen=tk.Label(fr,text="x=?  z=?",font=("Helvetica",11,"bold"),fg="#225522")
    fk_pen.pack(side=tk.LEFT)
    tk.Label(fr,text="   Pen A:",font=("Helvetica",10),fg="grey").pack(side=tk.LEFT,padx=(12,2))
    fk_a=tk.Label(fr,text="(?,?)",font=("Helvetica",9),fg="#888"); fk_a.pack(side=tk.LEFT)
    tk.Label(fr,text="  Pen B:",font=("Helvetica",10),fg="grey").pack(side=tk.LEFT,padx=(8,2))
    fk_b=tk.Label(fr,text="(?,?)",font=("Helvetica",9),fg="#888"); fk_b.pack(side=tk.LEFT)

    def _refresh(val=None):
        p1,p2=phi1_var.get(),phi2_var.get()
        phi1_lbl.config(text=f"{p1:+.1f}°"); phi1_rad.config(text=f"({math.radians(p1):.3f} rad)")
        phi2_lbl.config(text=f"{p2:+.1f}°"); phi2_rad.config(text=f"({math.radians(p2):.3f} rad)")
        fk = apply_joints(p1, p2)
        if fk is None:
            fk_pen.config(text="unreachable",fg="#aa2222")
            phi15_lbl.config(text="—"); phi18_lbl.config(text="—")
            fk_a.config(text="—"); fk_b.config(text="—")
        else:
            ax,az=fk['pen_a']; pbx,pbz=fk['pen_b']; px,pz=fk['pen']
            fk_pen.config(text=f"x={px:+.4f} m   z={pz:+.4f} m",fg="#225522")
            phi15_lbl.config(text=f"{fk['phi15']:+.2f}°")
            phi18_lbl.config(text=f"{fk['phi18_gz']:+.2f}°")
            fk_a.config(text=f"({ax:.4f},{az:.4f})")
            fk_b.config(text=f"({pbx:.4f},{pbz:.4f})")
            if recording and pen_down: trajectory.append((px,pz))
            _cv_update(px, pz, pen_down)

    ttk.Separator(root,orient="horizontal").grid(
        row=11,column=0,columnspan=4,sticky="ew",padx=16,pady=(4,0))

    # ── Inverse kinematics ────────────────────────────────────────────────────
    tk.Label(root,text="Move to (x, z)",font=("Helvetica",11,"bold"),fg="#444").grid(
        row=12,column=0,columnspan=4,sticky="w",padx=16,pady=(8,2))
    ikf=tk.Frame(root); ikf.grid(row=12,column=0,columnspan=4,sticky="w",padx=16,pady=(0,8))
    tk.Label(ikf,text="x (m):",font=("Helvetica",11)).pack(side=tk.LEFT)
    ik_x=tk.Entry(ikf,width=8,font=("Helvetica",11)); ik_x.insert(0,"0.25")
    ik_x.pack(side=tk.LEFT,padx=(4,12))
    tk.Label(ikf,text="z (m):",font=("Helvetica",11)).pack(side=tk.LEFT)
    ik_z=tk.Entry(ikf,width=8,font=("Helvetica",11)); ik_z.insert(0,"-0.10")
    ik_z.pack(side=tk.LEFT,padx=(4,12))
    flip_v=tk.BooleanVar()
    tk.Checkbutton(ikf,text="Flip",variable=flip_v,font=("Helvetica",10)
                   ).pack(side=tk.LEFT,padx=(0,12))
    ik_st=tk.Label(ikf,text="",font=("Helvetica",10),fg="grey"); ik_st.pack(side=tk.LEFT)

    def do_ik():
        try: x,z=float(ik_x.get()),float(ik_z.get())
        except ValueError: ik_st.config(text="Invalid",fg="red"); return
        res=inverse_kinematics(x,z,flip=flip_v.get())
        if res is None:
            ik_st.config(text=f"({x:.3f},{z:.3f}) unreachable",fg="red"); return
        p1,p2=res
        phi1_var.set(round(p1,1)); phi2_var.set(round(p2,1))
        ik_st.config(text=f"→ φ1={p1:+.1f}°  φ2={p2:+.1f}°",fg="#225522")
        _refresh()
    tk.Button(ikf,text="Move",command=do_ik,font=("Helvetica",11),width=8
              ).pack(side=tk.LEFT,padx=(0,8))

    ttk.Separator(root,orient="horizontal").grid(
        row=14,column=0,columnspan=4,sticky="ew",padx=16,pady=(4,0))

    # ── Buttons ───────────────────────────────────────────────────────────────
    bf=tk.Frame(root); bf.grid(row=14,column=0,columnspan=4,pady=(8,4))

    def reset():
        phi1_var.set(0.); phi2_var.set(45.); _refresh()
    tk.Button(bf,text="⟳  Reset",command=reset,font=("Helvetica",11),width=12
              ).pack(side=tk.LEFT,padx=6)

    rec_t=tk.StringVar(value="⏺  Record")
    def toggle_rec():
        global recording
        recording=not recording
        if recording: trajectory.clear(); rec_t.set("⏹  Stop")
        else: rec_t.set("⏺  Record")
    tk.Button(bf,textvariable=rec_t,command=toggle_rec,font=("Helvetica",11),width=12
              ).pack(side=tk.LEFT,padx=6)

    def do_svg():
        if not trajectory: messagebox.showinfo("SVG","Nothing recorded."); return
        f=save_svg(trajectory)
        if f: messagebox.showinfo("SVG",f"Saved {len(trajectory)} pts → {f}")
    tk.Button(bf,text="💾  Export SVG",command=do_svg,font=("Helvetica",11),width=14
              ).pack(side=tk.LEFT,padx=6)

    def do_plot():
        if not trajectory: messagebox.showinfo("Path","Nothing recorded."); return
        try:
            import matplotlib.pyplot as plt
            xs,zs=zip(*trajectory)
            fig,ax=plt.subplots(figsize=(6,6))
            ax.plot(xs,zs,"b.-",lw=1,ms=3)
            ax.plot(xs[0],zs[0],"go",ms=8,label="start")
            ax.plot(xs[-1],zs[-1],"rs",ms=8,label="end")
            ax.set_xlabel("x (m)"); ax.set_ylabel("z (m)")
            ax.set_title(f"Path  ({len(trajectory)} pts)")
            ax.set_aspect("equal"); ax.legend(); ax.grid(True)
            plt.tight_layout(); plt.show()
        except ImportError:
            messagebox.showerror("Error","pip install matplotlib")
    tk.Button(bf,text="📈  Show path",command=do_plot,font=("Helvetica",11),width=14
              ).pack(side=tk.LEFT,padx=6)

    # ── Pen / draw controls ───────────────────────────────────────────────────
    bf2=tk.Frame(root); bf2.grid(row=15,column=0,columnspan=4,pady=(0,16))

    pen_btn_text = tk.StringVar(value="Pen: DOWN")
    def toggle_pen():
        global pen_down
        pen_down = not pen_down
        pen_btn_text.set("Pen: DOWN" if pen_down else "Pen: UP  ")
        pen_btn.config(fg="#225522" if pen_down else "#aa2222")
    pen_btn = tk.Button(bf2,textvariable=pen_btn_text,command=toggle_pen,
                        font=("Helvetica",11),width=12,fg="#225522")
    pen_btn.pack(side=tk.LEFT,padx=6)

    def clear_traj():
        trajectory.clear()
        _cv_clear()
        ik_st.config(text="Path cleared.",fg="grey")
    tk.Button(bf2,text="🗑  Clear path",command=clear_traj,font=("Helvetica",11),width=14
              ).pack(side=tk.LEFT,padx=6)

    def load_and_draw():
        """
        Load a JSON segment file and drive the Gazebo plotter through it.

        Expected format — list of segments, each a pair of [x, z] points
        in millimetres:
            [[[x0, z0], [x1, z1]], [[x1, z1], [x2, z2]], ...]

        Travel moves (pen up) are made to each segment start; the segment
        itself is drawn with the pen down, interpolated every 5 mm.
        IK uses prev_phi1/prev_phi2 continuity to prevent joint flipping.
        """
        global pen_down
        path = filedialog.askopenfilename(
            title="Open segment file",
            filetypes=[("JSON files","*.json"),("All files","*.*")])
        if not path: return

        try:
            with open(path) as f:
                segments = json.load(f)
        except Exception as e:
            ik_st.config(text=f"Error reading file: {e}",fg="red"); return

        try:
            for seg in segments:
                (seg[0][0], seg[0][1]), (seg[1][0], seg[1][1])
        except Exception:
            ik_st.config(text="Bad format — expected [[[x,z],[x,z]], ...]",fg="red")
            return

        STEP_MM = 5.0

        def _move_to(x_m, z_m, draw):
            """Solve IK (with continuity), set joints, update pen state."""
            global pen_down
            p1 = phi1_var.get(); p2 = phi2_var.get()
            res = inverse_kinematics(x_m, z_m, prev_phi1=p1, prev_phi2=p2)
            if res is None:
                return False
            pen_down = draw
            pen_btn_text.set("Pen: DOWN" if draw else "Pen: UP  ")
            pen_btn.config(fg="#225522" if draw else "#aa2222")
            if not draw: _prev_cv[0] = None
            phi1_var.set(round(res[0],1)); phi2_var.set(round(res[1],1))
            _refresh()
            time.sleep(0.3)
            root.update()
            return True

        trajectory.clear()
        n_segs = len(segments)
        for i, seg in enumerate(segments):
            x0_mm,z0_mm = seg[0]; x1_mm,z1_mm = seg[1]
            x0_m,z0_m = x0_mm/1000, z0_mm/1000
            x1_m,z1_m = x1_mm/1000, z1_mm/1000
            ik_st.config(
                text=f"Seg {i+1}/{n_segs}  "
                     f"({x0_mm:.0f},{z0_mm:.0f})→({x1_mm:.0f},{z1_mm:.0f}) mm",
                fg="grey")
            root.update()
            _move_to(x0_m, z0_m, draw=False)          # lift, travel to start
            dx=x1_m-x0_m; dz=z1_m-z0_m
            dist=math.sqrt(dx**2+dz**2)
            n_steps=max(1,int(dist/(STEP_MM/1000)))
            for k in range(n_steps+1):
                t=k/n_steps
                _move_to(x0_m+t*dx, z0_m+t*dz, draw=True)

        ik_st.config(text=f"Done — {n_segs} segments drawn.",fg="#225522")

    tk.Button(bf2,text="📂  Load & Draw",command=load_and_draw,
              font=("Helvetica",11),width=14,fg="#225577"
              ).pack(side=tk.LEFT,padx=6)

    _refresh()
    root.mainloop()


if __name__ == "__main__":
    build_gui()