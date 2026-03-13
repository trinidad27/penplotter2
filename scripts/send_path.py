import json, math, subprocess, sys, time

L_short    = 0.050
# L_long     = 0.220
L_long = 0.205
PEN_TO_A   = 0.240
PEN_A_TO_B = 0.050

WORLD = "penplotter_world"
TOPIC = f"/gazebo/{WORLD}/penplotter/joint_cmd"
STEP_DELAY = 0.05
LIFT_DELAY = 0.2

def _ci(c1x,c1y,r1,c2x,c2y,r2):
    dx=c2x-c1x; dy=c2y-c1y; d=math.sqrt(dx**2+dy**2)
    if d<1e-9 or d>r1+r2+1e-9 or d<abs(r1-r2)-1e-9: return []
    a=(r1**2-r2**2+d**2)/(2*d); h=math.sqrt(max(0.0,r1**2-a**2))
    mx=c1x+a*dx/d; my=c1y+a*dy/d
    if h<1e-9: return [(mx,my)]
    return [(mx+h*(-dy/d),my+h*(dx/d)),(mx-h*(-dy/d),my-h*(dx/d))]

def forward_kinematics(t1d,t2d):
    t1=math.radians(t1d); t2=math.radians(t2d)
    ax=L_long*math.cos(t2); ay=L_long*math.sin(t2)
    m1x=L_short*math.cos(t1); m1y=L_short*math.sin(t1)
    pbx=m1x+ax; pby=m1y+ay
    dx=ax-pbx; dy=ay-pby; d=math.sqrt(dx**2+dy**2)
    if d<1e-9: return None
    return ax,ay,pbx,pby,ax+PEN_TO_A*dx/d,ay+PEN_TO_A*dy/d

def inverse_kinematics(x,y,prev_t1=None,prev_t2=None):
    solutions=[]
    for ax,ay in _ci(x,y,PEN_TO_A,0,0,L_long):
        t2=math.degrees(math.atan2(ay,ax))
        dxt=ax-x; dyt=ay-y; dt=math.sqrt(dxt**2+dyt**2)
        if dt<1e-9: continue
        pbx=ax+PEN_A_TO_B*dxt/dt; pby=ay+PEN_A_TO_B*dyt/dt
        m1x=pbx-ax; m1y=pby-ay
        if abs(math.sqrt(m1x**2+m1y**2)-L_short)>1e-6: continue
        t1=math.degrees(math.atan2(m1y,m1x))
        fk=forward_kinematics(t1,t2)
        if fk and math.sqrt((fk[4]-x)**2+(fk[5]-y)**2)<0.001:
            solutions.append((t1,t2))
    if not solutions: return None
    if prev_t1 is not None:
        solutions.sort(key=lambda s:(s[0]-prev_t1)**2+(s[1]-prev_t2)**2)
    return solutions[0]

def send_angles(t1,t2):
    msg=f"{t1:.4f},{t2:.4f}"
    cmd=["gz","topic","--publish",TOPIC,"--msg",f'data: "{msg}"']
    r=subprocess.run(cmd,capture_output=True,text=True)
    if r.returncode!=0: print(f"[WARN] {r.stderr.strip()}")

def main():
    if len(sys.argv)<2: print("Usage: python3 send_path.py segments.json"); sys.exit(1)
    with open(sys.argv[1]) as f: segments=json.load(f)
    print(f"Loaded {len(segments)} segments. Publishing to {TOPIC}")
    print("Starting in 2s — make sure Gazebo is running...")
    time.sleep(2)
    prev_t1,prev_t2=135.0,90.0
    for i,seg in enumerate(segments):
        x0,y0=seg[0][0]/1000,seg[0][1]/1000
        x1,y1=seg[1][0]/1000,seg[1][1]/1000
        print(f"[{i+1}/{len(segments)}] ({seg[0][0]:.1f},{seg[0][1]:.1f})→({seg[1][0]:.1f},{seg[1][1]:.1f})mm")
        r=inverse_kinematics(x0,y0,prev_t1,prev_t2)
        if r and (abs(r[0]-prev_t1)>90 or abs(r[1]-prev_t2)>90):
                print(f"  [WARN] big angle jump! prev=({prev_t1:.1f},{prev_t2:.1f}) new=({r[0]:.1f},{r[1]:.1f})")
        if r is None: print("  [SKIP] unreachable"); continue
        prev_t1,prev_t2=r; send_angles(*r); time.sleep(LIFT_DELAY)
        dx=x1-x0; dy=y1-y0; dist=math.sqrt(dx**2+dy**2)
        for k in range(1,max(1,int(dist/0.005))+1):
            f=k/max(1,int(dist/0.005))
            r=inverse_kinematics(x0+f*dx,y0+f*dy,prev_t1,prev_t2)
            if r is None: continue
            prev_t1,prev_t2=r
            fk=forward_kinematics(*r)
            if fk: print(f"  pen=({fk[4]*1000:.1f},{fk[5]*1000:.1f})mm  θ1={r[0]:+.1f}° θ2={r[1]:.1f}°")
            send_angles(*r); time.sleep(STEP_DELAY)
    print("Done.")

if __name__=="__main__": main()