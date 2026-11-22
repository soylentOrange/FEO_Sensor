import sys
S=13.0; out=[]; a=out.append
BAT=(0,0,32,20); XW,XL=17.5,21.0
XIAO=(32-XW,-0.5,32,20.5)
PB=7*2.54; GAP=0.3
BRD=(XIAO[0]-GAP-PB,0,XIAO[0]-GAP,PB)
def bp(c,r): return (BRD[0]+(c-0.5)*2.54, BRD[1]+(r-0.5)*2.54)
def xp(side,i): return ((XIAO[0]+XIAO[2])/2+(-7.62 if side=="L" else 7.62), XIAO[1]+10.5-3*2.54+i*2.54)
xc=bp(4.5,7)[0]; XH=(xc-6.2, BRD[3]-1.0, xc+6.2, BRD[3]+6.0)
OX,OY=150,150
X=lambda v: OX+(v-BRD[0])*S; Y=lambda v: OY+(v+1.8)*S
def rect(r,f,s,e=""): a(f'<rect x="{X(r[0])}" y="{Y(r[1])}" width="{(r[2]-r[0])*S}" height="{(r[3]-r[1])*S}" fill="{f}" stroke="{s}" stroke-width="1.5" {e}/>')
def cable(pts,col,dash=""):
    d=" ".join(("M" if i==0 else "L")+f" {X(p[0])} {Y(p[1])}" for i,p in enumerate(pts))
    a(f'<path d="{d}" fill="none" stroke="#fff" stroke-width="6" stroke-linejoin="round"/>')
    a(f'<path d="{d}" fill="none" stroke="{col}" stroke-width="3" stroke-linejoin="round" {dash}/>')
rect(BAT,"#cfd8dc","#607d8b",'stroke-dasharray="7,4" rx="4"')
a(f'<text x="{X(1)}" y="{Y(19.3)}" class="tiny" fill="#455a64">LiPo 32×20×4 quer (darunter)</text>')
# Kabel
p1=bp(3,2); p8=bp(6,2); g=bp(4,3); x1=bp(3,7); x4=bp(6,7)
d0=xp("L",0); d1=xp("L",1); gn=xp("R",1)
cable([p1,(p1[0],1.3),(13.6,1.3),(13.6,d0[1]),d0],"#1f77b4")
cable([p8,(13.0,p8[1]),(13.0,d1[1]),d1],"#d62728")
cable([g,(g[0],21.6),(32.9,21.6),(32.9,gn[1]),gn],"#222")
cable([x4,(x4[0]+1.1,x4[1]),(x4[0]+1.1,19.0),(24.0,19.0),(24.0,17.2)],"#e91e63",'stroke-dasharray="6,3"')
cable([(0.0,10.0),(-1.0,10.0),(-1.0,x1[1]-1.6),(x1[0],x1[1]-1.6),x1],"#c2185b")
cable([(0.0,11.4),(-1.8,11.4),(-1.8,x1[1]+0.6),(bp(4,7)[0],x1[1]+0.6),bp(4,7)],"#555")
a(f'<rect x="{X(-0.6)}" y="{Y(9.0)}" width="{1.4*S}" height="{3.4*S}" fill="#90a4ae" stroke="#455a64"/>')
a(f'<text x="{X(BRD[0])-8}" y="{Y(9.6)}" class="tiny" text-anchor="end">Akku-</text>')
a(f'<text x="{X(BRD[0])-8}" y="{Y(9.6)+15}" class="tiny" text-anchor="end">Abgang</text>')
# XIAO
rect(XIAO,"#1f4f8a","#0d2c52",'rx="6" fill-opacity="0.82"')
cx=(XIAO[0]+XIAO[2])/2
a(f'<rect x="{X(cx-4.5)}" y="{Y(-1.7)}" width="{9*S}" height="{3.6*S}" rx="6" fill="#bfc5cc" stroke="#555"/>')
a(f'<text x="{X(cx)}" y="{Y(0.6)}" class="tiny" text-anchor="middle">USB-C</text>')
a(f'<text x="{X(cx)}" y="{Y(9.8)}" class="lbl" fill="#fff" text-anchor="middle">XIAO</text>')
a(f'<text x="{X(cx)}" y="{Y(11.4)}" class="tiny" fill="#cfe0ff" text-anchor="middle">17,5 × 21</text>')
Ln=["D0/A0","D1","D2","D3","D4","D5","D6"]; Rn=["5V","GND","3V3","D10","D9","D8","D7"]
for i in range(7):
    for side,names in (("L",Ln),("R",Rn)):
        x,y=xp(side,i); n=names[i]; hot=n in("D0/A0","D1","GND")
        a(f'<rect x="{X(x)-7}" y="{Y(y)-7}" width="14" height="14" fill="{"#ffd34d" if hot else "#c9a227"}" stroke="#7a5c00"/>')
        a(f'<text x="{X(x)+(11 if side=="L" else -11)}" y="{Y(y)+4}" class="pin" fill="{"#fff" if hot else "#9fb8de"}" text-anchor="{"start" if side=="L" else "end"}" font-weight="{"bold" if hot else "normal"}">{n}</text>')
a(f'<rect x="{X(cx-3.8)}" y="{Y(15.4)}" width="{7.6*S}" height="{3.2*S}" rx="4" fill="none" stroke="#ff8a80" stroke-width="1.5" stroke-dasharray="4,3"/>')
a(f'<text x="{X(cx)}" y="{Y(16.8)}" class="tiny" fill="#ffcdd2" text-anchor="middle">BAT+ (Unterseite,</text>')
a(f'<text x="{X(cx)}" y="{Y(18.1)}" class="tiny" fill="#ffcdd2" text-anchor="middle">Lage prüfen)</text>')
# Platine
rect(BRD,"#e6d3a3","#8a7346",'rx="3" fill-opacity="0.82"')
for c in range(1,8):
    for r in range(1,8):
        x,y=bp(c,r); a(f'<circle cx="{X(x)}" cy="{Y(y)}" r="3" fill="#6b5a3a"/>')
(dx0,dy0),(dx1,dy1)=bp(3,2),bp(6,5); mx,my=(dx0+dx1)/2,(dy0+dy1)/2
a(f'<rect x="{X(mx-3.175)}" y="{Y(my-4.8)}" width="{6.35*S}" height="{9.6*S}" rx="3" fill="#262626"/>')
a(f'<text x="{X(mx)}" y="{Y(my)+5}" class="lbl" fill="#fff" text-anchor="middle">U1</text>')
for (c,r) in [(3,2),(3,3),(3,4),(3,5),(6,2),(6,3),(6,4),(6,5)]:
    x,y=bp(c,r); a(f'<rect x="{X(x)-6}" y="{Y(y)-3}" width="12" height="6" fill="#c0c0c0" stroke="#444"/>')
rect(XH,"#f2f2ea","#666",'rx="3"')
for k,c in enumerate((3,4,5,6),1):
    x,y=bp(c,7)
    a(f'<line x1="{X(x)}" y1="{Y(y)}" x2="{X(x)}" y2="{Y(XH[1]+1.4)}" stroke="#b8860b" stroke-width="3"/>')
    a(f'<text x="{X(x)}" y="{Y(XH[3]-1.0)}" class="tiny" text-anchor="middle">{k}</text>')
a(f'<text x="{X(xc)}" y="{Y(XH[3])+20}" class="sub" text-anchor="middle">▼ FSR-Stecker (XH flach)</text>')
# Masse
def dh(y,x0,x1,t):
    a(f'<line x1="{X(x0)}" y1="{Y(y)}" x2="{X(x1)}" y2="{Y(y)}" stroke="#444" marker-start="url(#ar)" marker-end="url(#ar)"/>')
    a(f'<text x="{(X(x0)+X(x1))/2}" y="{Y(y)-6}" class="dim" text-anchor="middle">{t}</text>')
def dv(x,y0,y1,t,left=False):
    a(f'<line x1="{X(x)}" y1="{Y(y0)}" x2="{X(x)}" y2="{Y(y1)}" stroke="#444" marker-start="url(#ar)" marker-end="url(#ar)"/>')
    a(f'<text x="{X(x)+(-6 if left else 6)}" y="{(Y(y0)+Y(y1))/2+4}" class="dim" text-anchor="{"end" if left else "start"}">{t}</text>')
W_=32-BRD[0]
dh(-3.6,BRD[0],32,f"≈{W_:.1f} mm")
dh(XH[3]+4.0,0,32,"Akku 32")
dh(XH[3]+4.0,BRD[0],0,f"{-BRD[0]:.1f}")
dv(34.6,-1.7,XH[3],f"≈{XH[3]+1.7:.0f} mm")
dv(BRD[0]-4.8,0,PB,"17,8",left=True)
dv(34.6+0.0,0,0,"")
a(f'<text x="{X(BRD[0])}" y="{Y(-1.5)}" class="tiny">Lochraster 7×7</text>')
# Legende
LX=X(34.6)+120
items=[("#1f77b4","Pin 1 → D0/A0","≈15 mm, direkt nach rechts"),("#d62728","Pin 8 → D1","≈5 mm, direkt nach rechts"),
("#222","GND (C4) → GND","unten um den XIAO herum"),("#c2185b","LiPo + → XH-1","Akku-Abgang links, kurz"),
("#e91e63","XH-4 → BAT+","unter dem XIAO"),("#555","LiPo − → XH-2 (GND)","BAT− = GND, kein Kabel zum Pad")]
a(f'<text x="{LX}" y="{OY-40}" class="h2">Kabel</text>')
for i,(c,t1,t2) in enumerate(items):
    yy=OY-16+i*42
    a(f'<rect x="{LX}" y="{yy}" width="26" height="9" rx="3" fill="{c}"/>')
    a(f'<text x="{LX+34}" y="{yy+9}" class="txt" font-weight="bold">{t1}</text>')
    a(f'<text x="{LX+34}" y="{yy+26}" class="sub">{t2}</text>')
ny=OY-16+6*42+16
notes=["Dicke wie bisher ≈11–12 mm","(Akku 4 + Schaum 1 + Platine 1,6 + OPV 4).",
"Platine steht 3,6 mm links über den Akku.","Alternativ XIAO 1,8 mm rechts überstehen",
"lassen, dann je 1,8 mm auf beiden Seiten."]
for i,t in enumerate(notes): a(f'<text x="{LX}" y="{ny+i*19}" class="txt">{t}</text>')
W=int(LX+300); H=int(Y(XH[3]+7))
svg=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">',
'<defs><marker id="ar" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10 z" fill="#444"/></marker></defs>',
'<style>.h1{font-size:22px;font-weight:bold}.h2{font-size:17px;font-weight:bold}.sub{font-size:12.5px;fill:#555}.lbl{font-size:15px;font-weight:bold}.pin{font-size:11px}.tiny{font-size:12px}.dim{font-size:13px;font-weight:bold;fill:#333}.txt{font-size:13.5px}</style>',
f'<rect width="{W}" height="{H}" fill="#fff"/>','<text x="40" y="36" class="h1">Anordnung quer: Akku 32×20, XIAO rechts, Platine links (Draufsicht)</text>',
f'<text x="40" y="58" class="sub">USB-C oben, XH unten · Akku unter XIAO und Platine · Kabel der Platine auf der Lötseite (zwischen Platine und Akku)</text>']
open(sys.argv[1],"w").write("\n".join(svg+out+['</svg>'])); print(W,H, BRD[0], XH[3])
