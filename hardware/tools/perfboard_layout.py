#!/usr/bin/env python3
"""Compact perfboard layout: MCP602 soldered directly, XIAO connected by wires,
JST-XH 4-pin (outer pins: LiPo+ switch, inner pins: FSR). Checks the netlist, draws the SVG."""
import itertools, sys

NX, NY = 7, 7
ROWS = "ABCDEFG"
def nm(p): return f"{ROWS[p[1]-1]}{p[0]}"

# Channels: amplifier B (5-7) = FSR stage, A (1-3) = Sallen-Key
U1 = {1: (3, 2), 2: (3, 3), 3: (3, 4), 4: (3, 5), 5: (6, 5), 6: (6, 4), 7: (6, 3), 8: (6, 2)}
pinnet = {1: "OUTSK", 2: "OUTSK", 3: "INSK+", 4: "GND", 5: "VREF", 6: "INF-", 7: "OUTF", 8: "VDD"}
XH = {1: (3, 7), 2: (4, 7), 3: (5, 7), 4: (6, 7)}
xhnet = {1: "BATIN", 2: "GND", 3: "INF-", 4: "BATOUT"}

SMD = [  # name, value, padA, padB, netA, netB
    ("R1", "100k", (1, 1), (1, 2), "OUTF", "X"),
    ("R2", "100k", (1, 3), (1, 4), "X", "INSK+"),
    ("C1", "100n", (2, 2), (2, 3), "OUTSK", "X"),
    ("C2", "47n", (2, 4), (2, 5), "INSK+", "GND"),
    ("C3", "100n", (4, 2), (5, 2), "GND", "VDD"),
    ("R3", "330k", (5, 3), (5, 4), "VDD", "VREF"),
    ("R4", "10k", (4, 5), (5, 5), "GND", "VREF"),
    ("Rf", "12k", (7, 3), (7, 4), "OUTF", "INF-"),
]
BRIDGES = [
    ((1, 1), (2, 1)), ((1, 2), (1, 3)), ((1, 3), (2, 3)), ((2, 2), (3, 2)), ((3, 2), (3, 3)),
    ((2, 4), (3, 4)), ((1, 4), (2, 4)), ((2, 5), (3, 5)), ((3, 5), (4, 5)),
    ((4, 2), (4, 3)), ((4, 3), (4, 4)), ((4, 4), (4, 5)), ((4, 5), (4, 6)), ((4, 6), (4, 7)),
    ((5, 2), (6, 2)), ((5, 2), (5, 3)), ((5, 4), (5, 5)), ((5, 5), (6, 5)),
    ((6, 3), (7, 3)), ((7, 2), (7, 3)), ((6, 4), (7, 4)), ((7, 4), (7, 5)), ((5, 6), (5, 7)),
]
WIRES = [  # net, polyline
    ("OUTF", [(7, 2), (7, 1), (2, 1)]),
    ("INF-", [(7, 5), (7, 6), (5, 6)]),
]
# cables (solder side) -> destination
CABLES = [
    ("VDD", (6, 2), "XIAO D1", "to pin 8"),
    ("OUTSK", (3, 2), "XIAO A0 (D0)", "to pin 1"),
    ("GND", (4, 3), "XIAO GND", "to GND bridge chain"),
    ("BATIN", (3, 7), "LiPo +", "to XH pin 1"),
    ("GND", (4, 7), "LiPo −", "to XH pin 2 (GND)"),
    ("BATOUT", (6, 7), "XIAO BAT+", "to XH pin 4"),
]

TERM = {}
for k, p in U1.items(): TERM[f"U1.{k}"] = (p, pinnet[k])
for k, p in XH.items(): TERM[f"XH.{k}"] = (p, xhnet[k])
for n, v, a, b, na, nb in SMD:
    TERM[f"{n}.a"] = (a, na); TERM[f"{n}.b"] = (b, nb)
for net, p, dst, how in CABLES: TERM[f"cable.{dst}"] = (p, net)

def seg(a, b):
    (x1, y1), (x2, y2) = a, b
    assert x1 == x2 or y1 == y2
    if x1 == x2:
        s = 1 if y2 > y1 else -1; return [(x1, y) for y in range(y1, y2 + s, s)]
    s = 1 if x2 > x1 else -1; return [(x, y1) for x in range(x1, x2 + s, s)]

parent = {}
def find(a):
    parent.setdefault(a, a)
    while parent[a] != a: parent[a] = parent[parent[a]]; a = parent[a]
    return a
def union(a, b): parent[find(a)] = find(b)

err = []
smdpads = {}
for n, v, a, b, na, nb in SMD:
    if abs(a[0]-b[0]) + abs(a[1]-b[1]) != 1: err.append(f"{n} nicht benachbart")
    for p in (a, b): smdpads.setdefault(p, []).append(n)
tht = set(U1.values()) | set(XH.values())
for p, l in smdpads.items():
    if len(l) > 1: err.append(f"{nm(p)} mehrere SMD {l}")
    if p in tht: err.append(f"{nm(p)} SMD auf THT")
for a, b in BRIDGES:
    if abs(a[0]-b[0]) + abs(a[1]-b[1]) != 1: err.append(f"Bruecke {a}{b}")
    union(a, b)
occupied = set(smdpads) | tht | {q for br in BRIDGES for q in br}
for net, pts in WIRES:
    allp = []
    for a, b in zip(pts, pts[1:]): allp += [q for q in seg(a, b) if q not in allp]
    for q in (pts[0], pts[-1]):
        if q in smdpads: err.append(f"Drahtende auf SMD-Pad {nm(q)}")
    for q in pts: union(pts[0], q)
    for q in allp:
        if q not in (pts[0], pts[-1]) and q in occupied: err.append(f"Draht {net} ueber belegtes Pad {nm(q)}")
    occupied |= set(allp)
for (t1, (p1, n1)), (t2, (p2, n2)) in itertools.combinations(TERM.items(), 2):
    if (n1 == n2) != (find(p1) == find(p2)):
        err.append(f"{t1}({n1}) vs {t2}({n2}) connected={find(p1)==find(p2)}")
if err:
    print("ERROR"); [print(" ", e) for e in err]; sys.exit(1)
print("Netlist OK:", len(TERM), "terminals")

# ---------- drawing ----------
NETC = {"GND": "#222222", "VDD": "#d62728", "OUTF": "#ff7f0e", "INF-": "#2ca02c", "VREF": "#9467bd",
        "OUTSK": "#1f77b4", "INSK+": "#11a3b5", "X": "#8c564b", "BATIN": "#c2185b", "BATOUT": "#e91e63"}
P = 58
MX, MY = 70, 112
PW = MX + (NX + 1) * P + 210
out = []
a = out.append
padnet = {}
for t, (p, n) in TERM.items(): padnet.setdefault(p, n)
rootnet = {find(p): n for p, n in padnet.items()}

def panel(ox, oy, bottom):
    X = lambda x: ox + MX + ((NX + 1 - x) if bottom else x) * P
    Y = lambda y: oy + MY + y * P
    a(f'<text x="{ox+MX}" y="{oy+28}" class="h2">{"SOLDER SIDE (bottom, mirrored)" if bottom else "COMPONENT SIDE (top)"}</text>')
    a(f'<text x="{ox+MX}" y="{oy+48}" class="sub">{"0805 parts, solder bridges, 2 wires, cable pads · column 1 on the right" if bottom else "only U1 (soldered directly) and XH header · dashed SMD = bottom side"}</text>')
    bx0 = min(X(1), X(NX)) - P*0.5; bx1 = max(X(1), X(NX)) + P*0.5
    a(f'<rect x="{bx0}" y="{Y(1)-P*0.5}" width="{bx1-bx0}" height="{NY*P}" rx="5" fill="{"#d9c08f" if bottom else "#e6d3a3"}" stroke="#8a7346" stroke-width="2"/>')
    # arrow towards the XIAO
    a(f'<text x="{(bx0+bx1)/2}" y="{Y(1)-P*1.35}" class="sub" text-anchor="middle">▲ top: cables to the XIAO (USB-C points up)</text>')
    for x in range(1, NX + 1):
        a(f'<text x="{X(x)}" y="{Y(1)-P*0.62}" class="ax" text-anchor="middle">{x}</text>')
    for y in range(1, NY + 1):
        a(f'<text x="{bx0-10}" y="{Y(y)+5}" class="ax" text-anchor="end">{ROWS[y-1]}</text>')
    for x in range(1, NX + 1):
        for y in range(1, NY + 1):
            if bottom:
                a(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{P*0.34}" fill="#c98a4b" stroke="#a5682d"/>')
                a(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{P*0.12}" fill="#3b2a14"/>')
            else:
                a(f'<circle cx="{X(x)}" cy="{Y(y)}" r="{P*0.12}" fill="#6b5a3a"/>')
    if not bottom:
        for n, v, pa, pb, na, nb in SMD:
            cx = (X(pa[0])+X(pb[0]))/2; cy = (Y(pa[1])+Y(pb[1]))/2; hor = pa[1] == pb[1]
            w, h = (P*0.79, P*0.49) if hor else (P*0.49, P*0.79)
            a(f'<rect x="{cx-w/2}" y="{cy-h/2}" width="{w}" height="{h}" fill="none" stroke="#7a6a4a" stroke-dasharray="4,3"/>')
            a(f'<text x="{cx}" y="{cy+4}" class="tiny" text-anchor="middle" fill="#7a6a4a">{n}</text>')
        # DIP-8 body (9.6 x 6.35 mm)
        cx = (X(3)+X(6))/2; cy = (Y(2)+Y(5))/2
        w = 6.35/2.54*P; h = 9.6/2.54*P
        a(f'<rect x="{cx-w/2}" y="{cy-h/2}" width="{w}" height="{h}" rx="3" fill="#262626" stroke="#000"/>')
        a(f'<path d="M {cx-9} {cy-h/2} a 9 9 0 0 0 18 0" fill="#e6d3a3"/>')
        a(f'<text x="{cx}" y="{cy-4}" class="lbl" fill="#fff" text-anchor="middle">U1</text>')
        a(f'<text x="{cx}" y="{cy+14}" class="tiny" fill="#ddd" text-anchor="middle">MCP602</text>')
        a(f'<circle cx="{cx-w/2+10}" cy="{cy-h/2+12}" r="4" fill="#ddd"/>')
        lab = {1: "1 OUT A→A0", 2: "2 −IN A", 3: "3 +IN A", 4: "4 GND", 5: "5 +IN B", 6: "6 −IN B", 7: "7 OUT B", 8: "8 VDD"}
        for k, p in U1.items():
            a(f'<rect x="{X(p[0])-P*0.22}" y="{Y(p[1])-P*0.12}" width="{P*0.44}" height="{P*0.24}" fill="#c0c0c0" stroke="#444"/>')
            left = p[0] == 3
            a(f'<text x="{X(p[0]) + (-P*0.3 if left else P*0.3)}" y="{Y(p[1])-P*0.2}" class="pin" text-anchor="{"end" if left else "start"}">{lab[k]}</text>')
        # XH B4B-XH-A: 12.4 x 5.75 mm, sticks out at the bottom
        cx = (X(3)+X(6))/2
        w = 12.4/2.54*P; h = 5.75/2.54*P
        y0 = Y(7) - P*0.9
        a(f'<rect x="{cx-w/2}" y="{y0}" width="{w}" height="{h}" rx="3" fill="#f2f2ea" stroke="#666" stroke-width="1.5"/>')
        a(f'<rect x="{cx-w/2+6}" y="{y0+6}" width="{w-12}" height="{h-18}" fill="none" stroke="#aaa"/>')
        names = {1: "LiPo+", 2: "FSR", 3: "FSR", 4: "BAT+"}
        for k, p in XH.items():
            a(f'<rect x="{X(p[0])-P*0.13}" y="{Y(p[1])-P*0.13}" width="{P*0.26}" height="{P*0.26}" fill="#d4af37" stroke="#7a5c00"/>')
            a(f'<text x="{X(p[0])}" y="{Y(p[1])+P*0.5}" class="pin" text-anchor="middle">{k}:{names[k]}</text>')
        a(f'<text x="{cx}" y="{y0+h+16}" class="lbl" text-anchor="middle">J1  JST-XH B4B-XH-A soldered flat (pins bent 90°), opening facing down</text>')
        a(f'<path d="M {X(3)} {y0+h-6} q {(X(6)-X(3))/2} 22 {X(6)-X(3)} 0" fill="none" stroke="#c2185b" stroke-width="2.5" stroke-dasharray="5,3"/>')
        a(f'<text x="{cx+w/2+8}" y="{y0+h-2}" class="tiny" fill="#c2185b">bridge in FSR plug = on</text>')
    else:
        for net, pts in WIRES:
            c = NETC[net]
            d = " ".join(("M" if i == 0 else "L") + f" {X(q[0])} {Y(q[1])}" for i, q in enumerate(pts))
            a(f'<path d="{d}" fill="none" stroke="#9aa0a6" stroke-width="9" stroke-linecap="round" stroke-linejoin="round"/>')
            a(f'<path d="{d}" fill="none" stroke="{c}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>')
            for q in (pts[0], pts[-1]):
                a(f'<circle cx="{X(q[0])}" cy="{Y(q[1])}" r="{P*0.28}" fill="#dfe3e6" stroke="{c}" stroke-width="2.5"/>')
        for pa, pb in BRIDGES:
            c = NETC.get(rootnet.get(find(pa)), "#888")
            x1, y1, x2, y2 = X(pa[0]), Y(pa[1]), X(pb[0]), Y(pb[1])
            if y1 == y2:
                a(f'<rect x="{min(x1,x2)}" y="{y1-P*0.2}" width="{abs(x2-x1)}" height="{P*0.4}" rx="{P*0.18}" fill="#e3e6e8" stroke="{c}" stroke-width="3"/>')
            else:
                a(f'<rect x="{x1-P*0.2}" y="{min(y1,y2)}" width="{P*0.4}" height="{abs(y2-y1)}" rx="{P*0.18}" fill="#e3e6e8" stroke="{c}" stroke-width="3"/>')
        for p in tht:
            c = NETC.get(padnet.get(p), "#777")
            a(f'<circle cx="{X(p[0])}" cy="{Y(p[1])}" r="{P*0.3}" fill="#e3e6e8" stroke="{c}" stroke-width="3"/>')
            a(f'<circle cx="{X(p[0])}" cy="{Y(p[1])}" r="{P*0.08}" fill="#555"/>')
            if p in U1.values():
                k = [k for k, q in U1.items() if q == p][0]
                a(f'<text x="{X(p[0])+P*0.2}" y="{Y(p[1])+P*0.36}" class="tiny2">{k}</text>')
        for n, v, pa, pb, na, nb in SMD:
            cx = (X(pa[0])+X(pb[0]))/2; cy = (Y(pa[1])+Y(pb[1]))/2; hor = pa[1] == pb[1]
            w, h = (P*0.79, P*0.49) if hor else (P*0.49, P*0.79)
            body = "#c9a46a" if n.startswith("C") else "#1a1a1a"
            a(f'<rect x="{cx-w/2}" y="{cy-h/2}" width="{w}" height="{h}" fill="{body}" stroke="#000"/>')
            t = P*0.15
            if hor:
                for sx in (cx-w/2, cx+w/2-t): a(f'<rect x="{sx}" y="{cy-h/2}" width="{t}" height="{h}" fill="#d8d8d8" stroke="#555" stroke-width="0.6"/>')
            else:
                for sy in (cy-h/2, cy+h/2-t): a(f'<rect x="{cx-w/2}" y="{sy}" width="{w}" height="{t}" fill="#d8d8d8" stroke="#555" stroke-width="0.6"/>')
        LBL = {"R1": (-0.3, 0.85), "C1": (-0.3, 1.75), "R2": (-0.3, 2.95), "C2": (-0.3, 5.75), "C3": (5.6, 0.1),
               "R3": (5.0, 2.55), "R4": (4.5, 5.6), "Rf": (8.0, 3.5)}
        for n, v, pa, pb, na, nb in SMD:
            cx = (X(pa[0])+X(pb[0]))/2; cy = (Y(pa[1])+Y(pb[1]))/2
            gx, gy = LBL[n]
            tx, ty = X(gx), Y(gy)
            a(f'<line x1="{cx}" y1="{cy}" x2="{tx}" y2="{ty-5}" stroke="#333" stroke-width="1"/>')
            a(f'<text x="{tx}" y="{ty}" class="smd" text-anchor="middle" stroke="#fff" stroke-width="4" paint-order="stroke">{n} {v}</text>')
        # cables
        for net, p, dst, how in CABLES:
            c = NETC[net]
            a(f'<circle cx="{X(p[0])}" cy="{Y(p[1])}" r="{P*0.42}" fill="none" stroke="{c}" stroke-width="3" stroke-dasharray="5,3"/>')

panel(10, 50, False)
panel(10 + PW, 50, True)
W = 2 * PW + 20
ly = 50 + MY + (NY + 1) * P + 95

L = []
L.append(f'<text x="40" y="{ly}" class="h2">Cables (solder side, dashed circles)</text>')
for i, (net, p, dst, how) in enumerate(CABLES):
    yy = ly + 24 + i * 21
    L.append(f'<rect x="40" y="{yy-11}" width="26" height="9" rx="3" fill="{NETC[net]}"/>')
    L.append(f'<text x="74" y="{yy}" class="txt"><tspan font-weight="bold">{dst}</tspan> ← {how} ({nm(p)})</text>')
L.append(f'<text x="40" y="{ly+24+6*21}" class="txt">BAT− on the XIAO is GND: LiPo − goes to board GND (XH-2),</text>')
L.append(f'<text x="40" y="{ly+24+7*21}" class="txt">the battery current returns to the XIAO through the GND wire.</text>')

bx = 600
L.append(f'<text x="{bx}" y="{ly}" class="h2">Bill of materials</text>')
bom = ["U1   MCP602-I/P (DIP-8), soldered directly",
       "R1, R2  100 kΩ 0805 1 % (fc ≈23 Hz, Q≈0.73)",
       "R3  330 kΩ 0805      R4  10 kΩ 0805",
       "Rf  12 kΩ 0805 (U_out ≤1.7 V for R_FSR ≥ 0.73 kΩ)",
       "C1 100 nF X7R   C2  47 nF X7R (0805)",
       "C3  100 nF 0805 X7R (decoupling)",
       "J1  JST-XH B4B-XH-A, soldered flat",
       "Perfboard 7×7 holes ≈ 18×18 mm",
       "6 stranded wires (AWG 28–30) to XIAO/LiPo"]
for i, t in enumerate(bom):
    L.append(f'<text x="{bx}" y="{ly+24+i*21}" class="mono">{t}</text>')

cx = 1020
L.append(f'<text x="{cx}" y="{ly}" class="h2">Nets</text>')
nets = [("VDD", "Pin 8, R3, C3 ← XIAO D1"), ("GND", "Pin 4, R4, C2, C3, XH-2"),
        ("VREF", "≈0.1 V: R3/R4 → pin 5"), ("INF-", "Pin 6: Rf, FSR (XH-3)"),
        ("OUTF", "Pin 7 (FSR stage, ≤1.7 V) → Rf, R1"), ("X", "Sallen-Key: R1, R2, C1"),
        ("INSK+", "Pin 3: R2, C2"), ("OUTSK", "Pin 1 = Pin 2: C1 → A0"),
        ("BATIN", "XH-1 ← LiPo +"), ("BATOUT", "XH-4 → XIAO BAT+")]
for i, (n, t) in enumerate(nets):
    yy = ly + 24 + i * 21
    L.append(f'<rect x="{cx}" y="{yy-11}" width="26" height="9" rx="3" fill="{NETC[n]}"/>')
    L.append(f'<text x="{cx+34}" y="{yy}" class="txt"><tspan font-weight="bold">{n}</tspan>  {t}</text>')

H = ly + 24 + 10 * 21 + 20
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" font-family="Helvetica, Arial, sans-serif">',
       '<style>.h1{font-size:22px;font-weight:bold}.h2{font-size:17px;font-weight:bold}.sub{font-size:12.5px;fill:#555}'
       '.ax{font-size:14px;fill:#555}.lbl{font-size:14px;font-weight:bold}.pin{font-size:12px}.tiny{font-size:11.5px}'
       '.tiny2{font-size:11px;font-weight:bold;fill:#333}.smd{font-size:13.5px;font-weight:bold}.txt{font-size:13.5px}'
       '.mono{font-size:13.5px;font-family:Menlo,Consolas,monospace}</style>',
       f'<rect width="{W}" height="{H}" fill="#fff"/>',
       '<text x="40" y="36" class="h1">Compact FSR amplifier: 7×7 holes (≈18×18 mm), XIAO connected by wires</text>',
       f'<text x="{W-40}" y="36" class="sub" text-anchor="end">Channels: B (pins 5–7) = FSR stage, A (pins 1–3) = Sallen-Key ≈23 Hz → A0</text>']
svg += out + L + ['</svg>']
open(sys.argv[1], "w").write("\n".join(svg))
print("SVG", W, H)
