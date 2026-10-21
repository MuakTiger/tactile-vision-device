"""
3단 케이크 구조 하우징 생성기

  1층(최상단)  기존 촉각 디스플레이 (full/7_22 set_final.stl, 원본 방향 그대로)
               173 x 200 x 34.8 판, 서보는 판 위로, 판 바닥 24개 구멍으로 서보 선이 아래로
  2층          Pico, PCA9685 x2, 스텝다운 x2, 배터리 x2 (케이블타이/테이프 고정)
               가운데 세로 칸막이로 PCA1(0x40) 계통 / PCA2(0x41) 계통 전원을 좌우로 분리
  3층(최하단)  Jetson + 보조배터리 + Logitech C920 (앞벽 아래 큰 창으로 밖을 봄)

한 번에 출력(Bambu Lab H2D, 1장 플레이트):
  2층을 90도 돌리면 3층 안에 쏙 들어간다. 3층의 받침턱은 Y쪽 벽에만 있어서
  돌려 넣은 2층과는 안 부딪히고, 조립할 때 원래 방향으로 올리면 그 턱 위에 걸친다.
  2층 바닥은 3층 바닥에서 NEST_Z_GAP 만큼 떠 있으므로 그 틈만 서포트로 받친다.
  X쪽엔 받침턱이 없어 생기는 좌우 틈은 막대 2개(같은 판 옆자리에 출력)를 끼워 메운다
  -> 조립하면 3층 테두리가 2층을 네 면 다 감싼다.
  -> print_plate_nested.stl 하나로 전부 같이 뽑는다.

실행:  python stack_tiers.py
출력:  tier2.stl, tier3.stl, tier3_side_fillers.stl (따로 출력용)
       print_plate_nested.stl (한 판 출력용), stack_assembled.stl (조립 상태 확인용), viewer.html
좌표:  바닥 중심이 원점, Z 위쪽.
"""
from pathlib import Path

import numpy as np
import trimesh
from manifold3d import CrossSection, JoinType, Manifold

HERE = Path(__file__).resolve().parent
TOP_STL = HERE.parent / "full" / "7_22 set_final.stl"

# ============================================================ PARAMETERS
# --- 1층 (기존 출력물 실측값) ---
PLATE_W, PLATE_L = 173.0, 200.0
# 판 바닥 선 구멍 영역(판 중심 기준). 1층을 180도 돌려 올려도 덮이도록 대칭으로 잡음
WIRE_HX, WIRE_HY = 75.5, 84.0
PLATE_REST = 6.0        # 판 가장자리가 2층 윗판에 걸치는 폭
PLATE_FIT = 1.0         # 판 둘레 턱과의 여유(한쪽)

# --- 공통 ---
WALL = 3.0
FLOOR = 3.0
R = 6.0                 # 바깥 모서리 둥글기
SEG = 64

# --- 2층 ---
T2_W, T2_L = 200.0, 236.0   # 바깥 X, Y  (T2_L - T2_W >= 2*NEST_GAP + 2*MIN_OVERLAP 이어야 겹쳐 출력 가능)
T2_INNER_H = 55.0           # 바닥 윗면 ~ 윗판 아랫면
DECK_T = 3.0                # 윗판 두께
LIP_H, LIP_T = 4.0, 2.0     # 1층 판 둘레 턱
DIV_T = 2.0                 # 칸막이 두께
DIV_LOW_H = 35.0            # 가로 칸막이 높이 (위로 배선 넘어감)
DIV_Y = (-55.0, -15.0)      # 가로 칸막이 위치: 배터리 | 스텝다운 | PCA
PICO_ZONE_Y = 70.0          # 가운데 칸막이가 끝나는 곳, 그 너머는 Pico 공용 칸
FLOOR_SLOT = (26.0, 12.0)   # 2층 바닥 케이블 구멍 (X, Y)
FLOOR_SLOT_X = (-70.0, -30.0, 30.0, 70.0)
FLOOR_SLOT_Y = (-85.0, -35.0, 10.0, 45.0)
PICO_SLOT_X = (-60.0, 0.0, 60.0)
PICO_SLOT_Y = 92.0

# --- 3층 ---
NEST_GAP = 5.0              # 겹쳐 출력할 때 2층과 3층 벽 사이 최소 간격
MIN_OVERLAP = 10.0          # 2층이 받침턱에 걸치는 최소 폭
STEP_Y = 7.0                # Y 방향 케이크 단차
T3_INNER_H = 77.0           # 바닥 윗면 ~ 2층 바닥 (AGX Orin 71.7 mm 도 들어감)
RIM_H = 6.0                 # 받침턱 위로 올라오는 테두리
# 좌우 틈 막대: 돌려 넣어 출력하느라 X쪽엔 받침턱을 못 두므로, 이 막대를 끼워 네 면을 감싼다
FILLER_CLEAR = 1.0          # 2층 옆면과의 여유
FILLER_FIT = 0.5            # 3층 벽과의 여유
FILLER_VENT = (8.0, 30.0)   # 막대 통풍 구멍 (X, Y)
FILLER_VENT_Y = (-75.0, -37.5, 0.0, 37.5, 75.0)
PLATE_GAP = 6.0             # 출력 판에서 부품 사이 간격
T3_WINDOW = (140.0, 40.0)   # +-X 벽 케이블/포트 창 (폭, 높이)
CAM_WINDOW = (120.0, 50.0)  # -Y(앞) 벽 바닥 C920 창 (폭, 높이). C920 본체 약 94 x 29 x 24
WIN_CHAMFER = 15.0          # 창 윗모서리 45도 깎기 (브리지 길이 줄임)

# --- 2층-3층 연결 구멍 (케이블타이 / M3 볼트), 2층 바닥과 3층 받침턱을 같이 관통 ---
TIE_X = (-70.0, 0.0, 70.0)  # 한 쌍씩 (x-5, x+5)
TIE_PITCH = 10.0
TIE_D = 4.0

# --- 출력 ---
BED = (300.0, 320.0)        # H2D 양쪽 노즐 공용 영역 (단일 노즐은 325 x 320) - 보수적으로
NEST_Z_GAP = 3.0            # 한 판 출력 시 2층 바닥을 3층 바닥에서 띄우는 높이 (이 틈만 서포트)
# ============================================================

# 유도 치수 (모두 반폭, 판 중심 = 원점)
T2_HX, T2_HY = T2_W / 2, T2_L / 2
DECK_Z = FLOOR + T2_INNER_H + DECK_T                    # 2층 윗판 윗면 = 1층 판 바닥
T2_H = DECK_Z + LIP_H
OPEN_HX, OPEN_HY = PLATE_W / 2 - PLATE_REST, PLATE_L / 2 - PLATE_REST

T3_IX = T2_HY + NEST_GAP                                # 돌려 넣은 2층의 긴 변이 X로 들어감
T3_HX = T3_IX + WALL
T3_HY = T2_HY + STEP_Y
T3_IY = T3_HY - WALL
LEDGE_LO = T2_HX + NEST_GAP                             # 돌려 넣은 2층과 안 부딪히는 한계
LEDGE_HI = T2_HY - MIN_OVERLAP                          # 2층이 걸칠 수 있는 한계
LEDGE_IY = (LEDGE_LO + LEDGE_HI) / 2                    # 받침턱 안쪽 끝
LEDGE_Z = FLOOR + T3_INNER_H
T3_H = LEDGE_Z + RIM_H
TIE_Y = (LEDGE_IY + T2_HY - WALL) / 2                   # 2층 바닥 안쪽이면서 받침턱 위

FILLER_X0, FILLER_X1 = T2_HX + FILLER_CLEAR, T3_IX - FILLER_FIT   # 막대 X 범위 (+X쪽)
FILLER_HY = T3_IY - 4.0                                 # 3층 안쪽 둥근 모서리 피함
FILLER_W = FILLER_X1 - FILLER_X0

assert OPEN_HX >= WIRE_HX + 3 and OPEN_HY >= WIRE_HY + 3, "윗판 구멍이 선 구멍을 못 덮음"
assert LEDGE_LO < LEDGE_HI, "T2_L - T2_W 를 키워야 겹쳐 출력 가능"
assert 2 * T3_HX <= BED[0] and 2 * T3_HY <= BED[1], "3층이 H2D 베드보다 큼"


# ------------------------------------------------------------ helpers
def rrect(hx, hy, r):
    return CrossSection.square((2 * (hx - r), 2 * (hy - r)), center=True).offset(r, JoinType.Round, circular_segments=SEG)


def prism(cs, z0, z1, scale_top=(1.0, 1.0)):
    return Manifold.extrude(cs, z1 - z0, scale_top=scale_top).translate((0, 0, z0))


def box(x0, y0, z0, x1, y1, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def mirrored_pair(m, normal):
    return m + m.mirror(normal)


def cyl(x, y, d, z0, z1):
    return Manifold.cylinder(z1 - z0, d / 2, circular_segments=SEG).translate((x, y, z0))


def wall_window(w, h, z0, c):
    """윗모서리를 45도로 깎은 창 단면 (u=벽 방향, v=z)."""
    return CrossSection([[(-w / 2, z0), (w / 2, z0), (w / 2, z0 + h - c), (w / 2 - c, z0 + h),
                          (-w / 2 + c, z0 + h), (-w / 2, z0 + h - c)]])


def window_x(w, h, z0, x0, x1):
    """X 벽(YZ 평면)을 x0~x1 두께로 뚫는 창."""
    return Manifold.extrude(wall_window(w, h, z0, WIN_CHAMFER), x1 - x0).transform(
        [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]])


def window_y(w, h, z0, y0, y1):
    """Y 벽(XZ 평면)을 y0~y1 두께로 뚫는 창."""
    return Manifold.extrude(wall_window(w, h, z0, WIN_CHAMFER), y1 - y0).transform(
        [[1, 0, 0, 0], [0, 0, -1, y1], [0, 1, 0, 0]])


def tie_holes(z0, z1):
    return union([cyl(x + dx, sy * TIE_Y, TIE_D, z0, z1)
                  for x in TIE_X for dx in (-TIE_PITCH / 2, TIE_PITCH / 2) for sy in (-1, 1)])


def union(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


# ------------------------------------------------------------ 2층
def make_tier2():
    ix, iy = T2_HX - WALL, T2_HY - WALL
    deck_bot = DECK_Z - DECK_T
    chamfer = max(ix - OPEN_HX, iy - OPEN_HY)          # 윗판 아래 45도 이하 경사 -> 서포트 불필요
    body = prism(rrect(T2_HX, T2_HY, R), 0, DECK_Z)
    cavity = (
        prism(rrect(ix, iy, R - WALL), FLOOR, deck_bot - chamfer)
        + prism(rrect(ix, iy, R - WALL), deck_bot - chamfer, deck_bot, (OPEN_HX / ix, OPEN_HY / iy))
    )
    opening = box(-OPEN_HX, -OPEN_HY, deck_bot - 1, OPEN_HX, OPEN_HY, DECK_Z + 1)
    fx, fy = PLATE_W / 2 + PLATE_FIT, PLATE_L / 2 + PLATE_FIT
    lip = prism(rrect(fx + LIP_T, fy + LIP_T, LIP_T) - CrossSection.square((2 * fx, 2 * fy), center=True),
                DECK_Z - 0.01, T2_H)

    # 칸막이: 가운데 세로벽(판 바로 아래까지)이 PCA1 / PCA2 계통을 좌우로 나눔,
    # 가로벽(낮음)은 배터리 | 스텝다운 | PCA 칸. +Y 끝은 Pico 공용 칸
    center = box(-DIV_T / 2, -iy - 0.5, FLOOR - 0.01, DIV_T / 2, PICO_ZONE_Y, DECK_Z - 1)
    cross = union([box(-ix - 0.5, y - DIV_T / 2, FLOOR - 0.01, ix + 0.5, y + DIV_T / 2, FLOOR + DIV_LOW_H)
                   for y in DIV_Y])

    sw, sl = FLOOR_SLOT
    slot = rrect(sw / 2, sl / 2, sl / 2 - 0.01)
    slots = union([prism(slot, -1, FLOOR + 1).translate((x, y, 0))
                   for x in FLOOR_SLOT_X for y in FLOOR_SLOT_Y]
                  + [prism(slot, -1, FLOOR + 1).translate((x, PICO_SLOT_Y, 0)) for x in PICO_SLOT_X])
    return body - cavity - opening + lip + center + cross - slots - tie_holes(-1, FLOOR + 1)


# ------------------------------------------------------------ 3층
def make_tier3():
    body = prism(rrect(T3_HX, T3_HY, R), 0, T3_H)
    cavity = prism(rrect(T3_IX, T3_IY, R - WALL), FLOOR, T3_H + 1)

    # Y 벽 받침턱: 아래는 45도 경사, 안쪽 끝 2 mm 수직
    depth = T3_IY + 1 - LEDGE_IY
    prof = CrossSection([[(LEDGE_IY, LEDGE_Z), (LEDGE_IY, LEDGE_Z - 2),
                          (T3_IY + 1, LEDGE_Z - 2 - depth), (T3_IY + 1, LEDGE_Z)]])   # (y, z), CCW
    ledge = Manifold.extrude(prof, 2 * T3_IX + 2).transform(   # (y, z, 길이) -> (x, y, z)
        [[0, 0, 1, -T3_IX - 1], [1, 0, 0, 0], [0, 1, 0, 0]])
    ledges = mirrored_pair(ledge, (0, 1, 0))

    ww, wh = T3_WINDOW
    windows = mirrored_pair(window_x(ww, wh, FLOOR + 2, T3_IX - 1, T3_HX + 1), (1, 0, 0))
    cw, ch = CAM_WINDOW
    cam = window_y(cw, ch, FLOOR, -T3_HY - 1, -T3_IY + 1)          # 앞벽, 바닥면부터
    z0 = LEDGE_Z - 2 - (TIE_Y + TIE_D - LEDGE_IY) - 1
    ties = tie_holes(z0, T3_H + 1) + mirrored_pair(filler_ties(z0, T3_H + 1), (1, 0, 0))
    shell = (body - cavity + ledges) ^ prism(rrect(T3_HX, T3_HY, R), -1, T3_H + 1)
    return shell - windows - cam - ties


# ------------------------------------------------------------ 좌우 틈 막대
def filler_ties(z0, z1):
    xc = (FILLER_X0 + FILLER_X1) / 2
    return union([cyl(xc + dx, sy * TIE_Y, TIE_D, z0, z1)
                  for dx in (-TIE_PITCH / 2, TIE_PITCH / 2) for sy in (-1, 1)])


def make_filler():
    """+X쪽 막대, 조립 위치. 양 끝이 Y 받침턱에 얹히고 윗면이 3층 테두리와 같은 높이.
    두 개 모두 이 모양 그대로(대칭이라 -X쪽에도 그대로 들어감)."""
    bar = box(FILLER_X0, -FILLER_HY, LEDGE_Z, FILLER_X1, FILLER_HY, T3_H)
    vw, vl = FILLER_VENT
    xc = (FILLER_X0 + FILLER_X1) / 2
    vents = union([prism(rrect(vw / 2, vl / 2, vw / 2 - 0.01), LEDGE_Z - 1, T3_H + 1).translate((xc, y, 0))
                   for y in FILLER_VENT_Y])
    return bar - vents - filler_ties(LEDGE_Z - 1, T3_H + 1)


def filler_flat(filler: Manifold) -> Manifold:
    """출력용: 원점 중심, 바닥 z=0, 긴 변이 X."""
    xc = (FILLER_X0 + FILLER_X1) / 2
    return filler.translate((-xc, 0, -LEDGE_Z)).rotate((0, 0, 90))


def print_layout(t2: Manifold, t3: Manifold, filler: Manifold):
    """H2D 한 판: 3층 + 그 안에 돌려 넣은 2층 + 3층 옆에 막대 2개. 판 중심 정렬."""
    flat = filler_flat(filler)
    ys = [T3_HY + PLATE_GAP + FILLER_W / 2 + i * (FILLER_W + PLATE_GAP) for i in range(2)]
    parts = [t3, nested_tier2(t2)] + [flat.translate((0, y, 0)) for y in ys]
    shift = -(ys[-1] + FILLER_W / 2 - T3_HY) / 2
    return [p.translate((0, shift, 0)) for p in parts]


# ------------------------------------------------------------ export
def to_trimesh(m: Manifold) -> trimesh.Trimesh:
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=True)


def load_top_module():
    """기존 1층을 원본 방향(서보 위) 그대로, 판 중심을 원점에 맞춘다."""
    top = trimesh.load(TOP_STL, force="mesh")
    b = top.bounds
    top.apply_translation([-(b[0][0] + b[1][0]) / 2, -(b[0][1] + b[1][1]) / 2, -b[0][2]])
    return top


def nested_tier2(t2: Manifold) -> Manifold:
    return t2.rotate((0, 0, 90)).translate((0, 0, FLOOR + NEST_Z_GAP))


VIEWER_HTML = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>3-Tier Stack Viewer</title>
<style>
 body{margin:0;background:#1e2127;color:#ddd;font:14px system-ui,sans-serif;overflow:hidden}
 #ui{position:absolute;top:12px;left:12px;background:#0009;padding:10px 14px;border-radius:8px;max-width:260px}
 #ui label{display:block;margin:4px 0} input[type=range]{width:170px}
 #info{margin-top:8px;font-size:12px;color:#aab;line-height:1.5}
</style></head><body>
<div id="ui"><b>3단 스택</b>
 <label><input type="radio" name="mode" id="mAsm" checked> 조립 상태</label>
 <label><input type="radio" name="mode" id="mPrint"> 출력 배치 (H2D 한 판)</label>
 <hr style="border-color:#444">
 <label><input type="checkbox" id="v1" checked> 1층 디스플레이 (회색)</label>
 <label><input type="checkbox" id="v2" checked> 2층 (파랑)</label>
 <label><input type="checkbox" id="v3" checked> 3층 (주황)</label>
 <label>분해 <input type="range" id="ex" min="0" max="150" value="0"></label>
 <label>단면 X <input type="range" id="cut" min="-140" max="140" value="140"></label>
 <div id="info"></div>
</div>
<script type="importmap">{"imports":{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
const DATA = __DATA__, META = __META__;
const r = new THREE.WebGLRenderer({antialias:true}); r.setSize(innerWidth,innerHeight); r.localClippingEnabled=true;
document.body.appendChild(r.domElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(0x1e2127);
const cam = new THREE.PerspectiveCamera(40, innerWidth/innerHeight, 1, 5000);
cam.up.set(0,0,1); cam.position.set(430,-520,420);
const ctl = new OrbitControls(cam, r.domElement); ctl.target.set(0,0,90); ctl.update();
scene.add(new THREE.HemisphereLight(0xffffff,0x444455,1.2));
const d = new THREE.DirectionalLight(0xffffff,1.6); d.position.set(200,-300,500); scene.add(d);
const clip = new THREE.Plane(new THREE.Vector3(-1,0,0), 140);
const loader = new STLLoader(); const M = {};
for (const [k, o] of Object.entries(DATA)) {
  const bin = Uint8Array.from(atob(o.b64), c => c.charCodeAt(0)).buffer;
  const g = loader.parse(bin); g.computeVertexNormals();
  M[k] = new THREE.Mesh(g, new THREE.MeshStandardMaterial({color:o.color, roughness:.6, metalness:.05,
    side:THREE.DoubleSide, clippingPlanes:[clip]}));
  scene.add(M[k]);
}
const grid = new THREE.GridHelper(500, 25, 0x555555, 0x333333); grid.rotation.x = Math.PI/2; scene.add(grid);
const [bw, bl] = META.bed;
const bed = new THREE.LineLoop(new THREE.BufferGeometry().setFromPoints(
  [[-bw/2,-bl/2],[bw/2,-bl/2],[bw/2,bl/2],[-bw/2,bl/2]].map(([x,y]) => new THREE.Vector3(x,y,0.5))),
  new THREE.LineBasicMaterial({color:0x7fdc7f}));
scene.add(bed);
const upd = () => {
  const e = +ex.value, pr = mPrint.checked;
  bed.visible = pr;
  M.top.visible = v1.checked && !pr; M.t2.visible = v2.checked && !pr;
  M.t3.visible = M.fill.visible = v3.checked && !pr;
  M.p2.visible = v2.checked && pr; M.p3.visible = M.pf.visible = v3.checked && pr;
  M.t2.position.z = META.t2z + e*0.6; M.fill.position.z = e*0.3; M.top.position.z = META.topz + e*1.3;
  M.p2.position.z = e;
  clip.constant = +cut.value;
  info.innerHTML = pr
    ? `초록 선 = H2D 출력 영역 ${bw}×${bl} mm<br>배치 ${META.plate}<br>2층은 90° 돌려 3층 안에, 막대 2개는 옆에`
    : `3층 ${META.t3}<br>2층 ${META.t2}<br>전체 높이 ${META.total} mm`;
};
for (const el of [mAsm,mPrint,v1,v2,v3,ex,cut]) el.oninput = upd; upd();
addEventListener('resize', () => { cam.aspect = innerWidth/innerHeight; cam.updateProjectionMatrix(); r.setSize(innerWidth,innerHeight); });
(function loop(){ requestAnimationFrame(loop); r.render(scene, cam); })();
</script></body></html>
"""


def write_viewer(path, items, meta):
    """items: {key: (trimesh, color_hex)} -> STL 내장 단일 HTML."""
    import base64
    import json
    data = {k: {"b64": base64.b64encode(trimesh.exchange.stl.export_stl(m)).decode(), "color": c}
            for k, (m, c) in items.items()}
    html = VIEWER_HTML.replace("__DATA__", json.dumps(data)).replace("__META__", json.dumps(meta))
    path.write_text(html, encoding="utf-8")


def dims(m):
    e = m.extents
    return f"{e[0]:.0f} × {e[1]:.0f} × {e[2]:.0f} mm"


def main():
    m2, m3, mf = make_tier2(), make_tier3(), make_filler()
    t2, t3 = to_trimesh(m2), to_trimesh(m3)
    fill_asm = to_trimesh(mirrored_pair(mf, (1, 0, 0)))
    flat = filler_flat(mf)
    fillers_flat = to_trimesh(flat.translate((0, -(FILLER_W + PLATE_GAP) / 2, 0))
                              + flat.translate((0, (FILLER_W + PLATE_GAP) / 2, 0)))
    p3, p2, *pf = [to_trimesh(p) for p in print_layout(m2, m3, mf)]
    plate = trimesh.util.concatenate([p3, p2] + pf)
    assert plate.extents[0] <= BED[0] and plate.extents[1] <= BED[1], f"출력 배치가 베드보다 큼: {plate.extents}"
    for name, m in (("tier2.stl", t2), ("tier3.stl", t3), ("tier3_side_fillers.stl", fillers_flat),
                    ("print_plate_nested.stl", plate)):
        m.export(HERE / name)
        print(f"{name}: {np.round(m.extents, 1)} mm, watertight={m.is_watertight}")

    top = load_top_module()
    top_z = LEDGE_Z + DECK_Z
    total = top_z + top.extents[2]
    assembled = trimesh.util.concatenate([t3, fill_asm, t2.copy().apply_translation([0, 0, LEDGE_Z]),
                                          top.copy().apply_translation([0, 0, top_z])])
    assembled.export(HERE / "stack_assembled.stl")
    print(f"stack_assembled.stl: {np.round(assembled.extents, 1)} mm (1층 포함, 확인용)")
    print(f"total height {total:.1f} mm  (tier3 {T3_H:.1f}, tier2 {T2_H:.1f}, ledge z {LEDGE_Z:.1f})")
    print(f"tier2 interior ~ {2 * (T2_HX - WALL):.0f} x {2 * (T2_HY - WALL):.0f} x {T2_INNER_H:.0f}")
    print(f"tier3 interior ~ {2 * T3_IX:.0f} x {2 * T3_IY:.0f} x {T3_INNER_H:.0f}")

    write_viewer(HERE / "viewer.html",
                 {"t3": (t3, "#e89a4c"), "fill": (fill_asm, "#c97a32"), "t2": (t2, "#5a9bd8"),
                  "top": (top, "#d8d8de"),
                  "p3": (p3, "#e89a4c"), "p2": (p2, "#5a9bd8"), "pf": (trimesh.util.concatenate(pf), "#c97a32")},
                 {"bed": BED, "t2z": LEDGE_Z, "topz": top_z,
                  "t2": dims(t2), "t3": dims(t3), "plate": dims(plate), "total": round(total)})
    print("viewer.html written")


if __name__ == "__main__":
    main()
