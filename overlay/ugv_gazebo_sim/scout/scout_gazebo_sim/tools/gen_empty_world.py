"""Generate worlds/empty.world: a 20x20 m obstacle arena plus a 26x20 m park with low and high terrain.

Usage (from this package dir):  python3 tools/gen_empty_world.py worlds/empty.world [curb_height_m]
Also writes models/park_terrain/meshes/park_terrain.stl. Then rebuild: colcon build --packages-select scout_gazebo_sim

Layout (world frame, robot spawns at the origin):
  arena  x -10..10  walls, boxes, pillars, low curbs and bumps; 4 m gate in the east wall at y -2..2
  park   x  10..36  terrain mesh with a sunken plaza (-0.6 m) and rolling hills/dips (south),
                    terrace B (0.5 m) and terrace C (1.0 m) in the north-east, linked by wide ramps
"""
import math
import os
import struct
import sys

# Low obstacles: above the wheel radius (0.08 m) so they block the robot, below the lidar plane
CURB_H = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
BUMP_R, BUMP_H = 0.12, CURB_H        # rounded bump: cylinder radius, height above floor

def mat(name):
    if name.startswith('<'):
        return name
    return f"<material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>{name}</name></script></material>"

def rgb(r, g, b):
    return f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse><specular>0.05 0.05 0.05 1</specular></material>"

GRASS, HEDGE, LEAVES = rgb(0.30, 0.55, 0.22), rgb(0.12, 0.38, 0.12), rgb(0.18, 0.48, 0.16)
CONCRETE, PATH, WATER = rgb(0.62, 0.62, 0.60), rgb(0.72, 0.66, 0.55), rgb(0.20, 0.42, 0.75)

# Low friction on obstacles: ODE uses the lower mu of the two surfaces, so the wheels can't
# get grip on a vertical face and climb it (the robot used to climb walls and flip)
SLIPPERY = "<surface><friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction></surface>"
# Anything the robot drives on needs grip (rubber on concrete/grass) or it slides back down even 5 deg
GRIPPY = "<surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>"

def link(name, pose, geom, material, surface=SLIPPERY, collide=True):
    col = f"<collision name='collision'><geometry>{geom}</geometry>{surface}</collision>" if collide else ""
    return f"""      <link name='{name}'>
        <pose>{pose}</pose>
        {col}
        <visual name='visual'><geometry>{geom}</geometry>{mat(material)}</visual>
      </link>"""

def model(name, links):
    return f"""    <model name='{name}'>
      <static>true</static>
{chr(10).join(links)}
    </model>"""

box = lambda x, y, z: f"<box><size>{x} {y} {z}</size></box>"
cyl = lambda r, l: f"<cylinder><radius>{r}</radius><length>{l}</length></cylinder>"
sph = lambda r: f"<sphere><radius>{r}</radius></sphere>"

out = []

# ---------------------------------------------------------------- floor and outer walls
# No ground_plane: it is infinite at z=0 and would cover the sunken parts of the park.
X0, X1, Y0, Y1 = -10.0, 36.0, -10.0, 10.0     # inner faces of the outer walls
PARK_X0 = 10.0                                # arena/park divider
T, H = 0.2, 1.0
out.append(model('arena_floor', [link('floor', '0 0 -0.05 0 0 0', box(20.4, 20.4, 0.1), 'Gazebo/Grey', GRIPPY)]))

# outer walls reach 1 m below the floor so there's no gap where the park terrain dips
WZ, WH = (H - 1.0) / 2, H + 1.0
L = X1 - X0
walls = [link('north', f'{(X0 + X1) / 2} {Y1 + T / 2} {WZ} 0 0 0', box(L + 2 * T, T, WH), 'Gazebo/Grey'),
         link('south', f'{(X0 + X1) / 2} {Y0 - T / 2} {WZ} 0 0 0', box(L + 2 * T, T, WH), 'Gazebo/Grey'),
         link('west', f'{X0 - T / 2} 0 {WZ} 0 0 0', box(T, Y1 - Y0, WH), 'Gazebo/Grey'),
         link('east', f'{X1 + T / 2} 0 {WZ} 0 0 0', box(T, Y1 - Y0, WH), 'Gazebo/Grey'),
         # arena east wall with a 4 m gate (y -2..2) into the park
         link('gate_north', f'{PARK_X0} 6 {H / 2} 0 0 0', box(T, 8.0, H), 'Gazebo/Grey'),
         link('gate_south', f'{PARK_X0} -6 {H / 2} 0 0 0', box(T, 8.0, H), 'Gazebo/Grey')]
out.append(model('arena_walls', walls))

# ---------------------------------------------------------------- arena (unchanged obstacles)
# interior walls (x, y, length, yaw)
inner = [(-4.0, 5.0, 6.0, 1.5708), (-5.0, -5.0, 6.0, 0.0), (6.0, -5.5, 5.0, 1.5708),
         (5.5, 5.0, 4.0, 0.0), (7.5, 6.5, 3.0, 1.5708), (0.5, 7.5, 4.0, 0.0)]
out.append(model('inner_walls', [link(f'wall_{i}', f'{x} {y} {H/2} 0 0 {yaw}', box(l, 0.15, H), 'Gazebo/Bricks')
                                  for i, (x, y, l, yaw) in enumerate(inner)]))

# boxes (x, y, sx, sy, sz, yaw)
boxes = [(3.5, -2.5, 0.6, 0.6, 0.8, 0.3), (-3.0, -2.0, 1.0, 0.5, 0.6, 0.0), (7.5, 1.5, 0.8, 0.8, 0.8, 0.7),
         (-7.5, 7.0, 1.2, 0.8, 0.8, 0.0), (2.5, 4.5, 0.7, 0.7, 0.6, 0.4), (-7.0, -8.0, 1.0, 1.0, 1.0, 0.0),
         (8.0, -8.5, 1.2, 0.6, 0.7, 0.5), (-1.5, -7.5, 0.8, 0.8, 0.8, 0.0), (-8.0, 2.5, 0.6, 1.4, 0.8, 0.0)]
out.append(model('boxes', [link(f'box_{i}', f'{x} {y} {sz/2} 0 0 {yaw}', box(sx, sy, sz), 'Gazebo/Wood')
                           for i, (x, y, sx, sy, sz, yaw) in enumerate(boxes)]))

# pillars (x, y, r)
pillars = [(4.0, 1.5, 0.2), (-2.0, 3.5, 0.25), (-6.0, -2.5, 0.3), (1.0, -5.5, 0.2), (8.0, 4.0, 0.25),
           (-8.5, -4.5, 0.2), (3.5, -7.5, 0.3), (-1.0, 8.5, 0.2)]
out.append(model('pillars', [link(f'pillar_{i}', f'{x} {y} {H/2} 0 0 0', cyl(r, H), 'Gazebo/DarkGrey')
                             for i, (x, y, r) in enumerate(pillars)]))

# low curbs: under the lidar plane -> invisible to a 2D lidar, but too high to drive over
curbs = [(2.5, 0.0, 3.0, 1.5708), (0.0, -3.0, 4.0, 0.0), (-2.5, 0.5, 3.0, 1.5708), (5.0, -2.0, 2.0, 0.0)]
out.append(model('low_curbs', [link(f'curb_{i}', f'{x} {y} {CURB_H/2} 0 0 {yaw}', box(l, 0.15, CURB_H), 'Gazebo/Yellow')
                               for i, (x, y, l, yaw) in enumerate(curbs)]))

# rounded bumps: cylinders lying on their side, sunk so only BUMP_H sticks out
bumps = [(0.0, 3.0, 3.0, 0.0), (-6.0, 0.5, 3.0, 1.5708), (5.0, 2.5, 2.5, 1.5708)]
out.append(model('speed_bumps', [link(f'bump_{i}', f'{x} {y} {BUMP_H - BUMP_R} 1.5708 0 {yaw + 1.5708}', cyl(BUMP_R, l), 'Gazebo/Orange')
                                 for i, (x, y, l, yaw) in enumerate(bumps)]))

# ---------------------------------------------------------------- park terrain (triangle mesh)
def smoothstep(e0, e1, v):
    t = min(max((v - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3 - 2 * t)

PLAZA = (15.0, -5.2, 1.5, 4.5, 0.6)          # sunken plaza: centre, flat radius, outer radius, depth
HILLS = [(24.0, -7.0, 0.8, 1.6),             # (x, y, height, sigma); negative height = dip
         (30.5, -6.5, 0.6, 1.3),
         (34.2, -8.0, 0.5, 1.2),
         (27.5, -8.5, -0.4, 1.0),
         (21.8, -9.0, 0.3, 1.0)]

def terrain_h(x, y):
    """Park ground height. Flat (0) wherever terraces, ramps and the gate are."""
    px, py, rf, ro, depth = PLAZA
    r = math.hypot(x - px, y - py)
    h = -depth if r < rf else (-depth * 0.5 * (1 + math.cos(math.pi * (r - rf) / (ro - rf))) if r < ro else 0.0)
    hills = sum(a * math.exp(-((x - hx) ** 2 + (y - hy) ** 2) / (2 * s * s)) for hx, hy, a, s in HILLS)
    # hills only south of the ramps (y < -3) and east of the plaza
    return h + hills * smoothstep(-3.0, -4.5, y) * smoothstep(19.5, 21.0, x)

STEP = 0.25
NX, NY = int((X1 - PARK_X0) / STEP), int((Y1 - Y0) / STEP)

def write_stl(path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    gx = [PARK_X0 + i * STEP for i in range(NX + 1)]
    gy = [Y0 + j * STEP for j in range(NY + 1)]
    hz = [[terrain_h(x, y) for x in gx] for y in gy]
    tris = []
    for j in range(NY):
        for i in range(NX):
            a, b = (gx[i], gy[j], hz[j][i]), (gx[i + 1], gy[j], hz[j][i + 1])
            c, d = (gx[i + 1], gy[j + 1], hz[j + 1][i + 1]), (gx[i], gy[j + 1], hz[j + 1][i])
            tris += [(a, b, c), (a, c, d)]             # counter-clockwise seen from above -> normal up
    with open(path, 'wb') as f:
        f.write(b'park terrain'.ljust(80, b' ') + struct.pack('<I', len(tris)))
        for p, q, r in tris:
            u, v = [q[k] - p[k] for k in range(3)], [r[k] - p[k] for k in range(3)]
            nrm = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
            ln = math.sqrt(sum(c * c for c in nrm)) or 1.0
            f.write(struct.pack('<12fH', *(c / ln for c in nrm), *p, *q, *r, 0))
    # steepest slope between neighbouring grid points, so the hills stay drivable
    worst = max(max(abs(hz[j][i + 1] - hz[j][i]), abs(hz[j + 1][i] - hz[j][i]))
                for j in range(NY) for i in range(NX))
    return len(tris), min(map(min, hz)), max(map(max, hz)), math.degrees(math.atan(worst / STEP))

pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ntri, hmin, hmax, slope = write_stl(os.path.join(pkg, 'models', 'park_terrain', 'meshes', 'park_terrain.stl'))
print(f'park terrain: {ntri} triangles, height {hmin:.2f}..{hmax:.2f} m, steepest slope {slope:.1f} deg')
mesh = "<mesh><uri>model://park_terrain/meshes/park_terrain.stl</uri></mesh>"
out.append(model('park_terrain', [link('ground', '0 0 0 0 0 0', mesh, GRASS, GRIPPY)]))

# ---------------------------------------------------------------- terraces and ramps
# terraces: (name, x0, x1, y0, y1, height). Brick retaining walls with a grass top.
terraces = [('terrace_b', 22.0, 30.0, 3.0, Y1, 0.5),
            ('terrace_c', 30.0, X1, 3.0, Y1, 1.0)]
ter = []
for name, ax, bx, ay, by, h in terraces:
    cx, cy, sx, sy = (ax + bx) / 2, (ay + by) / 2, bx - ax, by - ay
    ter.append(link(name, f'{cx} {cy} {h / 2} 0 0 0', box(sx, sy, h), 'Gazebo/Bricks', GRIPPY))
    ter.append(link(f'{name}_lawn', f'{cx} {cy} {h + 0.005} 0 0 0', box(sx - 0.1, sy - 0.1, 0.01), GRASS, collide=False))
out.append(model('terraces', ter))

RAMP_T, RAMP_W = 0.15, 2.5
def slope_link(name, x, y, z0, yaw, deg, h, w):
    """Concrete slope rising along yaw from its low edge at (x, y, z0) by h."""
    th = math.radians(deg)
    run, s = h / math.tan(th), h / math.sin(th)
    # tilt a thin box so its top face runs exactly from the low edge to the top edge
    u, z = run / 2 + RAMP_T / 2 * math.sin(th), z0 + h / 2 - RAMP_T / 2 * math.cos(th)
    cx, cy = x + u * math.cos(yaw), y + u * math.sin(yaw)
    return link(name, f'{cx:.4f} {cy:.4f} {z:.4f} 0 {-th:.4f} {yaw:.4f}', box(f'{s:.4f}', w, RAMP_T), CONCRETE, GRIPPY), run

def ramp_to(name, top_x, top_y, z0, yaw, deg, h):
    """Ramp whose top edge is at (top_x, top_y), climbing along yaw from z0 to z0 + h."""
    run = h / math.tan(math.radians(deg))
    lk, _ = slope_link(f'{name}_{deg}deg', top_x - run * math.cos(yaw), top_y - run * math.sin(yaw), z0, yaw, deg, h, RAMP_W)
    return lk

N, E = math.pi / 2, 0.0
ramps = [ramp_to('ramp_meadow_b_west', 22.0, 7.25, 0.0, E, 10, 0.5),   # meadow -> B, from the west
         ramp_to('ramp_meadow_b_long', 24.25, 3.0, 0.0, N, 5, 0.5),    # meadow -> B, long and gentle
         ramp_to('ramp_meadow_b_short', 28.25, 3.0, 0.0, N, 20, 0.5),  # meadow -> B, short and steep
         ramp_to('ramp_b_c', 30.0, 7.25, 0.5, E, 15, 0.5),             # B -> C
         ramp_to('ramp_meadow_c', 33.25, 3.0, 0.0, N, 25, 1.0)]        # meadow -> C, steepest
out.append(model('ramps', ramps))

# hedges along terrace edges (so the robot can't drive off a drop), with gaps at the ramps
def hedge(name, x0, y0, x1, y1, z0, hh=0.45, t=0.3):
    cx, cy, ln = (x0 + x1) / 2, (y0 + y1) / 2, math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    return link(name, f'{cx} {cy} {z0 + hh / 2} 0 0 {yaw}', box(ln, t, hh), HEDGE)
hedges = [hedge('hedge_b_s1', 22.0, 3.15, 23.0, 3.15, 0.5), hedge('hedge_b_s2', 25.5, 3.15, 27.0, 3.15, 0.5),
          hedge('hedge_b_s3', 29.5, 3.15, 30.0, 3.15, 0.5),
          hedge('hedge_b_w1', 22.15, 3.3, 22.15, 6.0, 0.5), hedge('hedge_b_w2', 22.15, 8.5, 22.15, Y1, 0.5),
          hedge('hedge_c_s1', 30.0, 3.15, 32.0, 3.15, 1.0), hedge('hedge_c_s2', 34.5, 3.15, X1, 3.15, 1.0),
          hedge('hedge_c_w1', 30.15, 3.3, 30.15, 6.0, 1.0), hedge('hedge_c_w2', 30.15, 8.5, 30.15, Y1, 1.0)]
out.append(model('hedges', hedges))

# ---------------------------------------------------------------- park furniture
def ground(x, y):
    for _, ax, bx, ay, by, h in terraces:
        if ax <= x <= bx and ay <= y <= by:
            return h
    return terrain_h(x, y)

# trees: trunk + canopy (canopy has collision so the Livox sees it overhead)
trees = [(11.3, 4.0), (11.4, 9.0), (16.3, 8.8), (16.6, 4.2), (20.0, 9.0), (21.0, -2.0), (12.0, -9.0),
         (21.2, -8.0), (25.6, -4.6), (29.0, -5.0), (33.0, -4.6), (35.0, -1.5), (26.5, -1.2),
         (24.2, 8.8), (27.0, 4.5), (34.6, 8.6), (31.5, 5.0)]
tl = []
for i, (x, y) in enumerate(trees):
    z, th = ground(x, y), 1.6 + 0.4 * (i % 3)
    tl.append(link(f'trunk_{i}', f'{x} {y} {z + th / 2} 0 0 0', cyl(0.13 + 0.03 * (i % 2), th), 'Gazebo/Wood'))
    tl.append(link(f'canopy_{i}', f'{x} {y} {z + th + 0.5} 0 0 0', sph(0.8 + 0.15 * (i % 3)), LEAVES))
out.append(model('trees', tl))

# benches (x, y, yaw), lamp posts, bins
benches = [(12.5, 1.1, 0.0), (16.0, 1.1, 0.0), (18.9, 4.0, N), (13.5, -5.2, N), (26.0, 9.4, 0.0), (33.0, 9.4, 0.0)]
lamps = [(11.5, -1.2), (15.0, 1.3), (19.0, -1.2), (21.6, 5.2), (22.6, -1.6), (30.6, 1.0), (29.5, 9.5)]
bins = [(17.5, 1.1), (23.0, 9.4)]
fl = []
for i, (x, y, yaw) in enumerate(benches):
    z = ground(x, y)
    fl.append(link(f'bench_{i}', f'{x} {y} {z + 0.22} 0 0 {yaw}', box(1.5, 0.45, 0.44), 'Gazebo/Wood'))
    fl.append(link(f'bench_{i}_back', f'{x - 0.2 * math.sin(yaw)} {y + 0.2 * math.cos(yaw)} {z + 0.6} 0 0 {yaw}',
                   box(1.5, 0.06, 0.35), 'Gazebo/Wood'))
for i, (x, y) in enumerate(lamps):
    z = ground(x, y)
    fl.append(link(f'lamp_{i}', f'{x} {y} {z + 1.5} 0 0 0', cyl(0.06, 3.0), 'Gazebo/DarkGrey'))
    fl.append(link(f'lamp_{i}_head', f'{x} {y} {z + 3.1} 0 0 0', sph(0.18), 'Gazebo/White', collide=False))
for i, (x, y) in enumerate(bins):
    fl.append(link(f'bin_{i}', f'{x} {y} {ground(x, y) + 0.4} 0 0 0', cyl(0.25, 0.8), 'Gazebo/DarkGrey'))
out.append(model('furniture', fl))

# pond on the meadow (stone rim + water), fountain in the middle of the sunken plaza
px, py, _, _, depth = PLAZA
water = [link('pond_rim', '13.8 6.5 0.15 0 0 0', cyl(1.7, 0.3), 'Gazebo/Grey'),
         link('pond_water', '13.8 6.5 0.305 0 0 0', cyl(1.55, 0.01), WATER, collide=False),
         link('fountain_basin', f'{px} {py} {-depth + 0.2} 0 0 0', cyl(0.9, 0.4), 'Gazebo/Grey'),
         link('fountain_water', f'{px} {py} {-depth + 0.405} 0 0 0', cyl(0.8, 0.01), WATER, collide=False),
         link('fountain_column', f'{px} {py} {-depth + 0.8} 0 0 0', cyl(0.12, 0.8), 'Gazebo/Grey')]
out.append(model('water', water))

# paths: visual-only gravel strips on flat ground (gate -> meadow -> ramps)
paths = [(16.25, 0.0, 12.5, 1.5, 0.0), (18.0, 3.6, 5.7, 1.5, N), (19.4, 7.25, 1.5, 1.5, 0.0),
         (24.25, -1.5, 2.5, 1.5, N)]
out.append(model('paths', [link(f'path_{i}', f'{x} {y} 0.004 0 0 {yaw}', box(l, w, 0.006), PATH, collide=False)
                           for i, (x, y, l, w, yaw) in enumerate(paths)]))

world = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- West: 20 x 20 m arena with walls, boxes and pillars (seen by the lidar), yellow curbs
         ({CURB_H*100:.0f} cm) and orange bumps ({BUMP_H*100:.0f} cm) that block the robot.
         East (through the gate at x=10): a park with a sunken plaza ({-hmin:.1f} m down), rolling hills
         (up to {hmax:.1f} m, steepest {slope:.0f} deg), terraces at 0.5 and 1.0 m, and ramps of 5-25 deg.
         Generated by tools/gen_empty_world.py. Origin is kept clear for spawning. -->

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.6 0.75 0.9 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
      <sky><clouds><speed>4</speed></clouds></sky>
    </scene>

    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Set world localization for GPS -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>45.15</latitude_deg>
      <longitude_deg>10.00</longitude_deg>
      <elevation>47.00</elevation>
      <heading_deg>180</heading_deg>
    </spherical_coordinates>

    <!-- Physics -->
    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

{chr(10).join(out)}
  </world>
</sdf>
"""
open(sys.argv[1], 'w').write(world)
