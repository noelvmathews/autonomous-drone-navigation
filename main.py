
import math
import random
from ursina import *

# --------------------------------------------------------------------------
# Physics & Flight Dynamic Parameters
# --------------------------------------------------------------------------
G = 9.81
MAX_THRUST = 2.2 * G
MAX_TILT = 38.0
ATTITUDE_RESPONSE = 7.5
YAW_RATE = 110.0
THROTTLE_UP = 0.65
THROTTLE_DOWN = 0.85
DRAG_LIN = 0.30
DRAG_QUAD = 0.035
DRAG_VERT = 0.55
CRASH_DESCENT = 4.8
CRASH_TILT = 18.0
DRONE_RADIUS = 0.95
WORLD_LIMIT = 220.0
CEILING = 160.0

BATTERY_DRAIN_BASE = 0.015
BATTERY_DRAIN_THRUST = 0.06

NUM_BUILDINGS = 36
NUM_RINGS = 10
RING_RADIUS = 3.8
RING_POINTS = 150
ALL_RINGS_BONUS = 500
PAD_HALF = 4.5

PADS = [
    {'name': 'ALPHA PAD', 'pos': (0.0, 0.0), 'color': color.cyan},
    {'name': 'BRAVO PAD', 'pos': (85.0, 45.0), 'color': color.lime},
    {'name': 'SKY HELIPAD', 'pos': (-70.0, -65.0), 'color': color.magenta},
]

CAMERA_MODES = ['CHASE', 'FPV (COCKPIT)', 'HIGH ORBIT TACTICAL']

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))

# --------------------------------------------------------------------------
# Application Setup
# --------------------------------------------------------------------------
app = Ursina(title='AeroSim: Autonomous Drone Navigation Lab', borderless=False)
window.exit_button.visible = False
window.fps_counter.enabled = True
window.color = color.hsv(215, 0.40, 0.95)
camera.fov = 85
camera.clip_plane_far = 2500

random.seed(42)

# --------------------------------------------------------------------------
# Environment & Obstacles
# --------------------------------------------------------------------------
class Building:
    def __init__(self, x, z, w, d, h):
        self.x, self.z, self.w, self.d, self.h = x, z, w, d, h
        hue = random.choice([205, 215, 230, 25, 40])
        col = color.hsv(hue, random.uniform(0.08, 0.28), random.uniform(0.40, 0.85))
        self.entity = Entity(model='cube', position=(x, h / 2, z), scale=(w, h, d),
                             color=col, texture='white_cube',
                             texture_scale=(max(1, w / 4), max(1, h / 4)))
        if h > 20:
            Entity(model='cube', position=(x, h + 2.5, z), scale=(0.3, 5, 0.3), color=color.gray)
            Entity(model='cube', position=(x, h + 5.2, z), scale=0.6, color=color.red)

    def hits(self, p, r):
        return (abs(p.x - self.x) < self.w / 2 + r and
                abs(p.z - self.z) < self.d / 2 + r and
                p.y < self.h + 0.2)

class Ring:
    SEGMENTS = 24

    def __init__(self, pos, facing_deg, ring_id):
        self.id = ring_id
        self.center = Vec3(*pos)
        rad = math.radians(facing_deg)
        self.normal = Vec3(math.sin(rad), 0, math.cos(rad))
        self.collected = False
        self.prev_side = None
        self.root = Entity(position=self.center, rotation_y=facing_deg)
        arc = 2 * math.pi * RING_RADIUS / self.SEGMENTS * 1.15
        self.segs = []
        for i in range(self.SEGMENTS):
            t = 2 * math.pi * i / self.SEGMENTS
            seg = Entity(parent=self.root, model='cube',
                         position=(math.cos(t) * RING_RADIUS, math.sin(t) * RING_RADIUS, 0),
                         rotation_z=math.degrees(t), scale=(0.35, arc, 0.35),
                         color=color.orange)
            self.segs.append(seg)

    def set_collected(self, value):
        self.collected = value
        for s in self.segs:
            s.color = color.lime if value else color.orange

    def set_target(self, is_target):
        if self.collected:
            return
        for s in self.segs:
            s.color = color.yellow if is_target else color.orange

def generate_city():
    ring_specs = []
    for i in range(NUM_RINGS):
        ang = math.radians(i * (360 / NUM_RINGS) + random.uniform(-10, 10))
        dist = random.uniform(32, 95)
        x, z = math.sin(ang) * dist, math.cos(ang) * dist
        y = random.uniform(8, 26)
        facing = math.degrees(ang) + 90 + random.uniform(-15, 15)
        ring_specs.append(((x, y, z), facing))

    keep_clear = [(0.0, 0.0, 18.0)] + [(p['pos'][0], p['pos'][1], 16.0) for p in PADS]
    keep_clear += [(r[0][0], r[0][2], 9.0) for r in ring_specs]

    buildings = []
    attempts = 0
    while len(buildings) < NUM_BUILDINGS and attempts < 2500:
        attempts += 1
        w, d = random.uniform(7, 16), random.uniform(7, 16)
        h = random.uniform(10, 52)
        x, z = random.uniform(-110, 110), random.uniform(-110, 110)
        half = max(w, d) / 2
        if any(math.hypot(x - cx, z - cz) < rad + half for cx, cz, rad in keep_clear):
            continue
        buildings.append(Building(x, z, w, d, h))
    return ring_specs, buildings

# --------------------------------------------------------------------------
# Drone Quadcopter Model & Propeller Blur
# --------------------------------------------------------------------------
class DroneModel:
    def __init__(self):
        self.pivot = Entity()
        self.body = Entity(parent=self.pivot, y=0.3)
        dark = color.hsv(220, 0.15, 0.16)
        # Carbon chassis
        Entity(parent=self.body, model='cube', scale=(0.75, 0.18, 1.05), color=dark)
        Entity(parent=self.body, model='cube', scale=(0.42, 0.14, 0.52), y=0.12, z=0.08, color=color.cyan)
        Entity(parent=self.body, model='sphere', scale=(0.28, 0.28, 0.28), z=0.55, y=0.02, color=color.azure) # FPV lens
        for rot in (45, -45):
            Entity(parent=self.body, model='cube', scale=(2.3, 0.07, 0.12), rotation_y=rot, color=color.dark_gray)
        self.props = []
        for sx in (-0.82, 0.82):
            for sz in (-0.82, 0.82):
                Entity(parent=self.body, model='cube', scale=(0.22, 0.15, 0.22), position=(sx, 0.05, sz), color=dark)
                front = sz > 0
                prop = Entity(parent=self.body, position=(sx, 0.17, sz))
                for a in (0, 90):
                    Entity(parent=prop, model='cube', scale=(1.0, 0.018, 0.09), rotation_y=a,
                           color=color.azure if front else color.white)
                self.props.append(prop)
        self.shadow = Entity(model='circle', rotation_x=90, scale=1.6, y=0.03,
                             color=color.hsv(0, 0, 0, 0.45), double_sided=True)

    def set_pose(self, pos, yaw, pitch, roll, spin):
        self.pivot.position = pos
        self.pivot.rotation_y = yaw
        self.body.rotation_x = pitch
        self.body.rotation_z = -roll
        for p in self.props:
            p.rotation_y += spin
        self.shadow.position = (pos.x, 0.05, pos.z)
        self.shadow.scale = _clamp(1.8 - pos.y * 0.025, 0.5, 1.8)

# --------------------------------------------------------------------------
# Simulation Controller
# --------------------------------------------------------------------------
class DroneFlightLab:
    def __init__(self):
        Sky()
        Entity(model='plane', scale=(600, 1, 600), texture='white_cube',
               texture_scale=(300, 300), color=color.hsv(110, 0.3, 0.48))
        for pad in PADS:
            cx, cz = pad['pos']
            Entity(model='cube', position=(cx, 0.04, cz), scale=(PAD_HALF * 2, 0.08, PAD_HALF * 2), color=pad['color'])
            Entity(model='cube', position=(cx + PAD_HALF, 7, cz + PAD_HALF), scale=(0.25, 14, 0.25), color=color.white)
            Entity(model='cube', position=(cx + PAD_HALF, 14.3, cz + PAD_HALF), scale=0.8, color=pad['color'])

        ring_specs, self.buildings = generate_city()
        self.rings = [Ring(pos, facing, i) for i, (pos, facing) in enumerate(ring_specs)]
        self.drone = DroneModel()
        self.cam_mode = 0
        self.orbit_angle = 0.0

        self._build_tactical_hud()
        self.reset()

    def _build_tactical_hud(self):
        Entity(parent=camera.ui, model='quad', color=color.hsv(0, 0, 0, 0.55),
               origin=(-0.5, 0.5), position=(-0.87, 0.47), scale=(0.46, 0.34))
        self.hud = Text(text='', position=(-0.85, 0.45), scale=1.0, color=color.white)
        self.nav = Text(text='', position=(0.0, 0.46), origin=(0, 0), scale=1.25, color=color.yellow)
        self.cam_text = Text(text='', position=(0.87, 0.47), origin=(0.5, 0.5), scale=0.95, color=color.white)
        self.msg = Text(text='', position=(0, 0.20), origin=(0, 0), scale=2.1, color=color.white)
        self.warn = Text(text='', position=(0, -0.36), origin=(0, 0), scale=1.5, color=color.red)

        # Battery Bar
        Entity(parent=camera.ui, model='quad', color=color.dark_gray, position=(-0.80, -0.42), scale=(0.30, 0.025))
        self.bat_bar = Entity(parent=camera.ui, model='quad', color=color.lime, origin=(-0.5, 0),
                              position=(-0.95, -0.42), scale=(0.30, 0.022))
        self.bat_label = Text(text='BATTERY: 100%', position=(-0.95, -0.39), scale=0.85, color=color.white)

        # Throttle Indicator
        Entity(parent=camera.ui, model='quad', color=color.hsv(0, 0, 0, 0.55),
               position=(0.84, -0.05), scale=(0.045, 0.36))
        self.thr_fill = Entity(parent=camera.ui, model='quad', color=color.azure, origin=(0, -0.5),
                               position=(0.84, -0.21), scale=(0.035, 0.0))
        Entity(parent=camera.ui, model='quad', color=color.white, position=(0.84, -0.21 + 0.17), scale=(0.05, 0.003))

    def reset(self):
        self.pos = Vec3(0, 0, 0)
        self.vel = Vec3(0, 0, 0)
        self.yaw, self.pitch, self.roll = 0.0, 0.0, 0.0
        self.throttle = 0.0
        self.battery = 100.0
        self.grounded = True
        self.crashed = False
        self.score = 0
        self.rings_done = 0
        self.flight_time = 0.0
        self.msg.text = ''
        for r in self.rings:
            r.set_collected(False)

    def step_simulation(self, dt):
        keys = held_keys
        # Battery drain
        if not self.grounded:
            drain = (BATTERY_DRAIN_BASE + self.throttle * BATTERY_DRAIN_THRUST) * (100.0 / 60.0) * dt
            self.battery = max(0.0, self.battery - drain)

        # Controls & Power cut
        can_thrust = self.battery > 0.0 and not self.crashed
        if can_thrust:
            if keys['space']: self.throttle += THROTTLE_UP * dt
            if keys['left shift'] or keys['right shift']: self.throttle -= THROTTLE_DOWN * dt
        else:
            self.throttle = max(0.0, self.throttle - THROTTLE_DOWN * 1.5 * dt)
        self.throttle = _clamp(self.throttle, 0.0, 1.0)

        # Yaw & Attitude
        self.yaw = (self.yaw + (keys['e'] - keys['q']) * YAW_RATE * dt) % 360
        airborne = self.pos.y > 0.25
        target_pitch = (keys['w'] - keys['s']) * MAX_TILT if (airborne and can_thrust) else 0.0
        target_roll = (keys['d'] - keys['a']) * MAX_TILT if (airborne and can_thrust) else 0.0
        self.pitch += (target_pitch - self.pitch) * min(1.0, ATTITUDE_RESPONSE * dt)
        self.roll += (target_roll - self.roll) * min(1.0, ATTITUDE_RESPONSE * dt)

        # Orientation & Vector Dynamics
        p, r, y = math.radians(self.pitch), math.radians(self.roll), math.radians(self.yaw)
        lx, ly, lz = math.sin(r), math.cos(p) * math.cos(r), math.sin(p)
        thrust_vec = (Vec3(math.cos(y), 0, -math.sin(y)) * lx +
                      Vec3(math.sin(y), 0, math.cos(y)) * lz +
                      Vec3(0, ly, 0))

        # Dynamic Wind Shear vector based on altitude
        wind = Vec3(math.sin(time.time() * 0.2) * 1.8, 0, math.cos(time.time() * 0.15) * 1.5) * (self.pos.y / 35.0)

        acc = thrust_vec * (self.throttle * MAX_THRUST) + Vec3(0, -G, 0) + wind
        speed = self.vel.length()
        acc -= self.vel * (DRAG_LIN + DRAG_QUAD * speed)
        acc.y -= self.vel.y * DRAG_VERT

        self.vel += acc * dt
        self.pos += self.vel * dt

        # Touchdown detection
        if self.pos.y <= 0 and self.vel.y <= 0:
            if not self.grounded:
                vsi = -self.vel.y
                tilt = math.degrees(math.acos(_clamp(ly, -1, 1)))
                if vsi > CRASH_DESCENT or tilt > CRASH_TILT:
                    self.crashed = True
                    self.msg.text = 'STRUCTURAL FAILURE: HARD CRASH\nPress R to reset'
                    self.msg.color = color.red
                else:
                    self.msg.text = 'SAFE TOUCHDOWN'
                    self.msg.color = color.lime
            self.pos.y, self.vel.y = 0, 0
            self.vel.x *= math.exp(-6.0 * dt)
            self.vel.z *= math.exp(-6.0 * dt)
            self.grounded = True
        elif self.pos.y > 0.05:
            self.grounded = False

        # Collision with Buildings
        for b in self.buildings:
            if b.hits(self.pos, DRONE_RADIUS):
                self.crashed = True
                self.msg.text = 'IMPACT DETECTED: COLLISION WITH STRUCTURE\nPress R to reset'
                self.msg.color = color.red
                break

        # Check Rings
        for ring in self.rings:
            if ring.collected: continue
            rel = (self.pos + Vec3(0, 0.3, 0)) - ring.center
            side = rel.dot(ring.normal)
            if ring.prev_side is not None and ring.prev_side * side < 0:
                if (rel - ring.normal * side).length() < RING_RADIUS:
                    ring.set_collected(True)
                    self.rings_done += 1
                    self.score += RING_POINTS
                    self.battery = min(100.0, self.battery + 8.0) # recharge reward!
            ring.prev_side = side

    def update(self):
        dt = min(time.dt, 1 / 20)
        if not self.crashed:
            self.step_simulation(dt)
            if not self.grounded: self.flight_time += dt

        # Props spin
        spin = 0 if self.crashed else 2800 * (0.2 + self.throttle) * dt
        self.drone.set_pose(self.pos, self.yaw, self.pitch, self.roll, spin)

        # Camera modes
        y = math.radians(self.yaw)
        fwd = Vec3(math.sin(y), 0, math.cos(y))
        if self.cam_mode == 0:
            target = self.pos - fwd * 8.5 + Vec3(0, 3.2, 0)
            camera.position = camera.position + (target - camera.position) * min(1.0, 7.0 * dt)
            camera.look_at(self.pos + Vec3(0, 0.8, 0))
        elif self.cam_mode == 1:
            camera.position = self.pos + Vec3(0, 0.35, 0) + fwd * 0.55
            camera.rotation = (self.pitch - 10, self.yaw, -self.roll)
        else:
            self.orbit_angle += 10 * dt
            a = math.radians(self.orbit_angle)
            camera.position = self.pos + Vec3(math.sin(a) * 40, 34, math.cos(a) * 40)
            camera.look_at(self.pos)

        # HUD Update
        spd = Vec3(self.vel.x, 0, self.vel.z).length()
        self.hud.text = (f'ALTITUDE : {self.pos.y:6.1f} m\n'
                         f'AIRSPEED : {spd * 3.6:6.1f} km/h\n'
                         f'VERT SPD : {self.vel.y:+6.1f} m/s\n'
                         f'HEADING  : {self.yaw:6.0f} deg\n'
                         f'GATE COMP: {self.rings_done}/{len(self.rings)}\n'
                         f'SCORE    : {self.score:6d}')
        self.thr_fill.scale_y = 0.34 * self.throttle
        self.bat_bar.scale_x = 0.30 * (self.battery / 100.0)
        self.bat_bar.color = color.lime if self.battery > 40 else (color.yellow if self.battery > 20 else color.red)
        self.bat_label.text = f'BATTERY: {self.battery:3.0f}%'
        self.cam_text.text = f'CAMERA: {CAMERA_MODES[self.cam_mode]} [C]'

        # Warnings
        if not self.grounded and not self.crashed:
            if self.battery <= 0: self.warn.text = 'LOW POWER: AUTO-DESCENT ACTIVE'
            elif self.pos.y < 8 and self.vel.y < -CRASH_DESCENT: self.warn.text = 'PULL UP: SINK RATE'
            else: self.warn.text = ''
        else:
            if not self.crashed: self.warn.text = ''

sim = DroneFlightLab()

def update(): sim.update()
def input(key):
    if key == 'r': sim.reset()
    elif key == 'c': sim.cam_mode = (sim.cam_mode + 1) % len(CAMERA_MODES)
    elif key == 'escape': application.quit()

if __name__ == '__main__':
    app.run()
