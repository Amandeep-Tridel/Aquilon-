#!/usr/bin/env python3
"""
Fake GPS + Moving Obstacle Detection with Advanced Avoidance for USV demo.

Simulates:
  - USV navigating a lawnmower path at 60 Hz simulation rate
  - 4 moving obstacles with different motion patterns
  - Sonar-like obstacle detection (range-based)
  - Advanced avoidance: PID heading, speed zones (stop/creep/slow/cruise/fast)
  - Emergency stop, velocity obstacle prediction, smooth recovery
  - 20 Hz marker updates for smooth pMarineViewer animation
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

# Lawnmower waypoints
WAYPOINTS = [
    (20, -20), (220, -20),
    (220, -50), (20, -50),
    (20, -80), (220, -80),
    (220, -110), (20, -110),
    (20, -140), (220, -140),
    (220, -170), (20, -170),
]

# --- Detection & Avoidance Zones (scaled for small obstacles) ---
DETECTION_RANGE = 30.0    # sonar detection range
SLOW_RANGE = 20.0         # slow down zone
CREEP_RANGE = 12.0        # creep speed zone
STOP_RANGE = 5.0          # emergency full stop

# --- Speed Limits ---
SPEED_CRUISE = 2.5
SPEED_FAST = 3.5
SPEED_SLOW = 1.5
SPEED_CREEP = 0.6
SPEED_STOP = 0.0

# --- PID Heading Controller ---
KP_HEADING = 3.0
KI_HEADING = 0.02
KD_HEADING = 1.0
MAX_INTEGRAL = 20.0
MAX_TURN_RATE = 60.0      # deg/s

# --- Speed PID ---
KP_SPEED = 2.0
KD_SPEED = 0.5
MAX_ACCEL = 2.0           # m/s^2
MAX_DECEL = 4.0           # m/s^2 (braking is faster)

# --- Timing ---
SIM_HZ = 60.0             # simulation tick rate
MARKER_HZ = 20.0          # marker publish rate
GPS_HZ = 10.0             # GPS data publish rate
MARKER_INTERVAL = int(SIM_HZ / MARKER_HZ)  # ticks between marker publishes
GPS_INTERVAL = int(SIM_HZ / GPS_HZ)

LAT_ORIGIN = 43.825300
LON_ORIGIN = -70.330400
METERS_PER_DEG_LAT = 111320.0
METERS_PER_DEG_LON = 111320.0 * math.cos(math.radians(LAT_ORIGIN))

# Stale labels to clear on startup
STALE_LABELS = [
    'obstacle_4', 'obstacle_5', 'obstacle_6', 'obstacle_7',
    'detect_4', 'detect_5', 'detect_6', 'detect_7',
    'OBS_0', 'OBS_1', 'OBS_2', 'OBS_3', 'OBS_4',
    'obs_0', 'obs_1', 'obs_2', 'obs_3', 'obs_4',
    'detect_range_0', 'detect_range_1', 'detect_range_2', 'detect_range_3',
]


def angle_diff(a, b):
    """Shortest signed angle difference a - b, in degrees [-180, 180]."""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def clamp(val, lo, hi):
    return max(lo, min(hi, val))


class PIDController:
    """PID controller with anti-windup."""

    def __init__(self, kp, ki, kd, max_integral=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral = clamp(self.integral + error * dt,
                              -self.max_integral, self.max_integral)
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class MovingObstacle:
    """An obstacle that moves in a pattern."""

    def __init__(self, cx, cy, radius, pattern, speed, color='red'):
        self.cx = cx
        self.cy = cy
        self.radius = radius
        self.pattern = pattern
        self.speed = speed
        self.color = color
        self.x = cx
        self.y = cy
        self.prev_x = cx
        self.prev_y = cy
        self.vx = 0.0
        self.vy = 0.0
        self.orbit_radius = 30.0
        self.t = 0.0
        # Pre-compute circle points for marker (16 points for small obstacles)
        self.n_pts = 16
        self._angles = [2.0 * math.pi * k / self.n_pts for k in range(self.n_pts)]
        self._cos = [math.cos(a) for a in self._angles]
        self._sin = [math.sin(a) for a in self._angles]

    def update(self, dt):
        self.prev_x = self.x
        self.prev_y = self.y
        self.t += dt
        if self.pattern == 'circle':
            self.x = self.cx + self.orbit_radius * math.cos(self.speed * self.t)
            self.y = self.cy + self.orbit_radius * math.sin(self.speed * self.t)
        elif self.pattern == 'horizontal':
            self.x = self.cx + self.orbit_radius * math.sin(self.speed * self.t)
            self.y = self.cy
        elif self.pattern == 'vertical':
            self.x = self.cx
            self.y = self.cy + self.orbit_radius * math.sin(self.speed * self.t)
        elif self.pattern == 'diagonal':
            self.x = self.cx + self.orbit_radius * math.sin(self.speed * self.t)
            self.y = self.cy + self.orbit_radius * math.cos(self.speed * self.t * 0.7)
        if dt > 0:
            self.vx = (self.x - self.prev_x) / dt
            self.vy = (self.y - self.prev_y) / dt

    def body_polygon(self):
        """Generate polygon string for obstacle body."""
        pts = [f'{self.x + self.radius * self._sin[k]:.1f},'
               f'{self.y + self.radius * self._cos[k]:.1f}'
               for k in range(self.n_pts)]
        return ':'.join(pts)

    def ring_polygon(self, ring_radius):
        """Generate polygon string for detection ring."""
        pts = [f'{self.x + ring_radius * self._sin[k]:.0f},'
               f'{self.y + ring_radius * self._cos[k]:.0f}'
               for k in range(self.n_pts)]
        return ':'.join(pts)


# 4 moving obstacles (small radii)
OBSTACLES = [
    MovingObstacle(100, -35, 4, 'horizontal', 0.3, 'red'),
    MovingObstacle(160, -80, 5, 'circle', 0.2, 'orange'),
    MovingObstacle(60, -120, 3, 'vertical', 0.25, 'red'),
    MovingObstacle(180, -150, 6, 'diagonal', 0.15, 'orange'),
]


class FakeGpsObstacleNode(Node):
    def __init__(self):
        super().__init__('fake_gps_obstacles')

        # GPS publishers
        self.pub_x = self.create_publisher(Float64, '/gps/x', 10)
        self.pub_y = self.create_publisher(Float64, '/gps/y', 10)
        self.pub_hdg = self.create_publisher(Float64, '/gps/heading', 10)
        self.pub_spd = self.create_publisher(Float64, '/gps/speed', 10)
        self.pub_lat = self.create_publisher(Float64, '/gps/lat', 10)
        self.pub_lon = self.create_publisher(Float64, '/gps/lon', 10)

        # Obstacle publishers
        self.pub_alert = self.create_publisher(String, '/obstacle/alert', 10)
        self.pub_range = self.create_publisher(Float64, '/obstacle/range', 10)
        self.pub_count = self.create_publisher(Float64, '/obstacle/count', 10)
        self.pub_view = self.create_publisher(String, '/view_marker', 10)

        # USV state
        self.wpt_idx = 0
        self.x = float(WAYPOINTS[0][0])
        self.y = float(WAYPOINTS[0][1])
        self.heading = 0.0
        self.actual_speed = 0.0
        self.target_speed = 0.0
        self.deployed = False
        self.detections = {}
        self.tick_count = 0
        self.state = 'IDLE'
        self._eff_min_dist = 999.0

        # PID controllers
        self.heading_pid = PIDController(KP_HEADING, KI_HEADING, KD_HEADING,
                                         MAX_INTEGRAL)
        self.speed_pid = PIDController(KP_SPEED, 0.0, KD_SPEED)

        # Recovery state
        self.stopped_time = 0.0
        self.clear_time = 0.0

        # Pre-allocate messages
        self._float_msg = Float64()
        self._str_msg = String()

        dt = 1.0 / SIM_HZ
        self.dt = dt
        self.timer = self.create_timer(dt, self.tick)
        self.create_timer(2.0, self.deploy)

        # Clear stale markers from previous runs
        self._stale_timer = self.create_timer(0.5, self.clear_stale_markers)
        self._stale_cleared = False

        self.get_logger().info('Fake GPS + Advanced Obstacle Avoidance started')
        self.get_logger().info(f'  {len(WAYPOINTS)} waypoints, {len(OBSTACLES)} obstacles')
        self.get_logger().info(f'  Sim={SIM_HZ}Hz, Markers={MARKER_HZ}Hz, GPS={GPS_HZ}Hz')
        self.get_logger().info(f'  Zones: detect={DETECTION_RANGE}m, slow={SLOW_RANGE}m, '
                               f'creep={CREEP_RANGE}m, stop={STOP_RANGE}m')
        for i, obs in enumerate(OBSTACLES):
            self.get_logger().info(
                f'  OBS {i}: ({obs.cx},{obs.cy}) r={obs.radius}m '
                f'pattern={obs.pattern} speed={obs.speed}')

    def clear_stale_markers(self):
        """One-shot cleanup of stale markers from previous runs."""
        if self._stale_cleared:
            return
        self._stale_cleared = True
        for label in STALE_LABELS:
            msg = String()
            msg.data = f'label={label},active=false'
            self.pub_view.publish(msg)
        self.get_logger().info(f'Cleared {len(STALE_LABELS)} stale markers')
        self._stale_timer.cancel()

    def deploy(self):
        if not self.deployed:
            self.deployed = True
            self.wpt_idx = 1
            self.state = 'CRUISE'
            self.get_logger().info('DEPLOYED — advanced obstacle avoidance active')

    def get_obstacle_threats(self):
        """Get all obstacles with distance, predicted CPA, and threat level."""
        threats = []
        usv_vx = self.actual_speed * math.sin(math.radians(self.heading))
        usv_vy = self.actual_speed * math.cos(math.radians(self.heading))

        for i, obs in enumerate(OBSTACLES):
            dx = obs.x - self.x
            dy = obs.y - self.y
            dist = math.sqrt(dx * dx + dy * dy) - obs.radius

            # Predict CPA using relative velocity
            rel_vx = obs.vx - usv_vx
            rel_vy = obs.vy - usv_vy
            rel_speed_sq = rel_vx * rel_vx + rel_vy * rel_vy

            if rel_speed_sq > 0.01:
                t_cpa = clamp(-(dx * rel_vx + dy * rel_vy) / rel_speed_sq,
                              0.0, 8.0)
                pred_dx = dx + rel_vx * t_cpa
                pred_dy = dy + rel_vy * t_cpa
                cpa_dist = math.sqrt(pred_dx * pred_dx + pred_dy * pred_dy) - obs.radius
            else:
                t_cpa = 0.0
                cpa_dist = dist

            closing = (dx * obs.vx + dy * obs.vy) < 0 if dist < DETECTION_RANGE else False

            threats.append({
                'id': i, 'obs': obs, 'dist': max(0.0, dist),
                'cpa_dist': max(0.0, cpa_dist), 't_cpa': t_cpa,
                'closing': closing, 'dx': dx, 'dy': dy,
            })
        return threats

    def compute_avoidance_heading(self, threats, target_x, target_y):
        """Compute desired heading blending waypoint direction + obstacle repulsion."""
        dx = target_x - self.x
        dy = target_y - self.y

        repulse_x = 0.0
        repulse_y = 0.0
        for t in threats:
            eff_dist = min(t['dist'], t['cpa_dist'])
            if eff_dist < DETECTION_RANGE:
                norm = math.sqrt(t['dx'] ** 2 + t['dy'] ** 2)
                if norm < 0.1:
                    continue

                if eff_dist < STOP_RANGE:
                    strength = 12.0
                elif eff_dist < CREEP_RANGE:
                    strength = 6.0 * (1.0 - eff_dist / CREEP_RANGE)
                elif eff_dist < SLOW_RANGE:
                    strength = 3.0 * (1.0 - eff_dist / SLOW_RANGE)
                else:
                    strength = 1.0 * (1.0 - eff_dist / DETECTION_RANGE)

                if t['closing']:
                    strength *= 1.5

                repulse_x -= (t['dx'] / norm) * strength
                repulse_y -= (t['dy'] / norm) * strength

        blend_x = dx + repulse_x * 3.0
        blend_y = dy + repulse_y * 3.0
        return math.degrees(math.atan2(blend_x, blend_y)) % 360.0

    def compute_target_speed(self, threats):
        """Determine target speed based on closest obstacle zone."""
        min_dist = 999.0
        any_closing = False

        for t in threats:
            eff = min(t['dist'], t['cpa_dist'])
            if eff < min_dist:
                min_dist = eff
            if t['closing'] and t['dist'] < SLOW_RANGE:
                any_closing = True

        self._eff_min_dist = min_dist

        if min_dist < STOP_RANGE:
            return SPEED_STOP, 'STOPPED'
        elif min_dist < CREEP_RANGE:
            frac = (min_dist - STOP_RANGE) / (CREEP_RANGE - STOP_RANGE)
            return SPEED_CREEP * frac, 'CREEP'
        elif min_dist < SLOW_RANGE:
            frac = (min_dist - CREEP_RANGE) / (SLOW_RANGE - CREEP_RANGE)
            spd = SPEED_CREEP + (SPEED_SLOW - SPEED_CREEP) * frac
            if any_closing:
                spd *= 0.7
            return spd, 'SLOW'
        elif min_dist < DETECTION_RANGE:
            frac = (min_dist - SLOW_RANGE) / (DETECTION_RANGE - SLOW_RANGE)
            return SPEED_SLOW + (SPEED_CRUISE - SPEED_SLOW) * frac, 'CRUISE'
        else:
            return SPEED_CRUISE, 'CRUISE'

    def tick(self):
        self.tick_count += 1
        dt = self.dt

        # Update obstacle positions every tick for smooth motion
        for obs in OBSTACLES:
            obs.update(dt)

        # Publish markers at MARKER_HZ
        if self.tick_count % MARKER_INTERVAL == 0:
            self.publish_obstacle_markers()

        # --- USV navigation with advanced avoidance ---
        if self.deployed and self.wpt_idx < len(WAYPOINTS):
            tx, ty = WAYPOINTS[self.wpt_idx]
            ddx = tx - self.x
            ddy = ty - self.y
            dist_to_wpt = math.sqrt(ddx * ddx + ddy * ddy)

            threats = self.get_obstacle_threats()
            self.target_speed, new_state = self.compute_target_speed(threats)

            # Clear time tracking
            if self._eff_min_dist > DETECTION_RANGE * 1.1:
                self.clear_time += dt
            else:
                self.clear_time = 0.0

            # Speed boost when all-clear
            if self.clear_time > 5.0 and new_state == 'CRUISE':
                self.target_speed = SPEED_FAST
                new_state = 'FAST'

            # Log state transitions
            if new_state != self.state:
                old_state = self.state
                self.state = new_state
                self.get_logger().info(
                    f'STATE: {old_state} -> {new_state} '
                    f'(spd={self.target_speed:.1f}, nearest={self._eff_min_dist:.0f}m)')
                if new_state == 'STOPPED':
                    self.stopped_time = 0.0
                    self.heading_pid.reset()

            # STOPPED: hold position, rotate to find escape
            if self.state == 'STOPPED':
                self.stopped_time += dt
                self.actual_speed = 0.0
                if self.stopped_time > 1.5:
                    desired = self.compute_avoidance_heading(threats, tx, ty)
                    hdg_err = angle_diff(desired, self.heading)
                    turn = clamp(hdg_err, -MAX_TURN_RATE * dt, MAX_TURN_RATE * dt)
                    self.heading = (self.heading + turn) % 360.0
                    escape_ok = all(
                        t['dist'] >= STOP_RANGE * 1.5 or
                        abs(angle_diff(self.heading,
                            math.degrees(math.atan2(t['dx'], t['dy'])) % 360.0)) > 60
                        for t in threats
                    )
                    if escape_ok and self.stopped_time > 2.5:
                        self.target_speed = SPEED_CREEP
                        self.state = 'CREEP'
                        self.get_logger().info('ESCAPE: creeping forward')
            else:
                # PID heading control
                desired_heading = self.compute_avoidance_heading(threats, tx, ty)
                hdg_err = angle_diff(desired_heading, self.heading)
                pid_out = self.heading_pid.update(hdg_err, dt)
                turn = clamp(pid_out * dt, -MAX_TURN_RATE * dt, MAX_TURN_RATE * dt)
                self.heading = (self.heading + turn) % 360.0

            # Speed PID: smooth accel/decel
            speed_err = self.target_speed - self.actual_speed
            speed_cmd = self.speed_pid.update(speed_err, dt)
            if speed_cmd > 0:
                accel = min(speed_cmd, MAX_ACCEL * dt)
            else:
                accel = max(speed_cmd, -MAX_DECEL * dt)
            self.actual_speed = clamp(self.actual_speed + accel, 0.0, SPEED_FAST)

            # Move USV
            if self.actual_speed > 0.005:
                step = self.actual_speed * dt
                rad = math.radians(self.heading)
                mx = step * math.sin(rad)
                my = step * math.cos(rad)
                if dist_to_wpt < step * 2:
                    self.x, self.y = float(tx), float(ty)
                    self.wpt_idx += 1
                    if self.wpt_idx >= len(WAYPOINTS):
                        self.get_logger().info('Lawnmower path complete!')
                    else:
                        nx, ny = WAYPOINTS[self.wpt_idx]
                        self.get_logger().info(
                            f'WPT {self.wpt_idx}/{len(WAYPOINTS)}: -> ({nx}, {ny})')
                else:
                    self.x += mx
                    self.y += my

        # --- Detection & alerts (at GPS rate to avoid spam) ---
        if self.tick_count % GPS_INTERVAL == 0:
            self.publish_detections()
            self.publish_gps()

    def publish_detections(self):
        """Publish obstacle detection data."""
        threats = self.get_obstacle_threats()
        in_range = [(t['id'], t['dist'], t['closing']) for t in threats
                    if t['dist'] < DETECTION_RANGE]

        nearest_dist = min((t['dist'] for t in threats), default=999.0)
        msg = Float64()
        msg.data = nearest_dist
        self.pub_range.publish(msg)

        msg = Float64()
        msg.data = float(len(in_range))
        self.pub_count.publish(msg)

        for obs_id, d, closing in in_range:
            obs = OBSTACLES[obs_id]
            if obs_id not in self.detections:
                self.detections[obs_id] = True
                closing_str = ' [CLOSING]' if closing else ''
                alert = (f'OBSTACLE DETECTED: id={obs_id}, '
                         f'pos=({obs.x:.0f},{obs.y:.0f}), r={obs.radius}m, '
                         f'range={d:.1f}m, pattern={obs.pattern}{closing_str}')
                self.get_logger().warn(alert)
                msg = String()
                msg.data = alert
                self.pub_alert.publish(msg)
            elif d < CREEP_RANGE:
                status = 'STOPPED' if d < STOP_RANGE else 'CREEPING'
                msg = String()
                msg.data = (f'{status}: id={obs_id}, range={d:.1f}m, '
                            f'speed={self.actual_speed:.1f}m/s')
                self.pub_alert.publish(msg)
            elif d < SLOW_RANGE:
                msg = String()
                msg.data = (f'SLOWING: id={obs_id}, range={d:.1f}m, '
                            f'speed={self.actual_speed:.1f}m/s')
                self.pub_alert.publish(msg)

        for obs_id in list(self.detections.keys()):
            obs = OBSTACLES[obs_id]
            d = math.sqrt((self.x - obs.x) ** 2 + (self.y - obs.y) ** 2) - obs.radius
            if d > DETECTION_RANGE * 1.2:
                del self.detections[obs_id]

    def publish_gps(self):
        """Publish GPS data."""
        lat = LAT_ORIGIN + self.y / METERS_PER_DEG_LAT
        lon = LON_ORIGIN + self.x / METERS_PER_DEG_LON

        for pub, val in [
            (self.pub_x, self.x), (self.pub_y, self.y),
            (self.pub_hdg, self.heading), (self.pub_spd, self.actual_speed),
            (self.pub_lat, lat), (self.pub_lon, lon),
        ]:
            msg = Float64()
            msg.data = val
            pub.publish(msg)

    def publish_obstacle_markers(self):
        """Publish obstacle positions as VIEW_POLYGON at high rate."""
        for i, obs in enumerate(OBSTACLES):
            # Obstacle body
            msg = String()
            msg.data = (f'pts={{{obs.body_polygon()}}},'
                        f'label=obstacle_{i},'
                        f'edge_color={obs.color},'
                        f'fill_color={obs.color},fill_transparency=0.4,'
                        f'edge_size=2')
            self.pub_view.publish(msg)

            # Small detection boundary ring
            ring_r = obs.radius + 6.0
            msg2 = String()
            msg2.data = (f'pts={{{obs.ring_polygon(ring_r)}}},'
                         f'label=detect_{i},'
                         f'edge_color=yellow,edge_size=1,'
                         f'vertex_size=0')
            self.pub_view.publish(msg2)

        # Survey boundary (first time only)
        if self.tick_count <= MARKER_INTERVAL:
            msg = String()
            msg.data = ('pts={20,-20:220,-20:220,-170:20,-170},'
                        'label=survey_area,edge_color=cyan,'
                        'edge_size=2,vertex_size=0')
            self.pub_view.publish(msg)


def main():
    rclpy.init()
    node = FakeGpsObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
