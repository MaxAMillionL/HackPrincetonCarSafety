"""
CARLA + Arduino Air-Quality Pull-Over Demo
- Reads air-quality from Arduino over serial (background thread)
- If poor air quality detected, car takes over, finds roadside spot, pulls over, parks, locks user controls
- After LOCK_DURATION, returns control
"""

import carla
import serial
import threading
import time
import math
import random
import sys
try:
    import pygame
    from pygame.locals import K_w, K_s, K_a, K_d, K_q
except Exception:
    pygame = None

# ====== CONFIG ======
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
TIMEOUT = 1000.0

SERIAL_PORT = 'COM5'          # <-- change for your system, e.g. '/dev/ttyUSB0'
BAUD_RATE = 9600
SERIAL_POLL_HZ = 5

AIR_QUALITY_THRESHOLD = 300   # adjust based on your Arduino sensor calibration (analog value)
LOCK_DURATION = 15            # seconds to keep the vehicle parked and user locked

VEHICLE_BP_FILTER = 'vehicle.*'  # blueprint filter
SPAWN_AT_RANDOM = True

# Controller tuning (rough starting values)
STEER_LOOKAHEAD = 6.0        # meters for pure pursuit lookahead
STEER_K = 1.0                # steering gain multiplier
PID_KP = 1.0
PID_KI = 0.0
PID_KD = 0.2

MAX_THROTTLE = 0.6
BRAKE_FORCE = 0.6

# ====== GLOBAL STATE ======
latest_air_quality = None
airlock = threading.Lock()
stop_threads = False

# Whether the simulation is under autonomous control (user locked out)
autonomous_lock = threading.Event()  # when set => autonomous takeover active (user locked)

# ====== SERIAL THREAD ======
def serial_reader_thread(serial_port, baud_rate):
    global latest_air_quality, stop_threads
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"[serial] Connected to {serial_port} at {baud_rate} baud")
    except Exception as e:
        print(f"[serial] Could not open serial port {serial_port}: {e}")
        return

    while not stop_threads:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                time.sleep(1.0 / SERIAL_POLL_HZ)
                continue
            # Expect numeric reading; try to parse
            try:
                val = int(''.join(ch for ch in line if ch.isdigit()))
                with airlock:
                    latest_air_quality = val
            except ValueError:
                # ignore non-numeric lines
                pass
        except Exception as e:
            print("[serial] read error:", e)
            break

    ser.close()
    print("[serial] thread exiting")

# ====== UTILS ======
def clamp(x, a, b):
    return max(a, min(b, x))

def vec3_to_tuple(v):
    return (v.x, v.y, v.z)

def distance(a, b):
    return a.distance(b)

# ====== SIMPLE PID FOR SPEED CONTROL ======
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, dt=0.05, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.int = 0.0
        self.prev_e = None
        self.min_out, self.max_out = output_limits

    def reset(self):
        self.int = 0.0
        self.prev_e = None

    def run(self, target, current):
        e = target - current
        self.int += e * self.dt
        der = 0.0 if (self.prev_e is None) else (e - self.prev_e) / self.dt
        self.prev_e = e
        out = self.kp * e + self.ki * self.int + self.kd * der
        if self.min_out is not None:
            out = max(self.min_out, out)
        if self.max_out is not None:
            out = min(self.max_out, out)
        return out

# ====== PURE PURSUIT STEERING ======
def get_forward_point(vehicle_transform, look_ahead):
    """
    compute a point ahead of the vehicle in world coordinates (vehicle local x forward)
    """
    loc = vehicle_transform.location
    forward = vehicle_transform.get_forward_vector()
    return carla.Location(x = loc.x + forward.x * look_ahead,
                          y = loc.y + forward.y * look_ahead,
                          z = loc.z + forward.z * look_ahead)

def pure_pursuit_steer(vehicle_transform, target_location, look_ahead=STEER_LOOKAHEAD):
    """
    Returns a steering angle [-1, 1] for the vehicle to head toward target_location.
    Uses geometric pure pursuit style.
    """
    # vehicle position and forward vector
    pos = vehicle_transform.location
    forward = vehicle_transform.get_forward_vector()

    # vector from vehicle to target in world coords
    to_target = target_location - pos
    # convert to local coordinates relative to vehicle orientation
    # local_x = forward dot to_target
    # local_y = right_vector dot to_target
    right = vehicle_transform.get_right_vector()
    local_x = forward.x * to_target.x + forward.y * to_target.y + forward.z * to_target.z
    local_y = right.x * to_target.x + right.y * to_target.y + right.z * to_target.z

    if local_x == 0 and local_y == 0:
        return 0.0
    # heading to target relative to vehicle front
    angle = math.atan2(local_y, local_x)
    # steering proportional to angle and a gain
    steer = clamp((angle * STEER_K), -1.0, 1.0)
    return steer

# ====== PATH PLANNING: build a short list of waypoints to parking spot ======
def compute_waypoint_path(map_api, start_loc, dest_loc, step=2.0):
    """
    Create a short sequence of waypoints between start and destination by sampling along map road.
    It's a pragmatic/simple approach: find waypoint at start, then iteratively move toward dest.
    """
    wp_start = map_api.get_waypoint(start_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    wp_end = map_api.get_waypoint(dest_loc, project_to_road=True, lane_type=carla.LaneType.Driving)
    path = []
    # simple: walk from start waypoint via `next` toward end (limited depth)
    # if on same road structure, next() will produce contiguous waypoints
    current = wp_start
    path.append(current)
    max_iters = 200
    it = 0
    while it < max_iters:
        it += 1
        next_wps = current.next(step)
        if not next_wps:
            break
        current = next_wps[0]
        path.append(current)
        if current.transform.location.distance(wp_end.transform.location) < 2.5:
            break
    return path

# ====== FIND ROAD-SIDE PARKING SPOT ======
def find_roadside_parking_transform(world_map, vehicle, forward_distance=8.0, lateral_offset=3.0):
    """
    Choose a point ahead in the driving lane and shift it to the right (lateral offset) to simulate roadside.
    If lane on right is available, will aim into that; otherwise simply offset from lane center.
    """
    transform = vehicle.get_transform()
    wp = world_map.get_waypoint(transform.location)
    # pick a waypoint forward along the lane
    next_wps = wp.next(forward_distance)
    if not next_wps:
        target_wp = wp
    else:
        target_wp = next_wps[0]

    # compute lateral offset to the right in world coordinates
    right = target_wp.transform.get_right_vector()
    park_loc = carla.Location(
        x = target_wp.transform.location.x + right.x * lateral_offset,
        y = target_wp.transform.location.y + right.y * lateral_offset,
        z = target_wp.transform.location.z
    )
    # rotate to align with road (yaw same as target waypoint)
    park_transform = carla.Transform(park_loc, target_wp.transform.rotation)
    return park_transform

# ====== DRIVE ALONG WAYPOINTS ======
def follow_waypoints(vehicle, waypoint_list, speed_controller, top_speed=5.0, stop_on_close=True, stop_distance=1.4):
    """
    Follow a list of carla.Waypoint objects using pure-pursuit steering and PID throttle control.
    top_speed is in m/s (CARLA uses m/s for physics)
    Returns when the final waypoint is reached or when autonomous_lock is cleared externally.
    """
    pid = speed_controller
    pid.reset()
    reached_final = False
    idx = 0
    # convert waypoints to transforms for target positions
    targets = [wp.transform.location for wp in waypoint_list]
    while idx < len(targets):
        if autonomous_lock.is_set() is False:
            # autonomous aborted externally
            print("[autonomy] takeover aborted, releasing.")
            break
        target = targets[idx]
        transform = vehicle.get_transform()
        v = vehicle.get_velocity()
        speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
        dist = transform.location.distance(target)

        # steering
        steer = pure_pursuit_steer(transform, target)

        # throttle PID to reach desired speed (we set target speed lower when approaching final)
        # if close to target, lower desired speed
        desired_speed = top_speed
        if idx >= len(targets)-1:
            desired_speed = min(2.0, top_speed)  # slower on final approach
        # if far but approaching, keep top_speed
        throttle_cmd = pid.run(desired_speed, speed)
        throttle = clamp(throttle_cmd, -1.0, 1.0)

        # convert throttle cmd to VehicleControl (positive throttle, otherwise brake)
        if throttle >= 0:
            vehicle.apply_control(carla.VehicleControl(throttle=clamp(throttle, 0.0, MAX_THROTTLE),
                                                      steer=steer,
                                                      brake=0.0))
        else:
            vehicle.apply_control(carla.VehicleControl(throttle=0.0,
                                                      steer=steer,
                                                      brake=min(abs(throttle), BRAKE_FORCE)))

        # small threshold to advance to next target
        if dist < stop_distance:
            idx += 1
        time.sleep(0.05)

    # stop vehicle at end
    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=False))
    time.sleep(0.2)
    return

# ====== KEYBOARD CONTROL (USER) ======
def run_pygame_keyboard_controller(vehicle):
    """
    Simple keyboard control loop. If autonomous_lock is set, inputs are ignored (locked).
    WASD for throttle/steer. Q to quit.
    """
    if pygame is None:
        print("[input] pygame not installed; skipping keyboard controller.")
        return
    pygame.init()
    screen = pygame.display.set_mode((300,100))
    pygame.display.set_caption("CARLA: User Control (W/S/A/D) - Q to quit")
    clock = pygame.time.Clock()
    throttle = 0.0
    steer = 0.0
    brake = 0.0

    running = True
    while running and not stop_threads:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # keydown/keyup handled via key.get_pressed
        keys = pygame.key.get_pressed()
        if autonomous_lock.is_set():
            # ignore user commands; keep braking lightly to ensure safety
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.5))
        else:
            throttle = 1.0 if keys[K_w] else 0.0
            brake = 1.0 if keys[K_s] else 0.0
            steer = -0.6 if keys[K_a] else (0.6 if keys[K_d] else 0.0)
            if keys[K_q]:
                # signal quit
                pygame.quit()
                sys.exit(0)
            vehicle.apply_control(carla.VehicleControl(throttle=throttle*0.6, steer=steer, brake=brake*0.8))
        clock.tick(30)

# ====== MAIN HIGH-LEVEL FLOW ======
def main():
    global latest_air_quality, stop_threads

    # Start serial thread
    ser_thread = threading.Thread(target=serial_reader_thread, args=(SERIAL_PORT, BAUD_RATE), daemon=True)
    ser_thread.start()

    # Connect to CARLA
    client = carla.Client(CARLA_HOST, CARLA_PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    map_api = world.get_map()
    blueprint_lib = world.get_blueprint_library()

    # Choose vehicle blueprint and spawn
    vehicles = blueprint_lib.filter(VEHICLE_BP_FILTER)
    vehicle_bp = random.choice(vehicles)
    spawn_point = random.choice(world.get_map().get_spawn_points()) if SPAWN_AT_RANDOM else world.get_map().get_spawn_points()[0]
    vehicle = None
    try:
        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    except Exception as e:
        print("Spawn failed:", e)
        stop_threads = True
        return

    print(f"[carla] spawned {vehicle.type_id} at {spawn_point.location}")

    # Start keyboard controller in a background thread (optional)
    input_thread = None
    if pygame is not None:
        input_thread = threading.Thread(target=run_pygame_keyboard_controller, args=(vehicle,), daemon=True)
        input_thread.start()
    else:
        print("[input] Pygame not available, user keyboard control disabled (autonomy still works).")

    # Build a PID controller instance for speed control
    pid = PIDController(kp=PID_KP, ki=PID_KI, kd=PID_KD, dt=0.05, output_limits=(-1.0, 1.0))

    try:
        print("[main] starting monitoring loop. Press Ctrl+C to quit.")
        while True:
            # Read latest sensor reading
            with airlock:
                aq = latest_air_quality

            if aq is not None:
                print(f"[sensor] air quality = {aq}")

            # Condition: poor air quality detected AND not already autonomous
            if aq is not None and aq > AIR_QUALITY_THRESHOLD and not autonomous_lock.is_set():
                print("[main] poor air quality threshold exceeded -> initiating autonomous pull-over.")
                # set autonomous flag so user input is ignored
                autonomous_lock.set()

                # Find a realistic roadside transform
                parking_transform = find_roadside_parking_transform(map_api, vehicle,
                                                                    forward_distance=8.0,
                                                                    lateral_offset=3.5)

                # Create waypoint path from current location to parking location
                start_loc = vehicle.get_transform().location
                path_wp = compute_waypoint_path(map_api, start_loc, parking_transform.location, step=3.0)
                # if path is trivial, add target waypoint manually using the parking transform
                if not path_wp:
                    # fallback: create a fake waypoint list via map.get_waypoint on current and parking
                    path_wp = []
                    try:
                        path_wp.append(map_api.get_waypoint(start_loc))
                        path_wp.append(map_api.get_waypoint(parking_transform.location))
                    except Exception:
                        pass

                # Follow the path under autonomy
                follow_waypoints(vehicle, path_wp, pid, top_speed=4.0)
                # Final small approach to the exact parking transform
                print("[main] final alignment toward exact parking transform...")
                # Slowly move straight toward exact parking pose
                t0 = time.time()
                while vehicle.get_transform().location.distance(parking_transform.location) > 1.2:
                    if not autonomous_lock.is_set():
                        break
                    transform = vehicle.get_transform()
                    v = vehicle.get_velocity()
                    speed = math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
                    steer = pure_pursuit_steer(transform, parking_transform.location)
                    throttle_cmd = pid.run(1.0, speed)  # crawl in slowly
                    throttle_cmd = clamp(throttle_cmd, -1.0, 0.5)
                    if throttle_cmd >= 0:
                        vehicle.apply_control(carla.VehicleControl(throttle=min(throttle_cmd, 0.3), steer=steer, brake=0.0))
                    else:
                        vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=steer, brake=min(abs(throttle_cmd), BRAKE_FORCE)))
                    time.sleep(0.05)
                    # safety timeout in case stuck
                    if time.time() - t0 > 10.0:
                        break

                # Park and engage handbrake
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                print("[main] vehicle parked and handbrake engaged. Locking user for", LOCK_DURATION, "seconds.")

                # Hold lock for LOCK_DURATION seconds (autonomy keeps control)
                park_end = time.time() + LOCK_DURATION
                while time.time() < park_end:
                    # maintain handbrake
                    vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
                    time.sleep(0.2)

                # Release lock and return control to user
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False))
                autonomous_lock.clear()
                print("[main] lock released. User controls restored.")

                # Optionally reset latest_air_quality to avoid immediate retrigger; user can tune this behavior
                with airlock:
                    latest_air_quality = None

            # small main loop sleep
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("[main] received KeyboardInterrupt - exiting")

    finally:
        # cleanup
        stop_threads = True
        autonomous_lock.clear()
        print("[main] destroying vehicle...")
        if vehicle is not None:
            try:
                vehicle.destroy()
            except Exception:
                pass
        print("[main] done. Exiting.")

if __name__ == "__main__":
    main()
