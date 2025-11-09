import carla
import random
import time
import serial
import pygame
from pygame.locals import K_ESCAPE, K_w, K_s, K_a, K_d, KEYDOWN, QUIT

# ====== CONFIGURATION ======
AIR_QUALITY_THRESHOLD = 300   # Adjust based on your sensor range
LOCKOUT_DURATION = 10         # seconds for cooldown
SERIAL_PORT = "COM5"          # Change to your Arduino COM port
BAUD_RATE = 9600
TIMEOUT = 1

# ====== INITIALIZATION ======
def connect_arduino():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        return ser
    except Exception as e:
        print(f"[Warning] Could not connect to Arduino: {e}")
        return None

def get_air_quality(ser):
    if ser and ser.in_waiting > 0:
        try:
            value = ser.readline().decode().strip()
            return int(value)
        except:
            return None
    return None

# ====== MAIN LOGIC ======
def main():
    # Connect to CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.filter('model3')[0]  # Tesla Model 3
    spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if vehicle is None:
        print("❌ Failed to spawn vehicle. Try restarting the map.")
        return
    print(f"✅ Spawned {vehicle.type_id}")

    # Attach a chase camera
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')
    camera_transform = carla.Transform(carla.Location(x=-6, z=3))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    spectator = world.get_spectator()

    # Setup Pygame for manual control
    pygame.init()
    screen = pygame.display.set_mode((400, 100))
    pygame.display.set_caption("CARLA: User Control")
    clock = pygame.time.Clock()

    control = carla.VehicleControl()
    manual_mode = True
    locked_out = False
    lockout_timer = 0

    ser = connect_arduino()

    print("🚗 Ready for manual driving. Use W/A/S/D. ESC to quit.")

    try:
        while True:
            # Update spectator to follow the car
            spectator.set_transform(
                vehicle.get_transform().transform(carla.Transform(carla.Location(x=-8, z=3)))
            )

            # --- Read air quality from Arduino ---
            aq_value = get_air_quality(ser)
            if aq_value:
                print(f"Air Quality: {aq_value}")
                if aq_value > AIR_QUALITY_THRESHOLD and not locked_out:
                    print("⚠️ Poor air quality detected — enabling autonomous safety mode.")
                    manual_mode = False
                    locked_out = True
                    vehicle.set_autopilot(True)
                    lockout_timer = time.time()

            # --- Lockout countdown ---
            if locked_out and time.time() - lockout_timer > LOCKOUT_DURATION:
                print("✅ Safety cooldown complete — returning control to user.")
                locked_out = False
                manual_mode = True
                vehicle.set_autopilot(False)

            # --- Manual control ---
            for event in pygame.event.get():
                if event.type == QUIT:
                    return
                elif event.type == KEYDOWN and event.key == K_ESCAPE:
                    return

            keys = pygame.key.get_pressed()
            if manual_mode:
                control.throttle = 0
                control.steer = 0
                control.brake = 0

                if keys[K_w]:
                    control.throttle = 0.6
                if keys[K_s]:
                    control.brake = 0.5
                if keys[K_a]:
                    control.steer = -0.4
                if keys[K_d]:
                    control.steer = 0.4

                vehicle.apply_control(control)

            clock.tick(30)

    finally:
        print("Cleaning up actors...")
        camera.destroy()
        vehicle.destroy()
        if ser:
            ser.close()
        pygame.quit()
        print("✅ Simulation ended.")

if __name__ == '__main__':
    main()
