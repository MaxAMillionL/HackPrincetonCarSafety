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

# ====== SERIAL HANDLING ======
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
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                value = int(line)
                return value
        except ValueError:
            pass
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

    # Setup Pygame for manual control + HUD
    pygame.init()
    screen = pygame.display.set_mode((400, 150))
    pygame.display.set_caption("CARLA: User Control + Air Quality HUD")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("Arial", 28, bold=True)
    small_font = pygame.font.SysFont("Arial", 20)

    control = carla.VehicleControl()
    manual_mode = True
    locked_out = False
    lockout_timer = 0

    ser = connect_arduino()
    aq_value = 0

    print("🚗 Ready for manual driving. Use W/A/S/D. ESC to quit.")

    try:
        while True:
            # --- Update spectator to follow the car ---
            vehicle_transform = vehicle.get_transform()
            cam_location = vehicle_transform.transform(carla.Location(x=-8, z=3))
            cam_rotation = vehicle_transform.rotation
            spectator.set_transform(carla.Transform(cam_location, cam_rotation))

            # --- Read air quality from Arduino ---
            new_aq_value = get_air_quality(ser)
            if new_aq_value is not None:
                aq_value = new_aq_value
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

            # --- Draw HUD ---
            screen.fill((20, 20, 20))
            hud_color = (0, 255, 0) if aq_value < 200 else (255, 200, 0) if aq_value < AIR_QUALITY_THRESHOLD else (255, 80, 80)
            text = font.render(f"Air Quality: {aq_value}", True, hud_color)
            status = "Manual Control" if manual_mode else "Autonomous Safety Mode"
            status_color = (100, 255, 100) if manual_mode else (255, 120, 0)
            mode_text = small_font.render(status, True, status_color)

            screen.blit(text, (40, 40))
            screen.blit(mode_text, (40, 90))
            pygame.display.flip()

            clock.tick(30)
            time.sleep(0.05)

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
