from coppeliasim_zmqremoteapi_client import *
import time

# ---------------- Connect ----------------
client = RemoteAPIClient()
sim = client.getObject('sim')
if not sim:
    print ("Not Connected")
else:

    print("Connected!")# ---------------- Robot ----------------
pioneer = sim.getObject('/PioneerP3DX')
left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

# Sensors
front_sensors = [
    sim.getObject('/PioneerP3DX/ultrasonicSensor[2]'),
    sim.getObject('/PioneerP3DX/ultrasonicSensor[3]'),
    sim.getObject('/PioneerP3DX/ultrasonicSensor[4]')
]
left_sensor = sim.getObject('/PioneerP3DX/ultrasonicSensor[1]')
right_sensor = sim.getObject('/PioneerP3DX/ultrasonicSensor[5]')

# Target (red cube)
SAFE_DISTANCE_TO_RED = 0.5
try:
    red_cube = sim.getObject('/Goal')
    target_found = True
except Exception:
    print("Red cube not found!")
    red_cube = None
    target_found = False

# ---------------- Motor Functions ----------------
def set_speed(left, right):
    sim.setJointTargetVelocity(left_motor, left)
    sim.setJointTargetVelocity(right_motor, right)

def stop_robot():
    set_speed(0, 0)

def move_forward(duration=0.15, speed=0.4):
    set_speed(speed, speed)
    sim.step()
    time.sleep(duration)
    stop_robot()

def turn_left(duration=0.25, speed=0.3):
    set_speed(-speed, speed)
    sim.step()
    time.sleep(duration)
    stop_robot()

def turn_right(duration=0.25, speed=0.3):
    set_speed(speed, -speed)
    sim.step()
    time.sleep(duration)
    stop_robot()

# ---------------- Distance to Target ----------------
def distance_to_target(obj_handle):
    pos_robot = sim.getObjectPosition(pioneer, -1)
    pos_target = sim.getObjectPosition(obj_handle, -1)
    dx = pos_target[0] - pos_robot[0]
    dy = pos_target[1] - pos_robot[1]
    return (dx**2 + dy**2)**0.5

# ---------------- Sensors ----------------
SAFE_FRONT = 0.7
SAFE_SIDE = 0.35

def obstacle(sensor, threshold):
    detection, distance, *_ = sim.readProximitySensor(sensor)
    return detection and distance < threshold

def check_sensors():
    front_blocked = any(obstacle(s, SAFE_FRONT) for s in front_sensors)
    left_blocked = obstacle(left_sensor, SAFE_SIDE)
    right_blocked = obstacle(right_sensor, SAFE_SIDE)
    return front_blocked, left_blocked, right_blocked


# ---------------- Memory ----------------
visited_paths = set()
prev_pos = None


def store_path(prev, curr):
    """Store path from prev -> curr"""
    if prev is None:
        return
    path = (round(prev[0], 2), round(prev[1], 2), round(curr[0], 2), round(curr[1], 2))
    visited_paths.add(path)


def has_been_here_from(prev, curr):
    """Check if we already traveled this path"""
    if prev is None:
        return False
    path = (round(prev[0], 2), round(prev[1], 2), round(curr[0], 2), round(curr[1], 2))
    return path in visited_paths


# ---------------- Main Loop ----------------
print("Starting wall-following navigation with path memory...")
for step in range(5000):
    stop_robot()
    sim.step()
    time.sleep(0.04)

    pos = sim.getObjectPosition(pioneer, -1)

    # Save the path from previous to current position
    store_path(prev_pos, pos)

    front, left, right = check_sensors()

    # Stop at red cube
    if target_found and distance_to_target(red_cube) < SAFE_DISTANCE_TO_RED + 0.1:
        move_forward(duration=0.05)  # small step
        stop_robot()
        print("Reached red cube! Stopping.")
        break

    # --- Decide move based on sensors and memory ---
    options = []
    # Approximate next positions
    next_forward = (pos[0] + 0.1, pos[1])
    next_left = (pos[0], pos[1] + 0.1)
    next_right = (pos[0], pos[1] - 0.1)

    if not front and not has_been_here_from(pos, next_forward):
        options.append("forward")
    if not left and not has_been_here_from(pos, next_left):
        options.append("left")
    if not right and not has_been_here_from(pos, next_right):
        options.append("right")

    # --- Make decision ---
    if "left" in options:
        print(f"Step {step}: Pos={pos}, Front={front}, Left={left}, Right={right}")
        print("Decision: Turn left and move forward")
        turn_left(duration=0.25)
        turn_left(duration=0.25)
        turn_left(duration=0.25)

        move_forward()
    elif "forward" in options:
        print(f"Step {step}: Pos={pos}, Front={front}, Left={left}, Right={right}")
        print("Decision: Move forward")
        move_forward()
    elif "right" in options:
        print(f"Step {step}: Pos={pos}, Front={front}, Left={left}, Right={right}")
        print("Decision: Turn right and move forward")
        turn_right(duration=0.25)
        move_forward()
    else:
        # Dead end → turn 180
        print(f"Step {step}: Pos={pos}, Front={front}, Left={left}, Right={right}")
        print("Dead end: Turning 180°")
        turn_left(duration=0.5)
        move_forward()

    # Save current pos as prev for next step
    prev_pos = pos
