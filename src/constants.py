"""
Centralized constants and tuning of constant during run time.
"""
# ============= HARDWARE Ports =============
# Motor ports
LEFT_MOTOR_PORT = "B"
RIGHT_MOTOR_PORT = "D"
DROPPER_MOTOR_PORT = "A"

# Sensor ports
LEFT_COLOR_PORT = 1
RIGHT_COLOR_PORT = 2
ULTRASONIC_PORT = 4
TOUCH_PORT = 3

# ============= Motor Control Constants =============
# Basic movement parameters
MOTOR_POWER = 30.5  # Power percentage (0-100)
MOTOR_DPS = 360  # Degrees per second for speed-based control
SPEED_MODIFIER = 2  # Multiplier for MOTOR_DPS

# Time-based movement constants
FORWARD_TIME_PER_BLOCK = 2.28  # Time in seconds to move one block forward
TURN_90_TIME = 2.32  # Time in seconds for a 90-degree turn

DROPPER_MOTOR_TIME = 0.7
DROPPER_MOTOR_DPS = 360

POSITION_TOLERANCE = 0.6

# ============= Environment Constants =============
BLOCK_SIZE = 25  # cm per grid block
ALIGNMENT_TOLERANCE = 5  # cm tolerance for wall distance verification

# ============= Sampling Constants =============
NB_COLOR_SAMPLING = 20  # Number of samples for color sensor readings

# ============= Map / Grid Constants ============= (x,y)
GRID_WIDTH = 5
GRID_HEIGHT = 5

# 1 is hallway, 2 is burning room, 3 is avoid room. (flipped upside down because 0,0 is top left)
GRID_MAP = [
    [1, 1, 3, 3, 3],
    [1, 1, 3, 3, 3],
    [1, 1, 1, 1, 0],
    [3, 3, 2, 2, 2],
    [3, 3, 2, 2, 2]
]

FIRE_STATION = (0, 0)

HALLWAY_PATH = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 2)]

ENTRANCE = (3, 2)  # Pos before burning room
BURNING_ROOM_ENTRY = (3, 3)  # First square of burning room

# SWeep of burning room (left to right circle)
BURNING_ROOM_SWEEP = [(2, 3), (2, 4), (3, 4), (4, 4), (4, 3), (3, 3), (2, 3)]

# Return path (opposite of path there)
RETURN_PATH = HALLWAY_PATH[::-1]

VALID_NEIGHBORS = {}
for y in range(GRID_HEIGHT):
    for x in range(GRID_WIDTH):
        if GRID_MAP[y][x] > 0:  # If valid position
            neighbors = []
            # Check all four directions
            for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT and
                        GRID_MAP[ny][nx] > 0):
                    neighbors.append((nx, ny))
            VALID_NEIGHBORS[(x, y)] = neighbors

# ============= Monte Carlo Localization Constants =============
# Number of particles for Monte Carlo localization
MCL_PARTICLE_COUNT = 100
# Standard deviation for motion model (in grid units)
MCL_MOTION_NOISE = 0.2
# Standard deviation for sensor model (in cm)
MCL_SENSOR_NOISE = 5.0
# Weight threshold for resampling
MCL_RESAMPLING_THRESHOLD = 0.5

# ============= Calibration Constants =============
MAX_GRID_ALIGNMENT_ATTEMPTS = 25
MAX_ENTRANCE_ALIGNMENT_ATTEMPTS = 5

# =========== Timing Related Constants ========

EXPECTED_TIME_TO_BURNING_ROOM = 20

# ============= Direction Constants =============
NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"

DIRECTION_VECTORS = {
    NORTH: (0, 1),
    EAST: (1, 0),
    SOUTH: (0, -1),
    WEST: (-1, 0)
}

# ============= Color Constants =============
COLOR_BLACK = "black"
COLOR_ORANGE = "orange"
COLOR_RED = "red"
COLOR_GREEN = "green"
COLOR_YELLOW = "yellow"
COLOR_PURPLE = "purple"
COLOR_WHITE = "white"
