"""
Centralized constants and tuning of constant during run time.
"""

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

# ============= HARDWARE Ports =============
# Motor ports
LEFT_MOTOR_PORT = "B"
RIGHT_MOTOR_PORT = "D"
DROPPER_MOTOR_PORT = "A"

# Sensor ports
LEFT_COLOR_PORT = 1
RIGHT_COLOR_PORT = 2
ULTRASONIC_PORT = 3
TOUCH_PORT = 4

# ============= Motor Control Constants =============
# Basic movement parameters
MOTOR_POWER = 30.5  # Power percentage (0-100)
MOTOR_DPS = 180     # Degrees per second for speed-based control
SPEED_MODIFIER = 2  # Multiplier for MOTOR_DPS

# Time-based movement constants
FORWARD_TIME_PER_BLOCK = 2.28  # Time in seconds to move one block forward
TURN_90_TIME = 2.32            # Time in seconds for a 90-degree turn

# ============= Environment Constants =============
BLOCK_SIZE = 24  # cm per grid block
ALIGNMENT_TOLERANCE = 5  # cm tolerance for wall distance verification

# ============= Sampling Constants =============
NB_COLOR_SAMPLING = 20  # Number of samples for color sensor readings

# ============= Mission Constants =============
MISSION_TIME_LIMIT = 180  # Mission time limit in seconds (3 minutes)

# ============= Map Constants =============

FIRE_STATION = (0,0)
# Hallway path (excluding room entry) (x,y)
HALLWAY_PATH = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 2)]

ENTRANCE = (3, 2)  # Position before entering the room
BURNING_ROOM_ENTRY = (3, 3)  # Entry point to burning room

# Room sweep pattern
BURNING_ROOM_SWEEP = [(2, 3), (3, 3), (4, 3), (2, 4), (3, 4), (4, 4)]

# Return path
RETURN_PATH = HALLWAY_PATH[::-1]


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

# Burning room boundaries
BURNING_ROOM_MIN_X = 2
BURNING_ROOM_MAX_X = 4
BURNING_ROOM_MIN_Y = 3
BURNING_ROOM_MAX_Y = 4

# ============= Monte Carlo Localization Constants =============
# Number of particles for Monte Carlo localization
MCL_PARTICLE_COUNT = 100
# Standard deviation for motion model (in grid units)
MCL_MOTION_NOISE = 0.1
# Standard deviation for sensor model (in cm)
MCL_SENSOR_NOISE = 5.0
# Weight threshold for resampling
MCL_RESAMPLING_THRESHOLD = 0.5

# ============= Calibration Constants =============
MAX_GRID_ALIGNMENT_ATTEMPTS = 10
MAX_ENTRANCE_ALIGNMENT_ATTEMPTS = 5




class RuntimeConstants:
    """
    Runtime-adjustable constants that can be tuned during program execution.
    Use this class when you need to adjust parameters during runtime based on calibration.
    """

    def __init__(self):
        # Initialize with default values from the module constants
        self.motor_power = MOTOR_POWER
        self.motor_dps = MOTOR_DPS
        self.speed_modifier = SPEED_MODIFIER
        self.forward_time_per_block = FORWARD_TIME_PER_BLOCK
        self.turn_90_time = TURN_90_TIME

    def adjust_power(self, new_power):
        """Adjust motor power constant."""
        self.motor_power = new_power
        return self.motor_power

    def adjust_speed(self, new_dps):
        """Adjust motor speed in degrees per second."""
        self.motor_dps = new_dps
        return self.motor_dps

    def adjust_forward_time(self, new_time):
        """Adjust time required to move one block forward."""
        self.forward_time_per_block = new_time
        return self.forward_time_per_block

    def adjust_turn_time(self, new_time):
        """Adjust time required for a 90-degree turn."""
        self.turn_90_time = new_time
        return self.turn_90_time

    def calibrate_from_test(self, test_results):
        """
        Adjust parameters based on test results.

        Args:
            test_results: Dictionary with calibration results
        """
        if 'motor_power' in test_results:
            self.motor_power = test_results['motor_power']

        if 'forward_time' in test_results:
            self.forward_time_per_block = test_results['forward_time']

        if 'turn_time' in test_results:
            self.turn_90_time = test_results['turn_time']

        return {
            'motor_power': self.motor_power,
            'forward_time_per_block': self.forward_time_per_block,
            'turn_90_time': self.turn_90_time
        }

def is_in_burning_room(x, y):
    """Check if coordinates are in the burning room."""
    return (BURNING_ROOM_MIN_X <= x <= BURNING_ROOM_MAX_X and
            BURNING_ROOM_MIN_Y <= y <= BURNING_ROOM_MAX_Y)