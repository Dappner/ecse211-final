import threading
from utils.brick import Motor, EV3ColorSensor, EV3UltrasonicSensor, TouchSensor
import time
import logging
import color_matching as colo
from utils.sound import Song, Sound

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

# Constants
SQUARE_SIZE = 24  # cm per grid square (US Sensor)
ALIGNMENT_TOLERANCE = 5  # cm tolerance for wall distance verification (US Sensor)
WHEELBASE = 15  # cm, approximate distance between tracks (adjust as needed)
TURN_SPEED = 300  # degrees per second for turns (used as limit)
MOVE_SPEED = 500  # degrees per second for forward movement

NB_COLOR_SAMPLING = 20  # number of times the color sensor samples a color
TURN_CALIBRATION = 1.0  # Adjust after testing (e.g., 0.9 or 1.1)

# Mission Timing Constraints
MISSION_TIME_LIMIT = 180  # 3 minutes in seconds
TIME_BUFFER_RETURN = 30  # 30 seconds buffer for returning to base
EXPECTED_TIME_TO_BURNING_ROOM = 20  # Expected time to reach burning room

# Constants from testing
MOTOR_DPS_CONST = 180
SPEED_MODIFIER = 2
MOTOR_DPS = MOTOR_DPS_CONST * SPEED_MODIFIER
MOTOR_POWER = 30.5

# PATHS
# Hallway path (excluding room entry) (x,y)
HALLWAY_PATH = [
    (0, 0),
    (1, 0),
    (0, 1),
    (1, 1),
    (0, 2),
    (1, 2),
    (2, 2),
    (3, 2),
]  # (4,2) doesn't really matter)

# (x,y)
ENTRANCE = (3, 2)  # Position before entering the room
BURNING_ROOM_ENTRY = (3, 3)  # Entry point to burning room
# Room sweep pattern
BURNING_ROOM_SWEEP = [(2, 3), (3, 3), (4, 3), (2, 4), (3, 4), (4, 4)]
# Return path
RETURN_PATH = [(3, 2), (2, 2), (1, 2), (0, 2), (1, 1), (0, 1), (1, 0), (0, 0)]

MAX_GRID_ALIGNMENT_ATTEMPTS = 10
MAX_ENTRANCE_ALIGNMENT_ATTEMPTS = 5

# Direction constants
NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"

DIRECTION_VECTORS = {NORTH: (0, 1), EAST: (1, 0), SOUTH: (0, -1), WEST: (-1, 0)}

# Color constants
COLOR_BLACK = "black"
COLOR_ORANGE = "orange"
COLOR_RED = "red"
COLOR_GREEN = "green"
COLOR_YELLOW = "yellow"
COLOR_PURPLE = "purple"
COLOR_WHITE = "white"

# Burning room border
BURNING_ROOM_MIN_X = 2
BURNING_ROOM_MAX_X = 4
BURNING_ROOM_MIN_Y = 3
BURNING_ROOM_MAX_Y = 4


def is_in_burning_room(x, y):
    """Check if given coordinates are inside the burning room."""
    return (
        BURNING_ROOM_MIN_X <= x <= BURNING_ROOM_MAX_X
        and BURNING_ROOM_MIN_Y <= y <= BURNING_ROOM_MAX_Y
    )


class SirenController:
    """Handles the creation and control of the siren sound."""

    def __init__(self):
        self.siren_thread = None
        self.siren_active = False
        self.create_siren()

    def create_siren(self):
        """Create the siren sound using the Sound module"""
        # Create high pitch siren sound
        self.siren_high = Sound(
            duration=0.4, volume=70, pitch="A5", cutoff=0.05, fs=8000
        )

        # Create low pitch siren sound
        self.siren_low = Sound(
            duration=0.4, volume=70, pitch="E5", cutoff=0.05, fs=8000
        )

        # Create silence gap
        self.silence = Song.create_silence(seconds=0.1)

        # Create the full siren pattern
        self.siren = Song([self.siren_high, self.silence, self.siren_low, self.silence])
        self.siren.compile()

    def start(self):
        """Start playing the siren in a separate thread"""
        if self.siren_thread is not None and self.siren_thread.is_alive():
            logger.info("Siren already playing")
            return

        self.siren_active = True
        self.siren_thread = threading.Thread(target=self._play_siren)
        self.siren_thread.daemon = (
            True  # Allow the thread to be terminated when the program exits
        )
        self.siren_thread.start()
        logger.info("Siren started")

    def stop(self):
        """Stop the siren"""
        if self.siren_thread is not None:
            self.siren_active = False
            self.siren.stop()
            logger.info("Siren stopped")

    def _play_siren(self):
        """Play the siren repeatedly until stopped"""
        while self.siren_active:
            self.siren.play()
            time.sleep(self.siren.duration)
            # The siren will automatically stop when the duration is up,
            # but we need to play it again if the siren is still active
            if not self.siren_active:
                break


class DriveSystem:
    """Handles movement and orientation of the robot."""

    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Initialize motors
        self.left_motor.set_limits(dps=MOVE_SPEED)
        self.right_motor.set_limits(dps=MOVE_SPEED)

        # Position tracking
        self.position = [0, 0]  # [x, y]
        self.orientation = NORTH

        logger.info(
            f"Drive system initialized at position {self.position}, orientation {self.orientation}"
        )

    def reset_motors(self):
        """Stop all motors by setting DPS to 0."""
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)
        logger.info("Motors reset")

    def set_motors_turn_left(self):
        """Set motors for left turn using power settings."""
        self.left_motor.set_power(-MOTOR_POWER)
        self.right_motor.set_power(MOTOR_POWER)
        logger.debug("Motors set for left turn")

    def set_motors_turn_right(self):
        """Set motors for right turn using power settings."""
        self.left_motor.set_power(MOTOR_POWER)
        self.right_motor.set_power(-MOTOR_POWER)
        logger.debug("Motors set for right turn")

    def advance_blocks(self, number):
        """Move forward a specified number of blocks based on tested timing values."""
        logger.info(f"Advancing {number} blocks")
        self.left_motor.set_dps(MOTOR_DPS)
        self.right_motor.set_dps(MOTOR_DPS)
        time.sleep(2.28 * number)
        self.reset_motors()

        # Update position based on orientation
        dx, dy = DIRECTION_VECTORS[self.orientation]
        self.update_position(dx * number, dy * number)
        logger.info(f"Advanced {number} blocks to position {self.position}")

    def move_forward_slightly(self, time_seconds=0.5):
        """Move forward slightly for small adjustments."""
        # TODO: We need to ensure that this doesn't throw everything off
        logger.debug(f"Moving forward slightly for {time_seconds} seconds")
        self.left_motor.set_power(MOTOR_POWER / 2)
        self.right_motor.set_power(MOTOR_POWER / 2)
        time.sleep(time_seconds)
        self.reset_motors()

    def move_backward_slightly(self, time_seconds=0.5):
        """Move backward slightly for small adjustments."""
        # TODO: We need to ensure that this doesn't throw everything off
        logger.debug(f"Moving forward slightly for {time_seconds} seconds")
        self.left_motor.set_power(-MOTOR_POWER / 2)
        self.right_motor.set_power(-MOTOR_POWER / 2)
        time.sleep(time_seconds)
        self.reset_motors()

    def turn_90_left(self, times=1):
        """Turn left 90 degrees (or multiple of 90) based on tested timing values."""
        logger.info(f"Turning left {90 * times} degrees")
        self.set_motors_turn_left()
        time.sleep(2.32 * times)
        self.reset_motors()

        # Update orientation
        directions = [NORTH, WEST, SOUTH, EAST]
        current_index = directions.index(self.orientation)
        new_index = (current_index + times) % 4
        self.orientation = directions[new_index]
        logger.info(f"New orientation: {self.orientation}")

    def turn_90_right(self, times=1):
        """Turn right 90 degrees (or multiple of 90) based on movement pattern."""
        logger.info(f"Turning right {90 * times} degrees")
        self.left_motor.set_power(MOTOR_POWER)
        self.right_motor.set_power(-MOTOR_POWER)
        time.sleep(2.32 * times)
        self.reset_motors()

        # Update orientation
        directions = [NORTH, EAST, SOUTH, WEST]
        current_index = directions.index(self.orientation)
        new_index = (current_index + times) % 4
        self.orientation = directions[new_index]
        logger.info(f"New orientation: {self.orientation}")

    def turn_slightly_left(self, time_seconds=0.2):
        """Make a small left turn adjustment without changing orientation."""
        logger.debug(f"Turning slightly left for {time_seconds} seconds")
        self.set_motors_turn_left()
        time.sleep(time_seconds)
        self.reset_motors()

    def turn_slightly_right(self, time_seconds=0.2):
        """Make a small right turn adjustment without changing orientation."""
        logger.debug(f"Turning slightly right for {time_seconds} seconds")
        self.set_motors_turn_right()
        time.sleep(time_seconds)
        self.reset_motors()

    def turn(self, target_direction):
        """Turn to face a target direction (NORTH, SOUTH, EAST, WEST)."""
        if self.orientation == target_direction:
            logger.info(f"Already facing {target_direction}, no turn needed")
            return

        old_orientation = self.orientation

        directions = [NORTH, EAST, SOUTH, WEST]
        current_index = directions.index(self.orientation)
        target_index = directions.index(target_direction)

        # Calculate the shortest turn
        diff = (target_index - current_index) % 4
        if diff == 1:  # Turn right once
            self.turn_90_right()
        elif diff == 2:  # Turn 180 degrees (two right or two left)
            self.turn_90_right(2)
        elif diff == 3:  # Turn left once
            self.turn_90_left()

        logger.info(f"Turned from {old_orientation} to {self.orientation}")

    def update_position(self, dx, dy):
        """Update robot's position based on movement."""
        old_position = self.position.copy()
        self.position[0] += dx
        self.position[1] += dy
        logger.info(f"Position updated from {old_position} to {self.position}")

    def stop(self):
        """Stop all movement."""
        self.reset_motors()
        logger.info("Motors stopped")


class SensorSystem:
    """Handles all sensors and color detection."""

    def __init__(self, left_color, right_color, ultrasonic, touch_sensor):
        self.left_color = left_color
        self.right_color = right_color
        # self.ultrasonic = ultrasonic
        self.ultrasonic = None
        self.touch_sensor = touch_sensor

        # Initialize sensors
        self.left_color.wait_ready()
        self.right_color.wait_ready()
        self.touch_sensor.wait_ready()

        # Ultrasonic may be optional
        if self.ultrasonic:
            try:
                self.ultrasonic.wait_ready()
            except:
                logger.warning("Ultrasonic sensor not available or not responding")
                self.ultrasonic = None

        logger.info("Sensor system initialized")

    def get_color_left(self):
        """Get RGB values from left color sensor."""
        rgb = []
        for i in range(NB_COLOR_SAMPLING):
            rgb.append(self.left_color.get_rgb())

        match = colo.match_unknown_color(rgb)
        logger.info(f"LEFT COLOR: {match}")
        return match

    def get_color_right(self):
        """Get RGB values from right color sensor."""
        rgb = []
        for i in range(NB_COLOR_SAMPLING):
            rgb.append(self.right_color.get_rgb())

        match = colo.match_unknown_color(rgb)
        logger.info(f"RIGHT COLOR: {match}")
        return match

    def check_color_match(self):
        """Returns Tuple (bool Match, left color, right color)"""
        left_color = self.get_color_left()
        right_color = self.get_color_right()
        return left_color == right_color, left_color, right_color

    def check_for_color(self, target_color):
        """Check if either sensor detects the target color."""
        left_color = self.get_color_left()
        right_color = self.get_color_right()

        return left_color == target_color or right_color == target_color

    def check_for_fire(self):
        """Check for red color (fire)."""
        return self.check_for_color(COLOR_RED)

    def check_for_entrance(self):
        """Check for orange color (entrance line)."""
        return self.check_for_color(COLOR_ORANGE)

    def check_for_furniture(self):
        """Check for green color (furniture)."""
        return self.check_for_color(COLOR_GREEN)

    def is_emergency_pressed(self):
        """Check if emergency button is pressed."""
        return self.touch_sensor.is_pressed()

    def get_wall_distance(self):
        """Get distance to nearest wall using ultrasonic sensor."""
        if not self.ultrasonic:
            logger.warning("Ultrasonic sensor not available")
            return None

        dist = self.ultrasonic.get_cm()
        logger.debug(f"Ultrasonic distance: {dist} cm")
        return dist


class FireExtinguisher:
    """Handles the fire extinguishing mechanism."""

    def __init__(self, dropper_motor):
        self.dropper_motor = dropper_motor
        self.fires_extinguished = 0

    def drop_cube(self):
        """Drop a foam cube to extinguish a fire."""
        logger.info("Dropping foam cube to extinguish fire")
        # TODO: Verify this works
        # Implementation of dropping mechanism
        self.dropper_motor.set_dps(-360)  # Rotate 180 degrees
        time.sleep(0.7)
        self.dropper_motor.set_dps(0)

        logger.info("Please pick up the Cube!")
        time.sleep(1)  # Wait for cube to drop

        self.fires_extinguished += 1
        logger.info(
            f"Fire extinguished. Total fires extinguished: {self.fires_extinguished}"
        )
        return True

    def get_fires_extinguished(self):
        """Return the count of fires extinguished."""
        return self.fires_extinguished


class Navigation:
    """Handles navigation logic, combining drive and sensors."""

    def __init__(self, drive_system, sensor_system):
        self.drive = drive_system
        self.sensors = sensor_system

    def align_with_grid(self):
        """Align robot with black grid lines using both color sensors."""
        logger.info("Aligning with grid...")

        attempts = 0
        max_attempts = MAX_GRID_ALIGNMENT_ATTEMPTS

        while attempts < max_attempts:
            on_black, side = self.sensors.is_on_black_line()

            if on_black:
                if side == "both":
                    logger.info("Aligned with black grid line")
                    self.drive.stop()
                    return True
                elif side == "left":
                    logger.debug("Left sensor on black, turning right slightly")
                    self.drive.turn_slightly_right(0.1)
                elif side == "right":
                    logger.debug("Right sensor on black, turning left slightly")
                    self.drive.turn_slightly_left(0.1)
            else:
                logger.debug("No black detected, moving forward slowly")
                self.drive.move_forward_slightly(0.2)

            self.drive.stop()
            attempts += 1

        logger.warning(f"Failed to align with grid after {max_attempts} attempts")
        return False

    def align_with_entrance(self):
        """Attempt to align with the orange entrance line."""
        logger.info("Trying to align with orange entrance line...")

        attempts = 0

        while attempts < MAX_ENTRANCE_ALIGNMENT_ATTEMPTS:
            found_orange, side = self.sensors.check_for_entrance()

            if found_orange:
                if side == "left":
                    logger.info("Orange line detected on left, adjusting position")
                    self.drive.turn_slightly_left(0.1)
                    self.drive.move_forward_slightly(0.3)
                elif side == "right":
                    logger.info("Orange line detected on right, adjusting position")
                    self.drive.turn_slightly_right(0.1)
                    self.drive.move_forward_slightly(0.3)

                # Check if both sensors are now on the orange line
                found_orange_again, new_side = self.sensors.check_for_entrance()
                if found_orange_again and new_side != side:
                    # Both sensors likely on the line now
                    logger.info("Successfully aligned with orange entrance line")
                    return True
            else:
                # No orange detected, make small search movements
                logger.info("No orange line detected, searching...")

                # Try small turns and forward movements to find the line
                if attempts % 2 == 0:
                    self.drive.turn_slightly_left(0.1)
                else:
                    self.drive.turn_slightly_right(0.1)

                self.drive.move_forward_slightly(0.2)

            attempts += 1

        logger.warning("Failed to align with entrance after multiple attempts")
        return False

    def verify_position(self):
        """Verify position using ultrasonic sensor and walls after alignment."""
        self.align_with_grid()
        dist = self.sensors.get_wall_distance()
        if dist is None:
            logger.warning("Ultrasonic sensor failed to return a distance")
            return False

        # Expected distance based on orientation and position
        if self.drive.orientation == EAST:  # East
            expected = (4 - self.drive.position[0]) * SQUARE_SIZE
        elif self.drive.orientation == NORTH:  # North
            expected = (4 - self.drive.position[1]) * SQUARE_SIZE
        elif self.drive.orientation == WEST:  # West
            expected = self.drive.position[0] * SQUARE_SIZE
        else:  # SOUTH
            expected = self.drive.position[1] * SQUARE_SIZE

        match = abs(dist - expected) < ALIGNMENT_TOLERANCE
        logger.info(
            f"Position verification: measured {dist} cm, expected {expected} cm, {'valid' if match else 'invalid'}"
        )
        return match

    def navigate_to(self, target_x, target_y, avoid_obstacles=True):
        """Navigate to a target position with grid alignment and obstacle avoidance."""
        dx = target_x - self.drive.position[0]
        dy = target_y - self.drive.position[1]
        logger.info(
            f"Navigating to ({target_x}, {target_y}) from {self.drive.position}, dx={dx}, dy={dy}"
        )

        # First move in X direction if needed
        if dx > 0:
            self.drive.turn(EAST)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(dx)
        elif dx < 0:
            self.drive.turn(WEST)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(-dx)

        # Then move in Y direction if needed
        if dy > 0:
            self.drive.turn(NORTH)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(dy)
        elif dy < 0:
            self.drive.turn(SOUTH)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(-dy)

        logger.info(f"Navigation complete, at position {self.drive.position}")

    def _avoid_obstacle(self):
        """Simple obstacle avoidance strategy."""
        logger.warning("Furniture detected, implementing avoidance")
        # Simple avoidance - turn left, move 1 block, turn right, move forward 1
        # TODO:
        current_orientation = self.drive.orientation
        self.drive.turn_90_left()
        self.drive.advance_blocks(1)
        self.drive.turn_90_right()
        self.drive.advance_blocks(1)
        # Return to original orientation
        self.drive.turn(current_orientation)

    def identify_room(self):
        """Identify the current room based on color patterns."""
        # Take multiple samples for reliability
        room_colors = []
        for _ in range(3):
            left_color = self.sensors.get_color_left()
            right_color = self.sensors.get_color_right()
            room_colors.extend([left_color, right_color])
            time.sleep(0.1)

        # Count occurrences of each color
        color_counts = {}
        for color in room_colors:
            if color in color_counts:
                color_counts[color] += 1
            else:
                color_counts[color] = 1

        logger.info(f"Room color analysis: {color_counts}")

        # Determine room type
        if COLOR_PURPLE in color_counts:
            return "burning room"
        elif COLOR_YELLOW in color_counts:
            return "avoid room"
        elif COLOR_WHITE in color_counts:
            return "hallway"

        return "unknown"


class MissionControl:
    """Manages the overall mission execution and state."""

    def __init__(self, drive, sensors, navigation, extinguisher, siren):
        self.drive = drive
        self.sensors = sensors
        self.navigation = navigation
        self.extinguisher = extinguisher
        self.siren = siren

        # Mission state
        self.mission_running = False
        self.emergency_stop_thread = None
        self.fires_detected = 0
        self.mission_start_time = None

    def start_emergency_monitor(self):
        """Start monitoring for emergency stop signal."""
        self.emergency_stop_thread = threading.Thread(
            target=self._monitor_emergency_stop
        )
        self.emergency_stop_thread.daemon = True
        self.emergency_stop_thread.start()
        logger.info("Emergency stop monitor started")

    def _monitor_emergency_stop(self):
        """Background thread for emergency stop monitoring."""
        while self.mission_running:
            if self.sensors.is_emergency_pressed():
                logger.warning("EMERGENCY STOP ACTIVATED")
                self.stop_mission()
                break
            time.sleep(0.1)

    def navigate_hallway(self):
        """Navigate through the hallway to the entrance."""
        logger.info("Starting hallway navigation")

        for x, y in HALLWAY_PATH[1:]:  # Skip first position
            if not self.mission_running:
                break

            logger.info(f"Navigating to checkpoint ({x}, {y})")
            self.navigation.navigate_to(x, y)

            # Check time limit
            if self._check_time_limit():
                break

        elapsed_time = time.time() - self.mission_start_time

        if elapsed_time > EXPECTED_TIME_TO_BURNING_ROOM:
            logger.warning(
                f"Navigation to entrance took longer than expected: {elapsed_time:.1f}s vs {EXPECTED_TIME_TO_BURNING_ROOM}s expected"
            )
        # Orient to face entrance
        self.drive.turn(NORTH)

        # Check if at entrance
        is_at_entrance = (
            self.drive.position[0] == ENTRANCE[0]
            and self.drive.position[1] == ENTRANCE[1]
        )

        if is_at_entrance:
            logger.info("Reached entrance position")
            return True
        else:
            logger.warning(
                f"Failed to reach entrance, current position: {self.drive.position}"
            )
            return False

    def enter_burning_room(self):
        """Enter the burning room if at entrance and orange line detected."""
        # First try to find and align with the orange entrance line
        found_orange, _ = self.sensors.check_for_entrance()

        if not found_orange:
            logger.info("Orange line not immediately visible, attempting to align")
            aligned = self.navigation.align_with_entrance()

            if not aligned:
                logger.warning(
                    "Failed to align with orange entrance, will attempt entry anyway"
                )

        # Enter room (once aligned or after attempting alignment)
        logger.info("Entering burning room")
        self.drive.advance_blocks(1)  # Move one block north to enter

        # Verify we're in the burning room
        room_type = self.navigation.identify_room()
        logger.info(f"Entered room identified as: {room_type}")

        if room_type == "burning room":
            logger.info("Successfully entered burning room")
            return True
        elif room_type == "avoid room":
            raise Exception(
                "For some reason it thinks we are in the avoid room. Color sensors!?"
            )
            logger.warning("Entered wrong room (avoid room)! Exiting immediately")
            # Return to hallway
            self.drive.turn(NORTH)
            self.drive.advance_blocks(1)
            return False
        else:
            logger.warning(f"Room type uncertain: {room_type}. Proceeding with caution")
            # Assume we're in the burning room but log the uncertainty
            return True

    def sweep_burning_room(self):
        """Search the burning room for fires and extinguish them."""
        logger.info("Starting systematic room sweep")

        for x, y in BURNING_ROOM_SWEEP:
            if not self.mission_running:
                break

            logger.info(f"Sweeping position ({x}, {y})")
            self.navigation.navigate_to(x, y)

            # Check for fire
            if self.sensors.check_for_fire():
                logger.info(f"Fire detected at position {self.drive.position}")
                self.fires_detected += 1

                # TODO: Move to drop on top of the relevant sensor
                # Drop
                self.extinguisher.drop_cube()
                # Wait one second (so we can move it out of the way and continue)

                # If we've found both fires, we can stop
                if self.extinguisher.get_fires_extinguished() >= 2:
                    logger.info("Both fires extinguished, ending sweep")
                    break

            # Check time limit
            if self._check_time_limit():
                break

        logger.info(
            f"Room sweep complete. Fires found: {self.fires_detected}, extinguished: {self.extinguisher.get_fires_extinguished()}"
        )
        return self.extinguisher.get_fires_extinguished() > 0

    def return_to_base(self):
        """Navigate back to the starting point."""
        logger.info("Beginning return journey to base")

        # First exit the room if needed
        if self.drive.position[1] >= 3:  # If in burning room (row 3 or 4)
            logger.info("Exiting burning room")
            self.navigation.navigate_to(BURNING_ROOM_ENTRY[0], BURNING_ROOM_ENTRY[1])
            self.navigation.navigate_to(ENTRANCE[0], ENTRANCE[1])

        # Follow return path
        for x, y in RETURN_PATH:
            if not self.mission_running:
                break

            # Skip if already at this position
            if self.drive.position[0] == x and self.drive.position[1] == y:
                continue

            logger.info(f"Returning via checkpoint ({x}, {y})")
            self.navigation.navigate_to(x, y)

            # Check time limit
            if self._check_time_limit():
                break

        # Check if successfully returned to base
        is_at_base = self.drive.position[0] == 0 and self.drive.position[1] == 0

        if is_at_base:
            logger.info("Successfully returned to base station (0,0)")
            return True
        else:
            logger.warning(
                f"Failed to return to base, stopped at {self.drive.position}"
            )
            return False

    def _check_time_limit(self):
        """Check if mission time limit (3 minutes) has been reached."""
        if self.mission_start_time is None:
            return

        elapsed_time = time.time() - self.mission_start_time
        if elapsed_time >= 180:  # 3 minutes
            logger.warning("Mission time limit (3 minutes) reached!")
            return True
        return False

    def stop_mission(self):
        """Stop the mission and all activities."""
        logger.info("Stopping mission")
        self.mission_running = False
        self.drive.stop()
        self.siren.stop()

    def drop_on_sensor(self, sensor: str):
        ROTATION_SECONDS = 0.9
        INCREMENT = 0.05
        FORWARD_MOVE = 0.4
        if sensor == "RIGHT":
            self.drive.turn_slightly_right(ROTATION_SECONDS)
            self.drive.move_forward_slightly(FORWARD_MOVE + 0.1)
            self.extinguisher.drop_cube()
            self.drive.move_backward_slightly(FORWARD_MOVE)
            self.drive.turn_slightly_left(ROTATION_SECONDS - INCREMENT)
            self.drive.move_forward_slightly(FORWARD_MOVE - 0.1)
        else:
            self.drive.turn_slightly_left(ROTATION_SECONDS)
            self.drive.move_forward_slightly(FORWARD_MOVE + 0.1)
            self.extinguisher.drop_cube()
            self.drive.move_backward_slightly(FORWARD_MOVE)
            self.drive.turn_slightly_right(ROTATION_SECONDS - INCREMENT)
            self.drive.move_forward_slightly(FORWARD_MOVE - 0.1)

    def run_mission(self):
        """Execute the full firefighting mission."""
        # Initialize mission
        self.mission_running = True
        self.mission_start_time = time.time()
        self.fires_detected = 0

        try:
            # Start emergency monitor
            self.start_emergency_monitor()

            # Start siren
            self.siren.start()

            # Navigate through hallway to entrance
            if self.navigate_hallway():
                # At entrance, try to enter burning room
                if self.enter_burning_room():
                    # In burning room, stop siren
                    self.siren.stop()

                    # Search for and extinguish fires
                    # TODO: Actually Sweep the Burning Room
                    # success = self.sweep_burning_room()

                    # Return to base
                    self.return_to_base()
                else:
                    logger.error("Failed to enter burning room")
            else:
                logger.error("Failed to reach entrance")

            # Mission complete
            elapsed_time = time.time() - self.mission_start_time
            logger.info(f"Mission completed in {elapsed_time:.1f} seconds")
            logger.info(
                f"Fires detected: {self.fires_detected}, extinguished: {self.extinguisher.get_fires_extinguished()}"
            )

        except Exception as e:
            logger.error(f"Error during mission: {e}")
        finally:
            # Ensure proper shutdown
            self.stop_mission()


class FirefighterRobot:
    """Main robot class that integrates all components."""

    def __init__(self):
        left_motor = Motor("B")
        right_motor = Motor("D")
        dropper_motor = Motor("A")
        left_color = EV3ColorSensor(1)
        right_color = EV3ColorSensor(2)
        ultrasonic = EV3UltrasonicSensor(3)
        touch_sensor = TouchSensor(4)

        # Create sub-systems
        self.drive_system = DriveSystem(left_motor, right_motor)
        self.sensor_system = SensorSystem(
            left_color, right_color, ultrasonic, touch_sensor
        )
        self.extinguisher = FireExtinguisher(dropper_motor)

        self.siren = SirenController()

        self.navigation = Navigation(self.drive_system, self.sensor_system)

        self.mission_control = MissionControl(
            self.drive_system,
            self.sensor_system,
            self.navigation,
            self.extinguisher,
            self.siren,
        )

        logger.info("FirefighterRobot fully initialized and ready")

    def run_mission(self):
        """Execute the firefighting mission."""
        self.mission_control.run_mission()

    def calibration_test(self):
        """Run tests of basic components."""
        logger.info("Starting calibration tests")

        try:
            # Test siren
            logger.info("Testing siren")
            self.siren.start()
            time.sleep(3)
            self.siren.stop()

            # Test sensors (before moving so we can put something down)
            logger.info("Testing sensors")
            left_color = self.sensor_system.get_color_left()
            right_color = self.sensor_system.get_color_right()
            logger.info(f"Color readings - Left: {left_color}, Right: {right_color}")

            if left_color == COLOR_RED:
                self.mission_control.drop_on_sensor("LEFT")
            elif right_color == COLOR_RED:
                self.mission_control.drop_on_sensor("RIGHT")

            time.sleep(1)

            if self.sensor_system.ultrasonic:
                distance = self.sensor_system.get_wall_distance()
                logger.info(f"Distance to wall: {distance} cm")

            # Test dropper
            logger.info("Testing fire extinguisher")
            self.extinguisher.drop_cube()

            logger.info("Calibration tests complete")

        except Exception as e:
            logger.error(f"Error during calibration: {e}")
        finally:
            # Clean shutdown
            self.drive_system.stop()
            self.siren.stop()

    def run_simple_path(self):
        """Run a simple path to test navigation."""
        logger.info("Running simple test path")

        try:
            # Start siren
            self.siren.start()

            # Navigate first few steps
            for x, y in HALLWAY_PATH[1:4]:  # Just the first few steps
                logger.info(f"Navigating to ({x}, {y})")
                self.navigation.navigate_to(x, y)

            # Return to start
            logger.info("Returning to start")
            self.navigation.navigate_to(0, 0)

            # Stop siren
            self.siren.stop()

        except Exception as e:
            logger.error(f"Error during test path: {e}")
        finally:
            self.drive_system.stop()
            self.siren.stop()


def main():
    """Main entry point for the firefighter robot mission."""
    robot = FirefighterRobot()

    try:
        robot.run_mission()
    except KeyboardInterrupt:
        logger.info("Mission interrupted by user")
    finally:
        # Stop everything
        if robot:
            robot.drive_system.stop()
            robot.siren.stop()


def calibration_testing():
    """Test basic robot functionality."""
    robot = FirefighterRobot()
    robot.calibration_test()


def simple_path_test():
    """Run a simple navigation test."""
    robot = FirefighterRobot()
    robot.run_simple_path()


if __name__ == "__main__":
    # main()
    calibration_testing()
    # simple_path_test()
