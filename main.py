import os
import threading
import sys
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

# Sound
sys.path.append(os.path.dirname(os.path.realpath(__file__)))

# Constants from testing
MOTOR_DPS_CONST = 180
SPEED_MODIFIER = 2
MOTOR_DPS = MOTOR_DPS_CONST * SPEED_MODIFIER
MOTOR_POWER = 30.5


# PATHS
# Hallway path (excluding room entry) (x,y)
HALLWAY_PATH = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2)]
# (x,y)
ENTRANCE = (3, 2)  # Position before entering the room
BURNING_ROOM_ENTRY = (3, 3)  # Entry point to burning room
# Room sweep pattern
BURNING_ROOM_SWEEP = [(2, 3), (3, 3), (4, 3), (2, 4), (3, 4), (4, 4)]
# Return path
RETURN_PATH = [(3, 2), (2, 2), (1, 2), (0, 2), (1, 1), (0, 1), (1, 0), (0, 0)]


MAX_GRID_ALIGNMENT_ATTEMPTS = 10

NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"


DIRECTION_VECTORS = {NORTH: (0, 1), EAST: (1, 0), SOUTH: (0, -1), WEST: (-1, 0)}

COLOR_BLACK = "black"
COLOR_ORANGE = "orange"
COLOR_RED = "red"
COLOR_GREEN = "green"
COLOR_YELLOW = "yellow"
COLOR_PURPLE = "purple"
COLOR_WHITE = "white"


class FirefighterRobot:
    def __init__(self):
        self.dropper_motor = Motor("C")
        self.left_motor = Motor("B")
        self.right_motor = Motor("D")

        self.left_color = EV3ColorSensor(1)
        self.right_color = EV3ColorSensor(2)
        self.ultrasonic = EV3UltrasonicSensor(3)
        self.touch_sensor = TouchSensor(4)

        # Siren Stuff
        self.siren_thread = None
        self.siren_active = False
        self.create_siren()

        # Initializing Limits and Constants
        self.left_motor.set_limits(dps=MOVE_SPEED)
        self.right_motor.set_limits(dps=MOVE_SPEED)
        self.position = [0, 0]  # [x, y]
        self.orientation = NORTH

        # Mission state
        self.mission_running = False
        self.emergency_stop_thread = None
        self.fires_detected = 0
        self.fires_extinguished = 0
        self.mission_start_time = None

        # Initializing Limits and Constants
        self.left_motor.set_limits(dps=MOVE_SPEED)
        self.right_motor.set_limits(dps=MOVE_SPEED)
        self.position = [0, 0]  # [x, y]
        self.orientation = NORTH

        # Waiting Ready
        self.left_color.wait_ready()
        self.right_color.wait_ready()
        self.touch_sensor.wait_ready()
        # self.ultrasonic.wait_ready()

        logger.info(
            f"Robot initialized at position {self.position}, orientation {self.orientation}"
        )

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

    def start_siren(self):
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

    def stop_siren(self):
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

    # def turn(self, angle_deg):
    #     """Turn in place by angle_deg (positive = right, negative = left)."""
    #     degrees = TURN_CALIBRATION * (WHEELBASE * math.pi * abs(angle_deg)) / (360 * 2)
    #     logger.info(f"Turning {angle_deg} degrees ({degrees:.1f} motor degrees)")
    #     if angle_deg > 0:  # Right turn
    #         self.left_motor.set_position_relative(degrees)
    #         self.right_motor.set_position_relative(-degrees)
    #     else:  # Left turn
    #         self.left_motor.set_position_relative(-degrees)
    #         self.right_motor.set_position_relative(degrees)
    #     self.left_motor.wait_is_stopped()
    #     self.right_motor.wait_is_stopped()
    #     old_orientation = self.orientation
    #     self.orientation = (self.orientation + angle_deg) % 360
    #     logger.info(
    #         f"Turn completed: orientation updated from {old_orientation} to {self.orientation}"
    #     )

    def stop(self):
        """Stop all movement."""
        self.reset_motors()
        logger.info("Motors stopped")

    def get_color_left(self):
        """Get RGB values from left color sensor."""
        rgb = []
        for i in range(NB_COLOR_SAMPLING):
            rgb.append(self.left_color.get_rgb())

        match = colo.match_unknown_color(rgb)
        logger.info("LEFT COLOR: ", match)
        return match

    def get_color_right(self):
        """Get RGB values from right color sensor."""
        rgb = []
        for i in range(NB_COLOR_SAMPLING):
            rgb.append(self.right_color.get_rgb())

        match = colo.match_unknown_color(rgb)
        logger.info("RIGHT COLOR: ", match)
        return match

    def check_color_match(self):
        """Returns Tuple (bool Match, left color, right color)"""
        left_color = self.get_color_left()
        right_color = self.get_color_right()
        return left_color == right_color, left_color, right_color

    def align_with_grid(self):
        """Align robot with black grid lines using both color sensors."""
        logger.info("Aligning with grid...")

        attempts = 0
        max_attempts = MAX_GRID_ALIGNMENT_ATTEMPTS

        while attempts < max_attempts:
            left_color = self.get_color_left()
            right_color = self.get_color_right()

            left_black = left_color == COLOR_BLACK
            right_black = right_color == COLOR_BLACK

            if left_black and right_black:
                logger.info("Aligned with black grid line")
                self.stop()
                return True
            elif left_black:
                logger.debug("Left sensor on black, turning right slightly")
                self.left_motor.set_power(MOTOR_POWER / 2)
                self.right_motor.set_power(-MOTOR_POWER / 2)
                time.sleep(0.1)
            elif right_black:
                logger.debug("Right sensor on black, turning left slightly")
                self.left_motor.set_power(-MOTOR_POWER / 2)
                self.right_motor.set_power(MOTOR_POWER / 2)
                time.sleep(0.1)
            else:
                logger.debug("No black detected, moving forward slowly")
                self.left_motor.set_power(MOTOR_POWER / 2)
                self.right_motor.set_power(MOTOR_POWER / 2)
                time.sleep(0.2)

            self.stop()
            attempts += 1

        logger.warning(f"Failed to align with grid after {max_attempts} attempts")
        return False

    def update_position(self, dx, dy):
        """Update robot's position based on movement."""
        old_position = self.position.copy()
        self.position[0] += dx
        self.position[1] += dy
        logger.info(f"Position updated from {old_position} to {self.position}")

    def get_wall_distances(self):
        """Get distance to nearest wall using ultrasonic sensor (cm)."""
        dist = self.ultrasonic.get_cm()
        logger.debug(f"Ultrasonic distance: {dist} cm")
        return dist

    def verify_position(self):
        """Optional backup: Verify position using ultrasonic sensor and walls after alignment."""
        self.align_with_grid()
        dist = self.get_wall_distances()
        if dist is None:
            logger.warning("Ultrasonic sensor failed to return a distance")
            return False

        # Expected distance based on orientation and position
        if self.orientation == EAST:  # East
            expected = (4 - self.position[0]) * SQUARE_SIZE
        elif self.orientation == NORTH:  # North
            expected = (4 - self.position[1]) * SQUARE_SIZE
        elif self.orientation == WEST:  # West
            expected = self.position[0] * SQUARE_SIZE
        else:  # SOUTH
            expected = self.position[1] * SQUARE_SIZE

        match = abs(dist - expected) < ALIGNMENT_TOLERANCE
        logger.info(
            f"Position verification: measured {dist} cm, expected {expected} cm, {'valid' if match else 'invalid'}"
        )

        return match

    def navigate_to(self, target_x, target_y):
        """Navigate to a target position with grid alignment."""
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        logger.info(
            f"Navigating to ({target_x}, {target_y}) from {self.position}, dx={dx}, dy={dy}"
        )

        # First move in X direction
        if dx > 0:
            self.turn(EAST)
            self.advance_blocks(dx)
            # self.align_with_grid()
        elif dx < 0:
            self.turn(WEST)
            self.advance_blocks(-dx)
            # self.align_with_grid()

        # Then move in Y direction
        if dy > 0:
            self.turn(NORTH)
            self.advance_blocks(dy)
            # self.align_with_grid()
        elif dy < 0:
            self.turn(SOUTH)
            self.advance_blocks(-dy)
            # self.align_with_grid()

        logger.info(f"Navigation complete, at position {self.position} \n")

    def identify_room(self):
        """Identify the Current Room"""
        # Take multiple samples to improve reliability
        left_colors = []
        right_colors = []

        for _ in range(3):
            left_colors.append(self.get_color_left())
            right_colors.append(self.get_color_right())
            time.sleep(0.1)

        # Most common color detection
        left_color = max(set(left_colors), key=left_colors.count)
        right_color = max(set(right_colors), key=right_colors.count)

        logger.info(f"Room identification - Left: {left_color}, Right: {right_color}")

        if COLOR_PURPLE in [left_color, right_color]:
            return "burning room"
        elif COLOR_YELLOW in [left_color, right_color]:
            return "avoid room"
        elif COLOR_WHITE in [left_color, right_color]:
            return "hallway"

        return "unknown"

    def drop_cube(self):
        """Drop a foam cube to extinguish a fire."""
        logger.info("Dropping foam cube to extinguish fire")
        # Implement the mechanism to drop a cube using the dropper motor
        # Example implementation (needs calibration in real testing):
        self.dropper_motor.set_position_relative(180)  # Rotate 180 degrees
        self.dropper_motor.wait_is_stopped()
        time.sleep(0.5)  # Wait for cube to drop

        self.fires_extinguished += 1
        logger.info(
            f"Fire extinguished. Total fires extinguished: {self.fires_extinguished}"
        )
        return True

    def check_emergency_stop(self):
        """Check if the emergency stop button (touch sensor) is pressed."""
        if self.touch_sensor.is_pressed():
            logger.warning("EMERGENCY STOP ACTIVATED")
            return True
        return False

    def start_emergency_stop_monitor(self):
        """Start a thread to monitor the emergency stop button."""
        self.emergency_stop_thread = threading.Thread(
            target=self._monitor_emergency_stop
        )
        self.emergency_stop_thread.daemon = True
        self.emergency_stop_thread.start()
        logger.info("Emergency stop monitor started")

    def _monitor_emergency_stop(self):
        """Background thread to continuously monitor emergency stop button."""
        while self.mission_running:
            if self.check_emergency_stop():
                logger.warning("Emergency stop triggered - stopping all operations")
                self.reset_and_stop()
                self.mission_running = False
                break
            time.sleep(0.1)  # Check every 100ms

    def sweep_burning_room(self):
        """Conduct a systematic sweep of the burning room looking for fires."""
        logger.info("Starting burning room sweep")

        for x, y in BURNING_ROOM_SWEEP:
            # Navigate to the next grid position
            self.navigate_to(x, y)

            # Check for fire
            if self.check_for_fire():
                logger.info(f"Fire detected at position {self.position}")
                self.fires_detected += 1

                # Drop cube to extinguish fire
                self.drop_cube()

                # If we've extinguished both fires, exit early
                if self.fires_extinguished >= 2:
                    logger.info("Both fires extinguished, exiting sweep")
                    break

        logger.info(
            f"Room sweep complete. Fires detected: {self.fires_detected}, extinguished: {self.fires_extinguished}"
        )

    def return_to_start(self):
        """Navigate back to the starting position (0,0)."""
        logger.info("Starting return journey to base (0,0)")

        # First exit the room if we're still in it
        if self.position[1] >= 3:  # If in rows 3 or 4 (burning room)
            logger.info("Exiting burning room")
            self.navigate_to(BURNING_ROOM_ENTRY[0], BURNING_ROOM_ENTRY[1])
            self.navigate_to(ENTRANCE[0], ENTRANCE[1])

        # Follow return path
        for x, y in RETURN_PATH:
            if self.position[0] == x and self.position[1] == y:
                continue  # Skip if we're already at this position
            self.navigate_to(x, y)

        logger.info("Return journey complete")

    def reset_and_stop(self):
        """Complete reset of robot, stopping all operations."""
        self.stop()
        self.stop_siren()
        logger.info("Robot reset and stopped")

    def run_mission(self):
        """Execute the full firefighting mission."""
        # Initialize mission
        self.mission_running = True
        self.mission_start_time = time.time()
        self.fires_detected = 0
        self.fires_extinguished = 0

        try:
            # Start emergency stop monitor
            self.start_emergency_stop_monitor()

            # Start siren
            self.start_siren()

            # 1. Navigate through hallway to entrance
            logger.info("Starting hallway navigation")
            for x, y in HALLWAY_PATH[
                1:
            ]:  # Skip the first position as we're already there
                logger.info(f"Navigating to ({x}, {y})")
                self.navigate_to(x, y)

                # Check mission time after each movement
                elapsed_time = time.time() - self.mission_start_time
                if elapsed_time >= 180:  # 3 minutes = 180 seconds
                    logger.warning("Mission time limit (3 minutes) reached!")
                    break

            # 2. Orient to enter the burning room
            self.turn(NORTH)

            # Check if we're at the entrance
            is_at_entrance = (
                self.position[0] == ENTRANCE[0] and self.position[1] == ENTRANCE[1]
            )

            if is_at_entrance:
                # Check for orange line to confirm
                if self.check_for_entrance():
                    logger.info("Entrance confirmed, entering burning room")

                    # 3. Enter the burning room
                    self.advance_blocks(1)  # Move one block north to enter

                    # Stop siren upon entering the room
                    self.stop_siren()

                    # 4. Sweep room for fires
                    self.sweep_burning_room()

                    # 5. Return to the start
                    if self.fires_extinguished > 0:
                        logger.info(
                            f"Successfully extinguished {self.fires_extinguished} fires, returning to base"
                        )
                        self.return_to_start()
                    else:
                        logger.warning("No fires were extinguished, returning to base")
                        self.return_to_start()
                else:
                    logger.error(
                        "At calculated entrance position but orange line not detected"
                    )
            else:
                logger.error(
                    f"Failed to reach entrance. Current position: {self.position}"
                )

            # Check final mission metrics
            elapsed_time = time.time() - self.mission_start_time
            logger.info(f"Mission complete in {elapsed_time:.1f} seconds")
            logger.info(
                f"Fires detected: {self.fires_detected}, Fires extinguished: {self.fires_extinguished}"
            )

        except Exception as e:
            logger.error(f"Error during mission: {e}")
        finally:
            # Ensure everything stops properly
            self.mission_running = False
            self.reset_and_stop()


def main():
    robot = FirefighterRobot()
    try:
        logger.info(f"Starting navigation at {robot.position}")

        robot.start_siren()

        # Navigate through hallway path
        for x, y in HALLWAY_PATH[1:]:
            logger.info(f"Target position: ({x}, {y})")
            robot.navigate_to(x, y)
            # Only verify position if robot seems unsure
            # if not robot.align_with_grid():
            #     logger.warning("Alignment failed, attempting position verification")
            #     if not robot.verify_position():
            #         logger.warning("Position verification failed, trying to continue")

        robot.turn(NORTH)
        # Check if we've reached the entrance
        is_at_entrance = (
            robot.position[0] == ENTRANCE[0] and robot.position[1] == ENTRANCE[1]
        )

        print("At Entrance?", is_at_entrance)

        robot.stop_siren()

    except Exception as e:
        logger.error(f"Error during navigation: {e}")
    finally:
        # Stops after error / termination
        robot.reset_and_stop()


def calibration_testing():
    robot = FirefighterRobot()

    # robot.move_forward(24)
    pass


def dropper_testing():
    robot = FirefighterRobot()
    pass


if __name__ == "__main__":
    main()
