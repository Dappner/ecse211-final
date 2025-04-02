from utils.brick import Motor, EV3ColorSensor, EV3UltrasonicSensor
import time
import math
import logging
import color_matching as colo

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger(__name__)

# Constants
SQUARE_SIZE = 24  # cm per grid square
WHEELBASE = 15  # cm, approximate distance between tracks (adjust as needed)
TURN_SPEED = 300  # degrees per second for turns (used as limit)
MOVE_SPEED = 500  # degrees per second for forward movement
NB_COLOR_SAMPLING = 20  # number of times the color sensor samples a color
ALIGNMENT_TOLERANCE = 5  # cm tolerance for wall distance verification
ORIENTATION_TOLERANCE = 10  # degrees tolerance for orientation checks
TURN_CALIBRATION = 1.0  # Adjust after testing (e.g., 0.9 or 1.1)


# Constants from testing

MOTOR_DPS_CONST = 180
SPEED_MODIFIER = 2
MOTOR_DPS = MOTOR_DPS_CONST * SPEED_MODIFIER
MOTOR_POWER = 30.5


# Hallway path (excluding room entry)
HALLWAY_PATH = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2)]
ENTRANCE = (3, 2)  # Position before entering the room
BURNING_ROOM_ENTRY = (3, 3)  # Entry point to burning room


NORTH = "NORTH"
SOUTH = "SOUTH"
EAST = "EAST"
WEST = "WEST"


DIRECTION_VECTORS = {NORTH: (0, 1), EAST: (1, 0), SOUTH: (0, -1), WEST: (-1, 0)}


class FirefighterRobot:
    def __init__(self):
        self.left_motor = Motor("A")
        self.right_motor = Motor("B")
        self.left_color = EV3ColorSensor(1)
        self.right_color = EV3ColorSensor(2)
        self.ultrasonic = EV3UltrasonicSensor(3)
        self.dropper_motor = Motor("C")

        # Initializing Limits and Constants
        self.left_motor.set_limits(dps=MOVE_SPEED)
        self.right_motor.set_limits(dps=MOVE_SPEED)
        self.position = [0, 0]  # [x, y]
        self.orientation = NORTH

        # Waiting Ready
        self.left_color.wait_ready()
        self.right_color.wait_ready()
        self.ultrasonic.wait_ready()

        logger.info(
            f"Robot initialized at position {self.position}, orientation {self.orientation}"
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

    def turn(self, target_direction):
        """Turn to face a target direction (NORTH, SOUTH, EAST, WEST)."""
        if self.orientation == target_direction:
            logger.info(f"Already facing {target_direction}, no turn needed")
            return

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

        logger.info(f"Turned from {self.orientation} to {target_direction}")
        self.orientation = target_direction

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
        while True:
            left_rgb = self.get_color_left()
            right_rgb = self.get_color_right()

            left_black = left_rgb == "black"
            right_black = right_rgb == "black"

            if left_black and right_black:
                logger.info("Aligned with black grid line")
                self.stop()
                break
            elif left_black:
                logger.debug("Left sensor on black, turning right")
                self.left_motor.set_dps(TURN_SPEED)
                self.right_motor.set_dps(-TURN_SPEED)
            elif right_black:
                logger.debug("Right sensor on black, turning left")
                self.left_motor.set_dps(-TURN_SPEED)
                self.right_motor.set_dps(TURN_SPEED)
            else:
                logger.debug("No black detected, moving forward")
                self.left_motor.set_dps(MOVE_SPEED)
                self.right_motor.set_dps(MOVE_SPEED)
            time.sleep(0.05)

            self.stop()
        self.stop()

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
            self.align_with_grid()
        elif dx < 0:
            self.turn(WEST)
            self.advance_blocks(-dx)
            self.align_with_grid()

        # Then move in Y direction
        if dy > 0:
            self.turn(NORTH)
            self.advance_blocks(dy)
            self.align_with_grid()
        elif dy < 0:
            self.turn(SOUTH)
            self.advance_blocks(-dy)
            self.align_with_grid()

        logger.info(f"Navigation complete, at position {self.position}")

    def identify_room(self):
        "Identify the Current Room"
        # TODO: improve this mechanic (maybe we need to realign if one is on black)

        left_color = self.get_color_left()
        right_color = self.get_color_left()

        if left_color == "purple":
            return "burning room"
        elif left_color == "yellow":
            return "avoid room"
        elif left_color == "white":
            return "hallway"

        return "unknown"


def main():
    robot = FirefighterRobot()
    logger.info(f"Starting navigation at {robot.position}")

    logger.info(f"Starting navigation at {robot.position}")

    # Navigate through hallway path
    for x, y in HALLWAY_PATH[1:]:
        logger.info(f"Target position: ({x}, {y})")
        robot.navigate_to(x, y)
        # Only verify position if robot seems unsure
        if not robot.align_with_grid():
            logger.warning("Alignment failed, attempting position verification")
            if not robot.verify_position():
                logger.warning("Position verification failed, trying to continue")

    # Check if we've reached the entrance
    is_at_entrance = (
        robot.position[0] == ENTRANCE[0] and robot.position[1] == ENTRANCE[1]
    )

    print("At Entrance?", is_at_entrance)

    # This is where we now align to the entrace.

    robot.stop()
    # Move forward backward to actually get Orange
    # Once
    # left_rgb = robot.get_rgb_left()
    # right_rgb = robot.get_rgb_right()
    # if robot.detect_color(left_rgb, "orange") and robot.detect_color(
    #     right_rgb, "orange"
    # ):
    #     logger.info("Detected orange entrance at (3,2), entering burning room")
    #     robot.move_forward(SQUARE_SIZE / 2)
    #     robot.update_position(0, 1)
    # else:
    #     logger.error("Entrance not found at (3,2), stopping")
    #     robot.stop()
    #     return
    #
    # room = robot.identify_room()
    # logger.info(f"Identified room: {room}")
    # if room == "avoid room":
    #     logger.error("Wrong room detected (yellow), stopping")
    #     robot.stop()
    # elif room == "burning room":
    #     logger.info("Successfully entered the burning room at (3,3)")
    # else:
    #     logger.error("Unknown room, stopping for safety")
    #     robot.stop()
    #
    robot.stop()


def calibration_testing():
    robot = FirefighterRobot()

    # robot.move_forward(24)
    pass


def dropper_testing():
    robot = FirefighterRobot()
    pass


if __name__ == "__main__":
    main()
# calibration_testing()
