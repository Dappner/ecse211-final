import logging
from src.constants import (
    MOVE_SPEED, MOTOR_DPS, MOTOR_POWER, TURN_TIME,
    NORTH, SOUTH, EAST, WEST, DIRECTION_VECTORS
)
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger("drive")


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
        time.sleep(2.48 * number)
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
        time.sleep(TURN_TIME * times)
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
        time.sleep(TURN_TIME * times)
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
