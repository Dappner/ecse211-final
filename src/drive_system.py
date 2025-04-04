import logging
from src.constants import (
    MOTOR_DPS, MOTOR_POWER,
    NORTH, SOUTH, EAST, WEST, DIRECTION_VECTORS, TURN_90_TIME, FORWARD_TIME_PER_BLOCK
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
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()

        # Orientation tracking -- no longer Position Tracking -- Corrected by the navigation class at times.
        self.orientation = NORTH

        logger.info(
            f"Drive system initialized"
        )

    def reset_motors(self):
        """Stop all motors by setting DPS to 0."""
        self.left_motor.set_dps(0)
        self.right_motor.set_dps(0)

    def set_motors_turn_left(self):
        """Set motors for left turn using power settings."""
        self.left_motor.set_power(-MOTOR_POWER)
        self.right_motor.set_power(MOTOR_POWER)

    def set_motors_turn_right(self):
        """Set motors for right turn using power settings."""
        self.left_motor.set_power(MOTOR_POWER)
        self.right_motor.set_power(-MOTOR_POWER)

    def advance_blocks(self, number):
        """Move forward a specified number of blocks based on tested timing values."""
        logger.debug(f"Advancing {number} blocks")
        self.left_motor.set_dps(MOTOR_DPS)
        self.right_motor.set_dps(MOTOR_DPS)
        time.sleep(FORWARD_TIME_PER_BLOCK * number)
        self.reset_motors()

    def move_forward_slightly(self, time_seconds=0.5):
        """Move forward slightly for small adjustments."""
        logger.debug(f"Moving forward slightly for {time_seconds} seconds")
        self.left_motor.set_power(MOTOR_POWER / 2)
        self.right_motor.set_power(MOTOR_POWER / 2)
        time.sleep(time_seconds)
        self.reset_motors()

    def move_backward_slightly(self, time_seconds=0.5):
        """Move backward slightly for small adjustments."""
        logger.debug(f"Moving forward slightly for {time_seconds} seconds")
        self.left_motor.set_power(-MOTOR_POWER / 2)
        self.right_motor.set_power(-MOTOR_POWER / 2)
        time.sleep(time_seconds)
        self.reset_motors()

    def turn_90_left(self, times=1):
        """Turn left 90 degrees (or multiple of 90) based on tested timing values."""
        logger.debug(f"Turning left {90 * times} degrees")
        self.set_motors_turn_left()
        time.sleep(TURN_90_TIME * times)
        self.reset_motors()

        # Update orientation
        directions = [NORTH, WEST, SOUTH, EAST]
        try:
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Internal Drive orientation updated to: {self.orientation}")
        except ValueError:
            logger.error(f"Current orientation '{self.orientation}' unknown during left turn!")

    def turn_90_right(self, times=1):
        """Turn right 90 degrees (or multiple of 90) based on movement pattern."""
        logger.debug(f"Turning right {90 * times} degrees")
        self.left_motor.set_power(MOTOR_POWER)
        self.right_motor.set_power(-MOTOR_POWER)
        time.sleep(TURN_90_TIME * times)
        self.reset_motors()

        # Update orientation
        directions = [NORTH, EAST, SOUTH, WEST]
        try:
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Internal Drive orientation updated to: {self.orientation}")
        except ValueError:
            logger.error(f"Current orientation '{self.orientation}' unknown during right turn!")

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
            logger.debug(f"Already facing {target_direction}, no turn needed")
            return

        old_orientation = self.orientation

        directions = [NORTH, EAST, SOUTH, WEST]
        try:
            current_index = directions.index(self.orientation)
            target_index = directions.index(target_direction)
        except ValueError:
            logger.error(f"Cannot turn: Unknown current ('{self.orientation}') or target ('{target_direction}') direction.")
            return # Avoid turning if directions are invalid

        # Calculates the shortest turn
        diff = (target_index - current_index) % 4
        if diff == 1:  # Turn right once
            self.turn_90_right()
        elif diff == 2:  # Turn 180 degrees (two right or two left)
            self.turn_90_right(2)
        elif diff == 3:  # Turn left once
            self.turn_90_left()

        logger.debug(f"Turned from {old_orientation} to {self.orientation}")

    def stop(self):
        """Stop all movement."""
        self.reset_motors()
