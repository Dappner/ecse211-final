import logging
from src.constants import (
    MOTOR_DPS, TURN_DPS,
    NORTH, SOUTH, EAST, WEST, TURN_90_TIME, FORWARD_TIME_PER_BLOCK
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

        self.forward_time_per_block = FORWARD_TIME_PER_BLOCK
        self.turn_90_time = TURN_90_TIME

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
        """Set motors for left turn using DPS."""
        self.left_motor.set_dps(-TURN_DPS)
        self.right_motor.set_dps(TURN_DPS)

    def set_motors_turn_right(self):
        """Set motors for right turn using DPS."""
        self.left_motor.set_dps(TURN_DPS)
        self.right_motor.set_dps(-TURN_DPS)

    def advance_blocks(self, number):
        """Move forward a specified number of blocks based on calibrated timing values."""
        logger.debug(f"Advancing {number} blocks")

        # Log encoder positions before movement
        left_pos_before = self.left_motor.get_position()
        right_pos_before = self.right_motor.get_position()

        # Set motors to move forward
        self.left_motor.set_dps(MOTOR_DPS)
        self.right_motor.set_dps(MOTOR_DPS)

        # Sleep for calibrated time
        time.sleep(self.forward_time_per_block * number)

        # Stop motors
        self.reset_motors()

        # Log encoder positions after movement for debugging
        left_pos_after = self.left_motor.get_position()
        right_pos_after = self.right_motor.get_position()

        left_delta = left_pos_after - left_pos_before
        right_delta = right_pos_after - right_pos_before

        logger.debug(f"Motor movement - Left: {left_delta}, Right: {right_delta} degrees")

        return left_delta, right_delta

    def move_forward_slightly(self, time_seconds=0.2):
        """Move forward slightly for small adjustments."""
        logger.debug(f"Moving forward slightly for {time_seconds} seconds")
        self.left_motor.set_dps(MOTOR_DPS / 2)
        self.right_motor.set_dps(MOTOR_DPS / 2)
        time.sleep(time_seconds)
        self.reset_motors()

    def move_backward_slightly(self, time_seconds=0.2):
        """Move backward slightly for small adjustments."""
        logger.debug(f"Moving backward slightly for {time_seconds} seconds")
        self.left_motor.set_dps(-MOTOR_DPS / 2)
        self.right_motor.set_dps(-MOTOR_DPS / 2)
        time.sleep(time_seconds)
        self.reset_motors()

    def turn_90_left(self, times=1):
        """Turn left 90 degrees (or multiple of 90) using DPS and timing."""
        logger.debug(f"Turning left {90 * times} degrees")

        # Track encoder positions for debugging
        left_pos_before = self.left_motor.get_position()
        right_pos_before = self.right_motor.get_position()

        self.set_motors_turn_left()
        time.sleep(self.turn_90_time * times)
        self.reset_motors()

        # Log encoder positions after turn
        left_pos_after = self.left_motor.get_position()
        right_pos_after = self.right_motor.get_position()

        left_delta = left_pos_after - left_pos_before
        right_delta = right_pos_after - right_pos_before

        logger.debug(f"Turn left movement - Left: {left_delta}, Right: {right_delta} degrees")

        # Update orientation
        directions = [NORTH, WEST, SOUTH, EAST]
        try:
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")
        except ValueError:
            logger.error(f"Current orientation '{self.orientation}' unknown during left turn!")

    def turn_90_right(self, times=1):
        """Turn right 90 degrees (or multiple of 90) using DPS and timing."""
        logger.debug(f"Turning right {90 * times} degrees")

        # Track encoder positions for debugging
        left_pos_before = self.left_motor.get_position()
        right_pos_before = self.right_motor.get_position()

        self.left_motor.set_dps(TURN_DPS)
        self.right_motor.set_dps(-TURN_DPS)
        time.sleep(self.turn_90_time * times)
        self.reset_motors()

        # Log encoder positions after turn
        left_pos_after = self.left_motor.get_position()
        right_pos_after = self.right_motor.get_position()

        left_delta = left_pos_after - left_pos_before
        right_delta = right_pos_after - right_pos_before

        logger.debug(f"Turn right movement - Left: {left_delta}, Right: {right_delta} degrees")

        # Update orientation
        directions = [NORTH, EAST, SOUTH, WEST]
        try:
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")
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
