import logging
import time
from utils.brick import Motor
from src.constants import (
    NORTH, SOUTH, EAST, WEST,
    MOTOR_ENCODER_COUNTS_PER_BLOCK,
    TURN_ENCODER_COUNTS_90,
    MOTOR_ENCODER_TOLERANCE
)

logger = logging.getLogger("drive")


class NewDriveSystem:
    """
    Enhanced driving system using motor encoders for more reliable navigation.
    Uses position-based control rather than time-based control for higher accuracy.
    """

    def __init__(self, left_motor, right_motor):
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Reset encoders at initialization
        self.reset_encoders()

        # Track orientation
        self.orientation = NORTH

        # PID-like parameters for position control
        self.position_kp = 30  # Position proportional gain
        self.position_kd = 70  # Position derivative gain

        # Set motors to use position control with these parameters
        self.left_motor.set_position_kp(self.position_kp)
        self.left_motor.set_position_kd(self.position_kd)
        self.right_motor.set_position_kp(self.position_kp)
        self.right_motor.set_position_kd(self.position_kd)

        # Set motor limits for safer operation
        self.left_motor.set_limits(power=70, dps=360)
        self.right_motor.set_limits(power=70, dps=360)

        # Calibration values (can be adjusted during runtime)
        self.turn_adjustment_factor = 1.0
        self.forward_adjustment_factor = 1.0

        logger.info("Improved Drive System initialized with position-based control")

    def reset_encoders(self):
        """Reset both motor encoders to zero."""
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        logger.debug("Motor encoders reset to zero")

    def stop(self):
        """Immediately stop all motors."""
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)
        logger.debug("Motors stopped")

    def wait_for_completion(self, timeout=10.0):
        """
        Wait for motors to complete their movements.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            bool: True if completed, False if timed out
        """
        start_time = time.time()

        # Busy wait with a small sleep to reduce CPU usage
        while time.time() - start_time < timeout:
            left_moving = self.left_motor.is_moving()
            right_moving = self.right_motor.is_moving()

            if not (left_moving or right_moving):
                return True

            time.sleep(0.05)  # Small sleep to prevent CPU hogging

        # If we reach here, we've timed out
        logger.warning(f"Motor movement timed out after {timeout} seconds")
        self.stop()  # Force stop if timed out
        return False

    def advance_blocks(self, num_blocks):
        """
        Move forward a specified number of blocks using encoder feedback.

        Args:
            num_blocks: Number of blocks to move forward

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info(f"Moving forward {num_blocks} blocks")

        # Get starting encoder positions
        left_start = self.left_motor.get_encoder()
        right_start = self.right_motor.get_encoder()

        # Calculate target encoder counts based on direction
        encoder_counts = int(MOTOR_ENCODER_COUNTS_PER_BLOCK * num_blocks * self.forward_adjustment_factor)

        if self.orientation == NORTH:
            left_target = left_start + encoder_counts
            right_target = right_start + encoder_counts
        elif self.orientation == SOUTH:
            left_target = left_start - encoder_counts
            right_target = right_start - encoder_counts
        elif self.orientation == EAST:
            left_target = left_start + encoder_counts
            right_target = right_start + encoder_counts
        elif self.orientation == WEST:
            left_target = left_start - encoder_counts
            right_target = right_start - encoder_counts

        # Set positions for both motors
        self.left_motor.set_position(left_target)
        self.right_motor.set_position(right_target)

        # Wait for movement to complete
        result = self.wait_for_completion()

        # Verify we reached the target position
        if result:
            left_final = self.left_motor.get_encoder()
            right_final = self.right_motor.get_encoder()

            left_error = abs(left_final - left_target)
            right_error = abs(right_final - right_target)

            if (left_error > MOTOR_ENCODER_TOLERANCE or
                    right_error > MOTOR_ENCODER_TOLERANCE):
                logger.warning(f"Position error: Left={left_error}, Right={right_error}")
                result = False

            logger.debug(
                f"Move completed: Left moved {left_final - left_start}, Right moved {right_final - right_start}")

        return result

    def move_forward_slightly(self, distance_factor=0.2):
        """
        Move forward slightly for small adjustments.

        Args:
            distance_factor: Fraction of a block to move (0.2 = 20% of a block)
        """
        logger.debug(f"Moving forward slightly ({distance_factor} block)")
        self.advance_blocks(distance_factor)

    def move_backward_slightly(self, distance_factor=0.2):
        """
        Move backward slightly for small adjustments.

        Args:
            distance_factor: Fraction of a block to move (0.2 = 20% of a block)
        """
        logger.debug(f"Moving backward slightly ({distance_factor} block)")
        self.advance_blocks(-distance_factor)

    def _turn_90(self, direction, num_turns=1):
        """
        Turn 90 degrees in the specified direction.

        Args:
            direction: 'left' or 'right'
            num_turns: Number of 90-degree turns to make

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info(f"Turning {direction} {90 * num_turns} degrees")

        # Get starting encoder positions
        left_start = self.left_motor.get_encoder()
        right_start = self.right_motor.get_encoder()

        # Calculate encoder counts for turn, adjusted by calibration factor
        encoder_counts = int(TURN_ENCODER_COUNTS_90 * num_turns * self.turn_adjustment_factor)

        if direction == 'left':
            left_target = left_start - encoder_counts
            right_target = right_start + encoder_counts
        else:  # right
            left_target = left_start + encoder_counts
            right_target = right_start - encoder_counts

        # Set positions for both motors
        self.left_motor.set_position(left_target)
        self.right_motor.set_position(right_target)

        # Wait for movement to complete
        result = self.wait_for_completion()

        # Update orientation if successful
        if result:
            directions = [NORTH, EAST, SOUTH, WEST]
            current_index = directions.index(self.orientation)

            if direction == 'left':
                new_index = (current_index - num_turns) % 4
            else:  # right
                new_index = (current_index + num_turns) % 4

            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")

            # Log actual movement
            left_final = self.left_motor.get_encoder()
            right_final = self.right_motor.get_encoder()
            logger.debug(
                f"Turn completed: Left moved {left_final - left_start}, Right moved {right_final - right_start}")

        return result

    def turn_90_left(self, num_turns=1):
        """Turn left 90 degrees (or multiple of 90)."""
        return self._turn_90('left', num_turns)

    def turn_90_right(self, num_turns=1):
        """Turn right 90 degrees (or multiple of 90)."""
        return self._turn_90('right', num_turns)

    def turn_slightly_left(self, angle_factor=0.2):
        """
        Turn slightly left without changing tracked orientation.

        Args:
            angle_factor: Fraction of a 90-degree turn (0.2 = 18 degrees)
        """
        logger.debug(f"Turning slightly left ({angle_factor * 90} degrees)")

        # Get starting encoder positions
        left_start = self.left_motor.get_encoder()
        right_start = self.right_motor.get_encoder()

        # Calculate partial turn
        encoder_counts = int(TURN_ENCODER_COUNTS_90 * angle_factor * self.turn_adjustment_factor)

        left_target = left_start - encoder_counts
        right_target = right_start + encoder_counts

        # Set positions for both motors
        self.left_motor.set_position(left_target)
        self.right_motor.set_position(right_target)

        # Wait for movement to complete
        self.wait_for_completion(timeout=2.0)  # Shorter timeout for small movements

    def turn_slightly_right(self, angle_factor=0.2):
        """
        Turn slightly right without changing tracked orientation.

        Args:
            angle_factor: Fraction of a 90-degree turn (0.2 = 18 degrees)
        """
        logger.debug(f"Turning slightly right ({angle_factor * 90} degrees)")

        # Get starting encoder positions
        left_start = self.left_motor.get_encoder()
        right_start = self.right_motor.get_encoder()

        # Calculate partial turn
        encoder_counts = int(TURN_ENCODER_COUNTS_90 * angle_factor * self.turn_adjustment_factor)

        left_target = left_start + encoder_counts
        right_target = right_start - encoder_counts

        # Set positions for both motors
        self.left_motor.set_position(left_target)
        self.right_motor.set_position(right_target)

        # Wait for movement to complete
        self.wait_for_completion(timeout=2.0)  # Shorter timeout for small movements

    def turn(self, target_direction):
        """
        Turn to face a specific direction (NORTH, SOUTH, EAST, WEST).

        Args:
            target_direction: Direction to face after turning

        Returns:
            bool: True if successful, False otherwise
        """
        if self.orientation == target_direction:
            logger.debug(f"Already facing {target_direction}, no turn needed")
            return True

        directions = [NORTH, EAST, SOUTH, WEST]
        try:
            current_index = directions.index(self.orientation)
            target_index = directions.index(target_direction)
        except ValueError:
            logger.error(f"Invalid direction: current='{self.orientation}', target='{target_direction}'")
            return False

        # Calculate the shortest turn
        diff = (target_index - current_index) % 4
        result = False

        if diff == 1:  # Turn right once
            result = self.turn_90_right()
        elif diff == 2:  # 180 degrees - two right turns
            result = self.turn_90_right(2)
        elif diff == 3:  # Turn left once (equivalent to three rights)
            result = self.turn_90_left()

        return result

    def calibrate_movement(self, expected_distance, actual_distance):
        """
        Adjust forward movement calibration based on actual vs expected distance.

        Args:
            expected_distance: Expected distance in blocks
            actual_distance: Actual distance moved in blocks
        """
        if actual_distance > 0:
            new_factor = expected_distance / actual_distance
            # Limit adjustment to reasonable range
            new_factor = max(0.8, min(1.2, new_factor))

            # Apply running average to smooth adjustments
            self.forward_adjustment_factor = 0.7 * self.forward_adjustment_factor + 0.3 * new_factor

            logger.info(f"Forward calibration adjusted: {self.forward_adjustment_factor:.3f}")

    def calibrate_turning(self, num_tests=1):
        """
        Calibrate turning by performing test turns and measuring results.

        Args:
            num_tests: Number of test turns to perform
        """
        logger.info("Starting turn calibration")

        # Save original orientation
        original_orientation = self.orientation

        # Perform a full 360-degree turn (4 right turns)
        start_left = self.left_motor.get_encoder()
        start_right = self.right_motor.get_encoder()

        for _ in range(num_tests):
            self.turn_90_right(4)  # Should end up at the same orientation

        end_left = self.left_motor.get_encoder()
        end_right = self.right_motor.get_encoder()

        # Calculate error in position
        left_error = abs(end_left - start_left)
        right_error = abs(end_right - start_right)

        # Adjust turn calibration if needed
        if left_error > MOTOR_ENCODER_TOLERANCE or right_error > MOTOR_ENCODER_TOLERANCE:
            # Calculate actual encoder counts used per 90 degrees
            total_counts_left = abs(end_left - start_left) / (4 * num_tests)
            total_counts_right = abs(end_right - start_right) / (4 * num_tests)
            avg_counts = (total_counts_left + total_counts_right) / 2

            if avg_counts > 0:
                # Adjust turn factor based on actual counts vs expected
                new_factor = TURN_ENCODER_COUNTS_90 / avg_counts
                # Limit adjustment to reasonable range
                new_factor = max(0.8, min(1.2, new_factor))

                # Apply running average to smooth adjustments
                self.turn_adjustment_factor = 0.7 * self.turn_adjustment_factor + 0.3 * new_factor

                logger.info(f"Turn calibration adjusted: {self.turn_adjustment_factor:.3f}")

        # Ensure we've returned to the original orientation
        if self.orientation != original_orientation:
            logger.warning(f"Orientation changed during calibration: {original_orientation} -> {self.orientation}")
            self.orientation = original_orientation

        return self.turn_adjustment_factor