import logging
import time
from utils.brick import Motor
from src.constants import (
    NORTH, SOUTH, EAST, WEST, MOTOR_ENCODER_COUNTS_PER_BLOCK, MOTOR_ENCODER_TOLERANCE, TURN_ENCODER_COUNTS_90,
    MOTOR_DPS  # We'll still use the DPS value from constants
)

logger = logging.getLogger("encoder_drive")



class NewDriveSystem:
    """
    Position Based    """

    def __init__(self, left_motor, right_motor):
        """Initialize the drive system with motors."""
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Reset encoders at initialization
        self.reset_encoders()

        # Track orientation
        self.orientation = NORTH

        # Calibration factors that can be adjusted during runtime
        self.forward_factor = 1.0
        self.turn_factor = 1.0

        # Track motion state
        self.is_moving = False

        # Reduced speed for precision movements (% of normal speed)
        self.precision_speed_factor = 0.6

        # Motor power settings
        self.normal_power = 70
        self.precision_power = 40  # Lower power for more precise movements

        # Small movement timing constants (seconds)
        self.small_turn_base_time = 0.15  # Base time for small turn movements
        self.small_move_base_time = 0.25  # Base time for small forward/backward movements

        # Set motor limits for safer operation
        self.set_motor_limits()

        logger.info("Encoder-based Drive System initialized")

    def set_motor_limits(self, power=70, dps=MOTOR_DPS):
        """Set motor power and speed limits."""
        self.left_motor.set_limits(power=power, dps=dps)
        self.right_motor.set_limits(power=power, dps=dps)
        logger.debug(f"Motor limits set: power={power}%, dps={dps}")

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

        # Initially give motors time to start moving
        time.sleep(0.2)

        # Busy wait with a small sleep to reduce CPU usage
        while time.time() - start_time < timeout:
            left_moving = self.left_motor.is_moving()
            right_moving = self.right_motor.is_moving()

            if not (left_moving or right_moving):
                # Motors have stopped, movement completed
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
            num_blocks: Number of blocks to move forward (negative for backward)

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info(f"Moving {'forward' if num_blocks > 0 else 'backward'} {abs(num_blocks)} blocks")

        # Calculate encoder counts based on direction, adjusted by calibration factor
        counts = int(MOTOR_ENCODER_COUNTS_PER_BLOCK * num_blocks * self.forward_factor)

        # Track starting positions for verification
        left_start = self.left_motor.get_encoder()
        right_start = self.right_motor.get_encoder()

        # Set relative positions for both motors
        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(counts)

        # Wait for movement to complete
        result = self.wait_for_completion()

        if result:
            # Verify movement completed correctly
            left_end = self.left_motor.get_encoder()
            right_end = self.right_motor.get_encoder()

            left_delta = left_end - left_start
            right_delta = right_end - right_start

            logger.debug(f"Movement completed: Left={left_delta}, Right={right_delta}, Target={counts}")

            # Check if movement was accurate within tolerance
            left_error = abs(left_delta - counts)
            right_error = abs(right_delta - counts)

            if left_error > MOTOR_ENCODER_TOLERANCE or right_error > MOTOR_ENCODER_TOLERANCE:
                logger.warning(f"Movement error detected: Left error={left_error}, Right error={right_error}")
                # Could adjust calibration here based on errors

        return result

    def move_forward_slightly(self, distance_factor=0.2):
        """
        Move forward slightly for small adjustments with improved precision.

        Args:
            distance_factor: Fraction of a block to move (0.2 = 20% of a block)
        """
        logger.debug(f"Moving forward slightly ({distance_factor} block)")

        # Use lower power for small movements
        self.set_motor_limits(power=self.precision_power)

        # For very small movements, use timed motion instead of position
        if distance_factor < 0.08:
            # Use direct motor control with reduced speed
            reduced_dps = MOTOR_DPS * self.precision_speed_factor
            self.left_motor.set_dps(reduced_dps)
            self.right_motor.set_dps(reduced_dps)

            # Scale time proportionally to distance
            movement_time = self.small_move_base_time * distance_factor / 0.2
            time.sleep(movement_time)

            self.stop()
            return

        # For larger small movements, use position control
        counts = int(MOTOR_ENCODER_COUNTS_PER_BLOCK * distance_factor * self.forward_factor)

        # Use position_relative for consistent movement
        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(counts)
        self.is_moving = True

        # Use shorter timeout for small movements
        self.wait_for_completion(timeout=max(1.0, 2.0 * distance_factor))

        # Reset to normal power
        self.set_motor_limits(power=self.normal_power)

    def move_backward_slightly(self, distance_factor=0.2):
        """
        Move backward slightly for small adjustments with improved precision.

        Args:
            distance_factor: Fraction of a block to move (0.2 = 20% of a block)
        """
        logger.debug(f"Moving backward slightly ({distance_factor} block)")

        # Use lower power for small movements
        self.set_motor_limits(power=self.precision_power)

        # For very small movements, use timed motion
        if distance_factor < 0.08:
            # Use direct motor control with reduced speed
            reduced_dps = MOTOR_DPS * self.precision_speed_factor
            self.left_motor.set_dps(-reduced_dps)
            self.right_motor.set_dps(-reduced_dps)

            # Scale time proportionally to distance
            movement_time = self.small_move_base_time * distance_factor / 0.2
            time.sleep(movement_time)

            self.stop()
            return

        # Use negative counts for backward movement
        counts = -int(MOTOR_ENCODER_COUNTS_PER_BLOCK * distance_factor * self.forward_factor)

        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(counts)
        self.is_moving = True

        self.wait_for_completion(timeout=max(1.0, 2.0 * distance_factor))

        # Reset to normal power
        self.set_motor_limits(power=self.normal_power)

    def turn_90_left(self, times=1):
        """
        Turn left 90 degrees (or multiple of 90).

        Args:
            times: Number of 90-degree turns to make

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info(f"Turning left {90 * times} degrees")

        # Calculate encoder counts for turn, adjusted by calibration factor
        counts = int(TURN_ENCODER_COUNTS_90 * times * self.turn_factor)

        # For left turn: left motor backward, right motor forward
        self.left_motor.set_position_relative(-counts)
        self.right_motor.set_position_relative(counts)

        # Wait for movement to complete
        result = self.wait_for_completion()

        if result:
            # Update orientation
            directions = [NORTH, WEST, SOUTH, EAST]  # Counter-clockwise order
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")

        return result

    def turn_90_right(self, times=1):
        """
        Turn right 90 degrees (or multiple of 90).

        Args:
            times: Number of 90-degree turns to make

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info(f"Turning right {90 * times} degrees")

        # Calculate encoder counts for turn, adjusted by calibration factor
        counts = int(TURN_ENCODER_COUNTS_90 * times * self.turn_factor)

        # For right turn: left motor forward, right motor backward
        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(-counts)

        # Wait for movement to complete
        result = self.wait_for_completion()

        if result:
            # Update orientation
            directions = [NORTH, EAST, SOUTH, WEST]  # Clockwise order
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")

        return result

    def turn_slightly_left(self, angle_factor=0.1):
        """
        Turn slightly left without changing tracked orientation.
        Uses very precise movements for improved alignment.

        Args:
            angle_factor: Fraction of a 90-degree turn (0.1 = 9 degrees)
        """
        logger.debug(f"Turning slightly left ({angle_factor * 90} degrees)")

        # Use lower power for fine movements
        self.set_motor_limits(power=self.precision_power)

        # For very small adjustments, use timed control instead of position
        if angle_factor < 0.05:
            # Use direct speed control with reduced speed
            reduced_dps = MOTOR_DPS * self.precision_speed_factor
            self.left_motor.set_dps(-reduced_dps)
            self.right_motor.set_dps(reduced_dps)

            # Scale time proportionally to angle
            movement_time = self.small_turn_base_time * angle_factor / 0.1
            time.sleep(movement_time)

            self.stop()
            return

        # For larger angle adjustments, use position control
        counts = int(TURN_ENCODER_COUNTS_90 * angle_factor * self.turn_factor)

        # Apply turn without updating orientation
        self.left_motor.set_position_relative(-counts)
        self.right_motor.set_position_relative(counts)
        self.is_moving = True

        # Wait for movement to complete with shorter timeout
        self.wait_for_completion(timeout=max(1.0, 2.0 * angle_factor))

        # Reset to normal power
        self.set_motor_limits(power=self.normal_power)

    def turn_slightly_right(self, angle_factor=0.1):
        """
        Turn slightly right without changing tracked orientation.
        Uses very precise movements for improved alignment.

        Args:
            angle_factor: Fraction of a 90-degree turn (0.1 = 9 degrees)
        """
        logger.debug(f"Turning slightly right ({angle_factor * 90} degrees)")

        # Use lower power for fine movements
        self.set_motor_limits(power=self.precision_power)

        # For very small adjustments, use timed control instead of position
        if angle_factor < 0.05:
            # Use direct speed control with reduced speed
            reduced_dps = MOTOR_DPS * self.precision_speed_factor
            self.left_motor.set_dps(reduced_dps)
            self.right_motor.set_dps(-reduced_dps)

            # Scale time proportionally to angle
            movement_time = self.small_turn_base_time * angle_factor / 0.1
            time.sleep(movement_time)

            self.stop()
            return

        # For larger angle adjustments, use position control
        counts = int(TURN_ENCODER_COUNTS_90 * angle_factor * self.turn_factor)

        # Apply turn without updating orientation
        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(-counts)
        self.is_moving = True

        # Wait for movement to complete with shorter timeout
        self.wait_for_completion(timeout=max(1.0, 2.0 * angle_factor))

        # Reset to normal power
        self.set_motor_limits(power=self.normal_power)

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

    def calibrate_forward_movement(self, actual_distance, expected_distance=1.0):
        """
        Calibrate forward movement based on actual vs expected distance.

        Args:
            actual_distance: Actual distance moved in blocks
            expected_distance: Expected distance in blocks (default: 1 block)
        """
        if actual_distance <= 0:
            logger.warning("Cannot calibrate with zero or negative distance")
            return

        new_factor = expected_distance / actual_distance

        # Limit adjustment to reasonable range (±20%)
        new_factor = max(0.8, min(1.2, new_factor))

        # Apply moving average to smooth changes
        self.forward_factor = 0.7 * self.forward_factor + 0.3 * new_factor
        logger.info(f"Forward calibration adjusted: factor={self.forward_factor:.3f}")

    def calibrate_turn_movement(self, actual_angle, expected_angle=90):
        """
        Calibrate turn movement based on actual vs expected angle.

        Args:
            actual_angle: Actual angle turned in degrees
            expected_angle: Expected angle in degrees (default: 90 degrees)
        """
        if actual_angle <= 0:
            logger.warning("Cannot calibrate with zero or negative angle")
            return

        new_factor = expected_angle / actual_angle

        # Limit adjustment to reasonable range (±20%)
        new_factor = max(0.8, min(1.2, new_factor))

        # Apply moving average to smooth changes
        self.turn_factor = 0.7 * self.turn_factor + 0.3 * new_factor
        logger.info(f"Turn calibration adjusted: factor={self.turn_factor:.3f}")