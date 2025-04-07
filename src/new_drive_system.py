import logging
import time
from utils.brick import Motor
from src.constants import (
    NORTH, SOUTH, EAST, WEST, MOTOR_ENCODER_COUNTS_PER_BLOCK, MOTOR_ENCODER_TOLERANCE, TURN_ENCODER_COUNTS_90,
    MOTOR_DPS, TURN_DPS  # We'll still use the DPS value from constants
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

        # Track whether in motion (important because we are using relative position)
        self.is_moving = False

        # Motor power settings
        self.normal_power = 70
        self.precision_power = 40

        self.current_power = None  # Initial state is unknown

        # Time to wait after changing power
        self.motor_settling_time = 0.5

        # Initialize motors with normal power
        self.update_motor_power(self.normal_power)

        logger.info("Drive System initialized")

    def update_motor_power(self, power):
        """Update motor power if different from current setting.

        Returns:
            bool: True if power was updated (and settling time applied)
        """
        if power != self.current_power:
            # Power changed, update limits and apply settling time
            self.left_motor.set_limits(power=power)
            self.right_motor.set_limits(power=power)
            self.current_power = power

            # Wait for settings to take effect
            time.sleep(self.motor_settling_time)
            logger.debug(f"Motor power updated to {power}%")
            return True

        # No change needed
        return False

    def set_motor_limits(self, power=70, dps=MOTOR_DPS):
        """Set motor power and speed limits."""
        # Only update and sleep if different from current power
        if power != self.current_power:
            self.left_motor.set_limits(power=power, dps=dps)
            self.right_motor.set_limits(power=power, dps=dps)
            self.current_power = power

            # Allow limits to take effect
            time.sleep(self.motor_settling_time)

            logger.debug(f"Motor limits set: power={power}%, dps={dps}")

    def reset_encoders(self):
        """Reset both motor encoders to zero."""
        self.left_motor.reset_encoder()
        self.right_motor.reset_encoder()
        logger.debug("Motor encoders reset to zero")

    def stop(self):
        """Stop all motors."""
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)
        time.sleep(0.1)  # Short delay to ensure motors stop
        logger.debug("Motors stopped")

    def set_motors_turn_left(self):
        """Set motors for left turn."""
        if self.current_left_dps != -TURN_DPS or self.current_right_dps != TURN_DPS:
            self.left_motor.set_dps(-TURN_DPS)
            self.right_motor.set_dps(TURN_DPS)
            self.current_left_dps = -TURN_DPS
            self.current_right_dps = TURN_DPS

    def set_motors_turn_right(self):
        """Set motors for right turn."""
        if self.current_left_dps != TURN_DPS or self.current_right_dps != -TURN_DPS:
            self.left_motor.set_dps(TURN_DPS)
            self.right_motor.set_dps(-TURN_DPS)
            self.current_left_dps = TURN_DPS
            self.current_right_dps = -TURN_DPS

    def reset_motors(self):
        """Stop all motors by setting DPS to 0."""
        if self.current_left_dps != 0 or self.current_right_dps != 0:
            self.left_motor.set_dps(0)
            self.right_motor.set_dps(0)
            self.current_left_dps = 0
            self.current_right_dps = 0
            # Give motors time to fully stop
            time.sleep(0.1)

    def wait_for_completion(self, timeout=10.0):
        """Wait for motors to complete their movements."""
        start_time = time.time()
        time.sleep(0.2)  # Initially give motors time to start moving

        while time.time() - start_time < timeout:
            left_moving = self.left_motor.is_moving()
            right_moving = self.right_motor.is_moving()

            if not (left_moving or right_moving):
                return True  # Movement completed

            time.sleep(0.05)  # Small sleep to prevent CPU hogging

        # Timeout occurred
        logger.warning(f"Motor movement timed out after {timeout} seconds")
        self.stop()
        return False

    def advance_blocks(self, num_blocks):
        """Move forward a specified number of blocks."""
        logger.info(f"Moving {'forward' if num_blocks > 0 else 'backward'} {abs(num_blocks)} blocks")

        self.stop()
        self.update_motor_power(self.normal_power)

        # Calculate encoder counts
        counts = int(MOTOR_ENCODER_COUNTS_PER_BLOCK * num_blocks * self.forward_factor)

        # Set relative positions
        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(counts)

        return self.wait_for_completion()

    def turn_90_left(self, times=1):
        """Turn left 90 degrees (or multiple of 90)."""
        logger.info(f"Turning left {90 * times} degrees")

        self.stop()
        self.update_motor_power(self.normal_power)

        # Calculate encoder counts
        counts = int(TURN_ENCODER_COUNTS_90 * times * self.turn_factor)

        # Set relative positions for turn
        self.left_motor.set_position_relative(-counts)
        self.right_motor.set_position_relative(counts)

        result = self.wait_for_completion()

        if result:
            # Update orientation
            directions = [NORTH, WEST, SOUTH, EAST]  # Counter-clockwise
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")

        return result

    def turn_90_right(self, times=1):
        """Turn right 90 degrees (or multiple of 90)."""
        logger.info(f"Turning right {90 * times} degrees")

        self.stop()
        self.update_motor_power(self.normal_power)

        # Calculate encoder counts
        counts = int(TURN_ENCODER_COUNTS_90 * times * self.turn_factor)

        # Set relative positions for turn
        self.left_motor.set_position_relative(counts)
        self.right_motor.set_position_relative(-counts)

        result = self.wait_for_completion()

        if result:
            # Update orientation
            directions = [NORTH, EAST, SOUTH, WEST]  # Clockwise
            current_index = directions.index(self.orientation)
            new_index = (current_index + times) % 4
            self.orientation = directions[new_index]
            logger.debug(f"Orientation updated to: {self.orientation}")

        return result

    def turn_slightly_left(self, time_seconds=0.2):
        """Make a small left turn adjustment."""
        logger.debug(f"Turning slightly left for {time_seconds} seconds")

        # Use precision power for small movements
        self.update_motor_power(self.precision_power)

        # Set motor speeds directly
        self.left_motor.set_dps(-TURN_DPS)
        self.right_motor.set_dps(TURN_DPS)

        time.sleep(time_seconds)
        self.stop()

    def turn_slightly_right(self, time_seconds=0.2):
        """Make a small right turn adjustment."""
        logger.debug(f"Turning slightly right for {time_seconds} seconds")

        # Use precision power for small movements
        self.update_motor_power(self.precision_power)

        # Set motor speeds directly
        self.left_motor.set_dps(TURN_DPS)
        self.right_motor.set_dps(-TURN_DPS)

        time.sleep(time_seconds)
        self.stop()

    def move_forward_slightly(self, time_seconds=0.2):
        """Move forward slightly."""
        logger.debug(f"Moving forward slightly for {time_seconds} seconds")

        # Use precision power for small movements
        self.update_motor_power(self.precision_power)

        # Set motor speeds directly
        self.left_motor.set_dps(MOTOR_DPS / 2)
        self.right_motor.set_dps(MOTOR_DPS / 2)

        time.sleep(time_seconds)
        self.stop()

    def move_backward_slightly(self, time_seconds=0.2):
        """Move backward slightly."""
        logger.debug(f"Moving backward slightly for {time_seconds} seconds")

        # Use precision power for small movements
        self.update_motor_power(self.precision_power)

        # Set motor speeds directly
        self.left_motor.set_dps(-MOTOR_DPS / 2)
        self.right_motor.set_dps(-MOTOR_DPS / 2)

        time.sleep(time_seconds)
        self.stop()

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
