import logging
import time
from src.constants import (
    NORTH, EAST, SOUTH, WEST,
    HALLWAY_PATH, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
    MAX_GRID_ALIGNMENT_ATTEMPTS, MAX_ENTRANCE_ALIGNMENT_ATTEMPTS, BLOCK_SIZE
)

logger = logging.getLogger("navigation")


class SimpleNavigation:
    """
    Simplified navigation system that uses hardcoded paths and minimal sensor feedback.
    Focuses on the core mission of navigating to and from the burning room.
    """

    def __init__(self, drive_system, sensor_system):
        self.drive = drive_system
        self.sensors = sensor_system

        # Position tracking
        self.current_position = [0, 0]  # Start at origin
        self.current_path_index = 0
        self.in_burning_room = False

        self.calibration_complete = False
        self.consecutive_readings_needed = 2

        logger.info("Simple navigation system initialized")

    def follow_hallway_path(self):
        """
        Navigate through the hallway path to the room entrance with strategic grid alignments.

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info("Starting hallway navigation")

        # Start at beginning of path
        self.current_path_index = 0

        target_pos = HALLWAY_PATH[1]

        logger.info(f"Moving from start position {self.current_position} to {target_pos}")
        success = self.calibrate_first_movement()
        if not success:
            logger.error(f"Failed to move to first position {target_pos}")
            return False

        self.current_position = list(HALLWAY_PATH[1])
        self.current_path_index = 1
        time.sleep(0.2)

        # Second move from (0,1) to (0,2)
        target_pos = HALLWAY_PATH[2]

        logger.info(f"Moving to position {target_pos}")
        success = self.move_to_position(target_pos)
        if not success:
            logger.error(f"Failed to move to second position {target_pos}")
            return False

        self.current_position = list(HALLWAY_PATH[2])
        self.current_path_index = 2
        time.sleep(0.2)

        # Turn to EAST
        self.drive.turn(EAST)

        # At position (0,2): turned EAST - Now perform grid alignment
        logger.info("At position (0,2) - Performing grid alignment after turn")
        if not self.align_with_grid():
            logger.warning("Grid alignment failed, continuing with caution")

        # Complete the rest of the hallway path
        for i in range(3, len(HALLWAY_PATH)):
            target_pos = HALLWAY_PATH[i]

            logger.info(f"Moving to position {target_pos}")
            success = self.move_to_position(target_pos)
            if not success:
                logger.error(f"Failed to move to position {target_pos}")
                return False

            self.current_position = list(target_pos)
            self.current_path_index = i
            time.sleep(0.2)
        # Turn north (facing burning room)
        self.drive.turn(NORTH)

        # At ENTRANCE position - check for orange line
        logger.info("At entrance position - checking for orange line")
        if not self.align_with_entrance():
            logger.warning("Entrance alignment failed, continuing with caution")

        logger.info("Hallway navigation completed")
        return True

    def calibrate_first_movement(self):
        """
        Perform distance-based calibration during the first movement.
        Measures distance before and after moving one block, then adjusts parameters.

        Returns:
            bool: True if calibration was successful, False otherwise
        """
        logger.info("Starting first movement calibration")

        # 1. Measure at first position
        initial_distance = self.sensors.get_wall_distance()
        if initial_distance is None:
            logger.warning("Could not get initial distance reading")
            initial_distance = 100  # Default assumption
        else:
            logger.info(f"Initial distance to wall: {initial_distance:.1f}cm")

        # Initial encoder positions
        left_pos_before = self.drive.left_motor.get_position()
        right_pos_before = self.drive.right_motor.get_position()

        # 2. Drive one block north
        logger.info("Moving forward one block for calibration")
        self.drive.turn(NORTH)  # Ensure we're facing north
        self.drive.advance_blocks(1)

        # 3. Measure final distance
        final_distance = self.sensors.get_wall_distance()
        if final_distance is None:
            logger.warning("Could not get final distance reading")
        else:
            logger.info(f"Final distance to wall: {final_distance:.1f}cm")

        # 4. Calculate adjustment factor if both measurements successful
        if initial_distance is not None and final_distance is not None:
            distance_moved = initial_distance - final_distance
            logger.info(f"Distance moved: {distance_moved:.1f}cm (expected {BLOCK_SIZE}cm)")

            # Only adjust if we have reasonable measurements
            if distance_moved > 10:  # At least 10cm movement detected
                actual_blocks_moved = distance_moved / BLOCK_SIZE
                logger.info(f"Actual blocks moved based on distance: {actual_blocks_moved:.2f}")

                # Apply reasonable adjustment factor
                if 0.5 < actual_blocks_moved < 1.5:
                    self._apply_calibration_adjustment(actual_blocks_moved)

        # 5. Record encoder movement for diagnostics
        left_pos_after = self.drive.left_motor.get_position()
        right_pos_after = self.drive.right_motor.get_position()

        left_delta = left_pos_after - left_pos_before
        right_delta = right_pos_after - right_pos_before
        encoder_avg = (abs(left_delta) + abs(right_delta)) / 2

        logger.info(f"Encoder movement - Left: {left_delta}, Right: {right_delta}, Avg: {encoder_avg}")

        # 6. Use grid align to align to grid
        alignment_success = self.align_with_grid()

        # If alignment successful, back up slightly to center in the block
        if alignment_success:
            logger.info("Backing up slightly to center in grid block")
            #TODO: Might have to be played with (value wise)
            self.drive.move_backward_slightly(0.3)

        self.calibration_complete = True
        logger.info("First movement calibration complete")

        return True

    def enter_burning_room(self):
        """
        Enter the burning room from the entrance position.

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info("Attempting to enter burning room")

        # First try to align with the orange entrance line
        aligned = self.align_with_entrance()
        if not aligned:
            logger.warning("Could not align with entrance")
            # Try direct move anyway

        # Enter room (move one block north)
        logger.info("Moving into burning room")
        self.drive.turn(NORTH)
        success = self.drive.advance_blocks(1)

        if success:
            self.in_burning_room = True
            logger.info("Successfully entered burning room")
        else:
            logger.error("Failed to enter burning room")

        return success

    def return_to_base(self):
        """
        Navigate from current position back to base.

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info("Starting return to base")

        # If in burning room, first exit to hallway
        if self.in_burning_room:
            # First move to burning room entrance if not already there
            if self.current_position != list(BURNING_ROOM_ENTRY):
                logger.info(f"Moving to burning room entry {BURNING_ROOM_ENTRY}")
                success = self.move_to_position(BURNING_ROOM_ENTRY)
                if not success:
                    logger.error("Failed to return to burning room entrance")
                    return False

            # Exit to hallway (move one block south)
            logger.info("Exiting burning room to hallway")
            self.drive.turn(SOUTH)
            success = self.drive.advance_blocks(1)
            if not success:
                logger.error("Failed to exit burning room")
                return False

            self.in_burning_room = False
            self.current_position = list(HALLWAY_PATH[-1])  # Update position to entrance

        # We should now be at RETURN_PATH[0], verify
        if self.current_position != list(RETURN_PATH[0]):
            logger.warning(f"Position mismatch: expected {RETURN_PATH[0]}, got {self.current_position}")
            # Update position anyway to continue return path
            self.current_position = list(RETURN_PATH[0])

        # Follow return path (reverse of hallway path)
        for i, next_pos in enumerate(RETURN_PATH[1:], 1):
            logger.info(f"Return path step {i}: moving to {next_pos}")
            success = self.move_to_position(next_pos)
            if not success:
                logger.error(f"Failed to move to position {next_pos}")
                return False

            # Update current position
            self.current_position = list(next_pos)

            # Pause between movements
            time.sleep(0.2)

        logger.info("Successfully returned to base")
        return True

    def align_with_grid(self):
        """
        Improved grid alignment function using gradual movements and stable readings.

        Returns:
            bool: True if successfully aligned, False otherwise
        """
        logger.info("Starting grid alignment...")
        attempts = 0
        consecutive_stable_readings = 0
        last_position = None

        while attempts < MAX_GRID_ALIGNMENT_ATTEMPTS:
            # Get line detection status
            on_black, position = self.sensors.is_on_black_line()

            # Verify reading stability by requiring consecutive matching readings
            if position == last_position:
                consecutive_stable_readings += 1
            else:
                consecutive_stable_readings = 0

            last_position = position

            if consecutive_stable_readings >= self.consecutive_readings_needed:
                if on_black:
                    if position == "BOTH":
                        logger.info("Both sensors aligned with grid line")
                        self.drive.stop()
                        # Backing up to center.
                        self.drive.advance_blocks(-0.3)
                        return True
                    elif position == "LEFT":
                        logger.info("Left sensor on grid line, turning slightly left")
                        self.drive.turn_slightly_left(0.1)
                    elif position == "RIGHT":
                        logger.info("Right sensor on grid line, turning slightly right")
                        # Reduce turn increment for finer control
                        self.drive.turn_slightly_right(0.1)
                else:
                    # No sensor on grid line -> Move Forward
                    logger.info("No grid line detected, inching forward")
                    self.drive.move_forward_slightly(0.2)

            self.drive.stop()

            # Pause slightly to let sensors and robot stabilize
            time.sleep(0.2)

            attempts += 1

        logger.warning(f"Grid alignment failed after {attempts} attempts")
        return False

    def align_with_entrance(self):
        """ Attempts to align with the orange entrance line"""
        logger.info("Attempting to align with room entrance (orange line)...")
        attempts = 0
        consecutive_stable_readings = 0
        last_position = None

        while attempts < MAX_ENTRANCE_ALIGNMENT_ATTEMPTS:
            # Get entrance line detection
            found_orange, position = self.sensors.check_for_entrance()

            # Check for reading stability
            if position == last_position:
                consecutive_stable_readings += 1
            else:
                consecutive_stable_readings = 0

            last_position = position

            # Only proceed with adjustment if readings are stable
            if consecutive_stable_readings >= self.consecutive_readings_needed:
                if found_orange:
                    if position == "BOTH":
                        logger.info("Both sensors aligned with entrance")
                        self.drive.stop()
                        return True
                    elif position == "LEFT":
                        logger.info("Left sensor on entrance line, turning slightly left")
                        self.drive.turn_slightly_left(0.1)
                    elif position == "RIGHT":
                        logger.info("Right sensor on entrance line, turning slightly right")
                        self.drive.turn_slightly_right(0.1)
                else:
                    # No sensor on grid line -> Move Forward
                    logger.debug("No grid line detected, inching forward")
                    self.drive.move_forward_slightly(0.2)

            self.drive.stop()

            time.sleep(0.2)

            attempts += 1

        logger.warning(f"Entrance alignment failed after {attempts} attempts")
        return False

    def _apply_calibration_adjustment(self, actual_blocks_moved):
        """
        Apply calibration adjustment based on actual movement.

        Args:
            actual_blocks_moved: Measured movement in block units
        """
        # Calculate adjustment factor (inverse of actual movement)
        adjustment_factor = 1.0 / actual_blocks_moved

        # Limit adjustment to reasonable range (±20%)
        adjustment_factor = max(0.8, min(1.2, adjustment_factor))

        # Store adjustment factor for future reference
        self.forward_adjustment_factor = adjustment_factor

        # Apply to drive system parameters if available
        if hasattr(self.drive, 'forward_time_per_block'):
            old_forward_time = self.drive.forward_time_per_block
            self.drive.forward_time_per_block = old_forward_time * adjustment_factor
            logger.info(
                f"Adjusted forward time: {self.drive.forward_time_per_block:.2f}s (factor: {adjustment_factor:.2f})")

        # Also adjust turn times proportionally
        if hasattr(self.drive, 'turn_90_time'):
            old_turn_time = self.drive.turn_90_time
            self.drive.turn_90_time = old_turn_time * adjustment_factor
            logger.info(f"Adjusted turn time: {self.drive.turn_90_time:.2f}s (factor: {adjustment_factor:.2f})")

    def move_to_position(self, target_pos):
        """
        Move from current position to an adjacent grid position.

        Args:
            target_pos: (x, y) target position

        Returns:
            bool: True if successful, False otherwise
        """
        # Calculate direction
        dx = target_pos[0] - self.current_position[0]
        dy = target_pos[1] - self.current_position[1]

        # Check if move is valid (only one grid cell in one direction)
        if abs(dx) + abs(dy) != 1:
            logger.error(f"Invalid move from {self.current_position} to {target_pos} - not adjacent")
            return False

        # Determine orientation
        orientation = None
        if dx == 1 and dy == 0:
            orientation = EAST
        elif dx == -1 and dy == 0:
            orientation = WEST
        elif dx == 0 and dy == 1:
            orientation = NORTH
        elif dx == 0 and dy == -1:
            orientation = SOUTH

        # Turn to face the right direction
        self.drive.turn(orientation)

        # Allow a small pause for stability after turning
        time.sleep(0.2)

        # Move forward one block
        success = self.drive.advance_blocks(1)

        if success:
            # Update position on success
            self.current_position = list(target_pos)
            return True
        else:
            logger.error(f"Failed to advance to {target_pos}")
            return False