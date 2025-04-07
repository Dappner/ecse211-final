import logging
import time
from src.constants import (
    NORTH, EAST, SOUTH, WEST, DIRECTION_VECTORS,
    HALLWAY_PATH, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
    COLOR_BLACK, COLOR_ORANGE, COLOR_RED,
    MAX_GRID_ALIGNMENT_ATTEMPTS, MAX_ENTRANCE_ALIGNMENT_ATTEMPTS
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

        # Position tracking - simply track position in the hardcoded path
        self.current_path_index = 0
        self.in_burning_room = False

        self.alignment_turn_speed = 0.05  # Reduced turn speed for precision
        self.alignment_forward_speed = 0.05  # Reduced forward speed for precision
        self.consecutive_readings_needed = 2

        logger.info("Simple navigation system initialized")

    def follow_hallway_path(self):
        """
        Navigate through the hallway path to the room entrance.

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info("Starting hallway navigation")

        # Start at beginning of path
        self.current_path_index = 0

        # Navigate each segment of the hallway path
        for i in range(1, len(HALLWAY_PATH)):
            prev_pos = HALLWAY_PATH[i - 1]
            current_pos = HALLWAY_PATH[i]

            success = self._move_to_adjacent_position(prev_pos, current_pos)
            if not success:
                logger.error(f"Failed to move from {prev_pos} to {current_pos}")
                return False

            # Update path index
            self.current_path_index = i

            # Pause between movements
            time.sleep(0.2)

        logger.info("Hallway navigation completed")
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
            # First move to burning room entrance
            current_pos = BURNING_ROOM_SWEEP[-1]  # Last position in sweep
            success = self._move_to_adjacent_position(current_pos, BURNING_ROOM_ENTRY)
            if not success:
                logger.error("Failed to return to burning room entrance")
                return False

            # Exit to hallway (move one block south)
            self.drive.turn(SOUTH)
            success = self.drive.advance_blocks(1)
            if not success:
                logger.error("Failed to exit burning room")
                return False

            self.in_burning_room = False

        # Follow return path (reverse of hallway path)
        current_pos = RETURN_PATH[0]

        for next_pos in RETURN_PATH[1:]:
            success = self._move_to_adjacent_position(current_pos, next_pos)
            if not success:
                logger.error(f"Failed to move from {current_pos} to {next_pos}")
                return False

            # Update current position for next move
            current_pos = next_pos

            # Pause between movements
            time.sleep(0.2)

        logger.info("Successfully returned to base")
        return True

    def _move_to_adjacent_position(self, start_pos, end_pos):
        """
        Move from one grid position to an adjacent grid position.

        Args:
            start_pos: (x, y) start position
            end_pos: (x, y) end position

        Returns:
            bool: True if successful, False otherwise
        """
        # Calculate direction
        dx = end_pos[0] - start_pos[0]
        dy = end_pos[1] - start_pos[1]

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

        if orientation is None:
            logger.error(f"Invalid move from {start_pos} to {end_pos} - not adjacent")
            return False

        # Check if a turn is needed
        turn_performed = orientation != self.drive.orientation

        # Turn to face the right direction
        self.drive.turn(orientation)

        # If we turned, allow a small pause for stability
        if turn_performed:
            time.sleep(0.2)
            self.align_with_grid()

        # Move forward one block
        success = self.drive.advance_blocks(1)

        if not success:
            logger.error(f"Failed to advance from {start_pos} to {end_pos}")
            return False

        # After movement, check for grid alignment to stay on track, but only if we turned

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

        # Use a smaller turn increment for finer control
        TURN_INCREMENT = self.alignment_turn_speed  # Reduced from previous value
        FORWARD_INCREMENT = self.alignment_forward_speed  # Reduced from previous value

        while attempts < MAX_GRID_ALIGNMENT_ATTEMPTS:
            # Get line detection status
            on_black, position = self.sensors.is_on_black_line()

            # Verify reading stability by requiring consecutive matching readings
            if position == last_position:
                consecutive_stable_readings += 1
            else:
                consecutive_stable_readings = 0

            last_position = position

            # Only proceed with adjustment if we have stable readings
            if consecutive_stable_readings >= self.consecutive_readings_needed:
                if on_black:
                    if position == "BOTH":
                        logger.info("Both sensors aligned with grid line")
                        self.drive.stop()
                        # Back up slightly to center over the line
                        self.drive.move_backward_slightly(0.2)
                        return True
                    elif position == "LEFT":
                        logger.info("Left sensor on grid line, turning slightly left")
                        self.drive.turn_slightly_left(TURN_INCREMENT)
                    elif position == "RIGHT":
                        logger.info("Right sensor on grid line, turning slightly right")
                        # Reduce turn increment for finer control
                        self.drive.turn_slightly_right(TURN_INCREMENT)
                else:
                    # Neither sensor on grid line
                    logger.debug("No grid line detected, inching forward")
                    # Use smaller increment for searching
                    self.drive.move_forward_slightly(FORWARD_INCREMENT)

            # Always stop motors between adjustments to avoid momentum issues
            self.drive.stop()

            # Pause slightly to let sensors and robot stabilize
            time.sleep(0.15)

            attempts += 1

            # If we've made several attempts without finding a line, try a slightly larger movement
            if attempts % 5 == 0 and not on_black:
                logger.info(f"Made {attempts} attempts, trying larger search movement")
                self.drive.move_forward_slightly(FORWARD_INCREMENT * 2)

        logger.warning(f"Grid alignment failed after {attempts} attempts")
        return False

    def align_with_entrance(self):
        """
        Improved entrance alignment using more patient approach with smaller movements.

        Returns:
            bool: True if successfully aligned, False otherwise
        """
        logger.info("Attempting to align with room entrance (orange line)...")
        attempts = 0
        consecutive_stable_readings = 0
        last_position = None

        # Reduced increments for finer control
        TURN_INCREMENT = self.alignment_turn_speed
        FORWARD_INCREMENT = self.alignment_forward_speed

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
                        logger.info("Left sensor detected entrance, turning slightly left")
                        self.drive.turn_slightly_left(TURN_INCREMENT)
                        time.sleep(0.1)
                    elif position == "RIGHT":
                        logger.info("Right sensor detected entrance, turning slightly right")
                        self.drive.turn_slightly_right(TURN_INCREMENT)
                        time.sleep(0.1)
                else:
                    # Try forward and backward small movements in alternation
                    if attempts % 3 == 0:
                        logger.debug("No entrance detected, moving slightly forward")
                        self.drive.move_forward_slightly(FORWARD_INCREMENT)
                    elif attempts % 3 == 1:
                        logger.debug("No entrance detected, moving slightly backward")
                        self.drive.move_backward_slightly(FORWARD_INCREMENT)
                    else:
                        # Alternate turn direction in search pattern
                        if attempts % 6 < 3:
                            self.drive.turn_slightly_left(TURN_INCREMENT * 2)
                        else:
                            self.drive.turn_slightly_right(TURN_INCREMENT * 2)

            # Always stop motors between adjustments
            self.drive.stop()

            # Pause to let sensors stabilize
            time.sleep(0.2)

            attempts += 1

        logger.warning(f"Entrance alignment failed after {attempts} attempts")
        return False