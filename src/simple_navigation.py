import logging
import time
from src.constants import (
    NORTH, EAST, SOUTH, WEST,
    HALLWAY_PATH, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
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
        Navigate through the hallway path to the room entrance with strategic grid alignments.

        Returns:
            bool: True if successful, False otherwise
        """
        logger.info("Starting hallway navigation")

        # Start at beginning of path
        self.current_path_index = 0

        # First move from (0,0) to (0,1)
        prev_pos = HALLWAY_PATH[0]  # (0,0)
        current_pos = HALLWAY_PATH[1]  # (0,1)

        logger.info(f"Moving from start position {prev_pos} to {current_pos}")
        success = self._move_to_adjacent_position(prev_pos, current_pos)
        if not success:
            logger.error(f"Failed to move to first position {current_pos}")
            return False

        self.current_path_index = 1
        time.sleep(0.2)

        # Second move from (0,1) to (0,2)
        prev_pos = HALLWAY_PATH[1]  # (0,1)
        current_pos = HALLWAY_PATH[2]  # (0,2)

        logger.info(f"Moving to position {current_pos}")
        success = self._move_to_adjacent_position(prev_pos, current_pos)
        if not success:
            logger.error(f"Failed to move to second position {current_pos}")
            return False

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
            prev_pos = HALLWAY_PATH[i - 1]
            current_pos = HALLWAY_PATH[i]

            logger.info(f"Moving to position {current_pos}")
            success = self._move_to_adjacent_position(prev_pos, current_pos)
            if not success:
                logger.error(f"Failed to move from {prev_pos} to {current_pos}")
                return False

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

        # Move forward one block
        success = self.drive.advance_blocks(1)

        if not success:
            logger.error(f"Failed to advance from {start_pos} to {end_pos}")
            return False

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