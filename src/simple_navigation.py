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
        aligned = self._align_with_entrance()
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

        # Turn to face the right direction
        self.drive.turn(orientation)

        # Move forward one block
        success = self.drive.advance_blocks(1)

        if not success:
            logger.error(f"Failed to advance from {start_pos} to {end_pos}")
            return False

        # After movement, check for grid alignment to stay on track
        self.align_with_grid()

        return True

    def align_with_grid(self):
        """Align robot with black grid lines using both color sensors."""
        logger.info("Aligning with grid...")

        attempts = 0

        while attempts < MAX_GRID_ALIGNMENT_ATTEMPTS:
            on_black = self.sensors.is_on_black_line()

            if on_black[0]:  # First element is boolean success
                if on_black[1] == "both":
                    logger.info("Aligned with black grid line")
                    self.drive.stop()
                    return True
                elif on_black[1] == "left":
                    logger.info("Left sensor on black, turning left slightly")
                    self.drive.turn_slightly_left(0.1)
                elif on_black[1] == "right":
                    logger.info("Right sensor on black, turning right slightly")
                    self.drive.turn_slightly_right(0.1)
            else:
                logger.debug("No black detected, moving forward slowly")
                self.drive.move_forward_slightly(0.2)

            self.drive.stop()
            attempts += 1

        logger.warning(f"Failed to align with grid after {MAX_GRID_ALIGNMENT_ATTEMPTS} attempts")
        return False

    def align_with_entrance(self):
        """Attempt to align with the orange entrance line."""
        logger.info("Trying to align with orange entrance line...")

        attempts = 0

        while attempts < MAX_ENTRANCE_ALIGNMENT_ATTEMPTS:
            found_orange, side = self.sensors.check_for_entrance()

            if found_orange:
                if side == "left":
                    logger.info("Orange line detected on left, adjusting position")
                    self.drive.turn_slightly_left(0.1)
                    self.drive.move_forward_slightly(0.3)
                elif side == "right":
                    logger.info("Orange line detected on right, adjusting position")
                    self.drive.turn_slightly_right(0.1)
                    self.drive.move_forward_slightly(0.3)

                # Check if both sensors are now on the orange line
                found_orange_again, new_side = self.sensors.check_for_entrance()
                if found_orange_again and new_side == "both":
                    # Both sensors now on the line
                    logger.info("Successfully aligned with orange entrance line")
                    return True
            else:
                # No orange detected, make small search movements
                logger.info("No orange line detected, searching...")

                # Try alternating small turns / forward movements
                if attempts % 2 == 0:
                    self.drive.turn_slightly_left(0.1)
                else:
                    self.drive.turn_slightly_right(0.1)

                self.drive.move_forward_slightly(0.2)

            attempts += 1

        logger.warning("Failed to align with entrance after multiple attempts")
        return False