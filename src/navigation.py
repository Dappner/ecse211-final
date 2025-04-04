import logging
from src.constants import (
    MAX_GRID_ALIGNMENT_ATTEMPTS, MAX_ENTRANCE_ALIGNMENT_ATTEMPTS,
    EAST, NORTH, WEST, SOUTH,
    COLOR_PURPLE, COLOR_YELLOW, COLOR_WHITE, VALID_NEIGHBORS, DIRECTION_VECTORS
)
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger("navigation")


class Navigation:
    """Handles navigation logic, combining drive and sensors."""

    def __init__(self, drive_system, sensor_system):
        self.drive = drive_system
        self.sensors = sensor_system
        logger.info("Navigation system initialized.")

    def align_with_grid(self):
        """Align robot with black grid lines using both color sensors."""
        logger.info("Aligning with grid...")

        attempts = 0

        while attempts < MAX_GRID_ALIGNMENT_ATTEMPTS:
            on_black, side = self.sensors.is_on_black_line()

            if on_black:
                if side == "both":
                    logger.info("Aligned with black grid line")
                    self.drive.stop()
                    return True
                elif side == "left":
                    logger.debug("Left sensor on black, turning right slightly")
                    self.drive.turn_slightly_right(0.1)
                elif side == "right":
                    logger.debug("Right sensor on black, turning left slightly")
                    self.drive.turn_slightly_left(0.1)
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
                if found_orange_again and new_side != side:
                    # Both sensors likely on the line now
                    logger.info("Successfully aligned with orange entrance line")
                    return True
            else:
                # No orange detected, make small search movements
                logger.info("No orange line detected, searching...")

                # Try alternating small turns / forward movements.
                if attempts % 2 == 0:
                    self.drive.turn_slightly_left(0.1)
                else:
                    self.drive.turn_slightly_right(0.1)

                self.drive.move_forward_slightly(0.2)

            attempts += 1

        logger.warning("Failed to align with entrance after multiple attempts")
        return False

    def verify_position_with_ultrasonic(self, expected_distance=None):
        """
        Verify position using ultrasonic sensor and walls.

        Args:
            expected_distance: Expected distance to wall

        Returns:
            bool: True if position verified, False otherwise
        """
        if not self.sensors.has_ultrasonic:
            logger.warning("Cannot verify position without ultrasonic sensor")
            return False

        # Get current distance
        dist = self.sensors.get_wall_distance()
        if dist is None:
            logger.warning("Ultrasonic sensor failed to return a distance")
            return False

        # If expected distance is explicitly provided, use it
        if expected_distance is not None:
            match = abs(dist - expected_distance) < 5  # 5cm tolerance
            logger.info(
                f"Position verification: measured {dist}cm, expected {expected_distance}cm, {'valid' if match else 'invalid'}")
            return match

        # Otherwise, try to estimate expected distance based on position and orientation
        # This is a simplified approach - would need to be customized based on the actual map
        logger.info(f"Wall distance: {dist}cm")
        return True

    def navigate_to(self, target_x, target_y, avoid_obstacles=True):
        """Navigate to a target position with grid alignment and obstacle avoidance."""
        dx = target_x - self.drive.position[0]
        dy = target_y - self.drive.position[1]
        logger.info(
            f"Navigating to ({target_x}, {target_y}) from {self.drive.position}, dx={dx}, dy={dy}"
        )

        # First move in X direction if needed
        if dx > 0:
            self.drive.turn(EAST)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(dx)
        elif dx < 0:
            self.drive.turn(WEST)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(-dx)

        # Then move in Y direction if needed
        if dy > 0:
            self.drive.turn(NORTH)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(dy)
        elif dy < 0:
            self.drive.turn(SOUTH)
            if avoid_obstacles and self.sensors.check_for_furniture():
                self._avoid_obstacle()
            self.drive.advance_blocks(-dy)

        logger.info(f"Navigation complete, at position {self.drive.position}")

    def check_for_obstacle(self):
        """Check if there's an obstacle in the path."""
        found_obstacle, _ = self.sensors.check_for_furniture()
        if found_obstacle:
            logger.warning("Obstacle detected in path")
            return True
        return False

    def _avoid_obstacle(self):
        """
        Smart obstacle avoidance strategy which checks for valid neighbors to move to...
        """
        logger.warning("Executing obstacle avoidance maneuver")

        # Remember original orientation
        original_orientation = self.drive.orientation

        # Get current position
        x, y = self.drive.position
        current_pos = (int(x), int(y))

        # Get direction we were trying to go
        dx, dy = DIRECTION_VECTORS[original_orientation]
        blocked_pos = (current_pos[0] + dx, current_pos[1] + dy)

        # Check if we have valid neighbors info for this position
        if current_pos in VALID_NEIGHBORS:
            neighbors = VALID_NEIGHBORS[current_pos]
            logger.info(f"Valid neighbors for position {current_pos}: {neighbors}")

            # Filter out the blocked position
            valid_moves = [pos for pos in neighbors if pos != blocked_pos]

            if valid_moves:
                # Just pick the first valid neighbor
                next_x, next_y = valid_moves[0]
                logger.info(f"Selected alternative path to ({next_x}, {next_y})")

                # Navigate to the selected neighbor
                self.navigate_to(next_x, next_y)

                logger.info("Obstacle avoidance completed")
                return True

        # TODO: Fallback for obstacle avoidance?
        raise Exception("Obstacle Avoidance Didn't work... Should we just drive here?")

    def identify_room(self):
        """Identify the current room based on color patterns."""
        # Take multiple samples for reliability
        room_colors = []
        for _ in range(3):
            left_color = self.sensors.get_color_left()
            right_color = self.sensors.get_color_right()
            room_colors.extend([left_color, right_color])
            time.sleep(0.1)

        # Count occurrences of each color
        color_counts = {}
        for color in room_colors:
            if color in color_counts:
                color_counts[color] += 1
            else:
                color_counts[color] = 1

        logger.info(f"Room color analysis: {color_counts}")

        # Determine room type
        if COLOR_PURPLE in color_counts:
            return "burning room"
        elif COLOR_YELLOW in color_counts:
            return "avoid room"
        elif COLOR_WHITE in color_counts:
            return "hallway"

        return "unknown"

    def find_fire(self):
        """
        Search for a fire in the current position.
        Performs a small search pattern if needed.

        Returns:
            tuple: (bool, str) - If fire found and which sensor(s)
        """
        # First, check if fire is directly visible
        found_fire, side = self.sensors.check_for_fire()
        if found_fire:
            logger.info(f"Fire detected on {side} side")
            return True, side

        # If not, perform a small search pattern
        logger.info("No fire immediately visible, performing search pattern")

        # Remember original position
        original_position = self.drive.position.copy()

        # Try looking left
        self.drive.turn_slightly_left(0.3)
        found_fire, side = self.sensors.check_for_fire()
        if found_fire:
            logger.info(f"Fire detected on {side} side after turning left")
            return True, side

        # Try looking right
        self.drive.turn_slightly_right(0.6)  # 0.3 back to center + 0.3 to right
        found_fire, side = self.sensors.check_for_fire()
        if found_fire:
            logger.info(f"Fire detected on {side} side after turning right")
            return True, side

        # Return to center
        self.drive.turn_slightly_left(0.3)

        logger.info("No fire detected after search pattern")
        return False, "none"

