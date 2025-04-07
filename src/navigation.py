import logging
import time
import math
import random
from collections import deque

from src.constants import (
    MAX_GRID_ALIGNMENT_ATTEMPTS, MAX_ENTRANCE_ALIGNMENT_ATTEMPTS, MCL_PARTICLE_COUNT,
    MCL_MOTION_NOISE, MCL_SENSOR_NOISE, MCL_RESAMPLING_THRESHOLD, BLOCK_SIZE,
    EAST, NORTH, WEST, SOUTH, DIRECTION_VECTORS, VALID_NEIGHBORS, GRID_MAP,
    GRID_HEIGHT, GRID_WIDTH, COLOR_ORANGE, COLOR_RED, COLOR_GREEN, COLOR_PURPLE,
    COLOR_YELLOW, COLOR_WHITE, ALIGNMENT_TOLERANCE, COLOR_BLACK, BURNING_ROOM_ENTRY
)

logger = logging.getLogger("navigation")


class Particle:
    """
    Represents a particle for Monte Carlo Localization (particle filter).
    """

    def __init__(self, x, y, orientation, weight=1.0):
        self.x = x
        self.y = y
        self.orientation = orientation
        self.weight = weight

    def __repr__(self):
        return f"Particle(x={self.x:.2f}, y={self.y:.2f}, orient='{self.orientation}', w={self.weight:.4f})"


class Navigation:
    """
    Navigation system which now includes position tracking, error correction,
    and sensor-based localization.
    """

    def __init__(self, drive_system, sensor_system):
        """
        Initialize navigation system with improved localization capabilities.

        Args:
            drive_system: DriveSystem object for movement control
            sensor_system: SensorSystem object for environmental sensing
        """
        self.drive = drive_system
        self.sensors = sensor_system

        # Position tracking
        self.estimated_position = [0, 0]  # [x, y] in grid coordinates
        self.position_confidence = 1.0  # 0.0 to 1.0
        self.expected_orientation = NORTH  # Expected orientation based on commands

        # Grid line detection
        self.last_grid_line_detection = None  # Last time we detected a grid line
        self.grid_line_confidence = 0.0  # Confidence in grid alignment

        # Path planning
        self.movement_history = deque(maxlen=10)
        self.last_valid_position = [0, 0]
        self.last_valid_orientation = NORTH
        self.current_path = []

        # Recovery strategies
        self.recovery_mode = False
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3

        # Wall following behavior
        self.wall_following = False
        self.wall_side = None  # 'left' or 'right'

        # Room identification
        self.current_room_type = "unknown"
        self.room_confidence = 0.0

        # Add a history of position estimates for smoothing
        self.position_history = deque(maxlen=5)
        for _ in range(5):
            self.position_history.append([0, 0])

        logger.info("Enhanced navigation system initialized")

    def _expected_wall_distance(self, x, y, orientation):
        """
        Calculate expected distance to wall based on position and orientation.

        Args:
            x: Grid x coordinate
            y: Grid y coordinate
            orientation: Direction facing

        Returns:
            float: Expected distance to wall in cm, or None if unknown
        """
        if not (0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT):
            return None

        # Get direction vector
        dx, dy = DIRECTION_VECTORS.get(orientation, (0, 0))
        if dx == 0 and dy == 0:
            return None

        # Count cells until wall or edge
        distance = 0
        current_x, current_y = x, y

        while True:
            next_x, next_y = current_x + dx, current_y + dy

            # Check if next position is valid
            if not (0 <= next_x < GRID_WIDTH and 0 <= next_y < GRID_HEIGHT):
                # Hit map boundary
                break

            # Check if next position is an obstacle/wall (value 0 in grid map)
            if next_x < len(GRID_MAP[0]) and next_y < len(GRID_MAP):
                if GRID_MAP[int(next_y)][int(next_x)] == 0:
                    break

            # Move to next position
            current_x, current_y = next_x, next_y
            distance += 1

            # Limit search distance
            if distance > max(GRID_WIDTH, GRID_HEIGHT):
                return None

        # Add fractional distance and convert to cm
        fractional_dist = math.sqrt((current_x - x) ** 2 + (current_y - y) ** 2)
        return fractional_dist * BLOCK_SIZE

    def _expected_color(self, x, y):
        """
        Determine expected color based on grid position.

        Args:
            x: Grid x coordinate
            y: Grid y coordinate

        Returns:
            str: Expected color, or None if unknown
        """
        if not (0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT):
            return None

        grid_value = GRID_MAP[y][x]

        # Map grid values to expected colors
        if grid_value == 1:
            return COLOR_WHITE  # Hallway
        elif grid_value == 2:
            return COLOR_PURPLE  # Burning room
        elif grid_value == 3:
            return COLOR_YELLOW  # Avoid room

        return None

    def _is_valid_position(self, x, y):
        """Check if a position is valid on the grid."""
        return (0 <= x < GRID_WIDTH and
                0 <= y < GRID_HEIGHT and
                GRID_MAP[y][x] > 0)


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
                    logger.info("Left sensor on black, turning right slightly")
                    self.drive.turn_slightly_left(0.1)
                elif on_black[1] == "right":
                    logger.info("Right sensor on black, turning left slightly")
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

    def navigate_to(self, target_x, target_y, avoid_obstacles=True):
        """
        Navigate to a target position with obstacle avoidance and position verification.

        Args:
            target_x: Target x coordinate
            target_y: Target y coordinate
            avoid_obstacles: Whether to check for and avoid obstacles

        Returns:
            bool: True if navigation successful, False otherwise
        """
        # First, localize to ensure we know our position
        if self.position_confidence < 0.7:
            self.localize()

        # Calculate path
        start_x, start_y = int(round(self.estimated_position[0])), int(round(self.estimated_position[1]))
        path = self._plan_path((start_x, start_y), (target_x, target_y))

        if not path:
            logger.warning(f"Could not find path to ({target_x}, {target_y})")
            return False

        logger.info(f"Planned path to ({target_x}, {target_y}): {path}")
        self.current_path = path

        # Follow path
        for i, (x, y) in enumerate(path[1:], 1):  # Skip start position
            prev_x, prev_y = path[i - 1]

            # Determine direction to move
            dx = x - prev_x
            dy = y - prev_y

            if dx == 1:  # Move east
                self.drive.turn(EAST)
            elif dx == -1:  # Move west
                self.drive.turn(WEST)
            elif dy == 1:  # Move north
                self.drive.turn(NORTH)
            elif dy == -1:  # Move south
                self.drive.turn(SOUTH)

            # Check for obstacles if enabled
            if avoid_obstacles and self.check_for_obstacle():
                logger.warning(f"Obstacle detected at ({prev_x}, {prev_y}) while moving to ({x}, {y})")
                success = self._avoid_obstacle()
                if not success:
                    logger.error("Failed to avoid obstacle, aborting navigation")
                    return False

                # Re-plan path after obstacle avoidance
                current_x, current_y = int(round(self.estimated_position[0])), int(round(self.estimated_position[1]))
                new_path = self._plan_path((current_x, current_y), (target_x, target_y))

                if not new_path:
                    logger.warning(f"Could not find new path to ({target_x}, {target_y}) after obstacle avoidance")
                    return False

                logger.info(f"Re-planned path: {new_path}")
                self.current_path = new_path

                # Start following the new path (recursively)
                return self.navigate_to(target_x, target_y, avoid_obstacles)

            # Move forward one block
            self.drive.advance_blocks(1)

            # Update particles based on movement
            self.update_particles_after_movement(dx, dy)

            # Get sensor readings and update position estimate
            sensor_data = self.sensors.get_sensor_data()
            self.update_particle_weights(sensor_data)
            self.resample_particles()
            self.update_position_estimate()

            # Verify position with ultrasonic sensor
            self.verify_position_with_ultrasonic()

            # Add to movement history for backtracking
            self.movement_history.append((prev_x, prev_y))

            # If position confidence is low, try to relocalize
            if self.position_confidence < 0.4:
                logger.warning("Position confidence low, attempting relocalization")
                if not self.localize():
                    logger.error("Failed to relocalize, aborting navigation")
                    return False

                # Check if we're significantly off course and need to replan
                current_x, current_y = int(round(self.estimated_position[0])), int(round(self.estimated_position[1]))
                if abs(current_x - x) > 0.5 or abs(current_y - y) > 0.5:
                    logger.warning(f"Off course: Expected ({x}, {y}), but at ({current_x}, {current_y})")

                    # Re-plan path from current estimated position
                    new_path = self._plan_path((current_x, current_y), (target_x, target_y))

                    if not new_path:
                        logger.warning(f"Could not find new path to ({target_x}, {target_y}) after relocalization")
                        return False

                    logger.info(f"Re-planned path: {new_path}")
                    self.current_path = new_path

                    # Start following the new path (recursively)
                    return self.navigate_to(target_x, target_y, avoid_obstacles)

        # End navigation - verify arrival at target
        current_x, current_y = int(round(self.estimated_position[0])), int(round(self.estimated_position[1]))
        success = abs(current_x - target_x) <= 0.5 and abs(current_y - target_y) <= 0.5

        if success:
            logger.info(f"Navigation complete, arrived at position ({current_x}, {current_y})")
        else:
            logger.warning(f"Navigation ended at ({current_x}, {current_y}), target was ({target_x}, {target_y})")

        return success

    def _opposite_direction(self, direction):
        """Return the opposite direction."""
        opposites = {
            NORTH: SOUTH,
            SOUTH: NORTH,
            EAST: WEST,
            WEST: EAST
        }
        return opposites.get(direction, NORTH)

    def identify_room(self):
        """
        Identify the current room based on color patterns.
        Uses multiple samples for robustness.

        Returns:
            str: Room type ('burning room', 'avoid room', 'hallway', or 'unknown')
        """
        logger.info("Identifying current room")

        # Take multiple samples for reliability
        room_colors = []
        for _ in range(5):  # Increased samples for better accuracy
            left_color = self.sensors.get_color_left()
            right_color = self.sensors.get_color_right()
            if left_color:
                room_colors.append(left_color)
            if right_color:
                room_colors.append(right_color)
            time.sleep(0.1)

        # Count occurrences of each color
        color_counts = {}
        for color in room_colors:
            if color in color_counts:
                color_counts[color] += 1
            else:
                color_counts[color] = 1

        logger.info(f"Room color analysis: {color_counts}")

        # Determine room type based on color majority
        total_samples = len(room_colors)
        if total_samples == 0:
            return "unknown"

        # Calculate percentages of each color
        color_percentages = {color: count / total_samples * 100 for color, count in color_counts.items()}

        # Determine room type using thresholds
        if COLOR_PURPLE in color_percentages and color_percentages[COLOR_PURPLE] > 30:
            return "burning room"
        elif COLOR_YELLOW in color_percentages and color_percentages[COLOR_YELLOW] > 30:
            return "avoid room"
        elif COLOR_WHITE in color_percentages and color_percentages[COLOR_WHITE] > 30:
            return "hallway"
        else:
            # Try to match with expected color at current position
            x, y = int(round(self.estimated_position[0])), int(round(self.estimated_position[1]))
            expected = self._expected_color(x, y)

            if expected == COLOR_PURPLE:
                return "burning room"
            elif expected == COLOR_YELLOW:
                return "avoid room"
            elif expected == COLOR_WHITE:
                return "hallway"

        return "unknown"

    def find_fire(self):
        """
        Search for a fire in the current position.
        Performs a small search pattern if needed.

        Returns:
            tuple: (found_fire, sensor_side) - Boolean if fire found and which sensor(s)
        """
        logger.info("Searching for fire")

        # First, check if fire is directly visible
        found_fire, side = self.sensors.check_for_fire()
        if found_fire:
            logger.info(f"Fire detected on {side} side")
            return True, side

        # If not, perform a small search pattern
        logger.info("No fire immediately visible, performing search pattern")

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

        # Try looking forward with a small forward movement
        self.drive.turn_slightly_left(0.3)  # Back to center
        self.drive.move_forward_slightly(0.3)
        found_fire, side = self.sensors.check_for_fire()
        if found_fire:
            logger.info(f"Fire detected on {side} side after moving forward")
            return True, side

        # Move back to original position
        self.drive.move_backward_slightly(0.3)

        logger.info("No fire detected after search pattern")
        return False, "NONE"
