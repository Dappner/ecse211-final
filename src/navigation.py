"""
Enhanced navigation module with robust positioning and error correction.
"""
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
    COLOR_YELLOW, COLOR_WHITE, ALIGNMENT_TOLERANCE
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
        Initialize navigation system.

        Args:
            drive_system: DriveSystem object for movement
            sensor_system: SensorSystem object for sensing
        """
        self.drive = drive_system
        self.sensors = sensor_system

        # Position tracking
        self.estimated_position = [0, 0]  # [x, y] in grid coordinates
        self.position_confidence = 1.0  # 0.0 to 1.0

        # Monte Carlo Localization
        self.particles = []
        self.initialize_particles()

        # Movement history for backtracking
        self.movement_history = deque(maxlen=10)

        # Last known valid position
        self.last_valid_position = [0, 0]
        self.last_valid_orientation = NORTH

        # Wall following state
        self.wall_following = False
        self.wall_side = None  # 'left' or 'right'

        # Path planning
        self.current_path = []

        logger.info("Enhanced navigation system initialized")

    def initialize_particles(self):
        """Initialize particles for Monte Carlo Localization."""
        self.particles = []
        for _ in range(MCL_PARTICLE_COUNT):
            # All particles start at the origin (0,0) facing NORTH
            self.particles.append(Particle(0, 0, NORTH, 1.0))
        logger.debug(f"Initialized {MCL_PARTICLE_COUNT} particles at origin")

    def update_particles_after_movement(self, dx, dy, rotation=None):
        """
        Update particle positions after movement.

        Args:
            dx: Change in x position
            dy: Change in y position
            rotation: New orientation if turned, or None if no turn
        """
        for particle in self.particles:
            # Apply motion model with noise
            noise_x = random.gauss(0, MCL_MOTION_NOISE)
            noise_y = random.gauss(0, MCL_MOTION_NOISE)

            if rotation:
                # If a rotation occurred, update orientation
                particle.orientation = rotation
                # Add small chance of wrong orientation after turn
                if random.random() < 0.05:  # 5% chance of error
                    orientations = [NORTH, EAST, SOUTH, WEST]
                    orientations.remove(rotation)
                    particle.orientation = random.choice(orientations)

            # Update position based on orientation and add noise
            if particle.orientation == NORTH:
                particle.y += dy + noise_y
                particle.x += noise_x
            elif particle.orientation == EAST:
                particle.x += dx + noise_x
                particle.y += noise_y
            elif particle.orientation == SOUTH:
                particle.y -= dy + noise_y
                particle.x += noise_x
            elif particle.orientation == WEST:
                particle.x -= dx + noise_x
                particle.y += noise_y

        logger.debug("Updated particles after movement")

    def update_particle_weights(self, sensor_measurements):
        """
        Update particle weights based on sensor measurements.

        Args:
            sensor_measurements: Dict of sensor readings
        """
        # Get wall distance measurement
        wall_distance = sensor_measurements.get('wall_distance')
        left_color = sensor_measurements.get('left_color')
        right_color = sensor_measurements.get('right_color')

        for particle in self.particles:
            # Start with current weight
            w = particle.weight

            # Check if particle is in valid position
            px, py = int(particle.x), int(particle.y)
            if not self._is_valid_position(px, py):
                particle.weight = 0.01  # Very low weight for invalid positions
                continue

            # Update weight based on wall distance if available
            if wall_distance is not None:
                expected_distance = self._expected_wall_distance(px, py, particle.orientation)
                if expected_distance is not None:
                    # Calculate weight based on difference between expected and measured
                    distance_diff = abs(wall_distance - expected_distance)
                    w *= math.exp(-distance_diff ** 2 / (2 * MCL_SENSOR_NOISE ** 2))

            # Update weight based on colors
            if left_color is not None and right_color is not None:
                expected_color = self._expected_color(px, py)
                if expected_color is not None:
                    # If colors match expectation, increase weight
                    if left_color == expected_color or right_color == expected_color:
                        w *= 1.5
                    else:
                        w *= 0.5

            particle.weight = max(0.001, w)  # Ensure minimum weight

        # Normalize weights
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 0:
            for particle in self.particles:
                particle.weight /= total_weight

        logger.debug("Updated particle weights based on sensor measurements")

    def resample_particles(self):
        """Resample particles based on weights using importance sampling."""
        # Calculate effective sample size to determine if resampling is needed
        weights = [p.weight for p in self.particles]
        effective_size = 1.0 / sum(w ** 2 for w in weights)

        # Only resample if effective size is below threshold
        if effective_size < MCL_PARTICLE_COUNT * MCL_RESAMPLING_THRESHOLD:
            # Create cumulative sum of weights for faster sampling
            cum_weights = []
            cum_sum = 0
            for w in weights:
                cum_sum += w
                cum_weights.append(cum_sum)

            # Resample using low variance sampler
            new_particles = []
            step = 1.0 / MCL_PARTICLE_COUNT
            offset = random.random() * step
            i = 0

            for m in range(MCL_PARTICLE_COUNT):
                u = offset + m * step
                while u > cum_weights[i]:
                    i += 1

                # Create new particle based on selected particle
                old = self.particles[i]
                new_particles.append(Particle(old.x, old.y, old.orientation, 1.0 / MCL_PARTICLE_COUNT))

            self.particles = new_particles
            logger.debug("Resampled particles")

    def update_position_estimate(self):
        """Update estimated position based on particle distribution."""
        if not self.particles:
            return

        # Calculate weighted average
        total_x = 0
        total_y = 0
        total_weight = 0

        for p in self.particles:
            total_x += p.x * p.weight
            total_y += p.y * p.weight
            total_weight += p.weight

        if total_weight > 0:
            avg_x = total_x / total_weight
            avg_y = total_y / total_weight

            # Find most common orientation
            orientation_counts = {
                NORTH: 0,
                EAST: 0,
                SOUTH: 0,
                WEST: 0
            }

            for p in self.particles:
                orientation_counts[p.orientation] += p.weight

            estimated_orientation = max(orientation_counts, key=orientation_counts.get)

            # Update estimated position
            self.estimated_position = [avg_x, avg_y]

            # Calculate position confidence based on particle dispersion
            variance_x = sum((p.x - avg_x) ** 2 * p.weight for p in self.particles) / total_weight
            variance_y = sum((p.y - avg_y) ** 2 * p.weight for p in self.particles) / total_weight
            std_dev = math.sqrt(variance_x + variance_y)

            # Map standard deviation to confidence (higher std_dev = lower confidence)
            self.position_confidence = max(0.0, min(1.0, 1.0 - std_dev / 2.0))

            # If confidence is high enough, update drive system position
            if self.position_confidence > 0.7:
                self.drive.update_position(
                    round(avg_x) - self.drive.position[0],
                    round(avg_y) - self.drive.position[1]
                )
                if self.drive.orientation != estimated_orientation:
                    logger.info(f"Correcting orientation from {self.drive.orientation} to {estimated_orientation}")
                    self.drive.orientation = estimated_orientation

            logger.debug(f"Updated position estimate to {self.estimated_position} "
                         f"with confidence {self.position_confidence:.2f}")

    def _expected_wall_distance(self, x, y, orientation):
        """
        Calculate expected distance to wall based on position and orientation.

        Args:
            x: Grid x coordinate
            y: Grid y coordinate
            orientation: Facing direction

        Returns:
            float: Expected distance to wall in cm, or None if unknown
        """
        if not (0 <= x < GRID_WIDTH and 0 <= y < GRID_HEIGHT):
            return None

        # Calculate distance to wall based on map
        distance = 0
        dx, dy = DIRECTION_VECTORS[orientation]

        current_x, current_y = x, y
        while (0 <= current_x < GRID_WIDTH and
               0 <= current_y < GRID_HEIGHT and
               GRID_MAP[current_y][current_x] > 0):
            current_x += dx
            current_y += dy
            distance += 1

        # Convert to cm
        return distance * BLOCK_SIZE

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

    def localize(self):
        """
        Perform active localization to determine position.
        This is called when position confidence is low.
        """
        logger.info("Performing active localization")

        # Get current sensor data
        sensor_data = self.sensors.get_sensor_data()

        # Update particles based on sensor data
        self.update_particle_weights(sensor_data)
        self.resample_particles()
        self.update_position_estimate()

        # If confidence is still low, perform movement to gather more data
        if self.position_confidence < 0.5:
            logger.info("Low position confidence, performing movement for localization")

            # Rotation provides lots of information for localization
            self.drive.turn_slightly_left(0.2)
            time.sleep(0.2)
            sensor_data = self.sensors.get_sensor_data()
            self.update_particle_weights(sensor_data)

            self.drive.turn_slightly_right(0.4)
            time.sleep(0.2)
            sensor_data = self.sensors.get_sensor_data()
            self.update_particle_weights(sensor_data)

            self.drive.turn_slightly_left(0.2)

            # Resample and update after movements
            self.resample_particles()
            self.update_position_estimate()

        # Report localization results
        logger.info(f"Localization complete - Estimated position: {self.estimated_position}, "
                    f"Confidence: {self.position_confidence:.2f}")

        # Save position if confidence is high
        if self.position_confidence > 0.7:
            self.last_valid_position = self.estimated_position.copy()
            self.last_valid_orientation = self.drive.orientation

        return self.position_confidence > 0.7

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
                    logger.debug("Left sensor on black, turning right slightly")
                    self.drive.turn_slightly_right(0.1)
                elif on_black[1] == "right":
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
            match = abs(dist - expected_distance) < ALIGNMENT_TOLERANCE
            logger.info(
                f"Position verification: measured {dist}cm, expected {expected_distance}cm, {'valid' if match else 'invalid'}")
            return match

        # Otherwise, try to estimate expected distance based on position and orientation
        expected = self._expected_wall_distance(
            int(self.estimated_position[0]),
            int(self.estimated_position[1]),
            self.drive.orientation
        )

        if expected is not None:
            match = abs(dist - expected) < ALIGNMENT_TOLERANCE
            logger.info(
                f"Position verification: measured {dist}cm, expected {expected}cm, {'valid' if match else 'invalid'}")
            return match

        logger.info(f"Wall distance: {dist}cm (no verification possible)")
        return True

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

    def _plan_path(self, start, goal):
        """
        Plan a path from start to goal using A* algorithm.

        Args:
            start: Tuple (x, y) of start position
            goal: Tuple (x, y) of goal position

        Returns:
            list: List of (x, y) positions forming the path, or empty list if no path found
        """
        if not self._is_valid_position(start[0], start[1]) or not self._is_valid_position(goal[0], goal[1]):
            logger.warning(f"Invalid start {start} or goal {goal} position")
            return []

        # A* algorithm implementation
        open_set = {start}
        closed_set = set()

        came_from = {}

        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}

        while open_set:
            # Find node in open_set with lowest f_score
            current = min(open_set, key=lambda pos: f_score.get(pos, float('inf')))

            if current == goal:
                # We've reached the goal, reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]  # Reverse path to get start->goal

            open_set.remove(current)
            closed_set.add(current)

            # Check neighbors
            if current in VALID_NEIGHBORS:
                for neighbor in VALID_NEIGHBORS[current]:
                    if neighbor in closed_set:
                        continue  # Already evaluated

                    # Calculate tentative g_score
                    tentative_g = g_score[current] + 1  # Cost is 1 for each grid movement

                    if neighbor not in open_set:
                        open_set.add(neighbor)
                    elif tentative_g >= g_score.get(neighbor, float('inf')):
                        continue  # Not a better path

                    # This path is the best so far, record it
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor, goal)

        # No path found
        logger.warning(f"No path found from {start} to {goal}")
        return []

    def _heuristic(self, a, b):
        """Manhattan distance heuristic for A* algorithm."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def check_for_obstacle(self):
        """
        Check if there's an obstacle in the path.
        Uses ultrasonic sensor and color sensors.
        """
        # Check with ultrasonic sensor if available
        if self.sensors.has_ultrasonic:
            distance = self.sensors.get_wall_distance()
            if distance is not None and distance < BLOCK_SIZE - ALIGNMENT_TOLERANCE:
                logger.warning(f"Obstacle detected at distance {distance}cm")
                return True

        # Check with color sensors for furniture (green)
        found_furniture, _ = self.sensors.check_for_furniture()
        if found_furniture:
            logger.warning("Green furniture detected by color sensors")
            return True

        return False

    def _avoid_obstacle(self):
        """
        Smart obstacle avoidance strategy.

        Returns:
            bool: True if obstacle successfully avoided, False otherwise
        """
        logger.warning("Executing obstacle avoidance maneuver")

        # Remember original orientation
        original_orientation = self.drive.orientation

        # Get current position
        x, y = int(round(self.estimated_position[0])), int(round(self.estimated_position[1]))
        current_pos = (x, y)

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
                # Pick the neighbor that's closest to our goal if we have a current path
                if self.current_path and len(self.current_path) > 0:
                    goal = self.current_path[-1]
                    valid_moves.sort(key=lambda pos: self._heuristic(pos, goal))

                next_x, next_y = valid_moves[0]
                logger.info(f"Selected alternative path to ({next_x}, {next_y})")

                # Determine direction to move
                move_dx = next_x - x
                move_dy = next_y - y

                if move_dx == 1:
                    self.drive.turn(EAST)
                elif move_dx == -1:
                    self.drive.turn(WEST)
                elif move_dy == 1:
                    self.drive.turn(NORTH)
                elif move_dy == -1:
                    self.drive.turn(SOUTH)

                # Ensure we're not still detecting the obstacle
                if self.check_for_obstacle():
                    logger.warning("Still detecting obstacle after turn, backing up slightly")
                    self.drive.move_backward_slightly(0.5)

                # Move forward
                self.drive.advance_blocks(1)

                # Update particles based on movement
                self.update_particles_after_movement(move_dx, move_dy)
                sensor_data = self.sensors.get_sensor_data()
                self.update_particle_weights(sensor_data)
                self.update_position_estimate()

                logger.info("Obstacle avoidance completed")
                return True

        # Fallback: try backing up and taking a different path
        logger.warning("No valid alternative path found, backing up")
        self.drive.turn(self._opposite_direction(original_orientation))
        self.drive.advance_blocks(1)

        # Update position estimate after backup
        dx, dy = DIRECTION_VECTORS[self._opposite_direction(original_orientation)]
        self.update_particles_after_movement(dx, dy)
        sensor_data = self.sensors.get_sensor_data()
        self.update_particle_weights(sensor_data)
        self.update_position_estimate()

        return False

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
        return False, "none"