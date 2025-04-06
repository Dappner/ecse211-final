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

        # Monte Carlo Localization
        self.particles = []
        self.initialize_particles()

        # Tracking particle dispersion to detect kidnapped robot problem
        self.dispersion_threshold = 0.8  # Threshold for particle dispersion warning

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

    def initialize_particles(self, num_particles=None, position=None, spread=0.2):
        """
        Initialize particles for Monte Carlo Localization with improved distribution.

        Args:
            num_particles: Number of particles to generate (default: MCL_PARTICLE_COUNT)
            position: Center position [x, y] for particles (default: current estimate)
            spread: Standard deviation for Gaussian distribution of particles
        """
        if num_particles is None:
            num_particles = MCL_PARTICLE_COUNT

        if position is None:
            position = self.estimated_position

        self.particles = []

        # Calculate how many particles to allocate to each orientation
        particles_per_orientation = num_particles // 4
        remaining = num_particles - (particles_per_orientation * 4)

        orientations = [NORTH, EAST, SOUTH, WEST]

        # Current orientation gets more particles for better precision
        current_orientation_idx = orientations.index(self.drive.orientation)

        # Distribute particles with preference to current orientation
        orientation_counts = [particles_per_orientation] * 4
        orientation_counts[current_orientation_idx] += remaining

        for i, orientation in enumerate(orientations):
            count = orientation_counts[i]
            for _ in range(count):
                # Use Gaussian distribution around current position estimate
                x = random.gauss(position[0], spread)
                y = random.gauss(position[1], spread)

                # Constrain to valid grid positions
                x = max(0, min(GRID_WIDTH - 1, x))
                y = max(0, min(GRID_HEIGHT - 1, y))

                # Higher initial weight to particles with current orientation
                weight = 1.5 if orientation == self.drive.orientation else 1.0

                self.particles.append(Particle(x, y, orientation, weight))

        # Normalize weights
        self._normalize_weights()

        logger.debug(f"Initialized {num_particles} particles around position {position}")

    def _normalize_weights(self):
        """Normalize particle weights to sum to 1.0"""
        total_weight = sum(p.weight for p in self.particles)
        if total_weight > 0:
            for p in self.particles:
                p.weight /= total_weight

    def update_particles_after_movement(self, dx, dy, rotation=None):
        """
        Update particle positions after movement with improved noise model.

        Args:
            dx: Change in x position
            dy: Change in y position
            rotation: New orientation if turned, or None if no turn
        """
        if not self.particles:
            self.initialize_particles()
            return

        # Calculate average position before update for comparison
        prev_x = sum(p.x * p.weight for p in self.particles)
        prev_y = sum(p.y * p.weight for p in self.particles)

        # Track expected orientation
        if rotation:
            self.expected_orientation = rotation

        # Update each particle with motion model and noise
        for particle in self.particles:
            # Add noise proportional to movement distance
            move_distance = math.sqrt(dx ** 2 + dy ** 2)
            # More movement = more uncertainty
            noise_factor = max(0.05, min(0.3, move_distance * 0.1))

            noise_x = random.gauss(0, MCL_MOTION_NOISE * noise_factor)
            noise_y = random.gauss(0, MCL_MOTION_NOISE * noise_factor)

            # Update orientation if specified
            if rotation:
                # Small chance of orientation error during turns
                if random.random() < 0.05:  # 5% chance of error
                    orientations = [NORTH, EAST, SOUTH, WEST]
                    orientations.remove(rotation)
                    particle.orientation = random.choice(orientations)
                else:
                    particle.orientation = rotation

            # Update position based on orientation
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

            # Constrain particles to valid map area
            particle.x = max(0, min(GRID_WIDTH - 1, particle.x))
            particle.y = max(0, min(GRID_HEIGHT - 1, particle.y))

            # Reduce weight if particle moved to invalid position
            if not self._is_valid_position(int(round(particle.x)), int(round(particle.y))):
                particle.weight *= 0.1

        # Calculate new average position
        self._normalize_weights()
        new_x = sum(p.x * p.weight for p in self.particles)
        new_y = sum(p.y * p.weight for p in self.particles)

        # Check if movement was realistic
        expected_dist = math.sqrt(dx ** 2 + dy ** 2)
        actual_dist = math.sqrt((new_x - prev_x) ** 2 + (new_y - prev_y) ** 2)

        # If movement discrepancy is too large, reduce confidence
        if abs(actual_dist - expected_dist) > 0.5 and expected_dist > 0.1:
            self.position_confidence *= 0.8
            logger.warning(
                f"Unexpected particle movement detected. Expected: {expected_dist:.2f}, Actual: {actual_dist:.2f}")

        logger.debug(f"Updated particles after movement dx={dx}, dy={dy}, rotation={rotation}")

    def update_particle_weights(self, sensor_measurements):
        """
        Update particle weights based on sensor measurements with improved weighting.

        Args:
            sensor_measurements: Dict of sensor readings
        """
        if not self.particles:
            self.initialize_particles()
            return

        # Extract sensor data
        wall_distance = sensor_measurements.get('wall_distance')
        left_color = sensor_measurements.get('left_color')
        right_color = sensor_measurements.get('right_color')

        # Get current orientation from drive system
        current_orientation = self.drive.orientation

        # Check for significant disparity between expected and actual orientation
        if self.expected_orientation != current_orientation:
            logger.warning(f"Orientation mismatch: expected {self.expected_orientation}, got {current_orientation}")
            # Reduce confidence if orientations don't match
            self.position_confidence *= 0.7

        total_weight = 0
        for particle in self.particles:
            # Start with current weight, but apply small decay to prevent overconfidence
            w = particle.weight * 0.95

            # Get integer grid position
            px, py = int(round(particle.x)), int(round(particle.y))

            # Check if position is valid
            if not self._is_valid_position(px, py):
                particle.weight = 0.001  # Very low but non-zero weight
                continue

            # Calculate weight based on color sensing
            expected_color = self._expected_color(px, py)
            if expected_color and (left_color or right_color):
                # Check if either sensor matches expected color
                color_match = (left_color == expected_color or right_color == expected_color)

                if color_match:
                    w *= 1.5  # Boost weight for color match
                else:
                    w *= 0.7  # Reduce weight for color mismatch

                # Special case for boundary detection (black lines)
                if left_color == COLOR_BLACK or right_color == COLOR_BLACK:
                    # Check if particle is near grid line
                    x_frac = abs(particle.x - round(particle.x))
                    y_frac = abs(particle.y - round(particle.y))

                    if x_frac < 0.1 or y_frac < 0.1:
                        w *= 2.0  # Significant boost for grid line alignment
                        self.grid_line_confidence = 0.9
                        self.last_grid_line_detection = time.time()
                    else:
                        w *= 0.3  # Significant penalty for detecting grid line in wrong position

            # Update weight based on wall distance if available
            if wall_distance is not None and particle.orientation in DIRECTION_VECTORS:
                expected_distance = self._expected_wall_distance(px, py, particle.orientation)
                if expected_distance is not None:
                    # Calculate weight based on difference between expected and measured
                    distance_diff = abs(wall_distance - expected_distance)

                    # Using Gaussian model for distance measurement
                    distance_weight = math.exp(-distance_diff ** 2 / (2 * MCL_SENSOR_NOISE ** 2))

                    # More weight to distance measurements when confidence is low
                    if self.position_confidence < 0.5:
                        w *= (0.2 + 0.8 * distance_weight)  # Blend with existing weight
                    else:
                        w *= (0.5 + 0.5 * distance_weight)

            # Orientation consistency check
            if particle.orientation != current_orientation:
                w *= 0.8  # Penalize particles with wrong orientation

            particle.weight = max(0.0001, min(w, 5.0))  # Clamp weight to reasonable range
            total_weight += particle.weight

        # Normalize weights if total is non-zero
        if total_weight > 0:
            for p in self.particles:
                p.weight /= total_weight

        # Calculate dispersion to detect localization problems
        self._calculate_dispersion()

        # Check grid line confidence decay over time
        if self.last_grid_line_detection:
            time_since_grid = time.time() - self.last_grid_line_detection
            if time_since_grid > 5.0:  # 5 seconds since last grid line
                self.grid_line_confidence *= 0.9  # Decay confidence

        logger.debug(f"Updated particle weights based on sensor data")

    def _calculate_dispersion(self):
        """Calculate particle dispersion to detect localization problems"""
        if not self.particles:
            return

        # Calculate weighted mean position
        mean_x = sum(p.x * p.weight for p in self.particles)
        mean_y = sum(p.y * p.weight for p in self.particles)

        # Calculate variance
        var_x = sum(((p.x - mean_x) ** 2) * p.weight for p in self.particles)
        var_y = sum(((p.y - mean_y) ** 2) * p.weight for p in self.particles)

        # Calculate total dispersion (variance)
        dispersion = var_x + var_y

        # Update confidence based on dispersion
        self.position_confidence = max(0.1, min(1.0, 1.0 - dispersion))

        # Log warning if dispersion is high
        if dispersion > self.dispersion_threshold:
            logger.warning(f"High particle dispersion detected: {dispersion:.3f}. Low confidence in position estimate.")

            # If dispersion is extremely high, consider reinitializing
            if dispersion > self.dispersion_threshold * 2 and self.position_confidence < 0.3:
                logger.warning("Extreme position uncertainty detected. Consider relocalizing.")
                self.recovery_mode = True

    def resample_particles(self):
        """
        Improved resampling algorithm with adaptive resampling
        based on effective sample size.
        """
        if not self.particles:
            self.initialize_particles()
            return

        # Calculate effective sample size
        weights = [p.weight for p in self.particles]
        n_eff = 1.0 / sum(w * w for w in weights)
        threshold = MCL_PARTICLE_COUNT * MCL_RESAMPLING_THRESHOLD

        # Only resample if effective sample size is below threshold
        if n_eff < threshold:
            # Create cumulative sum of weights
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
                while i < len(cum_weights) - 1 and u > cum_weights[i]:
                    i += 1

                # Create new particle based on selected particle with small jitter
                old = self.particles[i]
                jitter_x = random.gauss(0, 0.02)  # Small position noise during resampling
                jitter_y = random.gauss(0, 0.02)

                new_particle = Particle(
                    old.x + jitter_x,
                    old.y + jitter_y,
                    old.orientation,
                    1.0 / MCL_PARTICLE_COUNT
                )

                # Ensure particle is within bounds
                new_particle.x = max(0, min(GRID_WIDTH - 1, new_particle.x))
                new_particle.y = max(0, min(GRID_HEIGHT - 1, new_particle.y))

                new_particles.append(new_particle)

            self.particles = new_particles
            logger.debug(f"Resampled particles (effective sample size: {n_eff:.1f}/{MCL_PARTICLE_COUNT})")
        else:
            logger.debug(f"Skipped resampling (effective sample size: {n_eff:.1f}/{threshold:.1f})")

    def update_position_estimate(self):
        """Update estimated position based on particle distribution with improved filtering."""
        if not self.particles:
            self.initialize_particles()
            return

        # Calculate weighted average position
        total_x = 0
        total_y = 0
        total_weight = 0
        orientation_counts = {NORTH: 0, EAST: 0, SOUTH: 0, WEST: 0}

        for p in self.particles:
            total_x += p.x * p.weight
            total_y += p.y * p.weight
            total_weight += p.weight

            if p.orientation in orientation_counts:
                orientation_counts[p.orientation] += p.weight

        if total_weight > 0:
            avg_x = total_x / total_weight
            avg_y = total_y / total_weight

            # Store in history for temporal smoothing
            self.position_history.append([avg_x, avg_y])

            # Apply temporal smoothing using weighted average of recent positions
            # More recent positions have higher weight
            smoothed_x = 0
            smoothed_y = 0
            total_weight = 0

            for i, pos in enumerate(self.position_history):
                weight = (i + 1) / sum(range(1, len(self.position_history) + 1))
                smoothed_x += pos[0] * weight
                smoothed_y += pos[1] * weight
                total_weight += weight

            if total_weight > 0:
                # Update estimated position with smoothed value
                self.estimated_position = [smoothed_x, smoothed_y]

                # Determine most likely orientation
                if orientation_counts:
                    most_likely_orientation = max(orientation_counts.items(), key=lambda x: x[1])[0]

                    # Only update drive orientation if confidence is high enough
                    if self.position_confidence > 0.7 and most_likely_orientation != self.drive.orientation:
                        logger.info(
                            f"Correcting orientation from {self.drive.orientation} to {most_likely_orientation}")
                        self.drive.orientation = most_likely_orientation

                # Update last valid position if confidence is high
                if self.position_confidence > 0.6:
                    self.last_valid_position = self.estimated_position.copy()
                    self.last_valid_orientation = self.drive.orientation

                logger.debug(f"Updated position: {self.estimated_position}, confidence: {self.position_confidence:.2f}")

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

    def localize(self, full_rotation=False):
        """
        Perform active localization to determine position with improved sensing.

        Args:
            full_rotation: Whether to perform a full 360° rotation for better localization

        Returns:
            bool: True if localization successful (high confidence)
        """
        logger.info(f"Starting active localization (confidence: {self.position_confidence:.2f})")

        # Store initial orientation to return to it
        initial_orientation = self.drive.orientation

        # First check for grid lines or other landmarks
        self._check_for_landmarks()

        # Get current sensor data
        sensor_data = self.sensors.get_sensor_data()
        self.update_particle_weights(sensor_data)

        # If grid line detected recently, try to align with it precisely
        if self.last_grid_line_detection and time.time() - self.last_grid_line_detection < 2.0:
            self.align_with_grid()

        # If confidence is still low, perform active sensing (rotate to gather information)
        if self.position_confidence < 0.5 or full_rotation:
            logger.info("Performing active sensing to improve localization")

            # Take initial readings in current orientation
            sensor_data = self.sensors.get_sensor_data()
            self.update_particle_weights(sensor_data)

            # Perform rotation to gather sensor data from different directions
            orientations = [NORTH, EAST, SOUTH, WEST]
            current_idx = orientations.index(self.drive.orientation)

            if full_rotation:
                for i in range(1, 4):  # 3 turns to make a full circle
                    next_idx = (current_idx + i) % 4
                    next_orientation = orientations[next_idx]

                    # Turn to next orientation in sequence
                    self.drive.turn(next_orientation)
                    time.sleep(0.3)  # Pause to get stable readings

                    # Get sensor data and update
                    sensor_data = self.sensors.get_sensor_data()
                    self.update_particle_weights(sensor_data)
                    self.resample_particles()
            else:
                # Just look left and right from current orientation
                left_idx = (current_idx - 1) % 4
                right_idx = (current_idx + 1) % 4

                # Look left
                self.drive.turn(orientations[left_idx])
                time.sleep(0.3)
                sensor_data = self.sensors.get_sensor_data()
                self.update_particle_weights(sensor_data)

                # Look right (need to turn 180° from left orientation)
                self.drive.turn(orientations[right_idx])
                time.sleep(0.3)
                sensor_data = self.sensors.get_sensor_data()
                self.update_particle_weights(sensor_data)

            # Return to original orientation using the shortest path
            self.drive.turn(initial_orientation)
            time.sleep(0.3)

        # Final update after sensing
        sensor_data = self.sensors.get_sensor_data()
        self.update_particle_weights(sensor_data)
        self.resample_particles()
        self.update_position_estimate()

        # Check for grid alignment one more time
        self._check_for_landmarks()

        # Final update after all measurements
        self.update_position_estimate()

        logger.info(
            f"Localization complete - Position: ({self.estimated_position[0]:.2f}, {self.estimated_position[1]:.2f}), "
            f"Confidence: {self.position_confidence:.2f}")

        return self.position_confidence > 0.6

    def _check_for_landmarks(self):
        """Check for landmarks to improve localization with sensor offsets"""

        # Define sensor offsets relative to robot center
        HORIZONTAL_SENSOR_OFFSET = 2  # cm
        FORWARD_SENSOR_OFFSET_Y = 4  # cm forward of robot center
        ULTRASONIC_FORWARD_OFFSET = 3  # cm forward of robot center (adjust based on your robot design)

        # Convert to grid units for calculations
        sensor_offset_x_grid = HORIZONTAL_SENSOR_OFFSET / BLOCK_SIZE
        sensor_offset_y_grid = FORWARD_SENSOR_OFFSET_Y / BLOCK_SIZE
        us_offset_y_grid = ULTRASONIC_FORWARD_OFFSET / BLOCK_SIZE

        landmark_detected = False

        # Check for black grid lines
        on_black, sensor_side = self.sensors.is_on_black_line()
        if on_black[0]:  # If black line detected
            self.last_grid_line_detection = time.time()
            self.grid_line_confidence = 0.9

            # Calculate robot center position based on which sensor detected the line
            robot_offset_x = 0
            if sensor_side == "left":
                robot_offset_x = -sensor_offset_x_grid
            elif sensor_side == "right":
                robot_offset_x = sensor_offset_x_grid

            # Always apply the forward offset (assumes forward orientation)
            robot_offset_y = -sensor_offset_y_grid  # Negative because sensor is in front

            # Rotate offsets based on current orientation
            rotated_offset_x, rotated_offset_y = self._rotate_offset(
                robot_offset_x, robot_offset_y, self.drive.orientation)

            # Update particle weights with proper offsets
            for p in self.particles:
                # Calculate particle's sensor position
                sensor_x = p.x + rotated_offset_x
                sensor_y = p.y + rotated_offset_y

                # Check if particle's sensor (not center) is near a grid line
                x_near_line = abs(sensor_x - round(sensor_x)) < 0.1
                y_near_line = abs(sensor_y - round(sensor_y)) < 0.1

                if x_near_line or y_near_line:
                    p.weight *= 2.0  # Boost weight for particles near grid lines
                else:
                    p.weight *= 0.3  # Reduce weight for others

            landmark_detected = True

        # Check for entrance line (orange)
        found_orange, orange_side = self.sensors.check_for_entrance()
        if found_orange:
            # Calculate offsets similar to black line detection
            robot_offset_x = 0
            if orange_side == "left":
                robot_offset_x = -sensor_offset_x_grid
            elif orange_side == "right":
                robot_offset_x = sensor_offset_x_grid

            # Apply forward offset
            robot_offset_y = -sensor_offset_y_grid

            # Rotate offsets
            rotated_offset_x, rotated_offset_y = self._rotate_offset(
                robot_offset_x, robot_offset_y, self.drive.orientation)

            # Update particles near entrance
            entrance_pos = BURNING_ROOM_ENTRY
            for p in self.particles:
                # Calculate distance from particle's sensor to entrance
                sensor_x = p.x + rotated_offset_x
                sensor_y = p.y + rotated_offset_y

                dist_to_entrance = math.sqrt((sensor_x - entrance_pos[0]) ** 2 +
                                             (sensor_y - entrance_pos[1]) ** 2)

                if dist_to_entrance < 1.5:  # Within 1.5 grid cells of entrance
                    p.weight *= 3.0  # Significantly boost weight
                else:
                    p.weight *= 0.2  # Significantly reduce weight

            landmark_detected = True

        # Check for wall detection with ultrasonic sensor
        if self.sensors.has_ultrasonic:
            wall_distance = self.sensors.get_wall_distance()
            if wall_distance is not None and wall_distance < BLOCK_SIZE * 3:  # Only use reliable close measurements
                # Calculate ultrasonic sensor position using its offset
                us_offset_x, us_offset_y = self._rotate_offset(
                    0, us_offset_y_grid, self.drive.orientation)

                for p in self.particles:
                    # Calculate expected distance for this particle's sensor
                    expected_distance = self._expected_wall_distance(
                        p.x + us_offset_x, p.y + us_offset_y, p.orientation)

                    if expected_distance is not None:
                        # Calculate weight based on difference
                        distance_diff = abs(wall_distance - expected_distance)
                        distance_weight = math.exp(-distance_diff ** 2 / (2 * 5 ** 2))  # Gaussian with σ=5cm

                        # Apply weight
                        p.weight *= (0.3 + 0.7 * distance_weight)

                landmark_detected = True

        # Normalize weights if any landmark was detected
        if landmark_detected:
            self._normalize_weights()
            logger.info(f"Landmark detection updated particle weights")

        return landmark_detected

    def _rotate_offset(self, offset_x, offset_y, orientation):
        """Rotate sensor offsets based on robot orientation"""
        if orientation == NORTH:
            return offset_x, offset_y
        elif orientation == EAST:
            return offset_y, -offset_x
        elif orientation == SOUTH:
            return -offset_x, -offset_y
        elif orientation == WEST:
            return -offset_y, offset_x
        return offset_x, offset_y  # D


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
        return False, "NONE"

    def adjust_drive_timing(self, forward_factor=None, turn_factor=None):
        """Adjust drive system timing parameters based on calibration."""
        if forward_factor:
            self.drive.forward_time_per_block *= forward_factor
            logger.info(f"Adjusted forward time to {self.drive.forward_time_per_block:.3f}s")

        if turn_factor:
            self.drive.turn_90_time *= turn_factor
            logger.info(f"Adjusted turn time to {self.drive.turn_90_time:.3f}s")