import logging
import math

from src.constants import (
    NB_COLOR_SAMPLING,
    COLOR_RED, COLOR_ORANGE, COLOR_GREEN
)
import src.color_matching as colo
from collections import deque
import time

logger = logging.getLogger("sensors")


class SensorSystem:
    """Handles all sensors and color detection."""

    def __init__(self, left_color_sensor, right_color_sensor, ultrasonic_sensor=None, touch_sensor=None):
        """
        Initialize the sensor system.

        Args:
            left_color_sensor: Left color sensor object
            right_color_sensor: Right color sensor object
            ultrasonic_sensor: Ultrasonic sensor object (optional)
            touch_sensor: Touch sensor object (optional)
        """
        # Initialize sensors
        self.black_threshold = None
        self.left_color = left_color_sensor
        self.right_color = right_color_sensor
        self.touch_sensor = touch_sensor
        self.ultrasonic = ultrasonic_sensor

        # Wait for sensors to be ready
        self.left_color.wait_ready()
        self.right_color.wait_ready()
        if self.touch_sensor:
            self.touch_sensor.wait_ready()

        # Check if ultrasonic sensor is working
        self.has_ultrasonic = False
        if ultrasonic_sensor:
            try:
                self.ultrasonic.wait_ready()
                test_reading = self.ultrasonic.get_cm()
                if test_reading is not None:
                    self.has_ultrasonic = True
                    logger.info(f"Ultrasonic sensor initialized, test reading: {test_reading}cm")
            except Exception as e:
                logger.warning(f"Ultrasonic sensor initialization failed: {e}")

        # Sensor data filtering
        self.distance_history = deque(maxlen=5)  # Last 5 distance readings
        self.left_color_history = deque(maxlen=NB_COLOR_SAMPLING)  # Recent color detections
        self.right_color_history = deque(maxlen=NB_COLOR_SAMPLING)  # Recent color detections

        # Sensor calibration values
        self.ultrasonic_offset = 0  # Adjustment for ultrasonic readings
        self.calibrate_color_thresholds()
        self.color_confidence_threshold = 0.6  # Minimum confidence for color detection

        logger.info("Sensor system initialized")

    def calibrate_color_thresholds(self):
        """Calibrate color detection thresholds based on environment."""
        # Essentially analyzes how much light there is to come up with a realistic black threshold
        # Ensure the color sensors are not on black line during this calibration....l
        samples = 10
        left_samples = []
        right_samples = []

        logger.info("Calibrating color detection thresholds...")

        for _ in range(samples):
            left_rgb = self.left_color.get_rgb()
            right_rgb = self.right_color.get_rgb()

            if left_rgb and len(left_rgb) == 3:
                left_samples.append(sum(left_rgb))

            if right_rgb and len(right_rgb) == 3:
                right_samples.append(sum(right_rgb))

            time.sleep(0.1)

        # Calculate statistics
        if left_samples:
            left_avg = sum(left_samples) / len(left_samples)
            logger.info(f"Left color sensor average brightness: {left_avg:.1f}")

        if right_samples:
            right_avg = sum(right_samples) / len(right_samples)
            logger.info(f"Right color sensor average brightness: {right_avg:.1f}")

        # Adjust black threshold if needed
        if left_samples and right_samples:
            # Set black threshold as 40% of average brightness
            avg_brightness = (sum(left_samples) + sum(right_samples)) / (len(left_samples) + len(right_samples))
            self.black_threshold = int(avg_brightness * 0.35)
            logger.info(f"Black detection threshold set to: {self.black_threshold}")

    def track_grid_line(self, samples=5):
        """
        Track a grid line's position relative to the sensors over multiple readings.
        Useful for detecting whether the robot is approaching, crossing, or moving away from a line.

        Returns:
            dict: Line tracking information
        """
        results = []

        for _ in range(samples):
            on_line, position = self.is_on_black_line()
            results.append((on_line, position))
            time.sleep(0.05)

        # Count occurrences
        on_line_count = sum(1 for r in results if r[0])
        positions = [r[1] for r in results if r[0]]
        position_counts = {}

        for pos in positions:
            if pos in position_counts:
                position_counts[pos] += 1
            else:
                position_counts[pos] = 1

        # Analyze pattern
        line_status = {
            'detected': on_line_count > 0,
            'confidence': on_line_count / samples,
            'positions': position_counts
        }

        # Determine if crossing, approaching, or moving away
        if len(results) >= 3:
            # Simple pattern analysis
            if not results[0][0] and results[-1][0]:
                line_status['pattern'] = 'approaching'
            elif results[0][0] and not results[-1][0]:
                line_status['pattern'] = 'moving_away'
            elif all(r[0] for r in results):
                line_status['pattern'] = 'on_line'
            else:
                line_status['pattern'] = 'mixed'

        return line_status

    def get_line_alignment_suggestion(self):
        """
        Analyze sensor readings to suggest alignment corrections.

        Returns:
            tuple: (needs_adjustment, adjustment_type, confidence)
                - needs_adjustment: Boolean indicating if adjustment is needed
                - adjustment_type: One of "forward", "backward", "left", "right", or None
                - confidence: Confidence level for the suggestion (0.0-1.0)
        """
        line_info = self.track_grid_line(samples=3)

        # No line detected
        if not line_info['detected']:
            return True, "forward", 0.8  # Move forward to find a line

        # Calculate most common position
        if line_info['positions']:
            positions = line_info['positions']
            most_common = max(positions, key=positions.get)
            confidence = positions[most_common] / sum(positions.values())

            if most_common == "left":
                return True, "right", confidence  # Turn right to center
            elif most_common == "right":
                return True, "left", confidence  # Turn left to center
            elif most_common == "both":
                if line_info.get('pattern') == 'approaching':
                    return True, "forward", confidence  # Keep moving forward to cross line
                elif line_info.get('pattern') == 'moving_away':
                    return True, "backward", confidence  # Move backward to get back on line
                else:
                    return False, None, confidence  # Already aligned

        return False, None, 0.5

    def get_color_left(self):
        """
        Get robust color reading from left color sensor using consensus.

        Returns:
            str: Detected color, or None if unreliable
        """
        # Clear history if it's getting too inconsistent
        if len(self.left_color_history) > NB_COLOR_SAMPLING // 2:
            if len(set(self.left_color_history)) > NB_COLOR_SAMPLING // 2:  # Too many different colors
                self.left_color_history.clear()

        # Get multiple RGB samples
        rgb_values = []
        for _ in range(NB_COLOR_SAMPLING):
            rgb = self.left_color.get_rgb()
            if rgb and len(rgb) == 3:  # Ensure valid RGB
                rgb_values.append(rgb)

        if not rgb_values:
            logger.warning("Left color sensor returned no valid RGB values")
            return None

        # Use color matching algorithm
        try:
            color_match = colo.match_unknown_color(rgb_values)
            confidence = self._calculate_color_confidence(rgb_values, color_match)

            # Add to history
            self.left_color_history.append(color_match)

            # Only return color if confidence is high enough
            if confidence >= self.color_confidence_threshold:
                logger.debug(f"Left color: {color_match} (confidence: {confidence:.2f})")
                return color_match
            else:
                # Use majority voting from history
                if self.left_color_history:
                    color_counts = {}
                    for color in self.left_color_history:
                        color_counts[color] = color_counts.get(color, 0) + 1
                    majority_color = max(color_counts, key=color_counts.get)
                    majority_count = color_counts[majority_color]

                    if majority_count >= len(self.left_color_history) * 0.6:  # At least 60% agreement
                        logger.debug(f"Left color (history majority): {majority_color}")
                        return majority_color

                logger.debug(f"Left color detection uncertain: {color_match} (confidence: {confidence:.2f})")
                return None
        except Exception as e:
            logger.error(f"Error in left color detection: {e}")
            return None

    def get_color_right(self):
        """
        Get robust color reading from right color sensor using consensus.

        Returns:
            str: Detected color, or None if unreliable
        """
        # Same approach as get_color_left
        if len(self.right_color_history) > NB_COLOR_SAMPLING // 2:
            if len(set(self.right_color_history)) > NB_COLOR_SAMPLING // 2:  # Too many different colors
                self.right_color_history.clear()

        rgb_values = []
        for _ in range(NB_COLOR_SAMPLING):
            rgb = self.right_color.get_rgb()
            if rgb and len(rgb) == 3:
                rgb_values.append(rgb)

        if not rgb_values:
            logger.warning("Right color sensor returned no valid RGB values")
            return None

        try:
            color_match = colo.match_unknown_color(rgb_values)
            confidence = self._calculate_color_confidence(rgb_values, color_match)

            self.right_color_history.append(color_match)

            if confidence >= self.color_confidence_threshold:
                logger.debug(f"Right color: {color_match} (confidence: {confidence:.2f})")
                return color_match
            else:
                if self.right_color_history:
                    color_counts = {}
                    for color in self.right_color_history:
                        color_counts[color] = color_counts.get(color, 0) + 1
                    majority_color = max(color_counts, key=color_counts.get)
                    majority_count = color_counts[majority_color]

                    if majority_count >= len(self.right_color_history) * 0.6:
                        logger.debug(f"Right color (history majority): {majority_color}")
                        return majority_color

                logger.debug(f"Right color detection uncertain: {color_match} (confidence: {confidence:.2f})")
                return None
        except Exception as e:
            logger.error(f"Error in right color detection: {e}")
            return None

    def _calculate_color_confidence(self, rgb_values, color_match):
        """
        Calculate confidence of color detection based on consistency.

        Args:
            rgb_values: List of RGB readings
            color_match: Detected color

        Returns:
            float: Confidence value from 0 to 1
        """
        # Use distance to color model center as confidence metric
        # This would be ideally calculated from color_matching module
        # For now, use a simple proxy: consistency of RGB values
        if not rgb_values:
            return 0

        # Calculate average RGB
        avg_r = sum(rgb[0] for rgb in rgb_values) / len(rgb_values)
        avg_g = sum(rgb[1] for rgb in rgb_values) / len(rgb_values)
        avg_b = sum(rgb[2] for rgb in rgb_values) / len(rgb_values)

        # Calculate variance (dispersion from average)
        var_r = sum((rgb[0] - avg_r) ** 2 for rgb in rgb_values) / len(rgb_values)
        var_g = sum((rgb[1] - avg_g) ** 2 for rgb in rgb_values) / len(rgb_values)
        var_b = sum((rgb[2] - avg_b) ** 2 for rgb in rgb_values) / len(rgb_values)

        total_variance = var_r + var_g + var_b

        # Convert to confidence (lower variance = higher confidence)
        # Scale to 0-1 range (heuristic: 5000 is typical max variance)
        max_expected_variance = 5000
        confidence = max(0, min(1, 1 - (total_variance / max_expected_variance)))

        return confidence

    def get_brightness_values(self):
        """
        Get brightness values (sum of RGB) from both color sensors.

        Returns:
            tuple: (left_brightness, right_brightness)
        """
        left_rgb = self.left_color.get_rgb()
        right_rgb = self.right_color.get_rgb()

        left_brightness = sum(left_rgb) if left_rgb else 0
        right_brightness = sum(right_rgb) if right_rgb else 0

        return left_brightness, right_brightness

    def check_color_match(self):
        """Returns Tuple (bool Match, left color, right color)"""
        left_color = self.get_color_left()
        right_color = self.get_color_right()
        return left_color == right_color, left_color, right_color

    def check_for_color(self, target_color):
        """
        Check if either sensor reliably detects the target color.
        Uses majority voting for more reliable detection.
        Returns ( True if any, which sensors)
        """
        left_color = self.get_color_left()
        right_color = self.get_color_right()

        # Check which sensors detect the target color
        left_match = (left_color == target_color)
        right_match = (right_color == target_color)

        if left_match and right_match:
            return True, "BOTH"
        elif left_match:
            return True, "LEFT"
        elif right_match:
            return True, "RIGHT"
        else:
            return False, "NONE"

    def check_for_fire(self):
        """Check for red color (fire)."""
        return self.check_for_color(COLOR_RED)

    def check_for_entrance(self):
        """Check for orange color (entrance line)."""
        return self.check_for_color(COLOR_ORANGE)

    def check_for_furniture(self):
        """Check for green color (furniture)."""
        return self.check_for_color(COLOR_GREEN)

    def is_emergency_pressed(self):
        """Check if emergency button is pressed."""
        return self.touch_sensor.is_pressed()

    # Stuff for US
    def calibrate_ultrasonic(self, known_distance=None):
        """
        Calibrate ultrasonic sensor against a known distance.

        Args:
            known_distance: Actual distance in cm, if available
        """
        if not self.has_ultrasonic:
            logger.warning("Cannot calibrate: ultrasonic sensor not available")
            return

        # Take multiple readings
        readings = []
        for _ in range(10):
            dist = self.ultrasonic.get_cm()
            if dist is not None:
                readings.append(dist)
            time.sleep(0.1)

        if not readings:
            logger.warning("Calibration failed: no valid readings obtained")
            return

        # Calculate average reading
        avg_reading = sum(readings) / len(readings)

        # If known distance is provided, calculate offset
        if known_distance is not None:
            self.ultrasonic_offset = known_distance - avg_reading
            logger.info(f"Ultrasonic calibrated with offset {self.ultrasonic_offset}cm")
        else:
            # Just log the consistency of readings
            variance = sum((r - avg_reading) ** 2 for r in readings) / len(readings)
            std_dev = math.sqrt(variance)
            logger.info(f"Ultrasonic self-check: avg={avg_reading:.1f}cm, std_dev={std_dev:.1f}cm")

    def is_on_black_line(self):
        """
        Grid line detection  that is calibrated with the brightness values...
        Detects if either or both sensors are on a black line.

        Returns:
            tuple: (on_line, sensor_position)
                - on_line: True if any sensor is on a black line, False otherwise
                - sensor_position: "both", "left", "right", or None
        """
        # Take multiple readings for reliability
        left_vals = []
        right_vals = []

        for _ in range(3):  # Take 3 samples
            left_brightness, right_brightness = self.get_brightness_values()
            left_vals.append(left_brightness)
            right_vals.append(right_brightness)
            time.sleep(0.01)  # Small delay between readings

        # Calculate average brightness
        left_avg = sum(left_vals) / len(left_vals) if left_vals else 0
        right_avg = sum(right_vals) / len(right_vals) if right_vals else 0

        # Determine if sensors are on a black line
        left_on_black = left_avg < self.black_threshold
        right_on_black = right_avg < self.black_threshold

        logger.debug(f"Left brightness: {left_avg:.1f}, Right brightness: {right_avg:.1f}")
        logger.debug(
            f"Black threshold: {self.black_threshold}, Left on black: {left_on_black}, Right on black: {right_on_black}")

        if left_on_black and right_on_black:
            return True, "both"
        elif left_on_black:
            return True, "left"
        elif right_on_black:
            return True, "right"
        else:
            return False, None

    def get_wall_distance(self):
        """
        Get filtered distance to nearest wall using ultrasonic sensor.
        Takes multiple readings and applies filtering ( this should be more reliable)
        """
        if not self.has_ultrasonic:
            logger.warning("Ultrasonic sensor not available")
            return None
        
        # flush previous distance readings
        self.distance_history.clear()

        # Take multiple readings
        readings = []
        num_samples = 8  # Take 8 samples
        for _ in range(num_samples):
            try:
                raw_dist = self.ultrasonic.get_cm()
                if raw_dist is not None:
                    # Apply calibration offset
                    readings.append(raw_dist + self.ultrasonic_offset)
            except Exception as e:
                logger.debug(f"Error in ultrasonic reading: {e}")
            time.sleep(0.01)  # Small delay between readings

        if not readings:
            logger.warning("No valid ultrasonic readings obtained")
            return None

        # Remove outliers before averaging
        if len(readings) >= 4:
            readings.sort()
            # Remove lowest and highest values if we have enough readings
            trimmed_readings = readings[1:-1]
        else:
            trimmed_readings = readings

        # Calculate average from trimmed readings
        avg_dist = sum(trimmed_readings) / len(trimmed_readings)

        # Add to history for trend analysis
        self.distance_history.append(avg_dist)

        # Apply median filter to recent history for additional stability
        if len(self.distance_history) >= 3:
            sorted_history = sorted(self.distance_history)
            filtered_dist = sorted_history[len(sorted_history) // 2]  # Median

            # Calculate standard deviation to assess measurement stability
            variance = sum((r - avg_dist) ** 2 for r in trimmed_readings) / len(trimmed_readings)
            std_dev = math.sqrt(variance)

            stability = "stable" if std_dev < 2.0 else "unstable"
            logger.debug(
                f"Ultrasonic distance: {filtered_dist:.1f}cm (avg: {avg_dist:.1f}cm, σ: {std_dev:.1f}, {stability})")
            return filtered_dist
        else:
            logger.debug(f"Ultrasonic distance: {avg_dist:.1f}cm (based on {len(trimmed_readings)} readings)")
            return avg_dist

    def get_sensor_data(self):
        """    Get readings from all sensors for debugging and calibration.
    """
        data = {
            'left_color': self.get_color_left(),
            'right_color': self.get_color_right(),
            'emergency_pressed': self.is_emergency_pressed()
        }

        if self.has_ultrasonic:
            data['wall_distance'] = self.get_wall_distance()

        return data
