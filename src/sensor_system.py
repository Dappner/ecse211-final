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
        self.color_confidence_threshold = 0.6  # Minimum confidence for color detection

        logger.info("Sensor system initialized")

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
            return True, "both"
        elif left_match:
            return True, "left"
        elif right_match:
            return True, "right"
        else:
            return False, "none"

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

    def get_wall_distance(self):
        """
        Get filtered distance to nearest wall using ultrasonic sensor.
        Takes multiple readings and applies filtering ( this should be more reliable)
        """
        if not self.has_ultrasonic:
            logger.warning("Ultrasonic sensor not available")
            return None

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
