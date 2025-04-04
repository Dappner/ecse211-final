import logging
from src.constants import (
    NB_COLOR_SAMPLING,
    COLOR_RED, COLOR_ORANGE, COLOR_GREEN, LEFT_COLOR_PORT, RIGHT_COLOR_PORT, ULTRASONIC_PORT, TOUCH_PORT
)
import src.color_matching as colo
from utils.brick import EV3ColorSensor, TouchSensor, EV3UltrasonicSensor

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger("sensors")

class SensorSystem:
    """Handles all sensors and color detection."""

    def __init__(self, left_port=LEFT_COLOR_PORT, right_port=RIGHT_COLOR_PORT,
                 us_port=ULTRASONIC_PORT, touch_port=TOUCH_PORT):
        """
        Initialize the sensor system.

        Args:
            left_port: Port for left color sensor
            right_port: Port for right color sensor
            us_port: Port for ultrasonic sensor
            touch_port: Port for touch sensor
        """
        # Initialize sensors
        self.left_color = EV3ColorSensor(left_port)
        self.right_color = EV3ColorSensor(right_port)
        self.touch_sensor = TouchSensor(touch_port)

        # Initialize ultrasonic sensor if available
        try:
            self.ultrasonic = EV3UltrasonicSensor(us_port)
            self.ultrasonic.wait_ready()
            logger.info("Ultrasonic sensor initialized")
            self.has_ultrasonic = True
        except Exception as e:
            logger.warning(f"Ultrasonic sensor initialization failed: {e}")
            self.ultrasonic = None
            self.has_ultrasonic = False

        # Wait for sensors to be ready
        self.left_color.wait_ready()
        self.right_color.wait_ready()
        self.touch_sensor.wait_ready()

        logger.info("Sensor system initialized")

    def get_color_left(self):
        """Get RGB values from left color sensor."""
        rgb = []
        for i in range(NB_COLOR_SAMPLING):
            rgb.append(self.left_color.get_rgb())

        match = colo.match_unknown_color(rgb)
        logger.info(f"LEFT COLOR: {match}")
        return match

    def get_color_right(self):
        """Get RGB values from right color sensor."""
        rgb = []
        for i in range(NB_COLOR_SAMPLING):
            rgb.append(self.right_color.get_rgb())

        match = colo.match_unknown_color(rgb)
        logger.info(f"RIGHT COLOR: {match}")
        return match

    def check_color_match(self):
        """Returns Tuple (bool Match, left color, right color)"""
        left_color = self.get_color_left()
        right_color = self.get_color_right()
        return left_color == right_color, left_color, right_color

    def check_for_color(self, target_color):
        """Check if either sensor detects the target color."""
        left_color = self.get_color_left()
        right_color = self.get_color_right()

        return left_color == target_color or right_color == target_color

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

    def get_wall_distance(self):
        """Get distance to nearest wall using ultrasonic sensor."""
        if not self.ultrasonic:
            logger.warning("Ultrasonic sensor not available")
            return None

        dist = self.ultrasonic.get_cm()
        logger.debug(f"Ultrasonic distance: {dist} cm")
        return dist

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
