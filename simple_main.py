import logging
import time
import argparse
from utils.brick import Motor, EV3ColorSensor, EV3UltrasonicSensor, TouchSensor, wait_ready_sensors

from simple_src.new_drive_system import NewDriveSystem
from simple_src.simple_navigation import SimpleNavigation
from simple_src.fire_extinguisher import FireExtinguisher
from simple_src.simple_mission_control import SimpleMissionControl
from simple_src.siren_controller import SirenController
from simple_src.sensor_system import SensorSystem

from simple_src.constants import (
    LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, DROPPER_MOTOR_PORT,
    LEFT_COLOR_PORT, RIGHT_COLOR_PORT, ULTRASONIC_PORT, TOUCH_PORT, HALLWAY_PATH,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler()]
)

logger = logging.getLogger("main")


class FirefighterRobot:

    def __init__(self):
        logger.info("Initializing Firefighter Robot")

        # Initialize hardware components
        self.left_motor = Motor(LEFT_MOTOR_PORT)
        self.right_motor = Motor(RIGHT_MOTOR_PORT)
        self.dropper_motor = Motor(DROPPER_MOTOR_PORT)

        # Initialize sensors
        self.left_color = EV3ColorSensor(LEFT_COLOR_PORT)
        self.right_color = EV3ColorSensor(RIGHT_COLOR_PORT)
        self.ultrasonic = EV3UltrasonicSensor(ULTRASONIC_PORT)
        self.touch_sensor = TouchSensor(TOUCH_PORT)

        # Wait for sensors to initialize
        wait_ready_sensors()

        # Create subsystems using the improved components
        self.drive_system = NewDriveSystem(self.left_motor, self.right_motor)
        self.sensor_system = SensorSystem(
            self.left_color,
            self.right_color,
            self.ultrasonic,
            self.touch_sensor
        )
        self.extinguisher = FireExtinguisher(self.dropper_motor)
        self.navigation = SimpleNavigation(self.drive_system, self.sensor_system)
        self.siren = SirenController()

        # Initialize mission control
        self.mission_control = SimpleMissionControl(
            self.drive_system,
            self.sensor_system,
            self.navigation,
            self.extinguisher,
            self.siren
        )

        logger.info("Firefighter Robot initialization complete")

    def run_mission(self):
        """Run the complete firefighter mission."""
        logger.info("Starting firefighter mission")
        try:
            self.mission_control.run_mission()
        except KeyboardInterrupt:
            logger.info("Mission interrupted by user")
        except Exception as e:
            logger.error(f"Error during mission: {e}", exc_info=True)
        finally:
            self.stop()

    def stop(self):
        """Stop all robot systems."""
        logger.info("Stopping all systems")
        if hasattr(self, 'mission_control'):
            self.mission_control.stop_mission()
        if hasattr(self, 'drive_system'):
            self.drive_system.stop()
        if hasattr(self, 'siren'):
            self.siren.stop()

    def test_motors(self):
        """Test basic motor functionality."""
        logger.info("Testing motors")

        # Test simple movements
        logger.info("Testing forward movement")
        self.drive_system.advance_blocks(1)
        time.sleep(1)

        logger.info("Testing turns")
        self.drive_system.turn_90_right()
        time.sleep(1)
        self.drive_system.turn_90_left()
        time.sleep(1)

        logger.info("Motor tests complete")

    def test_sensors(self):
        """Test sensor functionality."""
        logger.info("Testing sensors")

        # Test color sensors
        left_color = self.sensor_system.get_color_left()
        right_color = self.sensor_system.get_color_right()
        logger.info(f"Color sensors - Left: {left_color}, Right: {right_color}")

        # Test ultrasonic sensor
        if self.sensor_system.has_ultrasonic:
            distance = self.sensor_system.get_wall_distance()
            logger.info(f"Ultrasonic sensor: {distance} cm")
        else:
            logger.info("Ultrasonic sensor not available")

        # Test grid line detection
        on_grid, position = self.sensor_system.is_on_black_line()
        logger.info(f"Grid line detection: {on_grid} ({position})")

        # Test fire detection
        fire_detected, side = self.sensor_system.check_for_fire()
        logger.info(f"Fire detection: {fire_detected} ({side})")

        logger.info("Sensor tests complete")

    def test_navigation(self):
        """Test basic navigation functionality."""
        logger.info("Testing navigation")

        # Test following hallway for first 2 steps
        logger.info("Testing partial hallway navigation")
        self.siren.start()

        # Follow first few steps of hallway path
        for i in range(min(3, len(HALLWAY_PATH) - 1)):
            next_pos = HALLWAY_PATH[i + 1]
            self.navigation.move_to_position(next_pos)
            time.sleep(1)

        self.siren.stop()
        logger.info("Navigation test complete")

    def test_grid_alignment(self):
        """
        Test the grid alignment functionality.
        Place robot where black grid lines are visible to sensors.
        """
        logger.info("Testing grid alignment functionality")

        # Try to align with grid lines
        logger.info("Attempting to align with grid lines...")
        aligned = self.navigation.align_with_grid()

        if aligned:
            logger.info("Grid alignment successful")
        else:
            logger.info("Grid alignment failed - check sensor positioning")

        # Displays sensor readings for debugging
        time.sleep(0.5)
        left_brightness, right_brightness = self.sensor_system.get_brightness_values()
        logger.info(f"Brightness values - Left: {left_brightness}, Right: {right_brightness}")
        logger.info(f"Black threshold: {self.sensor_system.black_threshold}")

        logger.info("Grid alignment test complete")

    def test_entrance_alignment(self):
        """
        Test the entrance line detection and alignment.
        Place robot near an orange line to test.
        """
        logger.info("Testing entrance (orange line) alignment")

        # First test if we detect orange
        found_orange, side = self.sensor_system.check_for_entrance()
        logger.info(f"Initial orange detection: {found_orange} (side: {side})")

        # Try to align with entrance
        aligned = self.navigation.align_with_entrance()

        if aligned:
            logger.info("Entrance alignment successful!")
        else:
            logger.info("Entrance alignment failed - check orange line visibility")

        logger.info("Entrance alignment test complete")

    def test_calibration(self):
        """
        Test the first movement calibration logic.
        This will move forward one block and calibrate timing parameters.
        """
        logger.info("Testing first movement calibration")

        # First measure the distance
        if self.sensor_system.has_ultrasonic:
            initial_distance = self.sensor_system.get_wall_distance()
            logger.info(f"Initial distance reading: {initial_distance} cm")

        # Perform calibration
        self.navigation.calibrate_first_movement()

        # Log the adjusted parameters
        if hasattr(self.drive_system, 'forward_time_per_block'):
            logger.info(f"Calibrated forward time: {self.drive_system.forward_time_per_block:.2f}s")
        if hasattr(self.drive_system, 'turn_90_time'):
            logger.info(f"Calibrated turn time: {self.drive_system.turn_90_time:.2f}s")

        logger.info("Calibration test complete")

    def test_emergency_stop(self):
        """
        Test the emergency stop button functionality.
        Press the touch sensor to trigger the stop.
        """
        logger.info("Testing emergency stop (touch sensor)")
        logger.info("Press the touch sensor to trigger emergency stop")

        # Start a simple movement to demonstrate stopping
        self.drive_system.set_motors_turn_left()

        # Monitor touch sensor for 10 seconds or until pressed
        timeout = time.time() + 10
        while time.time() < timeout:
            if self.sensor_system.is_emergency_pressed():
                logger.info("EMERGENCY STOP ACTIVATED")
                self.stop()
                break
            time.sleep(0.1)

        if not self.sensor_system.is_emergency_pressed():
            logger.info("Emergency stop test timed out - stopping anyway")
            self.stop()

        logger.info("Emergency stop test complete")

    def test_movement_accuracy(self):
        """
        Test movement accuracy by performing a square pattern and returning to start.
        Useful for verifying turn angles and movement distances.
        """
        logger.info("Testing movement accuracy with square pattern")

        # Record initial ultrasonic reading if available
        initial_distance = None
        if self.sensor_system.has_ultrasonic:
            initial_distance = self.sensor_system.get_wall_distance()
            logger.info(f"Initial distance: {initial_distance} cm")

        # Move in a square pattern: forward, right, forward, right, forward, right, forward
        for i in range(4):
            logger.info(f"Square side {i + 1}/4")
            self.drive_system.advance_blocks(1)
            time.sleep(0.5)
            self.drive_system.turn_90_right()
            time.sleep(0.5)

        # Check if we're back near our starting position
        if self.sensor_system.has_ultrasonic:
            final_distance = self.sensor_system.get_wall_distance()
            logger.info(f"Final distance: {final_distance} cm")

            if initial_distance is not None:
                difference = abs(final_distance - initial_distance)
                logger.info(f"Distance difference from start: {difference:.1f} cm")

                if difference < 10:
                    logger.info("Square movement was reasonably accurate")
                else:
                    logger.info("Square movement showed significant drift")

        logger.info("Movement accuracy test complete")

    def test_siren(self):
        """Test the siren sound."""
        logger.info("Testing siren for 5 seconds")
        self.siren.start()
        time.sleep(5)
        self.siren.stop()
        logger.info("Siren test complete")

    def test_dropper(self):
        """Test the cube dropping mechanism."""
        logger.info("Testing cube dropper mechanism")
        self.extinguisher.test_drop_mechanism()
        logger.info("Dropper test complete")


def main():
    """Main entry point for the program with enhanced testing options."""
    parser = argparse.ArgumentParser(description='Firefighter Robot Control')

    # Basic test options
    parser.add_argument('--test-motors', action='store_true', help='Test basic motor functionality')
    parser.add_argument('--test-sensors', action='store_true', help='Test sensor functionality')
    parser.add_argument('--test-navigation', action='store_true', help='Test navigation functionality')

    # New specialized test options
    parser.add_argument('--test-grid', action='store_true', help='Test grid alignment functionality')
    parser.add_argument('--test-entrance', action='store_true', help='Test entrance line detection')
    parser.add_argument('--test-calibration', action='store_true', help='Test movement calibration')
    parser.add_argument('--test-emergency', action='store_true', help='Test emergency stop button')
    parser.add_argument('--test-accuracy', action='store_true', help='Test movement accuracy with square pattern')
    parser.add_argument('--test-siren', action='store_true', help='Test siren sound')
    parser.add_argument('--test-dropper', action='store_true', help='Test cube dropper mechanism')

    args = parser.parse_args()

    # Create robot
    robot = FirefighterRobot()

    try:
        if args.test_motors:
            robot.test_motors()
        elif args.test_sensors:
            robot.test_sensors()
        elif args.test_navigation:
            robot.test_navigation()
        elif args.test_grid:
            robot.test_grid_alignment()
        elif args.test_entrance:
            robot.test_entrance_alignment()
        elif args.test_calibration:
            robot.test_calibration()
        elif args.test_emergency:
            robot.test_emergency_stop()
        elif args.test_accuracy:
            robot.test_movement_accuracy()
        elif args.test_siren:
            robot.test_siren()
        elif args.test_dropper:
            robot.test_dropper()
        else:
            robot.run_mission()

    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        robot.stop()


if __name__ == "__main__":
    main()