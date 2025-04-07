import logging
import time
from utils.brick import Motor, EV3ColorSensor, EV3UltrasonicSensor, TouchSensor, wait_ready_sensors
import argparse
from src.drive_system import DriveSystem
from src.sensor_system import SensorSystem
from src.fire_extinguisher import FireExtinguisher
from src.navigation import Navigation
from src.siren_controller import SirenController
from src.mission_control import MissionControl
from src.constants import COLOR_RED, HALLWAY_PATH, LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, DROPPER_MOTOR_PORT, \
    LEFT_COLOR_PORT, RIGHT_COLOR_PORT, ULTRASONIC_PORT, TOUCH_PORT, EAST, SOUTH, WEST, NORTH

# Configure logging for the main module
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)

logger = logging.getLogger(__name__)


class FirefighterRobot:
    """Main robot class that integrates all components."""

    def __init__(self):
        # Initialize hardware components
        logger.info("Initializing Robot")
        self.left_motor = Motor(LEFT_MOTOR_PORT)
        self.right_motor = Motor(RIGHT_MOTOR_PORT)
        self.dropper_motor = Motor(DROPPER_MOTOR_PORT)

        # Initialize sensors
        self.left_color = EV3ColorSensor(LEFT_COLOR_PORT)
        self.right_color = EV3ColorSensor(RIGHT_COLOR_PORT)
        self.ultrasonic = EV3UltrasonicSensor(ULTRASONIC_PORT)
        self.touch_sensor = TouchSensor(TOUCH_PORT)

        # Create sub-systems
        wait_ready_sensors()

        self.drive_system = DriveSystem(self.left_motor, self.right_motor)
        self.sensor_system = SensorSystem(
            self.left_color, self.right_color, self.ultrasonic, self.touch_sensor
        )
        self.extinguisher = FireExtinguisher(self.dropper_motor)
        self.siren = SirenController()
        self.navigation = Navigation(self.drive_system, self.sensor_system)

        # Init mission control and provide it the subsystems
        self.mission_control = MissionControl(
                self.drive_system,
                self.sensor_system,
                self.navigation,
                self.extinguisher,
                self.siren
            )

        self.initialized = True

        logger.info("FirefighterRobot fully initialized and ready")

    def run_mission(self):
        """Execute the firefighting mission."""
        logger.info("Starting firefighter mission")

        try:
            self.mission_control.run_mission()
        except KeyboardInterrupt:
            logger.info("Mission interrupted by user")
        except Exception as e:
            logger.error(f"Error during mission: {e}")
        finally:
            self.mission_control.stop_mission()

    def calibration_test(self):
        """Run tests of basic components."""
        logger.info("Starting calibration tests")

        try:
            # Test siren
            logger.info("Testing siren")
            self.siren.start()

            # Test sensors (before moving so we can put something down)
            logger.info("Testing sensors")
            left_color = self.sensor_system.get_color_left()
            right_color = self.sensor_system.get_color_right()
            logger.info(f"Color readings - Left: {left_color}, Right: {right_color}")

            # Test fire detection and cube dropping

            if left_color == COLOR_RED:
                self.mission_control.drop_on_sensor("LEFT")
            elif right_color == COLOR_RED:
                self.mission_control.drop_on_sensor("RIGHT")

            time.sleep(1)

            # Test distance sensor if available
            if self.sensor_system.ultrasonic:
                distance = self.sensor_system.get_wall_distance()
                logger.info(f"Distance to wall: {distance} cm")

            time.sleep(3)

            self.siren.stop()
            logger.info("Calibration tests complete")

        except Exception as e:
            logger.error(f"Error during calibration: {e}")
        finally:
            # Clean shutdown
            self.drive_system.stop()
            self.siren.stop()

    def run_simple_path(self):
        """Run a simple path to test navigation."""
        logger.info("Running simple test path")

        try:
            # Start siren
            self.siren.start()



            # Navigate first few steps
            for x, y in HALLWAY_PATH[1:4]:  # Just the first few steps
                logger.info(f"Navigating to ({x}, {y})")
                self.navigation.navigate_to(x, y)

            # Return to start
            logger.info("Returning to start")
            self.navigation.navigate_to(0, 0)

            # Stop siren
            self.siren.stop()

        except Exception as e:
            logger.error(f"Error during test path: {e}")
        finally:
            self.drive_system.stop()
            self.siren.stop()

    def test_nineties(self):
        """Test 90-degree turning capabilities."""
        logger.info("Testing 90-degree turns")
        try:
            self.drive_system.turn(EAST)
            time.sleep(1)
        except Exception as e:
            logger.error(f"Error during turn testing: {e}")
        finally:
            self.drive_system.stop()

    def test_grid_alignment(self):
        """Test the grid alignment feature."""
        logger.info("Testing grid alignment")

        try:
            # Try to find a grid line
            found_line = self.navigation.align_with_grid()

        except KeyboardInterrupt:
            logger.info("Test interrupted by user")
        except Exception as e:
            logger.error(f"Error during grid alignment test: {e}")
        finally:
            self.drive_system.stop()

def main():
    """Main entry point for the firefighter robot mission."""
    parser = argparse.ArgumentParser(description='Firefighter Robot Control')
    parser.add_argument('--test-grid', action='store_true', help='Run grid alignment test')
    parser.add_argument('--test-90', action='store_true', help='Run 90 test')

    args = parser.parse_args()

    robot = FirefighterRobot()

    try:
        if args.test_grid:
            robot.test_grid_alignment()
        elif args.test_90:
            robot.test_nineties()
        else:
            robot.run_mission()
    except KeyboardInterrupt:
        logger.info("Mission interrupted by user")
    finally:
        # Stop everything
        if robot:
            robot.drive_system.stop()
            robot.siren.stop()


def calibration_testing():
    """Test basic robot functionality."""
    robot = FirefighterRobot()
    robot.calibration_test()


def simple_path_test():
    """Run a simple navigation test."""
    robot = FirefighterRobot()
    robot.run_simple_path()


def testing_nineties():
    """Test robot turning capabilities."""
    robot = FirefighterRobot()
    robot.test_nineties()


if __name__ == "__main__":
    main()