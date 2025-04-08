#!/usr/bin/env python3
"""
Main program for the firefighter robot using encoder-based control.
"""
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

# Import constants
from simple_src.constants import (
    LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, DROPPER_MOTOR_PORT,
    LEFT_COLOR_PORT, RIGHT_COLOR_PORT, ULTRASONIC_PORT, TOUCH_PORT, HALLWAY_PATH
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[logging.StreamHandler()]
)

logger = logging.getLogger("main")


class FirefighterRobot:
    """Main robot class that integrates all components."""

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



def main():
    """Main entry point for the program."""
    parser = argparse.ArgumentParser(description='Firefighter Robot Control')
    parser.add_argument('--test-motors', action='store_true', help='Test motor functionality')
    parser.add_argument('--test-sensors', action='store_true', help='Test sensor functionality')
    parser.add_argument('--test-navigation', action='store_true', help='Test navigation functionality')

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
        else:
            # Run the complete mission
            robot.run_mission()
    except KeyboardInterrupt:
        logger.info("Program interrupted by user")
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
    finally:
        robot.stop()

if __name__ == "__main__":
    main()