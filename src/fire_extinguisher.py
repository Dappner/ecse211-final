import logging
import time

from src.constants import DROPPER_MOTOR_PORT, DROPPER_MOTOR_DPS, DROPPER_MOTOR_TIME
from utils.brick import Motor

logger = logging.getLogger("extinguisher")


class FireExtinguisher:
    """Handles the fire extinguishing mechanism."""

    def __init__(self, motor_port=DROPPER_MOTOR_PORT):
        """
        Initialize the fire extinguisher system.

        Args:
            motor_port: Port for the dropper motor
        """
        self.dropper_motor = Motor(motor_port)
        self.fires_extinguished = 0

        logger.info("Fire extinguisher initialized")

    def drop_cube(self):
        """Drop a foam cube to extinguish a fire."""
        logger.info("Dropping foam cube to extinguish fire")
        # Implementation of dropping mechanism - Rotates the motor to drop one sandbag
        self.dropper_motor.set_dps(-DROPPER_MOTOR_DPS)  # Rotate 180 degrees
        time.sleep(DROPPER_MOTOR_TIME)
        self.dropper_motor.set_dps(0)

        logger.info("Please pick up the Cube!")
        time.sleep(1)  # Wait for cube to drop

        self.fires_extinguished += 1
        logger.info(
            f"Fire extinguished. Total fires extinguished: {self.fires_extinguished}"
        )
        return True

    def get_fires_extinguished(self):
        """Return the count of fires extinguished."""
        return self.fires_extinguished

    def test_drop_mechanism(self):
        """Test the cube dropping mechanism without incrementing the counter."""
        logger.info("Testing drop mechanism")

        # Execute drop
        self.dropper_motor.set_dps(-DROPPER_MOTOR_DPS)
        time.sleep(DROPPER_MOTOR_TIME)
        self.dropper_motor.set_dps(0)

        logger.info("Drop mechanism test completed")
        return True

