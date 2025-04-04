"""
Mission control module for managing the robot's firefighting mission.
"""
import logging
import threading
import time
from src.constants import (
    MISSION_TIME_LIMIT, EXPECTED_TIME_TO_BURNING_ROOM,
    HALLWAY_PATH, ENTRANCE, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
    NORTH
)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler()],
)
logger = logging.getLogger("mission")


class MissionControl:
    """Manages the overall mission execution and state."""

    def __init__(self, drive, sensors, navigation, extinguisher, siren):
        self.drive = drive
        self.sensors = sensors
        self.navigation = navigation
        self.extinguisher = extinguisher
        self.siren = siren

        # Mission state
        self.mission_running = False
        self.emergency_stop_thread = None
        self.fires_detected = 0
        self.mission_start_time = None

    def start_emergency_monitor(self):
        """Start monitoring for emergency stop signal."""
        self.emergency_stop_thread = threading.Thread(
            target=self._monitor_emergency_stop
        )
        self.emergency_stop_thread.daemon = True
        self.emergency_stop_thread.start()
        logger.info("Emergency stop monitor started")

    def _monitor_emergency_stop(self):
        """Background thread for emergency stop monitoring."""
        while self.mission_running:
            if self.sensors.is_emergency_pressed():
                logger.warning("EMERGENCY STOP ACTIVATED")
                self.stop_mission()
                break
            time.sleep(0.1)

    def navigate_hallway(self):
        """Navigate through the hallway to the entrance."""
        logger.info("Starting hallway navigation")

        for x, y in HALLWAY_PATH[1:]:  # Skip first position
            if not self.mission_running:
                break

            logger.info(f"Navigating to checkpoint ({x}, {y})")
            self.navigation.navigate_to(x, y)

            # Check time limit
            if self._check_time_limit():
                break

        elapsed_time = time.time() - self.mission_start_time

        if elapsed_time > EXPECTED_TIME_TO_BURNING_ROOM:
            logger.warning(
                f"Navigation to entrance took longer than expected: {elapsed_time:.1f}s vs {EXPECTED_TIME_TO_BURNING_ROOM}s expected"
            )
        # Orient to face entrance
        self.drive.turn(NORTH)

        # Check if at entrance
        is_at_entrance = (
                self.drive.position[0] == ENTRANCE[0]
                and self.drive.position[1] == ENTRANCE[1]
        )

        if is_at_entrance:
            logger.info("Reached entrance position")
            return True
        else:
            logger.warning(
                f"Failed to reach entrance, current position: {self.drive.position}"
            )
            return False

    def enter_burning_room(self):
        """Enter the burning room if at entrance and orange line detected."""
        # First try to find and align with the orange entrance line
        found_orange, _ = self.sensors.check_for_entrance()

        if not found_orange:
            logger.info("Orange line not immediately visible, attempting to align")
            aligned = self.navigation.align_with_entrance()

            if not aligned:
                logger.warning(
                    "Failed to align with orange entrance, will attempt entry anyway"
                )

        # Enter room (once aligned or after attempting alignment)
        logger.info("Entering burning room")
        self.drive.advance_blocks(1)  # Move one block north to enter

        # Verify we're in the burning room
        room_type = self.navigation.identify_room()
        logger.info(f"Entered room identified as: {room_type}")

        if room_type == "burning room":
            logger.info("Successfully entered burning room")
            return True
        elif room_type == "avoid room":
            logger.warning("Entered wrong room (avoid room)! Exiting immediately")
            # Return to hallway
            self.drive.turn(NORTH)
            self.drive.advance_blocks(1)
            return False
        else:
            logger.warning(f"Room type uncertain: {room_type}. Proceeding with caution")
            # Assume we're in the burning room but log the uncertainty
            return True

    def sweep_burning_room(self):
        """Search the burning room for fires and extinguish them."""
        logger.info("Starting systematic room sweep")

        for x, y in BURNING_ROOM_SWEEP:
            if not self.mission_running:
                break

            logger.info(f"Sweeping position ({x}, {y})")
            self.navigation.navigate_to(x, y)

            # Check for fire
            fire_detected, sensor_side = self.sensors.check_for_fire()
            if fire_detected:
                logger.info(f"Fire detected at position {self.drive.position} on {sensor_side} sensor")
                self.fires_detected += 1

                # Drop a cube on the specific sensor that detected the fire
                if sensor_side == "left":
                    self.drop_on_sensor("LEFT")
                elif sensor_side == "right":
                    self.drop_on_sensor("RIGHT")
                else:  # Both sensors or unspecified
                    self.extinguisher.drop_cube()

                # If we've found both fires, we can stop
                if self.extinguisher.get_fires_extinguished() >= 2:
                    logger.info("Both fires extinguished, ending sweep")
                    break

            # Check time limit
            if self._check_time_limit():
                break

        logger.info(
            f"Room sweep complete. Fires found: {self.fires_detected}, extinguished: {self.extinguisher.get_fires_extinguished()}"
        )
        return self.extinguisher.get_fires_extinguished() > 0

    def return_to_base(self):
        """Navigate back to the starting point."""
        logger.info("Beginning return journey to base")

        # First exit the room if needed
        if self.drive.position[1] >= 3:  # If in burning room (row 3 or 4)
            logger.info("Exiting burning room")
            self.navigation.navigate_to(BURNING_ROOM_ENTRY[0], BURNING_ROOM_ENTRY[1])
            self.navigation.navigate_to(ENTRANCE[0], ENTRANCE[1])

        # Follow return path
        for x, y in RETURN_PATH:
            if not self.mission_running:
                break

            # Skip if already at this position
            if self.drive.position[0] == x and self.drive.position[1] == y:
                continue

            logger.info(f"Returning via checkpoint ({x}, {y})")
            self.navigation.navigate_to(x, y)

            # Check time limit
            if self._check_time_limit():
                break

        # Check if successfully returned to base
        is_at_base = self.drive.position[0] == 0 and self.drive.position[1] == 0

        if is_at_base:
            logger.info("Successfully returned to base station (0,0)")
            return True
        else:
            logger.warning(
                f"Failed to return to base, stopped at {self.drive.position}"
            )
            return False

    def _check_time_limit(self):
        """Check if mission time limit (3 minutes) has been reached."""
        if self.mission_start_time is None:
            return False

        elapsed_time = time.time() - self.mission_start_time
        if elapsed_time >= MISSION_TIME_LIMIT:  # 3 minutes
            logger.warning("Mission time limit (3 minutes) reached!")
            return True
        return False

    def stop_mission(self):
        """Stop the mission and all activities."""
        logger.info("Stopping mission")
        self.mission_running = False
        self.drive.stop()
        self.siren.stop()

    def drop_on_sensor(self, sensor: str):
        """Drop cube more precisely on a specific sensor."""
        ROTATION_SECONDS = 0.9
        FORWARD_MOVE = 0.5
        if sensor == "RIGHT":
            self.drive.turn_slightly_right(ROTATION_SECONDS)
            self.drive.move_forward_slightly(FORWARD_MOVE + 0.1)
            self.extinguisher.drop_cube()
            self.drive.move_backward_slightly(FORWARD_MOVE)
            self.drive.turn_slightly_left(ROTATION_SECONDS)
            self.drive.move_forward_slightly(FORWARD_MOVE - 0.1)
        else:
            self.drive.turn_slightly_left(ROTATION_SECONDS)
            self.drive.move_forward_slightly(FORWARD_MOVE + 0.1)
            self.extinguisher.drop_cube()
            self.drive.move_backward_slightly(FORWARD_MOVE)
            self.drive.turn_slightly_right(ROTATION_SECONDS)
            self.drive.move_forward_slightly(FORWARD_MOVE - 0.1)

    def run_mission(self):
        """Execute the full firefighting mission."""
        # Initialize mission
        self.mission_running = True
        self.mission_start_time = time.time()
        self.fires_detected = 0

        try:
            # Start emergency monitor
            self.start_emergency_monitor()

            # Start siren
            self.siren.start()

            # Navigate through hallway to entrance
            if self.navigate_hallway():
                # At entrance, try to enter burning room
                if self.enter_burning_room():
                    # In burning room, stop siren
                    self.siren.stop()

                    # Search for and extinguish fires
                    success = self.sweep_burning_room()
                    # Runs until we hit ~ 2 Min

                    # Return to base regardless of success
                    self.return_to_base()
                else:
                    logger.error("Failed to enter burning room")
                    self.return_to_base()
            else:
                logger.error("Failed to reach entrance")
                self.return_to_base()

            # Mission complete
            elapsed_time = time.time() - self.mission_start_time
            logger.info(f"Mission completed in {elapsed_time:.1f} seconds")
            logger.info(
                f"Fires detected: {self.fires_detected}, extinguished: {self.extinguisher.get_fires_extinguished()}"
            )

        except Exception as e:
            logger.error(f"Error during mission: {e}")
        finally:
            # Ensure proper shutdown
            self.stop_mission()