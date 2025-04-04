"""
Mission control module for managing the robot's firefighting mission.
"""
import logging
import threading
import time
from enum import Enum
from src.constants import (
    MISSION_TIME_LIMIT, EXPECTED_TIME_TO_BURNING_ROOM,
    HALLWAY_PATH, ENTRANCE, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
    NORTH, FIRE_STATION, EAST, SOUTH, WEST
)

logger = logging.getLogger("mission")


class MissionState(Enum):
    INITIALIZING = "initializing"
    NAVIGATING_HALLWAY = "navigating_hallway"
    ENTERING_ROOM = "entering_room"
    SWEEPING_ROOM = "sweeping_room"
    RETURNING_TO_BASE = "returning_to_base"
    COMPLETED = "completed"
    ABORTED = "aborted"
    ERROR = "error"


class MissionOutcome(Enum):
    """Enum representing the different possible outcomes of the mission."""
    SUCCESS = "success"  # Complete mission success
    PARTIAL_SUCCESS = "partial_success"  # Some fires extinguished
    TIMEOUT = "timeout"  # Time limit reached
    ERROR = "error"  # Error during mission
    EMERGENCY_STOP = "emergency_stop"  # Emergency stop activated
    NAVIGATION_FAILURE = "navigation_failure"  # Failed to navigate
    HARDWARE_FAILURE = "hardware_failure"  # Hardware malfunction


class MissionControl:
    """Manages the overall mission execution and state."""

    def __init__(self, drive, sensors, navigation, extinguisher, siren):
        self.drive = drive
        self.sensors = sensors
        self.navigation = navigation
        self.extinguisher = extinguisher
        self.siren = siren

        # Mission state
        self.mission_state = MissionState.INITIALIZING
        self.mission_running = False
        self.emergency_stop_thread = None
        self.fires_detected = 0
        self.mission_start_time = None
        self.mission_elapsed_time = 0

        # Tracking retries for any given operation. If exceed -> Return to base....
        self.retry_counts = {
            "hallway_navigation": 0,
            "room_entry": 0,
            "fire_detection": 0,
            "return_navigation": 0
        }
        self.max_retries = 3

    def start_emergency_monitor(self):
        """Start monitoring for emergency stop signal."""
        self.emergency_stop_thread = threading.Thread(
            target=self._monitor_emergency_stop
        )
        self.emergency_stop_thread.daemon = True
        self.emergency_stop_thread.start()
        logger.info("Emergency stop monitor started")

    def _monitor_emergency_stop(self):
        logger.debug("Emergency stop monitoring thread started")
        polling_interval = 0.1
        """Background thread for emergency stop monitoring."""
        while self.mission_running:
            if self.sensors.is_emergency_pressed():
                logger.warning("EMERGENCY STOP ACTIVATED")
                self.stop_mission()
                break
            time.sleep(polling_interval)

    def navigate_hallway(self):
        """Navigate through the hallway to the entrance."""
        logger.info("Starting hallway navigation")
        self.mission_state = MissionState.NAVIGATING_HALLWAY

        try:
            for i, (x, y) in enumerate(HALLWAY_PATH[1:], 1):  # Skip first position (start)
                if not self.mission_running:
                    return False

                checkpoint_name = f"checkpoint {i}/{len(HALLWAY_PATH) - 1}"
                logger.info(f"Navigating to {checkpoint_name} at position ({x}, {y})")

                success = self.navigation.navigate_to(x, y)

                if not success:
                    logger.warning(f"Failed to reach {checkpoint_name}")
                    if self.retry_counts["hallway_navigation"] < self.max_retries:

                        logger.info("Attempting recovery with relocalization")
                        self.retry_counts["hallway_navigation"] += 1
                        self.navigation.localize()
                        success = self.navigation.navigate_to(x, y)

                        if not success:
                            logger.error(f"Failed to reach {checkpoint_name} after recovery attempt")
                            return False
                    else:
                        logger.error(f"Exceeded maximum retries for hallway navigation")
                        return False

                # Check time limit
                if self._check_time_limit():
                    return False

            # END OF PATH TO BURNING ROOM
            current_pos = self.drive.position
            expected_pos = HALLWAY_PATH[-1]

            position_accurate = (abs(current_pos[0] - expected_pos[0]) <= 0.5 and
                                 abs(current_pos[1] - expected_pos[1]) <= 0.5)

            if not position_accurate:
                logger.warning(f"Position discrepancy at hallway end: expected {expected_pos}, got {current_pos}")
                # Try to use localization
                self.navigation.localize()

            # Face north
            self.drive.turn(NORTH)

            logger.info("Hallway navigation completed successfully")
            return True

        except Exception as e:
            logger.error(f"Error during hallway navigation: {e}")
            return False

    def enter_burning_room(self):
        """Enter the burning room if at entrance and orange line detected."""
        # First try to find and align with the orange entrance line
        logger.info("Attempting to enter burning room")
        self.mission_state = MissionState.ENTERING_ROOM

        try:
            # First check if we're at the entrance position
            current_pos = self.drive.position
            entrance_pos = ENTRANCE

            position_accurate = (abs(current_pos[0] - entrance_pos[0]) <= 0.5 and
                                 abs(current_pos[1] - entrance_pos[1]) <= 0.5)

            if not position_accurate:
                logger.warning(f"Not at entrance position: expected {entrance_pos}, got {current_pos}")
                # Try to navigate to entrance first
                success = self.navigation.navigate_to(entrance_pos[0], entrance_pos[1])
                if not success:
                    logger.error("Failed to reach entrance position")
                    return False

            # Look for orange entrance line
            found_orange, side = self.sensors.check_for_entrance()

            if not found_orange:
                logger.info("Orange entrance line not immediately visible, attempting to align")
                aligned = self.navigation.align_with_entrance()

                if not aligned:
                    if self.retry_counts["room_entry"] < self.max_retries:
                        logger.warning("Failed to align with entrance, attempting small search pattern")
                        self.retry_counts["room_entry"] += 1

                        # Try small search pattern
                        search_successful = self._search_for_entrance()
                        if not search_successful:
                            logger.error("Failed to find entrance after search")
                            return False
                    else:
                        logger.warning("Exceeded retries for room entry")
                        return False

            # Enter room (move one block north)
            logger.info("Entering burning room")
            self.drive.turn(NORTH)
            self.drive.advance_blocks(1)

            # Update navigation state
            self.navigation.update_particles_after_movement(0, 1)
            sensor_data = self.sensors.get_sensor_data()
            self.navigation.update_particle_weights(sensor_data)
            self.navigation.update_position_estimate()

            # Verify we're in the burning room by checking color patterns
            room_type = self.navigation.identify_room()
            logger.info(f"Room identification: {room_type}")

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

        except Exception as e:
            logger.error(f"Error during room entry: {e}")
            return False

    def sweep_burning_room(self):
        """
        Search the burning room for fires and extinguish them.
        Uses a predefined sweep pattern and fire detection.

        Returns:
            bool: True if sweep completed, False if aborted
        """
        logger.info("Starting burning room sweep")
        self.mission_state = MissionState.SWEEPING_ROOM

        try:
            for point_index, (x, y) in enumerate(BURNING_ROOM_SWEEP):
                if not self.mission_running:
                    return False

                logger.info(f"Sweeping position ({x}, {y}) - point {point_index + 1}/{len(BURNING_ROOM_SWEEP)}")

                # Navigate to sweep point
                success = self.navigation.navigate_to(x, y)
                if not success:
                    logger.warning(f"Failed to reach sweep point ({x}, {y})")
                    # Try to continue with next point
                    continue

                # Check for fire with more thorough search
                fire_found, sensor_side = self.navigation.find_fire()

                if fire_found:
                    logger.info(f"Fire detected at position {self.drive.position} on {sensor_side} sensor")
                    self.fires_detected += 1

                    # Drop cube on the specific sensor that detected the fire
                    if sensor_side == "left":
                        self.drop_on_sensor("LEFT")
                    elif sensor_side == "right":
                        self.drop_on_sensor("RIGHT")
                    else:  # Both sensors or unspecified
                        self.extinguisher.drop_cube()

                    # If we've found both fires, we can stop the sweep
                    if self.extinguisher.get_fires_extinguished() >= 2:
                        logger.info("Both fires extinguished, ending sweep")
                        break

                # Check time limit
                if self._check_time_limit():
                    return False

            fires_extinguished = self.extinguisher.get_fires_extinguished()
            logger.info(f"Room sweep complete. Fires found: {self.fires_detected}, extinguished: {fires_extinguished}")

            # If we haven't found all fires but still have time, do an additional scan
            if fires_extinguished < 2 and self.mission_elapsed_time < MISSION_TIME_LIMIT * 0.7:
                logger.info("Not all fires found, performing additional scan")
                self._perform_additional_fire_scan()

            return True

        except Exception as e:
            logger.error(f"Error during room sweep: {e}")
            return False

    def _perform_additional_fire_scan(self):
        """
        Perform additional scanning for fires if not all were found in the initial sweep.
        Uses a systematic scanning pattern.
        """
        logger.info("Starting additional fire scan")

        # Move to center of room for a systematic scan
        center_x = sum(point[0] for point in BURNING_ROOM_SWEEP) // len(BURNING_ROOM_SWEEP)
        center_y = sum(point[1] for point in BURNING_ROOM_SWEEP) // len(BURNING_ROOM_SWEEP)

        self.navigation.navigate_to(center_x, center_y)

        # Perform a 360-degree scan
        for direction in [NORTH, EAST, SOUTH, WEST]:
            if not self.mission_running:
                return

            self.drive.turn(direction)
            time.sleep(0.5)  # Pause to stabilize

            # Check for fire
            fire_found, sensor_side = self.navigation.find_fire()

            if fire_found:
                logger.info(f"Fire detected during additional scan on {sensor_side} sensor")
                self.fires_detected += 1

                # Drop cube on the specific sensor
                if sensor_side == "left":
                    self.drop_on_sensor("LEFT")
                elif sensor_side == "right":
                    self.drop_on_sensor("RIGHT")
                else:
                    self.extinguisher.drop_cube()

                # If we've now found all fires, we can stop
                if self.extinguisher.get_fires_extinguished() >= 2:
                    logger.info("All fires extinguished during additional scan")
                    return

    def return_to_base(self):
        """Navigate back to the starting point."""
        logger.info("Beginning return journey to base")

        # if in burning room -> exit burning room
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

        # If outcome not set, mark as error
        if self.mission_outcome is None:
            self.mission_outcome = MissionOutcome.ERROR

        # Log final status
        self._log_mission_status()

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
