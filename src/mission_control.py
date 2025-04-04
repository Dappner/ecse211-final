"""
Mission control module for managing the robot's firefighting mission.
"""
import logging
import threading
import time
from enum import Enum
from src.constants import (
    HALLWAY_PATH, ENTRANCE, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
    NORTH, POSITION_TOLERANCE
)

logger = logging.getLogger("mission")

MISSION_RETURN_TRIGGER = 120  # Start return to base after 2 minutes
MISSION_MAX_TIME = 180


class MissionState(Enum):
    INIT = "init"
    HALLWAY = "hallway"
    ROOM_ENTRY = "room_entry"
    SWEEP = "sweep"
    RETURN = "return"
    DONE = "done"


class MissionControl:
    """Manages the overall mission execution and state."""

    def __init__(self, drive, sensors, navigation, extinguisher, siren):
        self.drive = drive
        self.sensors = sensors
        self.navigation = navigation
        self.extinguisher = extinguisher
        self.siren = siren
        self.mission_running = False
        self.mission_start_time = None
        self.fires_detected = 0
        self.return_triggered = False
        self.max_retries = 3

    def run_mission(self):
        self.mission_running = True
        self.mission_start_time = time.time()
        state = MissionState.HALLWAY
        self.start_timer_monitor()  # Sets return after 2min
        self.start_emergency_monitor()  # Checks for emergency button

        self.siren.start()

        while self.mission_running and state != MissionState.DONE:
            est_pos = self.navigation.estimated_position
            logger.info(
                f"--- State: {state.value} --- Est. Pos: [{est_pos[0]:.2f}, {est_pos[1]:.2f}] --- Orientation: {self.drive.orientation} ---")

            if self.return_triggered:  # (sets to true after 2Min)
                state = MissionState.RETURN

            if state == MissionState.HALLWAY:
                state = MissionState.ROOM_ENTRY if self.navigate_hallway() else MissionState.RETURN

            elif state == MissionState.ROOM_ENTRY:
                if self.enter_burning_room():
                    self.siren.stop()
                    state = MissionState.SWEEP
                else:
                    state = MissionState.RETURN

            elif state == MissionState.SWEEP:
                self.sweep_burning_room()
                state = MissionState.RETURN

            elif state == MissionState.RETURN:
                self.return_to_base()
                state = MissionState.DONE

        elapsed_time = time.time() - self.mission_start_time
        logger.info(f"Ended Mission attempt in {elapsed_time:.1f} seconds")
        self.stop_mission()

    def navigate_hallway(self):
        """Navigate through the hallway to the entrance."""
        logger.info("Starting hallway navigation")

        for i, (x, y) in enumerate(HALLWAY_PATH[1:], 1):  # Skip first position (start)
            if not self.check_mission_status():  # Check if time elapsed
                return False

            if not self.attempt_navigation(x, y, "hallway_navigation"):
                return False

        current_pos = self.navigation.estimated_position
        expected_pos = HALLWAY_PATH[-1]

        # Use a slightly larger tolerance for floating point comparison
        if abs(current_pos[0] - expected_pos[0]) > POSITION_TOLERANCE or abs(
                current_pos[1] - expected_pos[1]) > POSITION_TOLERANCE:
            logger.warning(
                f"Position potentially off after hallway nav: expected {expected_pos}, got ~[{current_pos[0]:.2f}, {current_pos[1]:.2f}]. Localizing.")
            self.navigation.localize()
            # Re-check after localization
            current_pos = self.navigation.estimated_position
            if abs(current_pos[0] - expected_pos[0]) > POSITION_TOLERANCE or abs(
                    current_pos[1] - expected_pos[1]) > POSITION_TOLERANCE:
                logger.warning(f"Still off after localization. Attempting final move to {expected_pos}.")
                # Post localization attempts to go to the entrance.
                if not self.attempt_navigation(expected_pos[0], expected_pos[1], "hallway_final"):
                    return False
            else:
                logger.info("Position confirmed after localization.")

        self.drive.turn(NORTH)
        logger.info("Hallway navigation completed")

        return True

    def enter_burning_room(self):
        """Enter the burning room if at entrance and orange line detected."""
        # First try to find and align with the orange entrance line
        logger.info("Attempting to enter burning room")

        # First check if we're at the entrance position
        current_pos = self.navigation.estimated_position
        entrance_pos = ENTRANCE

        if abs(current_pos[0] - entrance_pos[0]) > POSITION_TOLERANCE or abs(
                current_pos[1] - entrance_pos[1]) > POSITION_TOLERANCE:
            logger.warning(
                f"Not at entrance position: expected {entrance_pos}, got ~[{current_pos[0]:.2f}, {current_pos[1]:.2f}]")
            logger.info(f"Attempting navigation to entrance: {entrance_pos}")
            if not self.navigation.navigate_to(entrance_pos[0], entrance_pos[1]):
                logger.error("Failed to reach entrance position")
                return False
            else:
                logger.info(f"Successfully navigated to entrance {entrance_pos}")

        # Checking for orange line.
        if not self.sensors.check_for_entrance()[0]:
            if not self.navigation.align_with_entrance():
                logger.error("Failed to align with entrance")
                raise Exception("Couldn't align with entrance.")

        # Enter room (move one block north)
        logger.info("Entering burning room")
        self.drive.turn(NORTH)
        self.drive.advance_blocks(1)

        # Update Localization
        self.navigation.update_particles_after_movement(0, 1)
        sensor_data = self.sensors.get_sensor_data()
        self.navigation.update_particle_weights(sensor_data)
        self.navigation.update_position_estimate()

        # TODO: High potential for failure due to bad color sensor (green yellow weakness)
        room_type = self.navigation.identify_room()
        logger.info(f"Room identified as: {room_type}")

        return room_type == "burning room"

    def sweep_burning_room(self):
        """Sweep room for fires with localization."""
        logger.info("Starting burning room sweep")
        for x, y in BURNING_ROOM_SWEEP:
            if not self.check_mission_status() or self.return_triggered:
                break
            if self.attempt_navigation(x, y, "sweep_navigation"):
                fire_found, sensor_side = self.navigation.find_fire()
                if fire_found:
                    self.fires_detected += 1
                    logger.info(f"Fire detected at {self.drive.position}")
                    self.drop_on_sensor(sensor_side)
                    if self.extinguisher.get_fires_extinguished() >= 2:
                        logger.info("All fires extinguished")
                        break

    def start_emergency_monitor(self):
        """Start monitoring for emergency stop signal."""

        def monitor():
            while self.mission_running:
                if self.sensors.is_emergency_pressed():
                    logger.warning("EMERGENCY STOP ACTIVATED")
                    self.stop_mission()
                    break
                time.sleep(0.1)

        thread = threading.Thread(
            target=monitor, daemon=True
        )
        thread.start()
        logger.info("Emergency stop monitor started")

    def start_timer_monitor(self):
        """Monitors time and forces return at 2 minute mark"""

        def monitor():
            while self.mission_running and not self.return_triggered:
                elapsed_time = time.time() - self.mission_start_time
                if elapsed_time >= 120:  # 2 minutes
                    logger.info("2-minute mark reached. Triggering return.")
                    self.return_triggered = True
                    break
                time.sleep(1)

        thread = threading.Thread(target=monitor, daemon=True)
        thread.start()
        logger.info("Time monitor started (triggers RTB)")

    def return_to_base(self):
        """Return to base with localization."""
        logger.info("Returning to base")
        current_pos_est = self.navigation.estimated_position

        if current_pos_est[1] >= (BURNING_ROOM_ENTRY[1] - POSITION_TOLERANCE):  # If y >= 3 (roughly)
            logger.info("Currently in burning room area, navigating out.")
            # Navigate to exit point first, then entrance
            if not self.attempt_navigation(BURNING_ROOM_ENTRY[0], BURNING_ROOM_ENTRY[1],
                                           "return_exit_room"): return False  # Added return check
            if not self.attempt_navigation(ENTRANCE[0], ENTRANCE[1],
                                           "return_to_entrance"): return False  # Added return check

        logger.info("Navigating hallway back to base.")
        for x, y in RETURN_PATH[1:]:  # Skip the first element which is ENTRANCE (just moved there.)
            if not self.check_mission_status():
                break
            if not self.attempt_navigation(x, y, "return_navigation"):
                # If navigation fails during return, log it but continue trying maybe? Or just fail.
                logger.error(f"Failed final navigation step to base ({x},{y})")
                return False  # False -> Fail

        return True

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

    def stop_mission(self):
        """Stop the mission and all activities."""
        logger.info("Stopping mission")
        self.mission_running = False
        self.drive.stop()
        self.siren.stop()

    def check_mission_status(self):
        """Check if mission should continue 3 min hard limit."""
        if not self.mission_running:
            return False
        elapsed_time = time.time() - self.mission_start_time
        if elapsed_time >= 180:  # 3-minute hard limit
            logger.warning("Max mission time (3 minutes) reached!")
            self.stop_mission()
            return False
        return True

    def attempt_navigation(self, x, y, operation_key):
        """Attempt navigation with retries and localization."""
        retries = 0
        while retries < self.max_retries:
            if self.navigation.navigate_to(x, y):
                return True
            retries += 1
            logger.warning(f"Navigation to ({x}, {y}) failed, retry {retries}/{self.max_retries}")
            self.navigation.localize()  # Relocalize on failure
            # Sync localization to drive
            if not self.check_mission_status():
                return False
        logger.error(f"Max retries exceeded for {operation_key}")
        return False
