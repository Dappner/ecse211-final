"""
Mission control module for managing the robot's firefighting mission.
"""
import logging
import threading
import time
from enum import Enum
from src.constants import (
    HALLWAY_PATH, ENTRANCE, BURNING_ROOM_ENTRY, BURNING_ROOM_SWEEP, RETURN_PATH,
    NORTH, POSITION_TOLERANCE, BLOCK_SIZE, EAST, WEST, SOUTH
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

    def __init__(self, drive, sensor_system, navigation, extinguisher, siren):
        self.drive = drive
        self.sensor_system = sensor_system
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

    def calibrate_and_update_position(self, target_x, target_y):
        """
        Improved calibration during first block navigation with better position tracking.

        Args:
            target_x: Target X coordinate (should be 0)
            target_y: Target Y coordinate (should be 1)

        Returns:
            tuple: (success, actual_distance_moved)
        """
        logger.info(f"Calibrating during first movement to ({target_x}, {target_y})")

        # Get initial position
        start_pos = self.navigation.estimated_position.copy()
        logger.info(f"Starting position: {start_pos}")

        # Get initial distance reading
        initial_distance = None
        if self.sensor_system.has_ultrasonic:
            initial_distance = self.sensor_system.get_wall_distance()
            if initial_distance is not None:
                logger.info(f"Initial distance to wall: {initial_distance:.1f}cm")

        # Execute movement (always one block north)
        current_forward_time = self.drive.forward_time_per_block
        logger.info(f"Moving forward one block with timing {current_forward_time:.2f}s")

        # Track the encoder positions to measure actual movement
        left_pos_before = self.drive.left_motor.get_position()
        right_pos_before = self.drive.right_motor.get_position()

        # Execute the move
        self.drive.advance_blocks(1)

        # Read encoder positions after movement
        left_pos_after = self.drive.left_motor.get_position()
        right_pos_after = self.drive.right_motor.get_position()

        left_delta = left_pos_after - left_pos_before
        right_delta = right_pos_after - right_pos_before
        encoder_avg = (abs(left_delta) + abs(right_delta)) / 2

        logger.info(f"Encoder movement - Left: {left_delta}, Right: {right_delta}, Avg: {encoder_avg}")

        # Measure actual distance moved
        actual_blocks_moved = 1.0  # Default assumption
        adjustment_factor = 1.0

        if initial_distance is not None:
            final_distance = self.sensor_system.get_wall_distance()
            if final_distance is not None:
                distance_moved = initial_distance - final_distance
                expected_distance = BLOCK_SIZE

                logger.info(f"Distance moved: {distance_moved:.1f}cm (expected {expected_distance}cm)")

                # Calculate actual blocks moved
                if distance_moved > 0:  # Only adjust if positive movement detected
                    actual_blocks_moved = distance_moved / BLOCK_SIZE
                    logger.info(f"Actual blocks moved: {actual_blocks_moved:.2f}")

                    # Calculate adjustment factor with limits
                    adjustment_factor = expected_distance / max(distance_moved, 1.0)  # Prevent division by zero
                    adjustment_factor = max(0.8, min(1.2, adjustment_factor))  # Limit adjustment to reasonable range

                    # Also adjust turn time by the same factor
                    old_turn_time = self.drive.turn_90_time
                    self.drive.turn_90_time = old_turn_time * adjustment_factor
                    logger.info(f"Adjusted turn time: {self.drive.turn_90_time:.2f}s (factor: {adjustment_factor:.2f})")

                    # Update forward time
                    old_forward_time = self.drive.forward_time_per_block
                    self.drive.forward_time_per_block = old_forward_time * adjustment_factor
                    logger.info(
                        f"Adjusted forward time: {self.drive.forward_time_per_block:.2f}s (factor: {adjustment_factor:.2f})")

        # Update particle positions based on actual movement (using actual blocks moved instead of assumed)
        self.navigation.update_particles_after_movement(0, actual_blocks_moved)

        # Get new sensor data for particle weights
        sensor_data = self.sensor_system.get_sensor_data()
        self.navigation.update_particle_weights(sensor_data)
        self.navigation.resample_particles()

        # Force position to be on the grid point
        self.navigation.estimated_position = [target_x, target_y]

        # Sync orientation with drive
        self.navigation.sync_orientation_with_drive()

        # Boost position confidence after calibration
        self.navigation.position_confidence = 0.8

        return True, actual_blocks_moved

    def check_and_align_grid(self):
        """
        Check for grid lines and use them to correct position if detected.
        This helps keep navigation on track without solely relying on particles.
        """
        on_black, sensor_side = self.sensor_system.is_on_black_line()

        if on_black:
            logger.info(f"Grid line detected on {sensor_side}, performing alignment and position correction")

            # Try to align with the grid line
            self.drive.stop()

            if sensor_side == "both":
                logger.info("Already aligned with grid line")
            elif sensor_side == "left":
                logger.info("Left sensor on black, turning right slightly")
                self.drive.turn_slightly_right(0.1)
            elif sensor_side == "right":
                logger.info("Right sensor on black, turning left slightly")
                self.drive.turn_slightly_left(0.1)

            # Assume we're on a grid line, so round position to nearest integer
            current_pos = self.navigation.estimated_position
            grid_x = round(current_pos[0])
            grid_y = round(current_pos[1])

            # Update navigation with corrected position (snap to grid)
            logger.info(f"Updating position from [{current_pos[0]:.2f}, {current_pos[1]:.2f}] to [{grid_x}, {grid_y}]")
            self.navigation.estimated_position = [grid_x, grid_y]

            # Force sync orientation
            self.navigation.sync_orientation_with_drive()

            # Boost position confidence since we know we're on a grid line
            self.navigation.position_confidence = max(self.navigation.position_confidence, 0.7)

            return True

        return False

    def navigate_hallway(self):
        """
        Navigate through the hallway with calibration during first movement.
        """
        logger.info("Starting hallway navigation")

        # First waypoint with calibration (0,1)
        first_target_x, first_target_y = HALLWAY_PATH[1]
        success, distance_moved = self.calibrate_and_update_position(
            first_target_x, first_target_y
        )

        if not success:
            logger.warning("Failed to navigate to first waypoint")
            return False

        # Continue with remaining hallway path (skip the first target since we just went there)
        for i, (x, y) in enumerate(HALLWAY_PATH[2:], 2):  # Start index at 2
            if not self.check_mission_status():  # Check if time elapsed
                return False

            if not self.attempt_navigation(x, y, "hallway_navigation"):
                return False
            self.check_and_align_grid()

        # Final position verification
        current_pos = self.navigation.estimated_position
        expected_pos = HALLWAY_PATH[-1]

        if abs(current_pos[0] - expected_pos[0]) > POSITION_TOLERANCE or abs(
                current_pos[1] - expected_pos[1]) > POSITION_TOLERANCE:
            logger.warning(
                f"Position potentially off after hallway nav: expected {expected_pos}, got ~[{current_pos[0]:.2f}, {current_pos[1]:.2f}]. Localizing.")
            self.navigation.localize()
            # Check again after running localization.

            current_pos = self.navigation.estimated_position
            if abs(current_pos[0] - expected_pos[0]) > POSITION_TOLERANCE or abs(
                    current_pos[1] - expected_pos[1]) > POSITION_TOLERANCE:
                logger.warning(f"Still off after localization. Attempting final move to {expected_pos}.")
                # Tries to go to entrance.
                if not self.attempt_navigation(expected_pos[0], expected_pos[1], "hallway_final"):
                    return False
            else:
                logger.info("Position confirmed after localization.")

        self.drive.turn(NORTH)
        self.navigation.align_with_entrance()
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
        if not self.sensor_system.check_for_entrance()[0]:
            if not self.navigation.align_with_entrance():
                logger.error("Failed to align with entrance")
                raise Exception("Couldn't align with entrance.")

        # Enter room (move one block north)
        logger.info("Entering burning room")
        self.drive.turn(NORTH)
        self.drive.advance_blocks(1)

        # Update Localization
        self.navigation.update_particles_after_movement(0, 1)
        sensor_data = self.sensor_system.get_sensor_data()
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
                if self.sensor_system.is_emergency_pressed():
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
