import logging
import threading
import time
from enum import Enum

logger = logging.getLogger("mission")


class MissionState(Enum):
    INIT = "init"
    HALLWAY = "hallway"
    ROOM_ENTRY = "room_entry"
    SWEEP = "sweep"
    RETURN = "return"
    DONE = "done"


class SimpleMissionControl:
    """
    Simplified mission control system focused on completing the core firefighting mission.
    """

    def __init__(self, drive, sensor_system, navigation, extinguisher, siren):
        self.drive = drive
        self.sensors = sensor_system
        self.navigation = navigation
        self.extinguisher = extinguisher
        self.siren = siren

        self.mission_running = False
        self.mission_start_time = None
        self.return_triggered = False
        self.fires_extinguished = 0

        # Maximum mission time
        self.MISSION_RETURN_TRIGGER = 120  # Start returning after 2 minutes
        self.MISSION_MAX_TIME = 180  # 3 minutes total

        logger.info("Simple mission control initialized")

    def run_mission(self):
        """
        Run the complete firefighting mission.
        """
        self.mission_running = True
        self.mission_start_time = time.time()
        state = MissionState.INIT

        # Start monitoring threads
        self._start_timer_monitor()
        self._start_emergency_monitor()

        # Start siren
        self.siren.start()

        logger.info("Starting firefighting mission")

        while self.mission_running and state != MissionState.DONE:
            logger.info(f"Current mission state: {state.value}")

            # Check if time limit reached
            if self.return_triggered:
                logger.info("Time limit reached - transitioning to return state")
                state = MissionState.RETURN

            # State machine
            if state == MissionState.INIT:
                # Initialize and calibrate
                self._calibrate_system()
                state = MissionState.HALLWAY

            elif state == MissionState.HALLWAY:
                # Navigate hallway to burning room entrance
                if self.navigation.follow_hallway_path():
                    state = MissionState.ROOM_ENTRY
                else:
                    logger.error("Failed to navigate hallway")
                    state = MissionState.RETURN

            elif state == MissionState.ROOM_ENTRY:
                # Enter burning room
                if self.navigation.enter_burning_room():
                    self.siren.stop()  # Stop siren after entering room
                    state = MissionState.SWEEP
                else:
                    logger.error("Failed to enter burning room")
                    state = MissionState.RETURN

            elif state == MissionState.SWEEP:
                # Sweep burning room for fires
                self._sweep_for_fires()
                state = MissionState.RETURN

            elif state == MissionState.RETURN:
                # Return to base
                if self.navigation.return_to_base():
                    state = MissionState.DONE
                else:
                    logger.error("Failed to return to base")
                    state = MissionState.DONE

        # Mission ended - stop everything
        self.stop_mission()

        elapsed_time = time.time() - self.mission_start_time
        logger.info(f"Mission completed in {elapsed_time:.1f} seconds")
        logger.info(f"Fires extinguished: {self.extinguisher.get_fires_extinguished()}")

        return True

    def _calibrate_system(self):
        """
        Perform initial system calibration.
        """
        logger.info("Calibrating system")

        # Calibrate color sensors
        self.sensors.calibrate_color_thresholds()

        # Calibrate ultrasonic sensor if available
        if self.sensors.has_ultrasonic:
            self.sensors.calibrate_ultrasonic()

        logger.info("System calibration complete")

    def _sweep_for_fires(self):
        """
        Sweep the burning room and extinguish fires.
        """
        logger.info("Sweeping burning room for fires")

        # Execute the sweep pattern
        if not self.navigation.sweep_burning_room():
            logger.error("Failed to complete burning room sweep")
            return

        # If no fires found during sweep, try once more at final position
        if self.extinguisher.get_fires_extinguished() == 0:
            logger.info("No fires found during sweep, checking final position")
            found_fire, side = self.navigation.find_fire()
            if found_fire:
                self._drop_on_fire(side)

    def _drop_on_fire(self, sensor_side):
        """
        Position robot and drop cube on detected fire.

        Args:
            sensor_side: Which sensor detected the fire ("LEFT", "RIGHT", or "BOTH")
        """
        logger.info(f"Positioning to extinguish fire detected on {sensor_side}")

        # Position robot based on which sensor detected fire
        if sensor_side == "LEFT":
            self.drive.turn_slightly_left(0.2)
            self.drive.move_forward_slightly(0.2)
        elif sensor_side == "RIGHT":
            self.drive.turn_slightly_right(0.2)
            self.drive.move_forward_slightly(0.2)
        elif sensor_side == "BOTH":
            # FUCK
            self.drive.move_forward_slightly(0.1)

        # Drop cube
        success = self.extinguisher.drop_cube()

        # Return to original position
        if sensor_side == "LEFT":
            self.drive.move_backward_slightly(0.2)
            self.drive.turn_slightly_right(0.2)
        elif sensor_side == "RIGHT":
            self.drive.move_backward_slightly(0.2)
            self.drive.turn_slightly_left(0.2)
        elif sensor_side == "BOTH":
            self.drive.move_backward_slightly(0.1)

        if success:
            logger.info("Fire extinguished successfully")
            self.fires_extinguished = self.extinguisher.get_fires_extinguished()
        else:
            logger.error("Failed to extinguish fire")

    def _start_timer_monitor(self):
        """
        Start a thread to monitor mission time.
        """
        def monitor():
            while self.mission_running:
                elapsed_time = time.time() - self.mission_start_time

                # Check for return trigger time
                if elapsed_time >= self.MISSION_RETURN_TRIGGER and not self.return_triggered:
                    logger.info(f"Mission time {elapsed_time:.1f}s reached return trigger point")
                    self.return_triggered = True

                # Check for mission max time
                if elapsed_time >= self.MISSION_MAX_TIME:
                    logger.warning(f"Mission maximum time {elapsed_time:.1f}s reached")
                    self.stop_mission()
                    break

                time.sleep(1)  # Check every second

        thread = threading.Thread(target=monitor, daemon=True)
        thread.start()
        logger.info("Timer monitor started")

    def _start_emergency_monitor(self):
        """
        Start a thread to monitor emergency stop button.
        """

        def monitor():
            while self.mission_running:
                if self.sensors.is_emergency_pressed():
                    logger.warning("EMERGENCY STOP ACTIVATED")
                    self.stop_mission()
                    break

                time.sleep(0.1)  # Check frequently

        thread = threading.Thread(target=monitor, daemon=True)
        thread.start()
        logger.info("Emergency stop monitor started")

    def stop_mission(self):
        """
        Stop the mission and all activities.
        """
        if not self.mission_running:
            return

        logger.info("Stopping mission")
        self.mission_running = False

        # Stop drive system
        self.drive.stop()

        # Stop siren
        self.siren.stop()

        if self.mission_start_time:
            elapsed_time = time.time() - self.mission_start_time
            logger.info(f"Mission stopped after {elapsed_time:.1f} seconds")