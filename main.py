from utils.brick import Motor, EV3ColorSensor, EV3UltrasonicSensor, configure_ports
import time
import math
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler()]
)
logger = logging.getLogger(__name__)

# Constants
SQUARE_SIZE = 24  # cm per grid square
WHEELBASE = 15    # cm, approximate distance between tracks (adjust as needed)
TURN_SPEED = 300  # degrees per second for turns (used as limit)
MOVE_SPEED = 500  # degrees per second for forward movement
COLOR_THRESHOLD = 50  # RGB difference threshold for color detection
ALIGNMENT_TOLERANCE = 5  # cm tolerance for wall distance verification
ORIENTATION_TOLERANCE = 10  # degrees tolerance for orientation checks
TURN_CALIBRATION = 1.0  # Adjust after testing (e.g., 0.9 or 1.1)

# Hallway path (excluding room entry)
HALLWAY_PATH = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 2), (1, 2), (2, 2), (3, 2), (4, 2)]
ENTRANCE = (3, 2)  # Position before entering the room
BURNING_ROOM_ENTRY = (3, 3)  # Entry point to burning room

# RGB color definitions 
#TODO: Adjust these values based on actual sensor readings
COLORS = {
    "white": [200, 200, 200],   # Hallway
    "purple": [128, 0, 128],    # Burning room
    "yellow": [255, 255, 0],    # Room to avoid
    "gren": [93, 130, 11],     # Green Card
    "black": [0, 0, 0],         # Grid lines
    "orange": [255, 165, 0]     # Entrance line
}

class FirefighterRobot:
    def __init__(self):
        self.left_motor, self.right_motor, self.left_color, self.right_color, self.ultrasonic = configure_ports(
            PORT_A=Motor, PORT_B=Motor, PORT_1=EV3ColorSensor, PORT_2=EV3ColorSensor, PORT_3=EV3UltrasonicSensor
        )
        self.left_motor.set_limits(dps=MOVE_SPEED)
        self.right_motor.set_limits(dps=MOVE_SPEED)
        self.position = [0, 0]  # [x, y]
        self.orientation = 0    # Degrees: 0 (east), 90 (north), 180 (west), 270 (south)
        self.left_color.wait_ready()
        self.right_color.wait_ready()
        self.ultrasonic.wait_ready()
        logger.info(f"Robot initialized at position {self.position}, orientation {self.orientation}")

    def move_forward(self, distance_cm):
        """Move forward a given distance in cm."""
        degrees = (distance_cm / (WHEELBASE * math.pi)) * 360
        logger.info(f"Moving forward {distance_cm} cm ({degrees:.1f} motor degrees)")
        self.left_motor.set_position_relative(degrees)
        self.right_motor.set_position_relative(degrees)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()
        logger.info("Move forward completed")

    def turn(self, angle_deg):
        """Turn in place by angle_deg (positive = right, negative = left)."""
        degrees = TURN_CALIBRATION * (WHEELBASE * math.pi * abs(angle_deg)) / (360 * 2)
        logger.info(f"Turning {angle_deg} degrees ({degrees:.1f} motor degrees)")
        if angle_deg > 0:  # Right turn
            self.left_motor.set_position_relative(degrees)
            self.right_motor.set_position_relative(-degrees)
        else:  # Left turn
            self.left_motor.set_position_relative(-degrees)
            self.right_motor.set_position_relative(degrees)
        self.left_motor.wait_is_stopped()
        self.right_motor.wait_is_stopped()
        old_orientation = self.orientation
        self.orientation = (self.orientation + angle_deg) % 360
        logger.info(f"Turn completed: orientation updated from {old_orientation} to {self.orientation}")

    def stop(self):
        """Stop all movement."""
        self.left_motor.set_power(0)
        self.right_motor.set_power(0)
        logger.info("Motors stopped")

    def get_rgb_left(self):
        """Get RGB values from left color sensor."""
        rgb = self.left_color.get_rgb()
        logger.debug(f"Left color sensor RGB: {rgb}")
        return rgb

    def get_rgb_right(self):
        """Get RGB values from right color sensor."""
        rgb = self.right_color.get_rgb()
        logger.debug(f"Right color sensor RGB: {rgb}")
        return rgb

    def detect_color(self, rgb, target_color):
        """Check if the given RGB matches the target color within threshold."""
        if rgb is None:
            logger.warning(f"Invalid RGB reading for {target_color} detection")
            return False
        match = all(abs(rgb[i] - COLORS[target_color][i]) < COLOR_THRESHOLD for i in range(3))
        logger.debug(f"Detecting {target_color} with RGB {rgb}: {'match' if match else 'no match'}")
        return match

    def align_with_grid(self):
        """Align robot with black grid lines using both color sensors."""
        logger.info("Aligning with grid...")
        while True:
            left_rgb = self.get_rgb_left()
            right_rgb = self.get_rgb_right()
            left_black = self.detect_color(left_rgb, "black")
            right_black = self.detect_color(right_rgb, "black")

            if left_black and right_black:
                logger.info("Aligned with black grid line")
                self.stop()
                break
            elif left_black:
                logger.debug("Left sensor on black, turning right")
                self.left_motor.set_dps(TURN_SPEED)
                self.right_motor.set_dps(-TURN_SPEED)
            elif right_black:
                logger.debug("Right sensor on black, turning left")
                self.left_motor.set_dps(-TURN_SPEED)
                self.right_motor.set_dps(TURN_SPEED)
            else:
                logger.debug("No black detected, moving forward")
                self.left_motor.set_dps(MOVE_SPEED)
                self.right_motor.set_dps(MOVE_SPEED)
            time.sleep(0.05)
        self.stop()

    def update_position(self, dx, dy):
        """Update robot's position based on movement."""
        old_position = self.position.copy()
        self.position[0] += dx
        self.position[1] += dy
        logger.info(f"Position updated from {old_position} to {self.position}")

    def get_wall_distances(self):
        """Get distance to nearest wall using ultrasonic sensor (cm)."""
        dist = self.ultrasonic.get_cm()
        logger.debug(f"Ultrasonic distance: {dist} cm")
        return dist

    def verify_position(self):
        """Verify position using ultrasonic sensor and walls after alignment."""
        self.align_with_grid()
        dist = self.get_wall_distances()
        if dist is None:
            logger.warning("Ultrasonic sensor failed to return a distance")
            return False
        if abs(self.orientation - 0) < ORIENTATION_TOLERANCE:  # East
            expected = (4 - self.position[0]) * SQUARE_SIZE
        elif abs(self.orientation - 90) < ORIENTATION_TOLERANCE:  # North
            expected = (4 - self.position[1]) * SQUARE_SIZE
        elif abs(self.orientation - 180) < ORIENTATION_TOLERANCE:  # West
            expected = self.position[0] * SQUARE_SIZE
        elif abs(self.orientation - 270) < ORIENTATION_TOLERANCE:  # South
            expected = self.position[1] * SQUARE_SIZE
        else:
            logger.warning(f"Orientation {self.orientation} not within tolerance of cardinal directions")
            return False
        match = abs(dist - expected) < ALIGNMENT_TOLERANCE
        logger.info(f"Position verification: measured {dist} cm, expected {expected} cm, {'valid' if match else 'invalid'}")
        return match

    def navigate_to(self, target_x, target_y):
        """Navigate to a target position with grid alignment."""
        dx = target_x - self.position[0]
        dy = target_y - self.position[1]
        logger.info(f"Navigating to ({target_x}, {target_y}) from {self.position}, dx={dx}, dy={dy}")

        target_orientation = None
        if dx > 0:
            target_orientation = 0    # East
        elif dx < 0:
            target_orientation = 180  # West
        elif dy > 0:
            target_orientation = 90   # North
        elif dy < 0:
            target_orientation = 270  # South

        if target_orientation is not None:
            delta_angle = (target_orientation - self.orientation + 180) % 360 - 180  # Shortest turn angle
            if abs(delta_angle) > ORIENTATION_TOLERANCE:
                logger.info(f"Adjusting orientation from {self.orientation} to {target_orientation}")
                self.turn(delta_angle)

        steps = abs(dx) + abs(dy)
        for step in range(steps):
            logger.info(f"Step {step + 1}/{steps}: Moving one square")
            self.move_forward(SQUARE_SIZE)
            self.align_with_grid()
            if dx != 0:
                self.update_position(dx // abs(dx) if dx != 0 else 0, 0)
                dx -= dx // abs(dx) if dx != 0 else 0
            elif dy != 0:
                self.update_position(0, dy // abs(dy) if dy != 0 else 0)
                dy -= dy // abs(dy) if dy != 0 else 0

    def identify_room(self):
        """Identify the current room based on floor color (average of both sensors)."""
        left_rgb = self.get_rgb_left()
        right_rgb = self.get_rgb_right()
        if left_rgb is None or right_rgb is None:
            logger.warning("One or both color sensors failed to return RGB values")
            return "unknown"
        avg_rgb = [(l + r) / 2 for l, r in zip(left_rgb, right_rgb)]
        logger.debug(f"Average RGB for room identification: {avg_rgb}")

        if self.detect_color(avg_rgb, "purple"):
            return "burning room"
        elif self.detect_color(avg_rgb, "yellow"):
            return "avoid room"
        elif self.detect_color(avg_rgb, "white"):
            return "hallway"
        return "unknown"

def main():
    robot = FirefighterRobot()
    logger.info(f"Starting navigation at {robot.position}")

    for x, y in HALLWAY_PATH[1:]:
        logger.info(f"Target position: ({x}, {y})")
        robot.navigate_to(x, y)
        if not robot.verify_position():
            logger.warning("Position verification failed, attempting to proceed after realignment")
            robot.align_with_grid()

    # Once 
    left_rgb = robot.get_rgb_left()
    right_rgb = robot.get_rgb_right()
    if robot.detect_color(left_rgb, "orange") and robot.detect_color(right_rgb, "orange"):
        logger.info("Detected orange entrance at (3,2), entering burning room")
        robot.move_forward(SQUARE_SIZE / 2)
        robot.update_position(0, 1)
    else:
        logger.error("Entrance not found at (3,2), stopping")
        robot.stop()
        return

    room = robot.identify_room()
    logger.info(f"Identified room: {room}")
    if room == "avoid room":
        logger.error("Wrong room detected (yellow), stopping")
        robot.stop()
    elif room == "burning room":
        logger.info("Successfully entered the burning room at (3,3)")
    else:
        logger.error("Unknown room, stopping for safety")
        robot.stop()

    robot.stop()

if __name__ == "__main__":
    main()