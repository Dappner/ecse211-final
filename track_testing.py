from utils.brick import wait_ready_sensors, Motor
import time

motor_left = Motor("D")
motor_right = Motor("A")

wait_ready_sensors()


def turn_90():
    MULTIPLIER = 4
    motor_left.reset_position()
    motor_right.reset_position()
    motor_left.set_position(MULTIPLIER * 360)
    motor_right.set_position(-MULTIPLIER * 360)
    return


def main():
    turn_90()


def reset_sensors():
    motor_left.set_power(0)
    motor_right.set_power(0)


if __name__ == "__main__":
    try:
        main()
    except Exception:
        reset_sensors()
