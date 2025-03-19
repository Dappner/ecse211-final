from utils.brick import wait_ready_sensors, Motor
import time

motor_left = Motor("D")
motor_right = Motor("A")

wait_ready_sensors()


def turn_90():
    print("Turning!")
    MULTIPLIER = 4
    motor_left.set_position(MULTIPLIER * 360)
    motor_right.set_position(-MULTIPLIER * 360)
    time.sleep(2)
    return


def main():
    print("Attempting Turn!")
    turn_90()


def reset_sensors():
    motor_left.set_power(0)
    motor_right.set_power(0)


if __name__ == "__main__":
    print("Start")
    try:
        main()
    except Exception:
        reset_sensors()
        print("whoops!")
