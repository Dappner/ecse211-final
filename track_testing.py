from utils.brick import wait_ready_sensors, Motor
import time

MOTOR_DPS = 90

motor_left = Motor("D")
motor_right = Motor("A")

wait_ready_sensors()


def turn_90():
    print("Turning!")
    motor_left.reset_encoder()
    motor_right.reset_encoder()

    motor_left.set_dps(MOTOR_DPS)
    motor_right.set_dps(-MOTOR_DPS)

    time.sleep(2)

    reset_motors()
    return


def advance_one():
    print("advancing one field")
    motor_left.set_dps(MOTOR_DPS)
    motor_right.set_dps(MOTOR_DPS)

    time.sleep(2)

    reset_motors()


def main():
    print("Attempting Turn!")
    turn_90()
    time.sleep(5)
    turn_90()


def reset_motors():
    motor_left.set_power(0)
    motor_right.set_power(0)


if __name__ == "__main__":
    print("Start")
    try:
        main()
    except Exception:
        reset_motors()
        print("whoops!")
