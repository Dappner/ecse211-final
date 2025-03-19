from utils.brick import wait_ready_sensors, Motor
import time


class FinalProject:
    def __init__(self):
        # Motor Port
        self.motor = Motor("D")

        # Wait for sensors to be ready
        wait_ready_sensors()

    def main_loop(self):
        """Main loop"""
        # Loop
        while True:
            # Check for start stop

            time.sleep(0.02)

    def reset_sensors(self):
        self.motor.set_power(0)


def main():
    finalProject = FinalProject()
    try:
        finalProject.main_loop()
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print("\nAn error occurred: ", e)
    finally:
        finalProject.reset_sensors()


if __name__ == "__main__":
    main()
