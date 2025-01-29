from flymambo import *


def main():
    try:
        mamboAddr = "D0:3A:96:D0:E6:3B"
        fly = FlyMambo(mamboAddr)
        fly.get_battery()
        fly.takeoff()
        for i in range(2):
            fly.move_mambo(move="FWD", amount=1)
        fly.move_mambo(move="RGT_TURN", degrees=180)
        fly.land()
        exit(0)
    except KeyboardInterrupt:
        fly.emergency()
    return


if __name__ == '__main__':
    main()
