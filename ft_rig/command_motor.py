from time import sleep
import sys
import ft_rig.motors as motors
from ft_rig.motors import SteeringMotor, ThrottleMotor

motors.init()

steering = SteeringMotor()
throttle = ThrottleMotor()

actions = {
    "forwards" : throttle.forwards,
    "up" : throttle.forwards,
    "backwards" : throttle.backwards,
    "down" : throttle.backwards,
    "left" : steering.left,
    "right" : steering.right
}

def execute(commands):
    global actions
    for command in commands:
        actions[command]()
        sleep(1)
        steering.stop()
        throttle.stop()
        sleep(1)

if len(sys.argv) == 1:
    execute(["up", "down", "left", "right"])
elif sys.argv[1] == "interactive":
    while True:
        print("command: ", end="")
        try:
            execute(input().split())
        except KeyError as e:
            print("unknown command given !!!")
else:
    execute(sys.argv[1:])
