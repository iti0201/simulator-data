"""
- soidab mustast joonest paremal
- valge > 70
- must < 11
 @author: Martin Ojamets
"""

from ev3dev import ev3


class Robot:
    def __init__(self):
        self.speed = 100
        self.btn = ev3.Button()
        self.motor_left = ev3.LargeMotor('outA')
        self.motor_right = ev3.LargeMotor('outD')
        self.color_sensor = ev3.ColorSensor('in2')
        self.color_sensor.mode = "COL-REFLECT"  # 0-100

    def drive(self):
        self.motor_left.run_forever(speed_sp=self.speed * 4)
        self.motor_right.run_forever(speed_sp=self.speed * 4)

    def turn(self, direction):
        if direction == "right":
            # print("turning right slowly with:")
            self.motor_right.stop()
            self.motor_left.run_forever(speed_sp=self.speed * 3)
        elif direction == "left":
            # print("turning left slowly with:")
            self.motor_left.stop()
            self.motor_right.run_forever(speed_sp=self.speed * 3)

    def turn_fast(self, direction):
        if direction == "right":
            # print("turning right with:", -self.speed)
            self.motor_right.run_forever(speed_sp=-self.speed)
            self.motor_left.run_forever(speed_sp=self.speed)
        elif direction == "left":
            # print("turning left with:", -self.speed)
            self.motor_left.run_forever(speed_sp=-self.speed) # Changed these
            self.motor_right.run_forever(speed_sp=self.speed)

    def stop(self):
        self.motor_left.stop()
        self.motor_right.stop()

    def sense_reflection(self):
        return self.color_sensor.value()


def isOnTrack(reflections, last_turn):
    # reflections on list, kus 2 elementi
    # 0 - straight (last - current = 0)
    # 1 - right (last - current > 0)
    # -1 - left (last - current < 0)
    # 2 - fast right
    # -2 - fast left
    last = reflections[0]
    current = reflections[1]
    if 15 < current < 35:
        if last == current:
            if current > 30:
                if last_turn == "right":
                    return -1
                elif last_turn == "left":
                    return 2
            return 0
        elif last - current > 0:
            return 1
        elif last - current < 0:
            return -1
    elif current <= 15:
        return 1
    elif current >= 35:
        return -1


def main():
    robot = Robot()
    last_turn = "right"
    reflections = []
    for x in range(2):
        reflections.append(robot.sense_reflection())
    try:
        while not robot.btn.any():
            print(str(reflections[0]) + " : " + str(reflections[1]))
            robot.drive()
            # lisab 3 iteratsiooni väärtused
            reflections.append(robot.sense_reflection())

            # hoiab 2 iteratsiooni v22rtused (kustutab hilisema)
            reflections.pop(0)

            if isOnTrack(reflections, last_turn) == 0:
                robot.drive()
            elif isOnTrack(reflections, last_turn) == 1:
                last_turn = "right"
                robot.turn_fast("right")
            elif isOnTrack(reflections, last_turn) == -1:
                last_turn = "left"
                robot.turn_fast("left")
            elif isOnTrack(reflections, last_turn) == 2:
                last_turn = "right"
                robot.turn("right")
        robot.stop()
    except KeyboardInterrupt:
        robot.stop()


if __name__ == "__main__":
    main()
