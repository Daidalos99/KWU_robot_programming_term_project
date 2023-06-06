#!/usr/bin/env python3
import sys
from geometry_msgs.msg import Twist
import rclpy


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
본 노드는 비글을 위해 만들어진 teleoperation 노드이다. 

created by 광운대 로봇학부 21학번 이일주.

움직임을 원하면 로봇의 좌표계를 기준으로 아래 키를 입력하면 된다


        왼쪽    중앙  오른쪽
전진        u    i    o
회전        j         l
후진        m    ,    .


속도를 증가시키고 싶으면 +키를 입력하고 감소시키고 싶으면 -키를 입력하면 된다.

속도의 기준은 비글의 max_motor_vel을 100으로 잡고 사용하였다.

"""

moveBindings = {
    # 전진방향 
    'i': (1, 1), #전진
    'o': (1, 0.5), #오른쪽으로 꺾으면서 전진
    'u': (0.5, 1), #왼쪽으로 꺾으면서 전진
    # 회전
    'j': (-0.5, 0.5), #제자리에서 왼쪽 회전
    'l': (0.5, -0.5), #제자리에서 오른쪽 회전
    # 후진 방향
    ',': (-1, -1), #후진
    '.': (-1, -0.5), #오른쪽으로 후진
    'm': (-0.5, -1)
}

speedBindings = {
    '+': (1.1, 1.1),
    '-': (.9, .9),
}

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tleft_wheel = %s\tright_wheel =  %s ' % (speed, turn)

def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = rclpy.create_node('move_beagle')

    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    left_speed = 50.0
    right_speed = 50.0
    left = 0.0
    right = 0.0
    status = 0.0


    try:
        print(msg)


        while True:
            
            key = getKey(settings) #키보드 입력받기
            
            if key in moveBindings.keys(): #기본 이동 키보드 입력시 
                left = moveBindings[key][0]
                right = moveBindings[key][1]
                print(vels(left*left_speed, right*right_speed))
            elif key in speedBindings.keys():
                left_speed = left_speed * speedBindings[key][0]
                right_speed = right_speed * speedBindings[key][1]

                print(vels(left_speed, right_speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                if (key == '\x03'): #ctrl+c를 입력받았을경우 while문 탈출
                    break

            twist = Twist()
            twist.linear.x = left * left_speed
            twist.angular.z = right * right_speed
            pub.publish(twist)

    except Exception as e:
        print(e)

        restoreTerminalSettings(settings)





if __name__ == '__main__':
    main()