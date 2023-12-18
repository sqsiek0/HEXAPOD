'''
    File info:
    Controlling the tilt, roll and pitch of the robot's body as well as its gait using the keyboard.
    Publishing command on topic "body_IK_calculations"
'''


# detecting the OS
import sys
import paho.mqtt.client as mqtt
import json
import time

# standard ros2 functionality
import rclpy

# importing our new message type
from hexapod_controller_interfaces.msg import BodyIKCalculate

rclpy.init()

    #TODO: Podmienić noda
node = rclpy.create_node('phone_operate')
pub_ = node.create_publisher(BodyIKCalculate, "body_IK_calculations", 10)

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as BodyIKCalculate message.

|---------------------------------------------------------------------------|
|                   List of keys used to change the values                  |
|---------------------------------------------------------------------------|
|   Translation:            |   Rotation:           |   Movement direction: |
|       Q   W   E           |       U   I           |       1               |
|       A   S   D           |       J   K           |       2               |
|                           |       M   ,           |       3               |
|                           |                       |                       |
|---------------------------|-----------------------|-----------------------|
|   W / S : X translation   |   U / I : X rotation  |   1 : move forward    |
|   A / D : Y translation   |   J / K : Y rotation  |   2 : move backward   |
|   Q / E : Z translation   |   M / , : Z rotation  |   3 : move stop       |
|                           |                       |                       |
|---------------------------------------------------------------------------|


|---------------------------------------------------------------------------|
|               List of keys used to change state of the robot              |
|---------------------------------------------------------------------------|
|   Controll mode / state:                                                  |
|       4   8                                                               |
|       5   9                                                               |
|       6   0                                                               |
|       7                                                                   |
|                                                                           |
|---------------------------------------------------------------------------|
|   4 : walk in a given direction (can be changed with 1 and 2)             |
|   5 : rotate the robot counter-clockwise                                  |
|   6 : rotate the robot clockwise                                          |
|   7 : enter the body manipulation mode (standing still)                   |
|   8 : enable the torque on every servo                                    |
|   9 : disable the torque on every servo                                   |
|                                                                           |
|   0 : idle state, do nothing                                              |
|                                                                           |
|---------------------------------------------------------------------------|

anything else : back to initial position

CTRL-C        : go back to initial position and quit
"""

chooseRobotState = {
    '0': "idle",
    '4': "walk",
    '5': "rotate_left",
    '6': "rotate_right",
    '7': "body_manipulation",
    '8': "torque_enable",
    '9': "torque_disable",
}

movementDirection = {
    '1': 1,
    '2': -1,
}

moveBindings = {
    'w': (10, 0, 0),
    's': (-10, 0, 0),
    'd': (0, 10, 0),
    'a': (0, -10, 0),
    'q': (0, 0, 10),
    'e': (0, 0, -10),
}

rotationBindings = {
    'u': (2, 0, 0),
    'i': (-2, 0, 0),
    'j': (0, 2, 0),
    'k': (0, -2, 0),
    'm': (0, 0, 2),
    ',': (0, 0, -2),
}

# TODO !!!
limits = {
    # X axis translation limits
    'transXMin': 0,
    'transXMax': 0,
    # Y axis translation limits
    'transYMin': 0,
    'transYMax': 0,
    # Z axis translation limits
    'transZMin': 0,
    'transZMax': 0,

    # X axis rotation limits
    'rotXMin': 0,
    'rotXMax': 0,
    # Y axis rotation limits
    'rotYMin': 0,
    'rotYMax': 0,
    # Z axis rotation limits
    'rotZMin': 0,
    'rotZMax': 0,
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


def positionToCalculate(transX, transY, transZ, rotX, rotY, rotZ):
    return 'currently:\n\ttransX: %s\n\ttransY: %s\n\ttransZ %s\n\trotX: %s\n\trotY: %s\n\trotZ: %s' % (transX, transY, transZ, rotX, rotY, rotZ)

def positionSubscriber(msg: BodyIKCalculate):
    # transX, transY, transZ, rotX, rotY, rotZ = msg
    transX = msg.position_of_the_body[0]
    transY = msg.position_of_the_body[1]
    transZ = msg.position_of_the_body[2]
    rotX = msg.position_of_the_body[3]
    rotY = msg.position_of_the_body[4]
    rotZ = msg.position_of_the_body[5]
    print(f'currently:\n\ttransX: {transX}\n\ttransY: {transY}\n\ttransZ {transZ}\n\trotX: {rotX}\n\trotY: {rotY}\n\trotZ: {rotZ}')
    
def on_message(client, userdata, message):
    print(f"Otrzymano wiadomość na temacie {message.topic} z treścią: {message.payload.decode()}")
    if message.topic == 'robot/walking':
        on_message_walking(message)
    elif message.topic == 'robot/turning':
        on_message_turning(message)
    elif message.topic == 'robot/translation':
        on_message_translation(message)
    elif message.topic == 'robot/rotate':
        on_message_rotation(message)
    # return f'currently:\n\ttransX: {transX}\n\ttransY: {transY}\n\ttransZ {transZ}\n\trotX: {rotX}\n\trotY: {rotY}\n\trotZ: {rotZ}'
    
def on_message_walking(message):
    print('Callback - walking')
    data = json.loads(message.payload)
    final_post(data, 'walking')
    
def on_message_turning(message):
    print('Callback - turning')
    data = json.loads(message.payload)
    final_post(data, 'turning')
    
def on_message_translation(message):
    print('Callback - translation')
    data = json.loads(message.payload)
    final_post(data, 'translation')
    
def on_message_rotation(message):
    print('Callback - rotation')
    data = json.loads(message.payload)
    final_post(data, 'rotation')
    
def final_post(data, type): 
    transX = 0
    transY = 0
    transZ = 0
    rotX = 0
    rotY = 0
    rotZ = 0
    status = 0
    direction = 1
    robot_state = "idle"
    
    try:
        if type == 'walking':
            print('walking')
            keyDirection = data['moveDirection']
            keyState = data['robotState']
            direction = movementDirection[keyDirection]
            robot_state = chooseRobotState[keyState]
        elif type == 'turning':
            print('turning')
            key = data['robotState']
            robot_state = chooseRobotState[key]
        elif type == 'translation':
            print('translation')
            key = data['translationState']
            transX = transX + moveBindings[key][0]
            transY = transY + moveBindings[key][1]
            transZ = transZ + moveBindings[key][2]
        elif type == 'rotation':
            print('rotation')
            key = data['rotationState']
            rotX = rotX + rotationBindings[key][0]
            rotY = rotY + rotationBindings[key][1]
            rotZ = rotZ + rotationBindings[key][2]
        else:
            transX = 0
            transY = 0
            transZ = 0
            rotX = 0
            rotY = 0
            rotZ = 0
            direction = 1
            robot_state = "idle"
    except Exception as e:
        print(e)
    # finally:
    #     cmd = BodyIKCalculate()
    #     cmd.position_of_the_body[0] = 0
    #     cmd.position_of_the_body[1] = 0
    #     cmd.position_of_the_body[2] = 0
    #     cmd.position_of_the_body[3] = 0
    #     cmd.position_of_the_body[4] = 0
    #     cmd.position_of_the_body[5] = 0
    #     cmd.move_direction = 1
    #     cmd.robot_state = "idle"
    #     pub_.publish(cmd)
        
    print(transX, transY, transZ, rotX, rotY, rotZ, direction, robot_state)
    cmd = BodyIKCalculate()
    cmd.position_of_the_body[0] = transX
    cmd.position_of_the_body[1] = transY
    cmd.position_of_the_body[2] = transZ
    cmd.position_of_the_body[3] = rotX
    cmd.position_of_the_body[4] = rotY
    cmd.position_of_the_body[5] = rotZ
    cmd.move_direction = direction
    cmd.robot_state = robot_state
    pub_.publish(cmd)
    
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
    else:
        print(f"Failed to connect, return code {rc}\n")
        
def on_subscribe(client, userdata, mid, granted_qos):
    print(f"Subscribed: {mid} with QoS: {granted_qos}")
    
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print(f"Unexpected disconnection: {rc}")

def main():
    settings = saveTerminalSettings()

    
    
    # sub_ = node.create_subscription(BodyIKCalculate, "body_IK_calculations", positionSubscriber, 10)

    transX = 0
    transY = 0
    transZ = 0
    rotX = 0
    rotY = 0
    rotZ = 0
    status = 0
    direction = 1
    robot_state = "idle"
    
    client = mqtt.Client()
    client.on_message = on_message
    client.on_connect = on_connect
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect
    
    
    try:
        client.connect("134.122.84.130", 1883, 60)  # Połącz z brokerem (IP, port, keepalive)
        client.subscribe('robot/walking')
        client.subscribe('robot/turning')
        client.subscribe('robot/translation')
        client.subscribe('robot/rotate')
        
        client.loop_start()
        print('Connected and loop started')
        
    except Exception as e:
        print(f"Can't connect to MQTT broker: {e}")
        
    try:
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Ending the program...")
        client.loop_stop()

    # while True:
    #     print('cos')
    #     key = getKey(settings)
    # try:
    #     print(msg)
    #     # print(positionToCalculate(transX, transY, transZ, rotX, rotY, rotZ))
    #     print("\n---------------------------\n")
    #     while True:
    #         #TODO: Zrobić by ciągnał z pliku variables
    #         key = getKey(settings)
    #         # print(positionToCalculate(transX, transY, transZ, rotX, rotY, rotZ))
    #         if (status == 14):
    #             print(msg)
    #         status = (status + 1)
    #         if key in moveBindings.keys():
    #             transX = transX + moveBindings[key][0]
    #             transY = transY + moveBindings[key][1]
    #             transZ = transZ + moveBindings[key][2]
    #         elif key in rotationBindings.keys():
    #             rotX = rotX + rotationBindings[key][0]
    #             rotY = rotY + rotationBindings[key][1]
    #             rotZ = rotZ + rotationBindings[key][2]              
    #         elif key in movementDirection.keys():
    #             direction = movementDirection[key]
    #         elif key in chooseRobotState.keys():
    #             robot_state = chooseRobotState[key]
    #         else:
    #             transX = 0
    #             transY = 0
    #             transZ = 0
    #             rotX = 0
    #             rotY = 0
    #             rotZ = 0
    #             direction = 1
    #             robot_state = "idle"
    #             if (key == '\x03'):
    #                 break

    #         cmd = BodyIKCalculate()
    #         cmd.position_of_the_body[0] = transX
    #         cmd.position_of_the_body[1] = transY
    #         cmd.position_of_the_body[2] = transZ
    #         cmd.position_of_the_body[3] = rotX
    #         cmd.position_of_the_body[4] = rotY
    #         cmd.position_of_the_body[5] = rotZ
    #         cmd.move_direction = direction
    #         cmd.robot_state = robot_state
    #         pub_.publish(cmd)

    # except Exception as e:
    #     print(e)

    # finally:
    #     cmd = BodyIKCalculate()
    #     cmd.position_of_the_body[0] = 0
    #     cmd.position_of_the_body[1] = 0
    #     cmd.position_of_the_body[2] = 0
    #     cmd.position_of_the_body[3] = 0
    #     cmd.position_of_the_body[4] = 0
    #     cmd.position_of_the_body[5] = 0
    #     cmd.move_direction = 1
    #     cmd.robot_state = "idle"
    #     pub_.publish(cmd)

    #     restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()