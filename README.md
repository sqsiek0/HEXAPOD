# HEXAPOD Robot 🤖🕷
<p align='center'>
<img src="./img/hexapod.png" width="375" height="260" />
</p>

## About:
This is the repository of the academic project which my friends and I are developing. By any means is it perfect, but we will work **HARD** to improve the functionality and make it work! 

What we want to achieve:

- Develop a functional control system for the **HEXAPOD** robot,
- Learn more about Raspberry Pi (i.e. HALF-Duplex UART communication),
- Learn about ROS2 software,
- Learn about Docker and contenerized environments,
- Learn about robotics (especially inverse kinematics),
- Learn about OOP programming in python and ROS2,
- Learn how to use Git & GitHub to collaborate.

### First success:
We created a closed loop control system utilizing:
- Publisher node,
- Subscriber node,
- ROS2 service,
- Turtlesim demo.

<p align='center'>
<img src="./img/turtle.png" width="375" height="260" />
</p>

### Successful connection:
We managed to connect the servos and made sure that all 18 of them were working! 🥳

<p align='center'>
<img src="./img/first_move.gif" width="375" height="260" />
</p>

### First Stand up:
After few attempts HEXAPOD can finaly stand up easily! Still a lot of work to do, but we are ready to migrate the prototype Python scripts to ROS2 to control all of the servos simultaneously.

<p align='center'>
<img src="./img/hexapod_standup.gif" width="375" height="260" />
</p>

### Controlling all of the servos /w Inverse Kinematics:
Many hours and iterations later we finally managed to use the ROS2 capabilities to move every leg simultaneously! To achieve this we used:
- improved code structure - we moved every leg into one file and made separate functions for each leg,
- custom interface, which is basically an array of positions for the servos,
- different packet structure for controlling the servos - we used a Dynamixel's class called "GroupSyncWrite" to minimize the number of the messages sent (just 6 now!),
- launch file, which spawns a separate process for each of the legs and for the body, allowing us to execute them at the same time.

<p align='center'>
<img src="./img/hexapod_synchro_test.gif" width="375" height="260" />
<span>&ensp;</span>
<img src="./img/hexapod_pushups.gif" width="375" height="260" />
<br>
<img src="./img/custom_interface.png" height="150" />
</p>
