# Anticollision system for UAV
The purpose of this code is to sense objects in four directions of the quadcopter(front, back, left and right), in order to avoide any contact between quadcopter and 
      objects such as, walls, trees, humans, etc and calculate the movement to avoide that object and sends that data to pixhawk. The sensors used in this code are 
    TFmini LIDar x1 in the front of quad and x3 Ultrasonic sensors(HC SR04) on other three sides. The pin connections are commented below, adjacent to the code line. 
      The sensors are connected to "Arduino UNO" and UNO is connected to "PIXHAWK" flight controller via TELEM2 port, which is used for serial communication (UART). 
