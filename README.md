The provided files contain the necessary project files, settings, source code and drivers utilized within the STM32 IDE. 

This robot may be used applicably within autonomous delivery systems or as a segway into automatic automobile driving features.
Some noticable features that enhance the safety and overall quality of this project include:
- Push Start / Stop through Timer Interrupt
- 6 Case Line Following Logic for Motors
- Obstacle Collision Detection ( Near Linear Deacceleration )
- Adaptive Speed Measures ( Braking Function to Counteract Forward Momentum )

The robot will be able to safely traverse through courses as desired without control of the user (disregarding start/stop)

Requirements:
1x STM32 IDE, Nucleo-L476RG Board
4x DC Motors
1x H Bridge Motor Driver 
3x IR Sensors
1x Ultrasonic Sensor
1x DC Power Supply (Li-Ion, 2x 3.7V)

Pin Assignments:
PC13 B1
PC0 SENSOR_LEFT
PC1 SENSOR_CENTER
PC3 SENSOR_RIGHT
PA6 HC-SR04 TRIG
PB9 HC-SR04 ECHO
PB0 IN1
PB1 IN2
PB2 IN3
PB10 IN4
PA0 TIM2 CH1
PA1 TIM2 CH2
PA5 Onboard LED

GETTING STARTED)
Import files into Existing Projects into Workspace --- This method will vary slightly depending on your version of STM32 IDE utilizied. 
Select Project Root; Nucleo-L476RG Board
Compile project onto Nucleo-L476RG Board
With your desired track, press B1 to begin. There is a 200ms delay until the robot begins driving. 
At any point, the user may press B1 once more to interrupt the service, freezing the functionality. 
