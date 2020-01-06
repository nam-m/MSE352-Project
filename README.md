## Servo DC Motor Speed Controller Using ARM MCU

![DC motor speed controller](https://user-images.githubusercontent.com/32988140/71801998-624cd080-3011-11ea-9c66-757fccebb2e8.jpg)

### Objective
To program a proportional controller to generate PWM signal to control the
speed of a Servo DC motor and display the Revolutions Per Minute (RPM) on 3-digit 7-segment. 

### List of Components
Component | Description
------------ | -------------
MCU | Tiva C Series TM4C123G LaunchPad
Servo DC Motor (12V) | AUB0812L-9X41 (Delta Electronics)
IDE | Code Composer Studio (CCS)
3-digit 7-segment LEDs | For Binary-coded decimal (BCD) display
N-channel MOSFET, potentiometer, resistors


### Description
The MCU needs to generate PWM signal to the Servo DC motor and be able to proportionally control
the speed of the motor. Then, the Servo DC motor encoder should give a closed loop feedback signal
from the motor shaft speed, which is also known as tachometer. 

From there, MCU must be set up to run a timer to start counting the edges of digital signal 
that is converted by ADC from Servo DC motor to MCU. 
In order to calculate the speed of the motor, the time is recorded for every three edge changes
because every three-edge change is one complete period of the feedback signal. 

Afterwards, the speed of the motor is determined from the average of three continuous samples. 
Next, another ADC in MCU is set up to read the reference speed from the potentiometer. 
The reference speed is then compared to the actual speed to find the error source. 
With the error source, the speed reading is adjusted to displayed on 3-digit 7-segment.
