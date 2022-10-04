# STM-Quadcopter
 A simple quadcopter built on the STM32F411CEU6 (black pill board) with 4 motors, a PPM receiver, ADC checking battery level and I2C communication with MPU6050 imu.
## Hardware
Below are images of the perfboard wired up with the I2C (SDA=Yellow, SCL=White), PWM motor outputs (Green), PPM RC receiver input (Blue) and the battery ADC voltage divider across the resistors.
<p align="center">
<img src="https://i.imgur.com/QVK6CNa.jpg" height="250"/>
<img src="https://i.imgur.com/rY7LFSb.jpg" height="250"/>
<img src="https://i.imgur.com/TVj17ee.jpg" height="250"/>
</p>
Below is a simple schematic showing the pin connections, this doesnt include and voltage regulation or requirements for the MCU as this is taken care of by the black pill board and MPU-6050 breakout board.
<p>
<img src="https://svgur.com/i/n85.svg">
</p>