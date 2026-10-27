# i2c addons  

The ZRSense and ZRCore board both have expansion ports for i2c devices. This catalogs the device you can add. We have written drivers for devices that may not be included in the packages we sell like the rangefinder and mpu6050 gyro accelerometer but you can buy these from amazon in module form and add them. 


## Rangefinder VL53L5C

We intially wanted to have this on the ZRSense board, but found several issues with it namely: it made troubleshooting the pcb very hard, it made the pcb more expensive, there is a complicated calibration routine, it's not very useful without something to get the heading (direction) and since it is on the head unit it is very limited on what it can sense because its only pointing in the direction of the head and the robot body blocks it. So we do not populate it on the boards and units we sell. We plan on creating a rotating version that is an add on to the robot.

![NotPopulated](https://github.com/zonemikel/ZoneRobotics/blob/main/i2c_addons/rangefinder_not_populated.PNG?raw=true)  

 We already have the drivers working, so if you need to have the rangefinding abilities as well as the heading and gyro features you can simply buy one off amazon and add it to the i2c header on the back of the ZRSense board, or the i2c expansion header on the ZRCore board.  
 
 If you solder the two boards together with power, ground, scl, sda it works out to be very compact and easy to add to the front of the ZRCar, if you have any questions feel free to ask in discord.

![addongyrorangefinder](https://github.com/zonemikel/ZoneRobotics/blob/main/i2c_addons/zrcar_sensor.jpg?raw=true)  

## Links to modules on amazon  

https://a.co/d/01IR4GiY VL53L5X $17.99

https://a.co/d/04xmXOyU MPU6050 $9.99 3PCS

