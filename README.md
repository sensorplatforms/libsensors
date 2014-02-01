libsensors Android Sensors HAL
==========

This is an InputEvent based Sensors HAL implementation for interfaceing to OSP based sensor-hubs.

# Setting up a build environment
  * clone into your android tree under hardware/sensorplatforms/libsensors
  * execute a top level build
  * 'mm' builds from this directory will work after prerequisites have been generated from the first top level build  
  
# Customizing for a platform
Platform specific changes should be done in their own branches.

For example if a platform needs a light and proximity sensor not handled by the sensorhub.  First branch from master with platform-\<boardname\>, then make the appropriate changes to sensors.cpp

Don't forget to change the LOCAL_MODULE name in Android.mk to match your platform's required name.

# Device Installation
During development, or on platforms where it is not desirable to flash the entire Android image, it is easiest to replace libsensors directly with adb.  

Assuming you've built from master (which names its output sensors.osp.so):
  * take note of the precise name of the existing sensors.<board-name>.so and rename or delete it
  * adb push your compiled sensors.osp.so 
  * softlink sensors.osp.so to the board name
  * for your new libsensors to be loaded do a full reboot. 
  * alternatively, kill the zygote process to restart the Android layer, keeping linux level processes active 
