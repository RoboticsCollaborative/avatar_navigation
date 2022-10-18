## Dependencies
`pip install pyserial`



Controller needs to run on the cumputer with the IMU USB input. Run the controller with:
`rosrun avatar_imu_controller imu_driver.py`

You will need to either change the udev rules or the permissions for the usb input. The simple way is:
`chmod 777 /dev/ttyUSB0`
