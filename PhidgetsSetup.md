# Introduction #

You will need to give your user permission to use the USB device

## Permissions ##
  * To use phidgets you must have permissions for the usb bus.
  * After plugging in the phidget the way to make sure you have permission is to
```
sudo chmod 777 -R /dev/bus/usb
```
Now remember that if you're worried about security you should actually set it for your specific user.