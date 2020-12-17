#### lepton_camera
* Follow libuvc from groupgets/libuvc,
```
~$ git clone https://github.com/groupgets/libuvc
~$ cd libuvc
~$ mkdir build
~$ cd build
~$ cmake ..
~$ make && sudo make install
```
* If error occurred make sure
```
~$ apt-get install libusb-1.0-0-dev
```
**Above installation process are utilized from [groupgets/purethermal1_uvc-capture](https://github.com/groupgets/purethermal1-uvc-capture)**
* Run ros node
```
~$ sudo su
~$ source /home/{$user}/catkin_ws/devel/setup.bash
~$ rosrun lepton_camera uvc-radiometry_node.py
```
