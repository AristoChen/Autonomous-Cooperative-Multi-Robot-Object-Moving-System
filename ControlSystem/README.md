# ControlSystem

ControlSystem should be put on the top of the space in order to capture the whole space with Raspberry pi Camera module.

ControlSystem will detect every quadrilateral in the screen, and record the coordinate of each point of the quadrilateral, and crop it for QRcode decoding, then we can know the position of the QRcode and recognize the item ID stored in the QRcode.

#### Hardware 
 - Raspberry Pi 3
 - Raspberry Pi Camera Module
#### SoftWare
 - OpenCV
 - Zbar

## Installation

##### OpenCV  
 We use an easy way to install OpenCV, instead of compiling the source code. 
```sh
$ sudo apt-get install libopencv-dev
```
##### Zbar
Download source code first from [Zbar](http://zbar.sourceforge.net/download.html), and then do the following commands 

```sh
$ sudo apt-get install python-gtk2-dev
$ sudo apt-get install libv4l-dev
$ sudo apt-get install libmagickwand-dev
$ sudo apt-get install qt4-dev-tools
$ cd /usr/include/linux
$ sudo ln -s ../libv4l1-videodev.h videodev.h
$ cd ~/Download
$ tar jxf zbar-0.10.tar.bz2
$ cd zbar-0.10
$ ./configure
$ make 
$ sudo make install
```


