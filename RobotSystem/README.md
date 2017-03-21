# RobotSystem

#### Hardware
 - Raspberry Pi 3
 - DC Motor * 2, wheel * 2, L293D IC chip * 1 
 - [meArm](https://shop.mime.co.uk/) * 1, servo motor * 4 
 - wire * N
 - 
 
#### SoftWare
We use [WiringPi](http://wiringpi.com/) to control Raspberry Pi GPIO, the following is installation commands.


```sh
$ git clone git://git.drogon.net/wiringPi
$ cd wiringPi
$ git pull origin
$ ./build
```
After installation, you can type the following commands to check whether it is successfully installed or not.

```sh
$ gpio –v
$ gpio readall
```

You can also use the example code below to check. First create a file called "TEST.c", then copy the code. 
```sh
#include <wiringPi.h>
#include <stdlib.h>
#include <stdio.h>

void init_pins();

int main()
{
	wiringPiSetup();
	init_pins();
	while(1)
	{
		digitalWrite(1, HIGH);
		delay(1000);
		digitalWrite(1, LOW);
		delay(500);
	}
}

void init_pins()
{
	pinMode(1, OUTPUT); //Pin 12, GPIO 18
}
```

then compile with the commands below, "-lwiringPi" tells your Raspberry Pi to compile with the WiringPi library, if you don't add that, compilation will failed.

```sh
$ gcc –o TEST TEST.c -lwiringPi
```

After Compilation, you can type the following command to execute. Remember to wire up a LED to PIN 12 of Raspberry Pi 3 before execute, also always remember to add "sudo" before executing a file that compile with WiringPi library
```sh
$ sudo ./TEST
```
