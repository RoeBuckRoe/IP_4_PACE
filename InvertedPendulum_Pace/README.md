# UKAEA-Inverted Pendulum

Inverted Pendulum project shared repository

## Description

This is a prototype application demonstrating the use of MARTe2 to implement an inverted pendulum control application.

## Test Status

Tested 22/09/2024 on WSL.  Still need to test two other platforms.

1. Linux x86_64
1. Raspbery Pi 4.

## Build

This assumes that yo have already cloned this repository in a working directory. If not, then

```
$ git clone https://github.com/AdamVStephen/inverted-pendulum
```

Next, clone and build both `MARTe2` and `MARTe2-components`. For example, on a Raspberry Pi 4 system

```
$ git clone https://github.com/ukaea/MARTe2.git
$ cd MARTe2
$ git checkout ukaea/rpi4
$ make -f Makefile.x86-linux
$ export MARTe2_DIR=$(pwd)
$ cd ..
$ git clone https://github.com/ukaea/MARTe2-components.git
$ cd MARTe2-components
$ git checkout ukaea/rpi4
$ make -f Makefile.x86-linux
$ export MARTe2_Components_DIR=$(pwd)
```

**Note:** The above `export` commands will only create the `MARTe2_DIR` and `MARTe2_Components_DIR` environment variables for your current terminal session. To avoid having to create these variables in other sessions, you can make them permanent by adding the appropriate lines to your shell configuration file (e.g. `~/.bashrc`):

```
export MARTe2_DIR=<MARTe2 path>
export MARTe2_Components_DIR=<MARTe2-components path>
```

**Note:** When building the `MARTe2` and `MARTe2-components` on the Raspberry Pi, you may see a build failure at the end
related to `libgtest.a`. This only affects the unit test builds, and can safely be ignored for this
project.

Then build the UKAEA/Sygensys components. Assuming that it is located at the same directory level as 
MARTe2 and MARTe2-components:

```
$ cd ../inverted-pendulum
# If building for x86
$ make -f Makefile.x86-linux
```

## Run

Having built the application's components, you can now run the application (bear in mind that the
environment variables `MARTe2_DIR` and `MARTe2_Components_DIR` must be set - see above).

### Running a test configuration

To run one of the test configurations in the `Configurations/Tests` directory, run the following commands:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/Tests/<name of test file> -s State1 --perf
```

See the relevant test report in the `Tests` folder on the Marte For Industry Sharepoint for the hardware and software configuration which needs to be set up before a given test. Note also the use of the `--perf` option to `Main.sh`: this applies various system and application settings which should improve the overall performance of the application. See `Documentation/RPi400/README.md` for more information about the performance settings and their effects.

### Generic serial example

This example demonstrates the use of the `RaspPiSerial` DataSource to listen on a serial port for 
messages, which are then passed into the MARTe2 application as a signal, which is logged to the 
console.

First, set up a pair of virtual serial ports linked together, using the `socat` utility:

```
$ socat -d -d pty,raw,echo=0 pty,raw,echo=0
... socat[8941] N PTY is /dev/pts/8
... socat[8941] N PTY is /dev/pts/9
.. socat[8941] N starting data transfer loop with FDs [5,5] and [7,7]
```

Edit the configuration file `Configurations/Example-RaspPiSerial.cfg` and set the `Port` of the 
`Serial` DataSource to one of the files created by `socat` e.g.

```
...
Port = "/dev/pts/9"
...
```

Then run the example MARTe2 application in another terminal. Execute the following command:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/Example-RaspPiSerial.cfg -s State1
...
```

Finally, send a message of the expected size (see the field `MessageSize` in the configuration file)
to the other serial port of the pair, e.g.:

```
echo "abcdefg" > /dev/pts/8
```

You should see the MARTe2 application emit the corresponding `Buffer` signal (as integers 
corresponding to the ASCII characters of the message):

```
...
[Information - LoggerBroker.cpp:152]: Buffer [0:8]:{ 97 98 99 100 101 102 103 10 0 } 
...
```

### STM32 example

Connect the STM32 development board to the Pi via serial port, and note the serial port it is connected 
on e.g. `/dev/serial0`. Run the STM32 application on the development board (see 
https://github.com/AndrewLarkins/UKAEA-Sygensys-STM32 for instructions).

Edit the example STM32 configuration file `Configurations/Example-STM32-FileLogging.cfg` and set 
the `Port` of the `STM32` DataSource e.g.:

```
...
Port = "/dev/serial0"
...
```

If you want to make any changes to the STM32 application, you'll need to connect up the board to
your computer via the micro-USB connector (which connects to the embedded ST-Link debug adapter).
Then run the STM32CubeIDE application as per the instructions at the above link.

You may also need to set the baud rate, depending on what configuration the STM32 application is 
using (this test has successfully been run at rates up to 460800 baud, and STM32 data rates up to
1000 data frames per second). To change the baud and/or data rate of the STM32 application:

* Edit `htim3.Init.Prescaler` in `main.c:MX_TIM3_Init` to set the data rate. For example, a value
  of `10000 - 1` will set a data rate of 1 data frame per second, `100 - 1` will give 100 data frames
  per second, and so forth.
* Edit `huart4.Init.BaudRate` in `main.c:MX_UART4_Init` to set the baud rate

In any event, after any edits, select `Run->Run` from the toolbar to launch the new application on
the STM32 board.

Then run the example STM32 MARTe application on the Pi. Execute the following command:

```
$ cd Startup
$ ./Main.sh -l RealTimeLoader -f ../Configurations/EduKit_STM32_InvertedGAM_FileLogging.cfg -s State1
...
```
After the preamble, no terminal output will be produced by the application. You can watch the data
accumulating into the log file by running e.g.

```
$ tail -f control_data_logs.csv
```

Where the name of the log file is defined in the MARTe2 configuration file (the argument to the `-f` flag). 
