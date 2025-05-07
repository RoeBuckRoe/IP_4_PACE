# IP_4_PACE

## Instalation and Setup

If you want to install MARTe2 on your own machine create a MARTe workspace folder and clone the following repository into the directory:

```
https://github.com/aneto0/MARTe2
```

Then build the tools from inside the directory by running:

```
$ cd MARTe2
$ make -f Makefile.x86-linux
$ export MARTe2_DIR=$(pwd)
$ LD_LIBRARY_PATH=MARTe2_DIR/Build/x86-linux/Core
```

Then, clone the standard library of components and install them using the instructions a below into the MARTe workspace folder:

https://github.com/aneto0/MARTe2-components 

Then, from inside that directory, run:

```
$ git clone https://github.com/aneto0/MARTe2-components.git
$ cd MARTe2-components
$ make -f Makefile.x86-linux
$ export MARTe2_Components_DIR=$(pwd)
```

> Before running the MARTe application be sure relevant paths are exported.

Once the environment is setup. The next step is to Install the Inverted Pendulum programs. 

Clone and build the files from the repo from inside the MARTe workspace by running the commands:

```
$ git clone https://github.com/RoeBuckRoe/IP_4_PACE.git
$ cd IP_4_PACE
$ make -f Makefile.x86-linux
```

Once the environment is fully setup, Movce on to the Run instructions.

## Run

Upload the corresponding STM32 code to the Edukit from the repo:

```
https://github.com/RokHari/InvertedPendulum-STM32
```

Connect the STM32 to a computer with the USB cable.

Set the correct device file (Port configuration of the MotorSTM32 data source in Configurations/Pendulum.cfg).

Navigate to Startup/ directory within the project and execute the following command:

```
sudo -E ./Main.sh -l RealTimeLoader -f ../Configurations/Pendulum.cfg -m StateMachine::START
```