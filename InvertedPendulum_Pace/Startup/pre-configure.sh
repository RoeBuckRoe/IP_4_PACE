#!/bin/bash

TARGET=x86-linux
if [ -z ${MARTe2_DIR+x} ]; then echo "Please set the MARTe2_DIR environment variable"; exit; fi
if [ -z ${MARTe2_Components_DIR+x} ]; then echo "Please set the MARTe2_Components_DIR environment variable"; exit; fi

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:.
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_DIR/Build/x86-linux/WaterTankGAM/Makefiles/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:Core/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_DIR/Build/x86-linux/Core/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/EPICSCA/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/LinuxTimer/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/LoggerDataSource/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/DAN/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/NI6259/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/NI6368/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/SDN/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/UDP/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/MDSWriter/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/RealTimeThreadSynchronisation/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/DataSources/FileDataSource/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/IOGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/PIDGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/BaseLib2GAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/ConversionGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/ConstantGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/FilterGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/HistogramGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/StatisticsGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/WaveformGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/Interfaces/BaseLib2Wrapper/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/Interfaces/SysLogger/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/Interfaces/EPICS/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/x86-linux/Components/GAMs/MessageGAM


LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/$TARGET/Components/DataSources/LinkDataSource/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/$TARGET/Components/DataSources/UDP/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/$TARGET/Components/DataSources/RealTimeThreadAsyncBridge/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/$TARGET/Components/GAMs/SSMGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$MARTe2_Components_DIR/Build/$TARGET/Components/GAMs/TriggerOnChangeGAM/

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/superuser/MARTe_ws/InvertedPendulum-MARTe2/Build/x86-linux/Components/GAMs/HomingGAM/
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/superuser/MARTe_ws/InvertedPendulum-MARTe2/Build/x86-linux/Components/GAMs/IPStatusGAM/

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/DataSources/EduKitSerial
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/DataSources/RaspPiSerial
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/DataSources/GPS
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/DataSources/STM32
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/GAMs/UTCTimestampingGAM
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/GAMs/DownsamplingGAM
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/GAMs/InversionGAM
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/GAMs/InvertedPendulumGAM
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/Lib/DataSourceSignalChecker
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/Lib/GAMSignalChecker
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../Build/$TARGET/Components/Lib/SerialBuffer


echo $LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH
