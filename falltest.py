import sys
sys.path.append('/home/pi/SensorModuleTest/BME280')
sys.path.append('/home/pi/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/2019/SensorModuleTest/GPS')
sys.path.append('/home/pi/2019/SensorModuleTest/Camera')
sys.path.append('/home/pi/SensorModuleTest/IM920')
sys.path.append('/home/pi/2020/Detection/Release')
sys.path.append('/home/pi/2020/Detection/Land')
sys.path.append('/home/pi/2020/Detection/ParachuteDetection')
sys.path.append('/home/pi/SensorModuleTest/Melting')
sys.path.append('/home/pi/SensorModuleTest/Motor')
sys.path.append('/home/pi/Detection/Others')
import time
import serial
import pigpio
import BME280
import TSL2561
import traceback

anylux = 0
anypress = 0.3
Anypress = 0
anyalt = 0
luxdata = []
bme280data = []
gpsdata = []
luxcount = 0
presscount = 0

luxreleasejudge = 0
pressreleasejudge = 0

PhaseChk = 0
