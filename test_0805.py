import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BMX055')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Wireless')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Camera')
sys.path.append('/home/pi/git/kimuralab/Detection/Landing_phase')
sys.path.append('/home/pi/git/kimuralab/Detection/Release_phase')
sys.path.append('/home/pi/git/kimuralab/Detection/Run_phase')
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/Detection/GoalDetection')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Calibration')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Stuck')
sys.path.append('/home/pi/git/kimuralab/Other')

import time
import serial
import pigpio
import traceback
from threading import Thread
import math
import cv2

import BME280
import BMX055
import GPS
import TSL2561
import Melting
import IM920
import BMX055
import Other
import pwm_control
import Stuck
import Calibration2
import gps_navigate
import ParaAvoidance
import ParaDetection
import Capture
import Land
import Release
import goaldetection

phaseChk = 0 #value of phase check
RX = 18

# --- For time setting --- #
t_start = 0
t_sleep_start = 0
t_release_start = 0
t_landing_start = 0
t_melting_start = 0
t_GoalDetection_start = 0

# --- For timeout --- #
t_release = 30
t_landing = 60

# --- For Sensor Data --- #
bme280data = [0.0, 0.0, 0.0, 0.0, 0.0]
GPS_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# --- For Judgement --- #
luxcount = 0
presscount = 0
Pcount = 0
GAcount = 0
luxjudge = 0
pressjudge = 0
gpsjudge = 0
anylux = 0
anypress = 0.3
anyalt = 0.1

# --- For Photo Path --- #
photo_path = '/home/pi/photo/photo'

# --- For Log path --- #
phaseLog = '/home/pi/log/phaseLog.txt'
releaseLog = '/home/pi/log/releaseLog.txt'
landingLog = '/home/pi/log/landingLog.txt'
meltingLog = '/home/pi/log/meltingLog.txt'
goalDetectionLog = '/home/pi/log/goalDetectionLog.txt'
errorLog = '/home/pi/log/errorLog.txt'
	
#--- difine goal latitude and longitude ---#
lon2 = 139.5430
lat2 = 35.553

pi = pigpio.pi()

def setup():
	global phaseChk
	pi.write(22,1)
	pi.write(24,0)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	TSL2561.tsl2561_setup()
	GPS.openGPS()

	with open(phaseLog, 'a') as f:
		pass

def close():
	pi.write(22,0)
	pi.write(24,0)
	GPS.closeGPS()

def timer(t):
	global cond
	time.sleep(t)
	cond = False

if __name__ == '__main__':
	try:
		print('Program Start {0}'.format(time.time()))
		t_start = time.time()

		# --- Setup Phase --- #
		setup()
		print('Start {0}'.format(phaseChk))
		if phaseChk == 1:
			IM920.Send('P1S')
			Other.saveLog(phaseLog, '1', 'Program Started', time.time() - t_start)
			IM920.Send('P1F')
			phaseChk += 1
			print('phaseChk = '+str(phaseChk))

		# --- Sleep Phase --- #
		if phaseChk == 2:
			IM920.Send('P2S')
			t_sleep_start = time.time()
			Other.saveLog(phaseLog, '2', 'Sleep Phase Started', time.time() - t_start)
			IM920.Send('P2F')
			phaseChk += 1
			print('phaseChk = '+str(phaseChk))

		
		# --- Release Phase --- #
		if phaseChk == 3:
			IM920.Send('P3S')
			Other.saveLog(phaseLog, '3', 'Release Phase Started', time.time() - t_start)
			t_release_start = time.time()
			print('Release Phase Started {}'.format(time.time() - t_start))

			# --- Release Judgement, 'while' is until timeout --- #
			while time.time() - t_release_start <= t_release:
				luxjudge,luxcount = Release.luxdetect(anylux)
				pressjudge,presscount = Release.pressdetect(anypress)
				if luxjudge == 1 or pressjudge == 1:
					Other.saveLog(releaseLog, 'Release Judge', time.time() - t_start, TSL2561.readLux(), BME280.bme280_read(), luxjudge, pressjudge)
					print('Rover has released')
					break
				else:
					print('Rover is still in the air')
					Other.saveLog(releaseLog, 'Release Judge', time.time() - t_start, TSL2561.readLux(), Release.luxdetect(anylux), BME280.bme280_read(), Release.pressdetect(anypress))
					IM920.Send('P3D')
			if t_release < time.time() - t_release_start:
				Other.saveLog(releaseLog, 'Release Judge by Timeout', time.time() - t_start)
				print('Release Timeout')
			IM920.Send('P3F')
			phaseChk += 1
			print('phaseChk = '+str(phaseChk))
			
		# --- Landing Phase --- #
		if phaseChk == 4:
			IM920.Send('P4S')
			Other.saveLog(phaseLog, '4', 'Landing Phase Started', time.time() - t_start)
			t_landing_start = time.time()
			print('Landing Phase Started {}'.format(time.time() - t_start))

			# --- Landing Judgement, 'while' is until timeout --- #
			while time.time() - t_landing_start <= t_landing:
				Pressjudge,Presscount = Land.Pressdetect(anypress)
				GPSjudge,GAcount = Land.gpsdetect(anyalt)
				if Pressjudge == 1 and GPSjudge == 1:
					Other.saveLog(landingLog, 'Landing Judge', time.time() - t_start, BME280.bme280_read(), Pressjudge, GPS.readGPS(), GPSjudge)
					print('Rover has Landed')
					break
				else:
					print('Rover is still in the air')
					Other.saveLog(landingLog, 'Rover is still in the air', time.time() - t_start, BME280.bme280_read(), Land.Pressdetect(anypress), GPS.readGPS(), Land.gpsdetect(anyalt))
					IM920.Send('P4D')
			if t_landing < time.time() - t_release_start:
				Other.saveLog(landingLog, 'Landing Judge by Timeout', time.time() - t_start)
				print('Landing Timeout')
			IM920.Send('P4F')
			phaseChk += 1
			print('phaseChk = '+str(phaseChk))

	except:
		print('else: close')
		close()
