import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi.git/kimuralab/SensorModuleTest/Wireless')
sys.path.append('/home/pi/git/kimuralab/Detection/Run_phase')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Calibration')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Stuck')
sys.path.append('/home/pi/git/kimuralab/Other')

import time
import serial
import pigpio
import traceback

import BME280
import GPS
import TSL2561
import Melting
import IM920
import BMX055
import Other
import pwm_control
import Stuck
import Calibration
import gps_navigate

phaseChk = 0 #value of phase check

# --- For time setting --- #
t_start = 0
t_sleep_start = 0
t_release_start = 0
t_landing_start = 0
t_melting_start = 0

# --- For Sensor Data --- #
bme280data = [0.0, 0.0, 0.0, 0.0, 0.0]
gpsdata = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# --- For Judgement --- #
luxcount = 0
presscount = 0
Pcount = 0
GAcount = 0
luxjudge = 0
pressjudge = 0
gpsjudge = 0

# --- For Log path --- #
phaseLog = '/home/pi/log/phaseLog.txt'
releaselog = 'home/pi/log/releaseLog.txt'
landingLog = 'home/pi/log/landingLog.txt'
meltingLog = 'home/pi/log/meltingLog.txt'
errorLog = 'home/pi/log/errorLog.txt'

pi = pigpio.pi()

def setup():
	global phaseChk
	pi.write(22,1)
	pi.write(24,1)
	BME280.bme2280_setup()
	BME280_calib_param()
	TSL2561.tsl2561_setup()
	GPS.openGPS()

	with open(phaseLog, 'a') as f:
		pass

def close():
	pi.write(22,0)
	pi.write(24,0)
	GPS.close()

if __name__ == '__main__'
	try:
		print('Program Start {0}'.format(time.time()))
		t_start = time.time()

		# --- Setup Phase --- #
		setup()
		print('Start {0}'.format(phaseChk))
		if phaseChk <= 1:
			IM920.Send('P1S')
			Other.SaveLog(phaseLog, '1', 'Program Started', time.time() - t_atart)
			IM920.Send('P1F')

		# --- Sleep Phase --- #
		if phaseChk <= 2:
			Im920.Send('P2S')
			t_sleep_start = time.time()
			Other.SaveLog(phaseLog, '2', 'Sleep Phase Started', time.time - t_start)
			Im920.Send('P2F')
		
		# --- Release Phase --- #
		if phaseChk <= 3:
			IM920.Send('P3S')
			Other.SaveLog(phaseLog, '3', 'Release Phase Started', time.time - t_start)
			t_relase_start = tim.time()
			print('Release Phase Started {}'.format(time.time - t_start))

			# --- Release Judgement, 'while' is until timeout --- #
			while time.time() - t_release_start <= t_release:
				luxjudge,luxcount = Release.luxdetect()
				pressjudge,presscount = Release.pressdetect()
				if luxjudge == 1 or pressjudge == 1:
					Other.saveLog(releaseLog, 'Release Judge', luxjudge, pressjudge)
					print('Rover has released')
					break
				else:
					print('Rover is still in the air')
					IM920.Send('P3D')
			else:
				Other.savsLog(releaseLog, 'Release Judge by Timeout')
				print('Release Timeout')
			IM920.Send('P3F')
		# --- Landing Phase --- #
		if phaseChk <= 4:
			IM920.Send('P4S')
			Other.saveLog(phaseLog, '4', 'Landing Phase Started', time.time - t_start)
			t_landing_start = time.time()
			print('Landing Phase Started' {}.format(time.time - t_start))

			# --- Landing Judgement, 'while' is until timeout --- #
			while time.time() - t_landing_start <= t_landing:
				Pressjudge,Presscount = Land.pressdetect()
				GPSjudge,GAcount = Land.gpsdetect()

