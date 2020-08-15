# -*- coding: utf-8 -*-
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
import picamera

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
import Calibration
import gps_navigate
import ParaAvoidance
import ParaDetection
import Capture
import Land
import Release
import goaldetection

phaseChk = 1 #value of phase check
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
goal_value = [0.0, 0.0, 0.0, 0.0]

# --- For Judgement --- #
luxcount = 0
presscount = 0
Pcount = 0
GAcount = 0
luxjudge = 0
pressjudge = 0
gpsjudge = 0
anylux = 200
anypress = 0.3
anyalt = 30

# --- For Photo Path --- #
photo_path = '/home/pi/photo/photo'

# --- For Log path --- #
phaseLog = '/home/pi/log/phaseLog.txt'
releaseLog = '/home/pi/log/releaseLog.txt'
landingLog = '/home/pi/log/landingLog.txt'
meltingLog = '/home/pi/log/meltingLog.txt'
ParaDetectionLog = '/home/pi/log/ParaDetectionLog.txt'
ParaAvoidanceLog = '/home/pi/log/ParaAvoidanceLog.txt'
CalibrationLog = '/home/pi/log/CalibrationLog.txt'
Run_GPSLog = '/home/pi/log/Run_GPSLog.txt'
goalDetectionLog = '/home/pi/log/goalDetectionLog.txt'
errorLog = '/home/pi/log/errorLog.txt'
	
#--- difine goal latitude and longitude ---#
lon2 = 139.910338 #unnga:139.5430 yakugaku:139.912130
lat2 = 35.917945 #unga:35.553 yakugaku:35.920528

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
			phaseChk += 7
			print('phaseChk = '+str(phaseChk))

		# --- Run Phase --- #
		if phaseChk == 8:
			IM920.Send('P8S')
			Other.saveLog(phaseLog, '8', 'Run Phase Started', time.time() - t_start)
			t_Run_start = time.time()
			print('Run Phase Started {}'.format(time.time() - t_start))

			direction = Calibration.calculate_direction(lon2,lat2)
			goal_distance = direction["distance"]
			goal_distance_before = goal_distance
			
			#------------- Calibration -------------#
			print('Calibration Start')
			#--- calculate offset ---#
			magdata = Calibration.magdata_matrix()
			magdata_offset = Calibration.calculate_offset(magdata)
			magx_off = magdata_offset[3]
			magy_off = magdata_offset[4]
			magz_off = magdata_offset[5]
			Other.saveLog(CalibrationLog, 'Calibration', time.time() - t_start, magdata, magx_off, magy_off, magz_off)
			time.sleep(1)
			# ------------- GPS navigate ------------- #
			while goal_distance >= 5:
				if goal_distance_before <= goal_distance:
					run = pwm_control.Run()
					run.stop()
					time.sleep(1)
					#--- calculate θ ---#
					data = Calibration.get_data()
					magx = data[0]
					magy = data[1]
					#--- 0 <= θ <= 360 ---#
					θ = Calibration.calculate_angle_2D(magx,magy,magx_off,magy_off)
					#------------- rotate contorol -------------#
					judge = Calibration.rotate_control(θ,lon2,lat2,t_start)
				else:
					#--- initialize goal distance before ---#
					goal_distance_before = goal_distance
				#------------- run straight -------------#
				print('Go straight')
				run = pwm_control.Run()
				run.straight_n()
				time.sleep(0.5)
				#--- calculate  goal direction ---#
				direction = Calibration.calculate_direction(lon2,lat2)
				#Other.saveLog(Run_GPSLog, 'Run_GPS', time.time() - t_start, goal_distance, land_point_distance, GPS.readGPS())
				goal_distance = direction["distance"]
				print('goal distance ='+str(goal_distance))
				#--- 0 <= azimuth <= 360 ---#
				azimuth = direction["azimuth1"]
				#--- calculate θ ---#
				data = Calibration.get_data()
				magx = data[0]
				magy = data[1]
				#--- 0 <= θ <= 360 ---#
				θ = Calibration.calculate_angle_2D(magx,magy,magx_off,magy_off)
				#--- if rover go wide left, turn right ---#
				#--- 15 <= azimuth <= 360 ---#
				if azimuth - 15 > θ and azimuth - 15 >= 0:
					print('turn right to adjustment')
					run = pwm_control.Run()
					run.turn_right_l()
					time.sleep(0.1)
				#--- 0 <= azimuth < 15 ---#
				elif azimuth - 15 < 0:
					azimuth += 360
					if azimuth - 15 > θ:
						print('turn right to adjustment')
						run = pwm_control.Run()
						run.turn_right_l()
						time.sleep(0.1)							

				#--- if rover go wide right, turn left ---#
				#--- 0 <= azimuth <= 345 ---#
				if θ > azimuth + 15 and  azimuth + 15 > 360:
					print('turn left to adjustment')
					run = pwm_control.Run()
					run.turn_left_l()
					time.sleep(0.1)
				#--- 345 < azimuth <= 360 ---#
				elif azimuth + 15 > 360:
					azimuth -= 360
					if θ > azimuth + 15:
						print('turn left to adjustment')
						run = pwm_control.Run()
						run.turn_left_l()
						time.sleep(0.1)
			
			phaseChk += 1
			run = pwm_control.Run()
			run.back()
			time.sleep(0.2)
			run.stop()
			time.sleep(1)
			IM920.Send('P8F')
			print('phaseChk = '+str(phaseChk))

		# --- Goal Detection Phase --- #
		if phaseChk == 9:
			IM920.Send('P9S')
			Other.saveLog(phaseLog, '9', 'Goal Detection Phase Started', time.time() - t_start)
			t_GoalDetection_start = time.time()
			print('Goal Detection Phase Started {}'.format(time.time() - t_start))
			try:
				# --- calculate the distance to the goal --- #
				direction = Calibration.calculate_direction(lon2,lat2)
				goal_distance = direction["distance"]
				print('goal_distance = ', goal_distance)
				# --- if the distance to the goal is within 5 meters --- #
				while goal_distance <= 5:
					goal_value[0] = 1
					# --- until the goal decision --- #
					while goal_value[0] != 0:
						goal_value = goaldetection.GoalDetection("/home/pi/photo/photo",200 ,20, 80, 7000)
						print("goalflug", goal_value[0], "goalarea",goal_value[1], "goalGAP", goal_value[2], "name", goal_value[3])
						Other.saveLog(goalDetectionLog, time.time() - t_start, goal_value, GPS.readGPS())
						if goal_value[0] != -1:
							# --- if the pixcel error is -30 or less, rotate left --- #
							if goal_value[2] <= -30.0:
								print('Turn left')
								run = pwm_control.Run()
								run.turn_left_l()
								time.sleep(0.5)
							# --- if the pixcel error is 30 or more, rotate right --- #
							elif 30 <= goal_value[2]:
								print('Turn right')
								run = pwm_control.Run()
								run.turn_right_l()
								time.sleep(0.5)
							# --- if the pixcel error is greater than -30 and less than 30, go straight --- #
							else:
								print('Go straight')
								run = pwm_control.Run()
								run.straight_h()
								time.sleep(1.0)
						else:
							print('No goal in this flame !')
							run = pwm_control.Run()
							run.turn_right()
							time.sleep(1)
			
					run = pwm_control.Run()
					run.stop()
					print('Rover has reached the Goal !')
					break

			except:
				run = pwm_control.Run()
				run.stop()
				print('\r\t except: Run stop')

			finally:
				run = pwm_control.Run()
				run.stop()
				GPS.closeGPS()
				print('Competition is Finish !')
			IM920.Send('P9F')

	except KeyboardInterrupt:
		close()
		print('Keyboard Interrupt')
		IM920.Send('KI')

	except:
		close()

	finally:
		print('end')
#8/15/12:00#
