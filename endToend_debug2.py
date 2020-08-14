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
				Presscount,Pressjudge = Land.Pressdetect(anypress)
				GAcount,GPSjudge = Land.gpsdetect(anyalt)
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

		# --- Melting Phase --- #
		if phaseChk == 5:
			IM920.Send('P5S')
			Other.saveLog(phaseLog, '5', 'Melting Phase Started', time.time() - t_start)
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Start")
			t_melting_start = time.time()
			print('Melting Phase Started {}'.format(time.time() - t_start))
			#Melting.Melting()
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Finished")
			IM920.Send('P5F')
			phaseChk += 1
			print('phaseChk = '+str(phaseChk))

		# --- Mission Phase --- #
		camera = picamera.PiCamera()
		camera.resolution = (320,240)
		camera.capture('/home/pi/photo/mission1.jpg')
		img = cv2.imread('/home/pi/photo/mission1.jpg')
		dst = cv2.resize(img, (img.shape[1]*2, img.shape[0]*2), interpolation=cv2.INTER_NEAREST)
		cv2.imwrite('/home/pi/photo/nearest1.jpg',dst)
		camera.resolution = (640,480)
		camera.capture('/home/pi/photo/mission2.jpg')

		# --- Parachute Detection Phase --- #
		if phaseChk == 6:
			IM920.Send('P6S')
			Other.saveLog(phaseLog, '6', 'Parachute Detection Phase Started', time.time() - t_start)
			t_ParaDetection_start = time.time()
			print('Parachute Detection Phase Started {}'.format(time.time() - t_start))

			#--- note GPS data at land point ---#
			longitude_land,latitude_land = ParaAvoidance.land_point_save()
			#--- initialize distance ---#
			global land_point_distance
			land_point_distance = 0
			t2 = time.time()
			t1 = t2
			#--- Parachute judge ---#
			#--- timeout is 60s ---#
			while t2 - t1 < 60:
				Luxflug = ParaDetection.ParaJudge(100)
				Other.saveLog(ParaDetectionLog, 'ParaDetection', time.time() - t_start, TSL2561.readLux(), Luxflug)				
				if Luxflug[0] == 1:
					break
				t1 =time.time()
				time.sleep(1)
				IM920.Send('P6F')
			phaseChk += 1
			print('phaseChk = '+str(phaseChk))

		while phaseChk == 7:
			# --- Parachute Avoidance Phase --- #
			if phaseChk == 7:
				IM920.Send('P7S')
				Other.saveLog(phaseLog, '7', 'Parachute Avoidance Phase Started', time.time() - t_start)
				t_ParaAvoidance_start = time.time()
				print('Parachute Avoidance Phase Started {}'.format(time.time() - t_start))

				while land_point_distance <= 1:
					try:
						#--- first parachute detection ---#
						flug, area, photoname = ParaDetection.ParaDetection("/home/pi/photo/photo",320,240,200,10,120)
						Other.saveLog(ParaAvoidanceLog, 'ParaAvoidance', time.time() - t_start, flug, area, photoname, GPS.readGPS())
						ParaAvoidance.Parachute_Avoidance(flug,t_start)
						land_point_distance = ParaAvoidance.Parachute_area_judge(longitude_land,latitude_land)

					except KeyboardInterrupt:
						print("Emergency!")
						run = pwm_control.Run()
						run.stop()

					except:
						run = pwm_control.Run()
						run.stop()
						print(traceback.format_exc())
				print('finish')
				IM920.Send('P7F')
				phaseChk += 1
				print('phaseChk = '+str(phaseChk))

			# --- Run Phase --- #
			if phaseChk == 8:
				IM920.Send('P8S')
				Other.saveLog(phaseLog, '8', 'Run Phase Started', time.time() - t_start)
				t_Run_start = time.time()
				print('Run Phase Started {}'.format(time.time() - t_start))

				direction = Calibration.calculate_direction(lon2,lat2)
				goal_distance = direction["distance"]
				stop_count = False
				while True:
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
					#--- calculate θ ---#
					data = Calibration.get_data()
					magx = data[0]
					magy = data[1]
					#--- 0 <= θ <= 360 ---#
					θ = Calibration.calculate_angle_2D(magx,magy,magx_off,magy_off)
					#------------- rotate contorol -------------#
					judge = Calibration.rotate_control(θ,lon2,lat2,t_start)
					#--- rotate control timeout ---#
					if judge == False:
						try:
							run = pwm_control.Run()
							run.straight_h()
							time.sleep(1)
						finally:
							run = pwm_control.Run()
							run.stop()
							time.sleep(1)					
					else:
						#--- judge = True (rotate control successed) ---#
						run = pwm_control.Run()
						run.stop()
						time.sleep(1)
						break
				# ------------- GPS navigate ------------- #
				while goal_distance >= 5 and land_point_distance >= 1:
					if stop_count == True:
						break
					while True:
						#--- calculate θ ---#
						data = Calibration.get_data()
						magx = data[0]
						magy = data[1]
						#--- 0 <= θ <= 360 ---#
						θ = Calibration.calculate_angle_2D(magx,magy,magx_off,magy_off)
						#------------- rotate contorol -------------#
						judge = Calibration.rotate_control(θ,lon2,lat2,t_start)
						#--- rotate control timeout ---#
						if judge == False:
							run = pwm_control.Run()
							run.stop()
							time.sleep(1)					
						else:
							#--- judge = True (rotate control successed) ---#
							run = pwm_control.Run()
							run.stop()
							time.sleep(1)
							break
					location = Stuck.stuck_detection1()
					longitude_past = location[0]
					latitude_past = location[1]
					#--- initialize count ---#
					loop_count = 0
					stuck_count = 0

					#------------- run straight -------------#
					while loop_count <= 5:
						print('Go straight')
						run = pwm_control.Run()
						run.straight_h()
						time.sleep(1)
						#--- calculate  goal direction ---#
						direction = Calibration.calculate_direction(lon2,lat2)
						land_point_distance = ParaAvoidance.Parachute_area_judge(longitude_land,latitude_land)
						Other.saveLog(Run_GPSLog, 'Run_GPS', time.time() - t_start, goal_distance, land_point_distance, GPS.readGPS())
						if land_point_distance <= 5:
							phaseChk -= 1							
							break
						goal_distance = direction["distance"]
						print('goal distance ='+str(goal_distance))
						if goal_distance <= 5:
							phaseChk += 1
							break
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
						run = pwm_control.Run()
						run.straight_h()
						time.sleep(0.5)
						#--- stuck detection ---#
						moved_distance = Stuck.stuck_detection2(longitude_past,latitude_past)
						if moved_distance >= 5:
							IM920.Send("Rover is moving now")
							print('Rover is moving now')
							stuck_count = 0
							location = Stuck.stuck_detection1()
							longitude_past = location[0]
							latitude_past = location[1]     
						else:
							#--- stuck escape ---#
							IM920.Send("Rover stucks now")
							print('Stuck!')
							Stuck.stuck_escape()
							loop_count -= 1
							stuck_count += 1
							if stuck_count >= 3:
								print("Rover can't move any more")
								stop_count = True
								phaseChk += 2
								break
						#--- calculate  goal direction ---#
						direction = Calibration.calculate_direction(lon2,lat2)
						land_point_distance = ParaAvoidance.Parachute_area_judge(longitude_land,latitude_land)
						Other.saveLog(Run_GPSLog, 'Run_GPS', time.time() - t_start, goal_distance, land_point_distance, GPS.readGPS())
						if land_point_distance <= 5:
							phaseChk -= 1							
							break
						goal_distance = direction["distance"]
						print('goal distance ='+str(goal_distance))
						if goal_distance <= 5:
							phaseChk += 1
							break
						loop_count += 1
						print('loop count is '+str(loop_count))
				
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
