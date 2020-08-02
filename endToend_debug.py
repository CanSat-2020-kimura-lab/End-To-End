import sys
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/BME280')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/GPS')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/TSL2561')
sys.path.append('/home/pi/git/kimuralab/SensorModuleTest/Melting')
sys.path.append('/home/pi.git/kimuralab/SensorModuleTest/Wireless')
sys.path.append('/home/pi/git/kimuralab/Detection/Run_phase')
sys.path.append('/home/pi/git/kimuralab/Detection/ParachuteDetection')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Calibration')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/ParaAvoidance')
sys.path.append('/home/pi/git/kimuralab/IntegratedProgram/Stuck')
sys.path.append('/home/pi/git/kimuralab/Other')

import time
import serial
import pigpio
import traceback
from threading import Thread

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
import ParaAvoidance
import ParaDetection
import Capture
import Land

phaseChk = 0 #value of phase check

# --- For time setting --- #
t_start = 0
t_sleep_start = 0
t_release_start = 0
t_landing_start = 0
t_melting_start = 0

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

# --- For Log path --- #
phaseLog = '/home/pi/log/phaseLog.txt'
releaselog = 'home/pi/log/releaseLog.txt'
landingLog = 'home/pi/log/landingLog.txt'
meltingLog = 'home/pi/log/meltingLog.txt'
errorLog = 'home/pi/log/errorLog.txt'
	
#--- difine goal latitude and longitude ---#
lon2 = 139.5430
lat2 = 35.553

pi = pigpio.pi()

def setup():
	global phaseChk
	pi.write(22,1)
	pi.write(24,1)
	BME280.bme2280_setup()
	BME280.calib_param()
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
		if phaseChk <= 1:
			IM920.Send('P1S')
			Other.saveLog(phaseLog, '1', 'Program Started', time.time() - t_atart)
			IM920.Send('P1F')

		# --- Sleep Phase --- #
		if phaseChk <= 2:
			IM920.Send('P2S')
			t_sleep_start = time.time()
			Other.saveLog(phaseLog, '2', 'Sleep Phase Started', time.time - t_start)
			IM920.Send('P2F')
		
		# --- Release Phase --- #
		if phaseChk <= 3:
			IM920.Send('P3S')
			Other.saveLog(phaseLog, '3', 'Release Phase Started', time.time - t_start)
			t_release_start = time.time()
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
				Other.saveLog(releaseLog, 'Release Judge by Timeout')
				print('Release Timeout')
			IM920.Send('P3F')
		# --- Landing Phase --- #
		if phaseChk <= 4:
			IM920.Send('P4S')
			Other.saveLog(phaseLog, '4', 'Landing Phase Started', time.time - t_start)
			t_landing_start = time.time()
			print('Landing Phase Started {}'.format(time.time - t_start))

			# --- Landing Judgement, 'while' is until timeout --- #
			while time.time() - t_landing_start <= t_landing:
				Pressjudge,Presscount = Land.Pressdetect()
				GPSjudge,GAcount = Land.gpsdetect()


		# --- Parachute Detection Phase --- #
		if phaseChk <= 5:
			IM920.Send('P5S')
			Other.saveLog(phaseLog, '5', 'Parachute Detection Phase Started', time.time - t_start)
			t_ParaDetection_start = time.time()
			print('Parachute Detection Phase Started {}'.format(time.time - t_start))

			#--- note GPS data at land point ---#
			longitude_land,latitude_land = ParaAvoidance.land_point_save()
			#--- initialize distance ---#
			global distance
			distance = 0
			t2 = time.time()
			t1 = t2
			#--- Parachute judge ---#
			#--- timeout is 60s ---#
			while t2 - t1 < 60:
				Luxflug = ParaDetection.ParaJudge(100)
				if Luxflug[0] == 1:
					break
				t1 =time.time()
				time.sleep(1)

		# --- Parachute Detection Phase --- #
		if phaseChk <= 6:
			IM920.Send('P6S')
			Other.saveLog(phaseLog, '6', 'Parachute Avoidance Phase Started', time.time - t_start)
			t_ParaAvoidance_start = time.time()
			print('Parachute Avoidance Phase Started {}'.format(time.time - t_start))

			while distance <= 5:
				try:
					#--- first parachute detection ---#
					flug, area, photoname = ParaDetection.ParaDetection("/home/pi/photo/photo",320,240,200,10,120)
					ParaAvoidance.Parachute_Avoidance(flug)
					distance = ParaAvoidance.Parachute_area_judge(longitude_land,latitude_land)

				except KeyboardInterrupt:
					print("Emergency!")
					run = pwm_control.Run()
					run.stop()

				except:
					run = pwm_control.Run()
					run.stop()
					print(traceback.format_exc())
			print('finish')

		# --- Run Phase --- #
		if phaseChk <= 7:
			IM920.Send('P7S')
			Other.saveLog(phaseLog, '7', 'Run Phase Started', time.time - t_start)
			t_Run_start = time.time()
			print('Run Phase Started {}'.format(time.time - t_start))

			direction = Calibration.calculate_direction(lon2,lat2)
			goal_distance = direction["distance"]
			# ------------- GPS navigate ------------- #
			while goal_distance >= 5:
				# ------------- Calibration ------------- #
				# --- calculate offset --- #
				magdata = Calibration.magdata_matrix()
				magdata_offset = Calibration.calculate_offset(magdata)
				magx_off = magdata_offset[3]
				magy_off = magdata_offset[4]
				magz_off = magdata_offset[5]
				time.sleep(1)
				#--- calculate θ ---#
				data = Calibration.get_data()
				magx = data[0]
				magy = data[1]
				#--- 0 <= θ <= 360 ---#
				θ = Calibration.calculate_angle_2D(magx,magy,magx_off,magy_off)
				#------------- rotate contorol -------------#
				Calibration.rotate_control(θ,lon2,lat2)
				location = Stuck.stuck_detection1()
				longitude_past = location[0]
				latitude_past = location[1]

			#------------- run straight -------------#
			for i in range(2):
				try:
					for i in range(2):
						run = pwm_control.Run()
						run.straight_h()
						time.sleep(1)
						#--- Send GPS data ---#
						try:
							while True:
								value = GPS.readGPS()
								latitude_new = value[1]
								longitude_new = value[2]
								print(value)
								print('longitude = '+str(longitude_new))
								print('latitude = '+str(latitude_new))
								time.sleep(1)
								if latitude_new != -1.0 and longitude_new != 0.0 :
									break
						except KeyboardInterrupt:
							GPS.closeGPS()
							print("\r\nKeyboard Intruppted, Serial Closed")

						except:
							GPS.closeGPS()
							print (traceback.format_exc())
						
						IM920.Send(GPS_data)
						#--- calculate goal direction ---#
						direction = Calibration.calculate_direction(lon2,lat2)
						goal_distance = direction["distance"]
						if goal_distance <= 5:
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
							run = pwm_control.Run()
							run.turn_right()
							time.sleep(0.5)
						#--- 0 <= azimuth < 15 ---#
						elif azimuth - 15 < 0:
							azimuth += 360
							if azimuth - 15 > θ:
								run = pwm_control.Run()
								run.turn_right()
								time.sleep(0.5)							

						#--- if rover go wide right, turn left ---#
						#--- 0 <= azimuth <= 345 ---#
						if θ > azimuth + 15 and  azimuth + 15 > 360:
							run = pwm_control.Run()
							run.turn_left()
							time.sleep(0.5)
						#--- 345 < azimuth <= 360 ---#
						elif azimuth + 15 > 360:
							azimuth -= 360
							if θ > azimuth + 15:
								run = pwm_control.Run()
								run.turn_left()
								time.sleep(0.5)
						#--- stuck detection ---#
						moved_distance = Stuck.stuck_detection2(longitude_past,latitude_past)
						if moved_distance >= 5:
							IM920.Send("rover moved!")
						else:
							#--- stuck escape ---#
							Stuck.stuck_escape()
						
						#--- Send GPS data ---#
						try:
							while True:
								value = GPS.readGPS()
								latitude_new = value[1]
								longitude_new = value[2]
								print(value)
								print('longitude = '+str(longitude_new))
								print('latitude = '+str(latitude_new))
								time.sleep(1)
								if latitude_new != -1.0 and longitude_new != 0.0 :
									break
						except KeyboardInterrupt:
							GPS.closeGPS()
							print("\r\nKeyboard Intruppted, Serial Closed")

						except:
							GPS.closeGPS()
							print (traceback.format_exc())
						
						IM920.Send(GPS_data)
						#--- calculate  goal direction ---#
						direction = Calibration.calculate_direction(lon2,lat2)
						goal_distance = direction["distance"]
						if goal_distance <= 5:
							break
					
				except KeyboardInterrupt:
					run = pwm_control.Run()
					run.stop()
				
				finally:
					run = pwm_control.Run()
					run.stop()
	except:
		pass
