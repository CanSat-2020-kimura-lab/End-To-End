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
sys.path.append('/home/pi/Detection')　

import time
import serial
import pigpio
import BME280
import TSL2561
import traceback
#import runtest

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

###ここからコピー
luxstr = ["lux1", "lux2"]			#variable to show lux returned variables
bme280str = ["temp", "pres", "hum", "alt"]	#variable to show bme280 returned variables
bmx055str = ["accx", "accy", "accz", "gyrx", "gyry", "gyrz", "dirx", "diry", "dirz"]	#variable to show bmx055 returned variables
gpsstr = ["utctime", "lat", "lon", "sHeight", "gHeight"]				#variable to show GPS returned variables

gpsData=[0.0,0.0,0.0,0.0,0.0]
bme280Data=[0.0,0.0,0.0,0.0]
bmx055data=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

t_setup = 60	#variable to set waiting time after setup
t = 1			#Unknown Variable
x = 300			#time for release(loopx)
y = 180			#time for land(loopy)

t_start  = 0.0	#time when program started

#lcount=0
acount=0
Pcount=0
GAcount=0
deltHmax=5
luxjudge = 0
pressjudge=0
pi=pigpio.pi()

paraExsist = 0 	#variable used for Para Detection

phaseLog = "/home/pi/log/phaseLog.txt"
waitingLog = "/home/pi/log/waitingLog.txt"
releaseLog = "/home/pi/log/releaseLog.txt"
landingLog = "/home/pi/log/landingLog.txt"
meltingLog = "/home/pi/log/meltingLog.txt"
paraAvoidanceLog = "/home/pi/log/paraAvoidanceLog.txt"

def setup():
	global phaseChk

	pi.set_mode(22,pigpio.OUTPUT)
	pi.write(22,1)	#IM920	Turn On
	pi.write(17,0)	#meltPin off
	time.sleep(1)
	BME280.bme280_setup()
	BME280.bme280_calib_param()
	BMX055.bmx055_setup()
	GPS.openGPS()

	with open(phaseLog, 'a') as f:
		pass

	phaseChk = int(Other.phaseCheck(phaseLog))
	print(phaseChk)

def close():
	GPS.closeGPS()
	pi.write(22, 0)
	pi.write(17,0)
	run = runtest.Run()　#変更しました
	run.stop()

if __name__ == "__main__":
	try:
		t_start = time.time() 
		# ------------------- Setup Phase --------------------- #
		print("Program Start  {0}".format(time.time())) #{}の中の0はなくてもいい
		setup()
		print(phaseChk)
		IM920.Send("Start")

		# ------------------- Waiting Phase --------------------- #
		Other.saveLog(phaseLog, "2", "Waiting Phase Started", time.time() - t_start)
		if(phaseChk <= 2):
			t_wait_start = time.time()
			while(time.time() - t_wait_start <= t_setup):
				Other.saveLog(waitingLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), TSL2561.readLux(), BMX055.bmx055_read())
				print("Waiting")
				IM920.Send("Sleep")
				time.sleep(1)
			IM920.Send("Waiting Finished")
			pi.write(22, 0)		#IM920 Turn Off

		# ------------------- Release Phase ------------------- #
		Other.saveLog(phaseLog, "3", "Release Phase Started", time.time() - t_start)
		if(phaseChk <= 3):
			tx1 = time.time()
			tx2 = tx1
			print("Releasing Judgement Program Start  {0}".format(time.time() - t_start))
			#loopx
			bme280Data=BME280.bme280_read()
			while (tx2-tx1<=x):
				IM920.Send("loopX")
				#luxjudge = Release.luxjudge()
				luxjudge = Release.luxdetect()
				if luxjudge == 1:
					break
				#pressjudge = Release.pressjudge()
				pressjudge = Release.pressdetect()
				if pressjudge == 1:
					break
				else:
					pass
		   			#print("now in rocket ,taking photo")
				Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(0.5)

				Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), TSL2561.readLux(), BME280.bme280_read(), BMX055.bmx055_read())
				#Other.saveLog(releaseLog, time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())				
				time.sleep(0.5)

				tx2=time.time()
			else:
				print("RELEASE TIMEOUT")
			print("THE ROVER HAS RELEASED")
			pi.write(22,1)
			time.sleep(2)
			IM920.Send("RELEASE")

		# ------------------- Landing Phase ------------------- #
		Other.saveLog(phaseLog, "4", "Landing Phase Started", time.time() - t_start)
		if(phaseChk <= 4):
			print("Landing Judgement Program Start  {0}".format(time.time() - t_start))
			ty1=time.time()
			ty2=ty1
			#loopy
			gpsdata = GPS.readGPS()
			bme280data=BME280.bme280_read()
			while(ty2-ty1<=y):
				IM920.Send("loopY")
				#pressjudge=Land.pressjudge()
				pressjudge=Land.pressdetect()
			#	gpsjudge=Land.gpsjudge()
				if pressjudge == 1: #and gpsjudge ==1:
					break
				elif pressjudge == 0 :#and gpsjudge==0:
			#	    print("Descend now taking photo")
			#	elif pressjudge==1 or gpsjudge==1:
			#	    print("landjudgementnow")
				gpssata = GPS.readGPS()
				bme280data=BME280.bme280_read()
			#	bmx055data=BMX055.bmx055_read()
				Other.saveLog(landingLog ,time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				Other.saveLog(landingLog ,time.time() - t_start, GPS.readGPS(), BME280.bme280_read(), BMX055.bmx055_read())
				time.sleep(1)
				ty2=time.time()
			else:
				print("LAND TIMEOUT")
			print("THE ROVER HAS LANDED")
			pi.write(22,1)
			IM920.Send("LAND")
			
		# ------------------- Melting Phase ------------------- #
		IM920.Send("Melt")
		Other.saveLog(phaseLog,"5", "Melting Phase Started", time.time() - t_start)
		if(phaseChk <= 5):
			print("Melting Phase Started")
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Start")
			Melting.Melting()
			Other.saveLog(meltingLog, time.time() - t_start, GPS.readGPS(), "Melting Finished")

		# ------------------- ParaAvoidance Phase ------------------- #	
		
		IM920.Send("Progam Finished")
		close()
		
	except KeyboardInterrupt:
		close()
		print("Keyboard Interrupt")
	except Exception as e:
		IM920.Send("error")
		close()
		Other.saveLog("/home/pi/log/errorLog.txt", time.time() - t_start, "Error")
		print(e.message)
		
#--- 引継ぎ ---#
#--- コピー ---#
# 2019の37~116をコピー
# 2019の186~193をコピー
#---変更点---#
#13行目を追加
#94,95行目を今年のモーターの仕様に変更
