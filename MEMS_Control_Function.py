import numpy as np
import time
import math
import matplotlib.pyplot as plt
import spidev
import RPi.GPIO as GPIO
import threading


#Open SPI bus and configuration
spi=spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000 #Set mode and speed

#Set GPIO mode
GPIO.setmode(GPIO.BCM)
SYNC_PIN=17     #Define SYNC pin
XFCLK_PIN=22	#Define XFCLK pin
YFCLK_PIN=27	#Define YFCLK pin
ENABLE_PIN=23	#Define Enable pin
GPIO.setup(SYNC_PIN,GPIO.OUT) #Setup SYNC pin
GPIO.setwarnings(False)


    
#Function to send data via SPI
def spi_transfer(data):
	spi.xfer2(data)

#Function to enable driver
def Enable_Driver():
	GPIO.setup(ENABLE_PIN,GPIO.OUT)
	GPIO.setup(ENABLE_PIN,GPIO.HIGH)
	
#Function to disable driver
def Disable_Driver():
	GPIO.setup(ENABLE_PIN,GPIO.LOW)
	
#Function to set DAC output
def dac_write(channel,data):
	spi.mode=1 #CPOL=1,CPHA=0\idle=low,negedgeloaded
	GPIO.output(SYNC_PIN,GPIO.LOW) #Select the DAC by setting SYNC pin low
	command=[(0x18)|channel,(data>>8)&0xFF,data&0xFF]
	#Send data via SPI
	spi_transfer(command)
	GPIO.output(SYNC_PIN,GPIO.HIGH) #Select the DAC by setting SYNC pin high
	
#Initialization sequence Fucntion
def initialize_dac():
	#FULL RESET command
	spi.mode=0
	GPIO.output(SYNC_PIN,GPIO.LOW) #Select the DAC by setting SYNC pin low
	spi_transfer([0x28,0x00,0x01])
	time.sleep(0.01)
	#ENABLE INTERNAL REFERENCE command
	spi_transfer([0x38,0x00,0x01])
	time.sleep(0.01)
	#ENABLE ALL DAC CHANNELS command
	spi_transfer([0x20,0x00,0x0F])
	time.sleep(0.01)
	#ENABLE SOFTWARE LDAC command
	spi_transfer([0x30,0x00,0x00])
	time.sleep(0.01)
	GPIO.output(SYNC_PIN,GPIO.HIGH) #Select the DAC by setting SYNC pin high
	print("DAC Initialization completed!")

#FCLK clock generation
class ClockSignalGenerator:
	def __init__(self,pin1,pin2,frequency):
		self.pin1=pin1
		self.pin2=pin2
		self.frequency=frequency
		GPIO.setup(self.pin1,GPIO.OUT)
		GPIO.setup(self.pin2,GPIO.OUT)
		self._running=False

	def generate_fclk(self):
		while self._running:
			GPIO.output(self.pin1,GPIO.HIGH)
			GPIO.output(self.pin2,GPIO.HIGH)
			time.sleep(1/(2*self.frequency))
			GPIO.output(self.pin1,GPIO.LOW)
			GPIO.output(self.pin2,GPIO.LOW)
			time.sleep(1/(2*self.frequency))
		
	def start(self):
		if not self._running:
			self._running=True
			self._generate_thread=threading.Thread(target=self.generate_fclk)
			self._generate_thread.start()
			
	def stop(self):
		if self._running:
			self._running=False
			self._generate_thread.join()
			GPIO.cleanup([self.pin1,self.pin2])
			GPIO.setup(self.pin1,GPIO.OUT)
			GPIO.setup(self.pin2,GPIO.OUT)
#Reset the bias value of all channel
def Reset_AllBias(Voltage):
	Vtx=Voltage_to_dacvalue(Voltage)
	dac_write(7,Vtx)
	print(f"Reset AllBias to {Voltage}")
	
#Convert analog voltage to digital
def Voltage_to_dacvalue(voltage):
	Voltage_digivalue=int((voltage/200)*65535)
	Voltage_digivalue=max(0,min(65535,Voltage_digivalue)) #Ensure the digital value is within the range of 0-65535
	return Voltage_digivalue

#MEMS Rotation Control_X
def Rotation_Control_X(Xbias,Xdiff,XdiffMax):
	if -XdiffMax<=Xdiff<=XdiffMax:
		XF_Vtx=Voltage_to_dacvalue(Xbias+(Xdiff)/2)
		XV_Vtx=Voltage_to_dacvalue(Xbias-(Xdiff)/2)
		dac_write(0,XF_Vtx)#A channel present X+
		dac_write(1,XV_Vtx)#B channel present X-
		#time.sleep(0.005)
	else:
		print("Vdiff loaded Wrong")
		Reset_AllBias()
		
#MEMS Rotation Control_Y
def Rotation_Control_Y(Ybias,Ydiff,YdiffMax):
	if -YdiffMax<=Ydiff<=YdiffMax:
		YF_Vtx=Voltage_to_dacvalue(Ybias+(Ydiff)/2)
		YV_Vtx=Voltage_to_dacvalue(Ybias-(Ydiff)/2)
		dac_write(3,YF_Vtx)#D channel present Y+
		dac_write(2,YV_Vtx)#C channel present Y-
		#time.sleep(0.005)
	else:
		print("Vdiff loaded Wrong")
		Reset_AllBias()
		
#Convert normalized coordinate to Diff
def Coordi_to_Diff(normalized_X,normalized_Y,Vdiff):
	X_Voltage=normalized_X*Vdiff
	Y_Voltage=normalized_Y*Vdiff
	return X_Voltage,Y_Voltage

#Calculate lissajous voltage
def calculate_lissajous_voltage(X_diff,Y_diff,delta,t,n):
	x_diff=X_diff*np.sin(t)		#delta:phase of lissajous;n:ratio of lissajous frequency,different n represent different profiles
	y_diff=Y_diff*np.sin(n*t+delta)
	return x_diff,y_diff
	
#Print  Lissajour
def Print_Lissajour(Vdiff,delta,t_values,n,Vbias,VdiffMax):
    for t in t_values:
        x_diff, y_diff = calculate_lissajous_voltage(Vdiff, Vdiff, delta, t, n)
        #plt.plot(x_diff, y_diff, 'b.')
        Rotation_Control_X(Vbias, y_diff, VdiffMax)
        Rotation_Control_Y(Vbias, x_diff, VdiffMax)

#Region Scan Class
class RegionScan:
    def __init__(self, step_size=0.01):
        self.step_size = step_size
        self.generator = self._point_generator()
        self.stop = False

    def _point_generator(self):
        x = -1
        while x <= 1:
            y = -1
            while y <= 1:
                yield x, y
                y += self.step_size
            x += self.step_size

    def traverse_points(self):
        self.stop = False  # 重置停止标志
        while not self.stop:
            try:
                point = next(self.generator)
                print(point)  # 或者你想对点进行的其他操作
            except StopIteration:
                break

    def stop_traverse(self):
        self.stop = True

    def reset_generator(self):
        self.generator = self._point_generator()

#Coarse Alignment
def Coarse_Alignment(Received_Data,Hit_area,Scanner):
	Scanner.reset_generator()  # 从头开始扫描
	Xaxis_length=Received_Data[4]
	Yaxis_length=Received_Data[5]
	def condition():		   # 使用外部变量进行条件判断
		return (Xaxis_length - Yaxis_length) < Hit_area
	Scanner.traverse_points(check_condition=condition)


#Pirnt AIRS
def Print_AIRS(Vdiff,VdiffMax,WIDTH,HEIGHT,Vbias):
	coordinates = [
	(2, 2), (2.714, 3.857), (3.429, 5.286), (4.143, 6.714),
    (4.857, 8.143), (5.571, 9.571),(5, 8),(5.4, 7.4),(5.8, 6.8),(6.2, 6.2), 
    (6.6, 5.6), (7.0, 5.0),(8, 2),(9, 2), (9,3),(9,4),(9,5),(9,6),(9,7),
    (9, 8),(13.5000, 8.0000),(14.1508, 7.8515), (14.6727, 7.4352),
    (14.9624, 6.8338),(14.9624, 6.1662), (14.6727, 5.5648), 
    (14.1508, 5.1485),(13.5000, 5.0000), (12.0000, 5.0000),(11,5),(10,5),(9,5),(12,5), (15.0000, 2.0000),
    (17.0000, 2.0000), (20.5000, 2.0000), (21.1508, 2.1485),
    (21.6727, 2.5648), (21.9624, 3.1662), (21.9624, 3.8338),
    (21.6727, 4.4352), (21.1508, 4.8515), (20.5000, 5.0000),
    (18.5000, 5.0000), (17.8492, 5.1485), (17.3273, 5.5648),
    (17.0376, 6.1662), (17.0376, 6.8338), (17.3273, 7.4352),
    (17.8492, 7.8515), (18.5000, 8.0000), (22.0000, 8.0000)]
	AIRS_coordinates = np.array(coordinates)
	
	for vector in AIRS_coordinates:
		x,y=vector
		x_diff = ((x/WIDTH)*2-1)*Vdiff
		y_diff = ((y/HEIGHT)*2-1)*Vdiff
		Rotation_Control_X(Vbias, y_diff, VdiffMax)
		Rotation_Control_Y(Vbias, x_diff, VdiffMax)
		time.sleep(0.001)
	
	for vector in reversed(AIRS_coordinates):
		a,b=vector
		x_diff = ((a/WIDTH)*2-1)*Vdiff
		y_diff = ((b/HEIGHT)*2-1)*Vdiff
		Rotation_Control_X(Vbias, y_diff, VdiffMax)
		Rotation_Control_Y(Vbias, x_diff, VdiffMax)
		time.sleep(0.001)
	
def Matplot():
	plt.xlabel('X_diff')
	plt.ylabel('Y_diff')
	plt.title('Lissajour curve')
	plt.grid(True)
	plt.show()
