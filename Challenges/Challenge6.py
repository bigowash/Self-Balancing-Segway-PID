from multiprocessing.pool import IMapUnorderedIterator
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
# from pyb import Pin, Timer
from oled_938 import OLED_938
from mpu6050 import MPU6050
# import time

from array import array
from audio import MICROPHONE
import random

# for the challenge
from neopixel import NeoPixel

import micropython
micropython.alloc_emergency_exception_buf(200)
from pyb import ExtInt

# create neopixel object
np = NeoPixel(Pin("Y12", Pin.OUT), 8)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.clear()

oled.draw_text(0,30,'Challenge 6')
oled.draw_text(0,45,'Theo and Arturo')
oled.display()

b_LED = LED(4)

# the colors we want for each pixel
gradient = [
    (0,255,0),
    (125,323,0),
    (170,208,0),
    (202,181,0),
    (225,151,0),
    (242,118,0),
    (252,79,0),
    (255,0,0)
]

def light(c):
	num = c*2//2.5 # tuned values that make c usually in between 1-8
	print(num)
	for i in range(8):
		if i < 8:
			if i < num:
				np[i] = gradient[i]
				np.write()
				pyb.delay(1)
			else:
				np[i] = (0, 0, 0)
				np.write()
				pyb.delay(1)

# IMU connected to X9 and X10
imu = MPU6050(1, False)    	# Use I2C port 1 on Pyboard

A1 = Pin('X3', Pin.OUT_PP)
A2 = Pin('X4', Pin.OUT_PP)
B1 = Pin('X7', Pin.OUT_PP)
B2 = Pin('X8', Pin.OUT_PP)

PWMA = Pin('X1')
PWMB = Pin('X2')

pot = pyb.ADC(Pin('X11'))

tim = Timer(2, freq = 1000)
motorA = tim.channel(1, Timer.PWM, pin = PWMA)
motorB = tim.channel(2, Timer.PWM, pin = PWMB)

# For motor Speeds
A_sense = Pin('Y4', Pin.PULL_NONE)
B_sense = Pin('Y6', Pin.PULL_NONE)

speed = 0

A_state = 0
A_speed = 0
A_count = 0
B_state = 0
B_speed = 0
B_count = 0

desired_A = 0
desired_B = 0

# --------------- For Beat Detection ---------------------

# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 5.0		# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 500	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)
ticD = pyb.millis()			# mark time now in msec

# -------------- IMU Reading ---------------------

def pitch_read(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt*0.001) + (1-alpha)*theta # complementary filter
    return (pitch, pitch_dot)

# -------------- Motor Control --------------------\

def A_back(value):
    A1.low()
    A2.high()
    motorA.pulse_width_percent(value)

def A_forward(value):
    A1.high()
    A2.low()
    motorA.pulse_width_percent(value)

def A_stop():
    motorA.pulse_width_percent(0)

def B_forward(value):
    B1.low()
    B2.high()
    motorB.pulse_width_percent(value)

def B_back(value):
    B1.high()
    B2.low()
    motorB.pulse_width_percent(value)

def B_stop():
    motorB.pulse_width_percent(0)

def Forward(val):
    A_forward(val)     # move forward
    B_forward(val)
def Backward(val):
    A_back(val)        #move backward
    B_back(val)
def Right(val):
    A_forward(val)      # move right
    B_back(val)
def Left(val):
    A_back(val)       # move left
    B_forward(val)
def Stop():             #stop
    A_stop()
    B_stop()

# ------------ Interrupts Start ---------------------

# speed
def isr_motorA(dummy):
    global A_count
    A_count += 1

def isr_speed_timer(dummy):
    global A_count
    global A_speed
    global B_count
    global B_speed
    A_speed = A_count
    A_count = 0
    B_speed = B_count
    B_count = 0

def isr_motorB(dummy):
    global B_count
    B_count += 1

# Create external interrupts for motorA Hall Effect Sensor
motorA_int = ExtInt('Y4', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorA)
motorB_int = ExtInt('Y7', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorB)

# Create timerinterrupts at 100msec intervals
speed_timer = pyb.Timer(4, freq = 10)
speed_timer.callback(isr_speed_timer)

# ------------ Interrupts End ---------------------
# ----------------- PID ---------------------------

class PID(object):
    def __init__(self, kp, kd, ki):
        self.cumulative_pitch_error = 0
        self.prev_error = 0
        # self.target = target
        self.kp = kp
        self.kd = kd
        self.ki = ki

    def getPWM(self, pitch, pitch_dot, target, dt):
        
        if (dt == 0):
            # print("no time")
            return 0

        error = pitch - target
        
        if (abs(error) <= 0.2):
        # if (abs(error) <= 1.1):
            # print("upright")
            return 0

        proportional = self.kp*error
        
        # # kd part - differential
        differential = self.kd * pitch_dot

        self.cumulative_pitch_error += error * dt*0.001

        # ki part - integral
        integral = self.ki*self.cumulative_pitch_error

        # summation
        w = proportional+differential+integral

        # if w > 50:
        #     print(w, proportional, differential, integral)

        if (w > 100):
            # print(w, proportional, differential, integral)
            # b_LED.on()
            return 100
        if (w < -100):
            # print(w, proportional, differential, integral)
            # b_LED.on()
            return -100
        # b_LED.off()
        return w

# ------------ Read move ----------------

f = open('moves.txt', 'r')
moves = str (f.read())
f.close()

counter = 0

def getMove():
    global counter

    intrs = moves[counter]
    counter += 1

    return intrs

# -------------- Start ------------------

# kp = 8.9
kp = 16
# kd = 0.76
kd = 1
# kd = 0.8
# ki = 3.5
ki = 3.7
# ki = 0  

# start pid object
pid = PID(kp, kd, ki)

# for IMU reading
tic = pyb.millis()
ticI = pyb.millis()	

# starting params
pitch = 0

timer = tic

movement = 0
prevVal = 0 

while True:  # Main program loop

    toc = pyb.millis()

    deltat = toc - tic

    tic = pyb.millis()  # Reset tic

    pitch, pitch_dot = pitch_read(pitch, deltat, 0.99) # alpha: larger => longer time constant

    val = pid.getPWM(pitch, pitch_dot, 0.56, deltat)
    
    # depending on current move, get values for motors

    if movement == 0:
        val = pid.getPWM(pitch, pitch_dot, 0.56, deltat)
    elif movement == 1:
        val = pid.getPWM(pitch, pitch_dot, 0.6, deltat)
    elif movement == 2:
        val = pid.getPWM(pitch, pitch_dot, 0.52, deltat)

    if (val > 0):
        A_back(val)
        B_back(val)
    else:
        A_forward(-val)
        B_forward(-val)

    if audio.buffer_is_filled():		# semaphore signal from ISR - set if buffer is full

        # Fetch instantaneous energy
        E = audio.inst_energy()			# fetch instantenous energy
        audio.reset_buffer()			# get ready for next epoch

		# compute moving sum of last 50 energy epochs with circular buffer
        sum_energy = sum_energy - e_buf[e_ptr] + E
        e_buf[e_ptr] = E			# over-write earliest energy with most recent
        e_ptr = (e_ptr + 1) % M		# increment e_ptr with wraparound - 0 to M-1
        average_energy = sum_energy/M
        
        # Compute ratio of instantaneous energy/average energy
        c = E/average_energy
        
        light(c) # neopixel strip

        if (pyb.millis()-ticD > MIN_BEAT_PERIOD):	# if longer than minimum period
           
            if (c>BEAT_THRESHOLD):		# look for a beat

                # blue led flash
                b_LED.on()
                pyb.delay(10)
                b_LED.off()

                dd = getMove()

                if (dd == "F"):
                    movement = 0
                    # val = pid.getPWM(pitch, pitch_dot, 1, deltat)
                elif (dd == "B"):
                    movement = 1
                    # val = pid.getPWM(pitch, pitch_dot, 0, deltat)
                elif(dd == "S"):
                    movement = 2

                ticD = pyb.millis()		# reset tic
        audio.reset_buffer()				# reset status flag

