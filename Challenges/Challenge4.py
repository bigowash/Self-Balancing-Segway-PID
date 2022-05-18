# from shutil import move
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from oled_938 import OLED_938
from mpu6050 import MPU6050
from array import array
from audio import MICROPHONE
import time
import random

import micropython
micropython.alloc_emergency_exception_buf(200)
from pyb import ExtInt

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64, external_vcc=False, i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()

oled.draw_text(0,30,'Challenge 4')
oled.draw_text(0,45,'Theo and Arturo')
oled.display()

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

b_LED = LED(4)

#For Beat Detection
# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 2.0		# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 500	# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)
tic = pyb.millis()			# mark time now in msec

# -------------- Motor Control --------------------

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

while True:				# Main program loop
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
        if (pyb.millis()-tic > MIN_BEAT_PERIOD):	# if longer than minimum period
            if (c>BEAT_THRESHOLD):		# look for a beat

                b_LED.on()
                pyb.delay(10)
                b_LED.off()

                dd = random.randint(0, 4)
                
                if (dd == 0):
                    Forward(50)
                elif (dd == 1):
                    Backward(50)
                elif(dd==2):
                    Left(50)
                elif(dd==3):
                    Right(50)
                elif(dd==4):
                    Stop()

                tic = pyb.millis()		# reset tic
        buffer_full = False				# reset status flag