import pyb
from pyb import Pin, Timer
from oled_938 import OLED_938
from mpu6050 import MPU6050

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
# oled.clear()

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

# -------------- IMU Reading ---------------------

def read_imu(dt):
	
    global g_pitch
    alpha = 0.7    # larger = longer time constant
    pitch = int(imu.pitch())
    roll = int(imu.roll())
    g_pitch = alpha*(g_pitch + imu.get_gy()*dt*0.001) + (1-alpha)*pitch
    
    oled.clear()
    oled.line(96, 26, pitch, 24, 1)
    oled.line(32, 26, g_pitch, 24, 1)
    oled.draw_text(0,0," Rol | PITCH |")
    oled.draw_text(83,0, "filtered")
    oled.display()
		
    return pitch
g_pitch = 0   

def pitch_read(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt*0.01) + (1-alpha)*theta
    return (pitch, pitch_dot)
pitch = 0
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
motorB_int = ExtInt('Y6', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorB)

# Create timerinterrupts at 100msec intervals
speed_timer = pyb.Timer(4, freq = 10)
speed_timer.callback(isr_speed_timer)

# ------------ Interrupts End ---------------------

# for IMU reading
tic = pyb.millis()	

while True:

    # for IMU reading
    toc = pyb.millis()
    # pitch = read_imu(toc-tic)
    # alpha: larger => longer time constant
    pitch, pitch_dot = pitch_read(pitch, toc-tic, 0.5)
    tic = pyb.millis()
    roll = imu.roll()
    
    # falling backwards
    if (pitch > -1):
        A_back(pitch)
        # B_back(roll)
    elif (pitch < -1):
        A_forward(-pitch)
        # B_forward(-roll)

    # falling backwards
    if (roll > -1):
        # A_back(pitch)
        B_back(roll)
    elif (roll < -1):
        # A_forward(-pitch)
        B_forward(-roll)

    oled.draw_text(0,20,'Motor A:{:5.2f} rps'.format(A_speed/39))
    oled.draw_text(0,45,'Motor B:{:5.2f} rps'.format(B_speed/39))
    oled.display()

    
    