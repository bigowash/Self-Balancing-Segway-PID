import pyb
from pyb import Pin, Timer
from oled_938 import OLED_938
from mpu6050 import MPU6050
# import time
import micropython
micropython.alloc_emergency_exception_buf(200)
# from pyb import ExtInt

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

b_LED = LED(4)

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

# # ------------ Interrupts Start ---------------------

# # speed
# def isr_motorA(dummy):
#     global A_count
#     A_count += 1

# def isr_speed_timer(dummy):
#     global A_count
#     global A_speed
#     global B_count
#     global B_speed
#     A_speed = A_count
#     A_count = 0
#     B_speed = B_count
#     B_count = 0

# def isr_motorB(dummy):
#     global B_count
#     B_count += 1

# # Create external interrupts for motorA Hall Effect Sensor
# motorA_int = ExtInt('Y4', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorA)
# motorB_int = ExtInt('Y7', ExtInt.IRQ_RISING, Pin.PULL_NONE, isr_motorB)

# # Create timerinterrupts at 100msec intervals
# speed_timer = pyb.Timer(4, freq = 10)
# speed_timer.callback(isr_speed_timer)

# # ------------ Interrupts End ---------------------
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
            print(w, proportional, differential, integral)
            b_LED.on()
            return 100
        if (w < -100):
            print(w, proportional, differential, integral)
            b_LED.on()
            return -100
        b_LED.off()
        return w

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

fell = 0
while fell == 0:

    toc = pyb.millis()

    deltat = toc - tic
    # print(deltat)

    tic = pyb.millis()  # Reset tic
    # pyb.delay(20)
    
    pitch, pitch_dot = pitch_read(pitch, deltat, 0.99) # alpha: larger => longer time constant

    val = pid.getPWM(pitch, pitch_dot, 0.56, deltat)

    if (pitch < -40 or pitch > 40):
        fell = 1

    # # falling backwards
    if (val > 0):
        A_back(val)
        B_back(val)
    else:
        A_forward(-val)
        B_forward(-val)
