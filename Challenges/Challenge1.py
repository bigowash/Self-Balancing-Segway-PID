import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from audio import MICROPHONE

# for the challenge
from neopixel import NeoPixel

# create neopixel object
np = NeoPixel(Pin("Y12", Pin.OUT), 8)

#  The following two lines are needed by micropython
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
i2c = pyb.I2C(2, pyb.I2C.MASTER)
devid = i2c.scan()				# find the I2C device number
oled = OLED_938(
    pinout={"sda": "Y10", "scl": "Y9", "res": "Y8"},
    height=64,
    external_vcc=False,
    i2c_devid=i2c.scan()[0],
)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Arturo and Theo')
oled.display()

# define ports for microphone, LEDs and trigger out (X5)

b_LED = LED(4)		# flash for beats on blue LED

def flash():		# routine to flash blue LED when beat detected
	b_LED.on()
	pyb.delay(10)
	b_LED.off()

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
	num = c*6//2.5 # tuned values that make c usually in between 1-8
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

# Create timer interrupt - one every 1/8000 sec or 125 usec
pyb.disable_irq()
sample_timer = pyb.Timer(7, freq=8000)	# set timer 7 for 8kHz

N = 160				# number of sample to calculate instant energy
mic = ADC(Pin('Y11'))
audio = MICROPHONE(sample_timer, mic, N)
pyb.enable_irq(True)
oled.draw_text(0,20, 'Ready to GO')	# Useful to show what's happening?
oled.display()
pyb.delay(100)

# Calculate energy over 50 epochs, each 20ms (i.e. 1 sec)
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 2.0		# threshold for c to indicate a beat
MIN_BEAT_PERIOD = 500		# no beat less than this

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs

tic = pyb.millis()			# mark time now in msec

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
		
		light(c)

		if (pyb.millis()-tic > MIN_BEAT_PERIOD):	# if longer than minimum period
			if (c>BEAT_THRESHOLD):		# look for a beat
				flash()					# beat found, flash blue LED
				tic = pyb.millis()		# reset tic
		audio.reset_buffer()				# reset status flag
