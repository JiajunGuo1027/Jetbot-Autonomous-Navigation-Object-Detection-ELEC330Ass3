import smbus2
import RPi.GPIO as GPIO
from time import sleep
import i2cEncoderMiniLib


def EncoderChangeLeft():
    print ('Changed Left: %d' % (encoderLeft.readCounter32()))

def EncoderChangeRight():
    print ('Changed Right: %d' % (encoderRight.readCounter32()))
    
    
def EncoderPushLeft():
    print ('Encoder Pushed! Left')
	
def EncoderPushRight():
    print ('Encoder Pushed! Right')
 
def EncoderReleaseLeft():
    print ('Encoder Released! Left')

def EncoderReleaseRight():
    print ('Encoder Released! Right')


def EncoderDoublePushLeft():
    print ('Encoder Double Push! Left')


def EncoderDoublePushRight():
    print ('Encoder Double Push! Right')


def EncoderLongPushLeft():
    print ('Encoder Long Push! Left')

def EncoderLongPushRight():
    print ('Encoder Long Push! Right')


def EncoderMaxLeft():
    print ('Encoder max! Left')

def EncoderMaxRight():
    print ('Encoder max! Right')
    
    
def EncoderMinLeft():
    print ('Encoder min! Left')

def EncoderMinRight():
    print ('Encoder min! Right')
    

def Encoder_INT_left(self):
    encoderLeft.updateStatus()

def Encoder_INT_right(self):
    encoderRight.updateStatus()
    

GPIO.setmode(GPIO.BCM)
bus = smbus2.SMBus(0)
INT_pin = 4
GPIO.setup(INT_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

encoderRight = i2cEncoderMiniLib.i2cEncoderMiniLib(bus, 0x20)
encoderLeft = i2cEncoderMiniLib.i2cEncoderMiniLib(bus, 0x24)

encconfig = ( i2cEncoderMiniLib.WRAP_ENABLE | i2cEncoderMiniLib.DIRE_RIGHT | i2cEncoderMiniLib.IPUP_ENABLE | i2cEncoderMiniLib.RMOD_X1)
encoderRight.begin(encconfig)
encoderLeft.begin(encconfig)


encoderRight.writeCounter(0)
encoderRight.writeMax(35)
encoderRight.writeMin(-20)
encoderRight.writeStep(1)
encoderRight.writeDoublePushPeriod(50)

encoderLeft.writeCounter(0)
encoderLeft.writeMax(35)
encoderLeft.writeMin(-20)
encoderLeft.writeStep(1)
encoderLeft.writeDoublePushPeriod(50)


encoderRight.onChange = EncoderChangeRight 
# when th encdoer value changes it runs that function  EncoderChangeRight 
#this function prints the encoder value to the terminaal using encoderRight.readCounter32()
# we can use readCounter32() to get the int value for the encder later and find out position
encoderRight.onButtonPush = EncoderPushRight
encoderRight.onButtonRelease = EncoderReleaseRight
encoderRight.onButtonDoublePush = EncoderDoublePushRight
encoderRight.onButtonLongPush = EncoderLongPushRight
encoderRight.onMax = EncoderMaxRight
encoderRight.onMin = EncoderMinRight


encoderLeft.onChange = EncoderChangeLeft
encoderLeft.onButtonPush = EncoderPushLeft
encoderLeft.onButtonRelease = EncoderReleaseLeft
encoderLeft.onButtonDoublePush = EncoderDoublePushLeft
encoderLeft.onButtonLongPush = EncoderLongPushLeft
encoderLeft.onMax = EncoderMaxLeft
encoderLeft.onMin = EncoderMinLeft


encoderRight.autoconfigInterrupt()
print ('Board ID right code: 0x%X' % (encoderRight.readIDCode()))
print ('Board  right Version: 0x%X' % (encoderRight.readVersion()))

encoderLeft.autoconfigInterrupt()
print ('Board ID left code: 0x%X' % (encoderLeft.readIDCode()))
print ('Board  left Version: 0x%X' % (encoderLeft.readVersion()))

GPIO.add_event_detect(INT_pin, GPIO.FALLING, callback=Encoder_INT_left, bouncetime=10)


while True:
    if GPIO.input(INT_pin) == False: #
       Encoder_INT_left(INT_pin) #
       Encoder_INT_right(INT_pin)
    pass
    
    
