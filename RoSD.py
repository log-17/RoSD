# coding=utf-8
# !/usr/bin/python

import RPi.GPIO as GPIO
import time

'''
the pulse width is from 0.5ms to 2.5ms

W refers to the pulse width
P refers to the duty cycle
T refers to the cycle
Theta refers to the angle

W(ms)  P(%)  Theta(°)
0.5ms  2.5     0
1.5ms  7.5     90
2.5ms  12.5    180

you can draw a line graph to calculate a and b

formula:
W = T * P
P = b + Theta * a / 180
'''
# sg90 micro-servo motor
servo_pin = 12
frequency = 50
a = 10
b = 2.5

# ultrasonic sensor
trig_pin = 15  # send
echo_pin = 16  # receive

btn_pin = 22


def setup():
    global pwm
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    GPIO.setup(servo_pin, GPIO.OUT)  # G18
    GPIO.setup(trig_pin, GPIO.OUT)  # G22
    GPIO.setup(echo_pin, GPIO.IN)  # G23
    GPIO.setup(btn_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # G17, pull-up resistor
    GPIO.add_event_detect(btn_pin, GPIO.BOTH, callback=detect, bouncetime=200)  # detect events, debounce for 200ms

    pwm = GPIO.PWM(servo_pin, frequency)
    pwm.start(0)
    time.sleep(2)


def detect(channel):
    rotate(GPIO.input(btn_pin))


def rotate(x):
    if x == 0:  # Initializing Mode
        print "button pressed"
        for angle in range(0, 181, 30):
            set_angle(angle)
    if x == 1:  # Detecting Mode
        print "button not pressed"
        while True:
            for angle in range(180, -1, -30):
                set_angle(angle)
            for angle in range(0, 181, 30):
                set_angle(angle)


def set_angle(angle):       
    duty_cycle = b + angle * a / 180
    pwm.ChangeDutyCycle(duty_cycle)
    print "angle =", angle, "-> duty cycle =", duty_cycle
    measure()
    time.sleep(1)


def measure():
    GPIO.output(trig_pin, 1)  #  1
    time.sleep(0.00001)  # 10us
    GPIO.output(trig_pin, 0)

    # start sending ultrasound
    while GPIO.input(echo_pin) == 0:
        pass
    start = time.time()

    # receive returned ultrasound
    while GPIO.input(echo_pin) == 1:
        pass
    end = time.time()

    # compute distance
    distance = (end - start) * 340 / 2 * 100
    print('%.2f' % distance)


def loop():
    while True:
        pass


def destroy():
    print "done"
    set_angle(0)
    GPIO.cleanup()


if __name__ == '__main__':
    print "starting"
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
