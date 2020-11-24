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
servo_pin = 22
frequency = 50
a = 10
b = 2.5

# ultrasonic sensor
trig_pin = 15  # send
echo_pin = 16  # receive

# active buzzer
buzzer_pin = 11

# button
btn_pin = 12

init_data = {}
detect_data = {}


def setup():
    global pwm
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    GPIO.setup(servo_pin, GPIO.OUT)  # G25
    GPIO.setup(trig_pin, GPIO.OUT)  # G22
    GPIO.setup(echo_pin, GPIO.IN)  # G23
    GPIO.setup(buzzer_pin, GPIO.OUT)  # G17
    GPIO.output(buzzer_pin, GPIO.HIGH)  # high -> no sound
    GPIO.setup(btn_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # G18, pull-up resistor
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
            measure(init_data, angle)
            time.sleep(1)
    if x == 1:  # Detecting Mode
        print "button not pressed"
        while True:
            for angle in range(180, -1, -30):
                set_angle(angle)
                measure(detect_data, angle)
                time.sleep(1)
            for angle in range(0, 181, 30):
                set_angle(angle)
                measure(detect_data, angle)
                time.sleep(1)

    for key, value in init_data.items():
        print str(key) + ":" + str(value)


def set_angle(angle):
    duty_cycle = b + angle * a / 180
    pwm.ChangeDutyCycle(duty_cycle)
    print "angle =", angle, "-> duty cycle =", duty_cycle


def measure(dict, angle):
    GPIO.output(trig_pin, 1)
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
    dict[angle] = round(distance, 2)
    print("%.2f" % distance)


def judge_distance(init_data, detect_data):
    pass


def buzzer_on():
    GPIO.output(buzzer_pin, GPIO.LOW)
    print "start"


def buzzer_off():
    GPIO.output(buzzer_pin, GPIO.HIGH)
    print "stop"


def beep(x):
    buzzer_on()
    time.sleep(x)
    buzzer_off()
    time.sleep(x)


def loop():
    while True:
        pass


def destroy():
    print "done"
    set_angle(0)
    GPIO.output(buzzer_pin, GPIO.HIGH)
    GPIO.cleanup()


if __name__ == '__main__':
    print "starting"
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        destroy()
