# -- coding: utf-8 --

from tkinter import *
import tkinter.font as font
import RPi.GPIO as GPIO
import time
import socket
import threading
import cv2
import pyzbar
from pyzbar.pyzbar import decode
from PIL import Image, ImageTk

from encoder import Encoder
from qrCodeReceiver import QrCodeReceiver

## MARK: ------------ GPIO ------------

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# DC Motor Encoder
dcMotorEncoderA = 22    # White
dcMotorEncoderB = 27    # Orange

# linear Actuator Encoder
linearEncoderA = 24
linearEncoderB = 25

## ------------ Encoder Class callback ------------
def measure(value, direction):
	print(f"Value: {value}, Direction: {direction}")

encMotor = Encoder(dcMotorEncoderA, dcMotorEncoderB, measure)
encLinear = Encoder(linearEncoderA, linearEncoderB, measure)

# 24V dc motor
dcMotorPWM = 13
dcMotorDIR = 19

# 12V linear actuator
linearPWM = 10
linearDIR = 9

M1 = 30 # 24V dc motor pwm
M2 = 30 # 12V linear actuator pwm
steps = 3200 # step motor steps
delay = 0.00001 # step motor delay

# GPIO setup
GPIO.setup(dcMotorPWM, GPIO.OUT)
GPIO.setup(dcMotorDIR, GPIO.OUT)
GPIO.setup(linearPWM, GPIO.OUT)
GPIO.setup(linearDIR, GPIO.OUT)

# PWM setup
dcMotorPWM = GPIO.PWM(dcMotorPWM, 100)
dcMotorPWM.start(0)
linearPWM = GPIO.PWM(linearPWM, 100)
linearPWM.start(0)

if M1 > 80:
    M1 = 80

if M2 > 30:
    M2 = 30

t2 = 0
            
# qr_receiver = qrCodeReceiver()

## MARK: -------- QRCodeScan --------

# enableQRScan = False

class QRCodeScanner:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.ret, self.frame = self.camera.read()
        self.running = True
        self.frame = None
        self.resized_frame = None
        self.enableQRScan = False
        
        self.name = '순수현'
        self.height = "178"
        self.weight = "64"
        
        self.shoulderWidth = 0
        self.shoulderHeight = 8
        self.tiltUpTime = 9
        self.tiltDownTime = 7
        self.upTime = 9

    def start(self):
        threading.Thread(target=self.process_frames).start()
        return self

    def process_frames(self):
        while True:
            if not self.running:
                return
            self.ret, self.frame = self.camera.read()
            if self.ret:
                # Resize the frame for faster processing
                frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                resized_frame = cv2.resize(frame_rgb, (640, 480))

                qr_codes = decode(resized_frame)

                for qr_code in qr_codes:
                    qr_code_data = qr_code.data.decode("utf-8")
                    
                    print("QR code:", qr_code_data)
                    # TODO: 실제 시연 data쓸거면 주석 해제하고 QR 정보 맞게 잘 넣어야 함.
                    # self.parse(qr_code_data)
                    # time.sleep(10.0)
                    
                    # TODO: dimmedLabel, scanLabel, toggleAutoMode NOT IN THIS CLASS; SHOULD BE REFACTORED
                    dimmedLabel.place_forget()
                    scanLabel.place_forget()
                    toggleAutoMode()
                    return
                
                scan_frame = Image.fromarray(resized_frame)
                scan_image = ImageTk.PhotoImage(scan_frame)
                if self.enableQRScan:
                    scanLabel.config(image=scan_image)

    def stop(self):
        self.running = False
        self.camera.release()
        cv2.destroyAllWindows()
        
    def parse(self, data):
        key_value_pairs = data.split(",")

        for pair in key_value_pairs:
            key, value = pair.split(':')

            key = key.strip()
            value = value.strip()

            if key == "name":
                self.name = value
            elif key == "height":
                self.height = value
            elif key == 'weight':
                self.weight = value
                
            elif key == "t0":
                self.t0 = int(value)
            elif key == "t1":
                self.t1 = int(value)
            elif key == "t2":
                self.t2 = int(value)
                
            elif key == "어깨너비":
                self.shoulderWidth = int(value)
            elif key == "어깨높이":
                self.shoulderHeight = int(value)
            elif key == "숙이기":
                self.tiltUpTime = int(value)
            elif key == "세우기":
                self.tiltDownTime = int(value)
            elif key == "기립":
                self.upTime = int(value)
            print(key, value)

        # infoLabel.config(text = f"{scanner.height} cm\n{scanner.weight} kg")
        

scanner = QRCodeScanner()


## MARK: -------- AutoController --------

beforeShoulderHeight = 0.0
beforeTiltAngle = 0.0
shoulderToStandHeight = 0.0
afterShoulderHeight = 0.0

def setting(shoulderWidth, shoulderHeight):
    """
    처음 환자 어깨 높이로 어깨걸이 높이 조정
    추후, 어깨걸이 높이 조정 전 체스트 가이드 너비 조절 추가 필요
    """
    global dcMotorPWM, beforeShoulderHeight
    
    # TODO: 체스트 가이드 너비 조절
    
    ### UpButton: 볼스크류 상승
    beforeShoulderHeight = encMotor.value # For accurate control
    GPIO.output(dcMotorDIR, GPIO.LOW)
    dcMotorPWM.ChangeDutyCycle(M1)
    print("Setting > dcMotorPWM activated")
    
    try:
        motorUpTime = time.time()
        while True:
            current = time.time()
            # TODO: value 양수 음수 및 값 체크, 적절한 시간 차이 체크
            if (encMotor.value > beforeShoulderHeight + shoulderHeight) or (current - motorUpTime > 1.0):
                break
            print("Setting > Uptime: {}", current - motorUpTime)
            time.sleep(0.1)
    except Exception as e:
        print("Setting> Error {}!", e)
        # TODO: Kill methods
    dcMotorPWM.ChangeDutyCycle(0)
    print("Setting > dcMotorPWM deactivated")
    
    # Enable autoStanding
    autoAdjustingButton.configure(image = adjustedImage, state = "disabled")
    autoStandingButton.bind(clicked, autoStandingButtonDidTap)
    autoStandingButton.bind(released, autoStandingButtonDidRelease)
    autoStandingButton.config(image = autoStandingImage, state = "normal")
    autoSittingButton.config(image = autoSittingDisabledImage, state = "disabled")
    print("Setting done")
    
def stand(tiltAngle, standHeight):
    """
    1. 리니어 액츄에이터를 구동하여 환자의 상체를 기울임
    2. DC 모터를 구동하여 환자를 기립시킴
    """
    global linearPWM, dcMotorPWM, beforeTiltAngle, shoulderToStandHeight, afterShoulderHeight
    
    ### TiltUpButton: 액츄에이터 틸트업
    beforeTiltAngle = encLinear.value
    GPIO.output(linearDIR, GPIO.LOW)
    linearPWM.ChangeDutyCycle(M2)
    print("Stand > linearPWM activated")
    
    try:
        linearTiltUpTime = time.time()
        while True:
            current = time.time()
            # TODO: value 양수 음수 및 값 체크, 적절한 시간 차이 체크
            if (encLinear.value > beforeTiltAngle + tiltAngle) or (current - linearTiltUpTime > 1.0):
                break
            print("Stand > tiltUptime: {}", current - linearTiltUpTime)
            time.sleep(0.1)
    except Exception as e:
        print("Stand tiltUp > Error {}!", e)
        # TODO: Kill methods
    linearPWM.ChangeDutyCycle(0)
    print("Stand > linearPWM deactivated")
    
    time.sleep(1)
    
    ### UpButton: 볼스크류 상승
    afterShoulderHeight = encMotor.value  # Save the current encoder value to utilize in "use()"
    GPIO.output(dcMotorDIR, GPIO.LOW)
    dcMotorPWM.ChangeDutyCycle(M1)
    print("Stand > dcMotorPWM activated")
    try:
        motorUpTime = time.time()
        while True:
            current = time.time()
            # TODO: value 양수 음수 및 값 체크, 적절한 시간 차이 체크
            if (encMotor.value > afterShoulderHeight + standHeight) or (current - motorUpTime > 1.0):
                break
            print("Stand > motorUptime: {}", current - motorUpTime)
            time.sleep(0.1)
    except Exception as e:
        print("Stand motorUp > Error {}!", e)
        # TODO: Kill methods
    dcMotorPWM.ChangeDutyCycle(0)
    print("Stand > dcMotorPWM deactivated")
    
    
    autoStandingButton.config(image = autoStandingDisabledImage, state = "disabled")
    autoSittingButton.config(image = autoSittingImage, state = "normal")
    autoSittingButton.bind(clicked, autoSittingButtonDidTap)
    autoSittingButton.bind(released, autoSittingButtonDidRelease)
    print("standing done")
    
def sit(afterShoulderHeight, beforeTiltAngle, beforeShoulderHeight):
    """
    1. 볼스크류가 하강한 만큼 DC 모터를 구동함
    2. 리니어 액츄에이터를 구동하여 기울어 있는 환자의 상체를 세움
    3. DC 모터를 구동하여 어깨 높이에 맞게 조정한 어깨 걸이를 낮춤
    """
    global dcMotorPWM, linearPWM, dcMotorDIR, linearDIR
    
    ### DownButton: 볼스크류 하강(상승한 만큼)
    GPIO.output(dcMotorDIR, GPIO.HIGH)
    dcMotorPWM.ChangeDutyCycle(M1)
    print("Sit > dcMotorPWM activated")
    try:
        motorDownTime = time.time()
        while True:
            current = time.time()
            # TODO: value 양수 음수 및 값 체크, 적절한 시간 차이 체크
            if encMotor.value < afterShoulderHeight or current - motorDownTime > 1.0:
                break
            print("Sit > motorDownTime 1: {}", current - motorDownTime)
            time.sleep(0.1)
    except Exception as e:
        print("Sit motorDown 1 > Error {}!", e)
        # TODO: Kill methods
    dcMotorPWM.ChangeDutyCycle(0)
    print("Sit > dcMotorPWM deactivated")
    
    ### TiltDownButton: 액츄에이터 틸트다운
    GPIO.output(linearDIR, GPIO.HIGH)
    linearPWM.ChangeDutyCycle(M2)
    print("Sit > linearPWM activated")
    try:
        linearTiltDownTime = time.time()
        while True:
            current = time.time()
            # TODO: value 양수 음수 및 값 체크, 적절한 시간 차이 체크
            if encLinear.value > beforeTiltAngle or current - linearTiltDownTime > 1.0:
                break
            print("Sit > linearDownTime: {}", current - linearTiltDownTime)
            time.sleep(0.1)
    except Exception as e:
        print("Sit linearDown > Error {}!", e)
        # TODO: Kill methods
    linearPWM.ChangeDutyCycle(0)
    print("Sit > linearPWM deactivated")
    time.sleep(1)
    
    dcMotorPWM.ChangeDutyCycle(M1)
    print("Sit > dcMotorPWM activated")
    try:
        motorDownTime = time.time()
        while True:
            current = time.time()
            # TODO: value 양수 음수 및 값 체크, 적절한 시간 차이 체크
            if (encMotor.value < beforeShoulderHeight) or (current - motorDownTime > 1.0):
                break
            print("Sit > motorDownTime 2: {}", current - motorDownTime)
            time.sleep(0.1)
    except Exception as e:
        print("Sit motorDown 2 > Error {}!", e)
        # TODO: Kill methods
    dcMotorPWM.ChangeDutyCycle(0)
    print("Sit > dcMotorPWM deactivated")
    toggleAutoMode()
    print("Sit done")
    
def stop():
    global linearPWM, dcMotorPWM, canceled
    linearPWM.ChangeDutyCycle(0)
    dcMotorPWM.ChangeDutyCycle(0)
    canceled = True
    upButton.config(image = upImage)
    downButton.config(image = downImage)
    tiltUpButton.config(image = tiltUpImage)
    tiltDownButton.config(image = tiltDownImage)

        
## MARK: -------- Window --------

window = Tk()
window.attributes("-fullscreen", True)
window.bind("<F11>", lambda event: window.attributes("-fullscreen", not window.attributes("-fullscreen")))
window.bind("<Escape>", lambda event: window.attributes("-fullscreen", False))

## MARK: ------------ UI ------------

# Variables

isAutoMode = False

scannedPersonName = "\(name)"
scannedPersonHeight = 0

clicked = "<Button-1>"
released = "<ButtonRelease-1>"

buttonSize = 163

repeatdelay = 10
repeatinterval = 10

background = "#FFFFFF"

leftFirstColumnX = 30
leftSecondColumnX = 220
rightFirstColumnX = 644
rightSecondColumnX = 834

firstRowY = 30
secondRowY = 220
thirdRowY = 410

handsButtonSize =  100
handsButtonY = 470

leftModeHandsButtonX1 = 774
leftModeHandsButtonX2 = 894

rightModeHandsButtonX1 = 30
rightModeHandsButtonX2 = 140

allModeHandsButtonX1 = 210
allModeHandsButtonX2 = 714

canceled = False
dcBottomPressed = False
dcTopPressed = False
limitBottomPressed = False
limitTopPressed = False

# Button Methods

def widenButtonDidTap(event):
    widenButton.config(image = widenPressedImage)
    print("widenButtonDidTap")

def widenButtonIsPressing():
    print("widenButtonIsPressing")

def widenButtonDidRelease(event):
    widenButton.config(image = widenImage)
    print("widenButtonDidRelease")
       

def narrowButtonDidTap(event):
    narrowButton.config(image = narrowPressedImage)
    print("narrowButtonDidTap")

def narrowButtonIsPressing():
    print("narrowButtonIsPressing")

def narrowButtonDidRelease(event):
    narrowButton.config(image = narrowImage)
    print("narrowButtonDidRelease")

def upButtonDidTap(event):
    print("upButtonDidTap")
    GPIO.output(dcMotorDIR, GPIO.LOW)
    dcMotorPWM.ChangeDutyCycle(M1)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지

def upButtonIsPressing():
    print("upButtonIsPressing")
    upButton.config(image = upPressedImage)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지

def upButtonDidRelease(event):
    global dcTopPressed
    print("upButtonDidRelease")
    dcTopPressed = True
    # upButton.config(image = upImage)
    # dcMotorPWM.ChangeDutyCycle(0)

def downButtonDidTap(event):
    print("downButtonDidTap")
    GPIO.output(dcMotorDIR, GPIO.HIGH)
    dcMotorPWM.ChangeDutyCycle(M1)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지

def downButtonIsPressing():
    print("downButtonIsPressing")
    downButton.config(image = downPressedImage)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지

def downButtonDidRelease(event):
    global dcBottomPressed
    print("downButtonDidRelease")
    dcBottomPressed = True
    # downButton.config(image = downImage)
    # dcMotorPWM.ChangeDutyCycle(0)

def tiltUpButtonDidTap(event):
    print("tiltUpButtonDidTap")
    GPIO.output(linearDIR, GPIO.LOW)
    linearPWM.ChangeDutyCycle(M2)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지
        
    # GPIO.output(linearDIR, GPIO.LOW)
    # linearPWM.ChangeDutyCycle(M2)

def tiltUpButtonIsPressing():
    global limitTopPressed
    limitTopPressed = True
    print("tiltUpButtonIsPressing")
    tiltUpButton.config(image = tiltUpPressedImage)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지

def tiltUpButtonDidRelease(event):
    global limitTopPressed
    print("tiltUpButtonDidRelease")
    limitTopPressed = True
    # tiltUpButton.config(image = tiltUpImage)
    # linearPWM.ChangeDutyCycle(0)

def tiltDownButtonDidTap(event):
    print("tiltDownButtonDidTap")
    GPIO.output(linearDIR, GPIO.HIGH)
    linearPWM.ChangeDutyCycle(M2)
    #TODO: 일정 엔코더 값을 넘으면 안전을 위해 강제 정지

def tiltDownButtonIsPressing():
    global limitBottomPressed
    limitBottomPressed = True
    print("tiltDownButtonIsPressing")
    tiltDownButton.config(image = tiltDownPressedImage)
        

def tiltDownButtonDidRelease(event):
    global limitBottomPressed
    print("tiltDownButtonDidRelease")
    limitBottomPressed = True
    # tiltDownButton.config(image = tiltDownImage)
    # linearPWM.ChangeDutyCycle(0)

def scanQRButtonDidTap():
    # global enableQRScan
    print("scanQRButtonDidTap")
    dimmedLabel.place(relx = 0.5, rely = 0.5, width = 1024, height = 600, anchor = "center")
    scanLabel.place(relx=0.5, rely=0.5, width=400, height=400, anchor="center")
    # scanner.enableQRScan = True
    time.sleep(3.0)
    dimmedLabel.place_forget()
    scanLabel.place_forget()
    toggleAutoMode()


def autoStandingButtonDidTap(event):
    print("autoStandingButtonDidTap")
    autoStandingButton.config(image = autoStandingPressedImage)
    # TODO: scanner 변수 추가하기
    stand(scanner.tiltAngle, scanner.standHeight)

def autoStandingButtonIsPressing():
    print("autoStandingButtonIsPressing")

def autoStandingButtonDidRelease(event):
    print("autoStandingButtonDidRelease")
    autoStandingButton.config(image = autoStandingImage)

def autoSittingButtonDidTap(event):
    global t2
    print("autoSittingButtonDidTap")
    autoSittingButton.config(image = autoSittingPressedImage)
    # TODO: scanner 변수 추가하기
    sit(afterShoulderHeight, beforeTiltAngle, beforeShoulderHeight)

def autoSittingButtonIsPressing():
    print("autoSittingButtonIsPressing")

def autoSittingButtonDidRelease(event):
    print("autoSittingButtonDidRelease")
    autoSittingButton.config(image = autoSittingImage)

# Layout Methods

def setLayout():
    window.grid_columnconfigure(3, weight=1)
    
    # widenButton.grid(row = 0, column = 0, padx = 30, pady = 30)
    # narrowButton.grid(row = 0, column = 1)
    # armpitLabel.grid(row = 0, column = 2, padx = 30)
    spacer1.grid(row = 0, column = 3, sticky = "ew")
    scanQRButton.grid(row = 0, column = 5, padx = 30)

    upButton.grid(row = 1, column = 0, padx = 30)
    downButton.grid(row = 1, column = 1)
    updownLabel.grid(row = 1, column= 2, padx = 30)
    spacer2.grid(row = 1, column = 3, sticky = "ew")
    autoAdjustingButton.grid(row = 1, column = 5, padx = 30)

    tiltUpButton.grid(row = 2, column = 0, padx = 30, pady = 30)
    tiltDownButton.grid(row = 2, column = 1)
    tiltLabel.grid(row = 2, column= 2, padx = 30)
    spacer3.grid(row = 2, column = 3, sticky = "ew")
    autoStandingButton.grid(row = 2, column = 4)
    autoSittingButton.grid(row = 2, column = 5, padx = 30)
    
    # TODO: For Test Version
    # exitAutoModeButton.grid(row = 0, column = 4, padx = 30)
    pauseButton.grid(row = 0, column = 0, padx = 30, pady = 30)
    

# UI Methods

def enableManualButtons():
    # armpitLabel.config(image = armpitImage)
    updownLabel.config(image = updownImage)
    tiltLabel.config(image = tiltImage)
    # widenButton.config(image = widenImage, state = "normal")
    # widenButton.bind(clicked, widenButtonDidTap)
    # widenButton.bind(released, widenButtonDidRelease)
    # narrowButton.config(image = narrowImage, state = "normal")
    # narrowButton.bind(clicked, narrowButtonDidTap)
    # narrowButton.bind(released, narrowButtonDidRelease)
    upButton.config(image = upImage, state = "normal")
    upButton.bind(clicked, upButtonDidTap)
    upButton.bind(released, upButtonDidRelease)
    downButton.config(image = downImage, state = "normal")
    downButton.bind(clicked, downButtonDidTap)
    downButton.bind(released, downButtonDidRelease)
    tiltUpButton.config(image = tiltUpImage, state = "normal")
    tiltUpButton.bind(clicked, tiltUpButtonDidTap)
    tiltUpButton.bind(released, tiltUpButtonDidRelease)
    tiltDownButton.config(image = tiltDownImage, state = "normal")
    tiltDownButton.bind(clicked, tiltDownButtonDidTap)
    tiltDownButton.bind(released, tiltDownButtonDidRelease)

def disableManualButtons():
    # armpitLabel.config(image = armpitDisabledImage)
    updownLabel.config(image = updownDisabledImage)
    tiltLabel.config(image = tiltDisabledImage)
    # widenButton.config(image = widenDisabledImage, state = "disabled")
    # widenButton.unbind(clicked)
    # widenButton.unbind(released)
    # narrowButton.config(image = narrowDisabledImage, state = "disabled")
    # narrowButton.unbind(clicked)
    # narrowButton.unbind(released)
    upButton.config(image = upDisabledImage, state = "disabled")
    upButton.unbind(clicked)
    upButton.unbind(released)
    downButton.config(image = downDisabledImage, state = "disabled")
    downButton.unbind(clicked)
    downButton.unbind(released)
    tiltUpButton.config(image = tiltUpDisabledImage, state = "disabled")
    tiltUpButton.unbind(clicked)
    tiltUpButton.unbind(released)
    tiltDownButton.config(image = tiltDownDisabledImage, state = "disabled")
    tiltDownButton.unbind(clicked)
    tiltDownButton.unbind(released)

def enableAutoButtons():
    autoStandingButton.config(image = autoStandingImage)
    autoStandingButton.bind(clicked, autoStandingButtonDidTap)
    autoStandingButton.bind(released, autoStandingButtonDidRelease)
    autoStandingButton.config(state = "normal")

    autoSittingButton.config(image = autoSittingImage)
    autoSittingButton.bind(clicked, autoSittingButtonDidTap)
    autoSittingButton.bind(released, autoSittingButtonDidRelease)
    autoSittingButton.config(state = "normal")

def disableAutoButtons():
    autoAdjustingButton.config(image=autoAdjustingDisabledImage, state='disabled')
    window.after(500, lambda: autoAdjustingButton.config(image = autoAdjustingDisabledImage))
    
    autoStandingButton.config(image = autoStandingDisabledImage)
    autoStandingButton.unbind(clicked)
    autoStandingButton.unbind(released)
    autoStandingButton.config(state = "disabled")

    autoSittingButton.config(image = autoSittingDisabledImage)
    autoSittingButton.unbind(clicked)
    autoSittingButton.unbind(released)
    autoSittingButton.config(state = "disabled")

def toggleAutoMode():
    global isAutoMode, dcMotorPWM, linearPWM
    if isAutoMode:
        stopAutoAdjustingButtonAnimation()
        # exitAutoModeButton.grid_forget()
        infoLabel.grid_forget()
        enableManualButtons()
        disableAutoButtons()
        dcMotorPWM.ChangeDutyCycle(0)
        linearPWM.ChangeDutyCycle(0)
    else:
        # exitAutoModeButton.grid(row = 0, column = 5, padx = 30)
        infoLabel.grid(row = 1, column = 4, sticky = "e")
        disableManualButtons()
        # enableAutoButtons()
        autoAdjustingButton.config(image=autoAdjustingImage, state='normal')
    isAutoMode = not isAutoMode

def cancelScanQRMode(event):
    dimmedLabel.place_forget()
    scanLabel.place_forget()
    
isAnimating = False

def autoAdjusting():
    global isAnimating
    isAnimating = True
    animateAdjustingGIF(0)
    # TODO: scanner 변수 추가하기
    setting(scanner.shoulderWidth, scanner.shoulderHeight)

def animateAdjustingGIF(frameIndex):
    global isAnimating
    global isAnimated
    autoAdjustingButton.configure(image=autoAdjustingGIFframes[frameIndex])
    if isAnimating:
        window.after(500, lambda: animateAdjustingGIF((frameIndex + 1) % len(autoAdjustingGIFframes)))

def stopAutoAdjustingButtonAnimation():
    global isAnimating
    isAnimating = False
    window.after(500, lambda: autoAdjustingButton.config(image = adjustedImage))
    


## MARK: -------- UI Components --------

spacer1 = Label(bg = background)
spacer2 = Label(bg = background)
spacer3 = Label(bg = background)

armpitImage = PhotoImage(file = "./images/armpit.png")
armpitDisabledImage = PhotoImage(file = "./images/armpit_disabled.png")
armpitLabel = Label(
    image = armpitImage,
    background = background
)

updownImage = PhotoImage(file = "./images/updown.png")
updownDisabledImage = PhotoImage(file = "./images/updown_disabled.png")
updownLabel = Label(
    image = updownImage,
    background = background
)

tiltImage = PhotoImage(file = "./images/tilt.png")
tiltDisabledImage = PhotoImage(file = "./images/tilt_disabled.png")
tiltLabel = Label(
    image = tiltImage,
    background = background
)

widenImage = PhotoImage(file = "./images/widen.png")
widenPressedImage = PhotoImage(file = "./images/widen_pressed.png")
widenDisabledImage = PhotoImage(file = "./images/widen_disabled.png")
widenButton = Button(
    image = widenImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = widenButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval,
)
widenButton.bind(clicked, widenButtonDidTap)
widenButton.bind(released, widenButtonDidRelease)

narrowImage = PhotoImage(file = "./images/narrow.png")
narrowPressedImage = PhotoImage(file = "./images/narrow_pressed.png")
narrowDisabledImage = PhotoImage(file = "./images/narrow_disabled.png")
narrowButton = Button(
    image = narrowImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = narrowButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
narrowButton.bind(clicked, narrowButtonDidTap)
narrowButton.bind(released, narrowButtonDidRelease)

upImage = PhotoImage(file = "./images/up.png")
upPressedImage = PhotoImage(file = "./images/up_pressed.png")
upDisabledImage = PhotoImage(file = "./images/up_disabled.png")
upButton = Button(
    image = upImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = upButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
upButton.bind(clicked, upButtonDidTap)
upButton.bind(released, upButtonDidRelease)

downImage = PhotoImage(file = "./images/down.png")
downPressedImage = PhotoImage(file = "./images/down_pressed.png")
downDisabledImage = PhotoImage(file = "./images/down_disabled.png")
downButton = Button(
    image = downImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = downButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
downButton.bind(clicked, downButtonDidTap)
downButton.bind(released, downButtonDidRelease)

tiltUpImage = PhotoImage(file = "./images/tiltUp.png")
tiltUpPressedImage = PhotoImage(file = "./images/tiltUp_pressed.png")
tiltUpDisabledImage = PhotoImage(file = "./images/tiltUp_disabled.png")
tiltUpButton = Button(
    image = tiltUpImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = tiltUpButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
tiltUpButton.bind(clicked, tiltUpButtonDidTap)
tiltUpButton.bind(released, tiltUpButtonDidRelease)

tiltDownImage = PhotoImage(file = "./images/tiltDown.png")
tiltDownPressedImage = PhotoImage(file = "./images/tiltDown_pressed.png")
tiltDownDisabledImage = PhotoImage(file = "./images/tiltDown_disabled.png")
tiltDownButton = Button(
    image = tiltDownImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = tiltDownButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
tiltDownButton.bind(clicked, tiltDownButtonDidTap)
tiltDownButton.bind(released, tiltDownButtonDidRelease)

scanQRImage = PhotoImage(file = "./images/scanQR.png")
scanQRButton = Button(
    image = scanQRImage,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = scanQRButtonDidTap,
)

autoAdjustingImage = PhotoImage(file = "./images/autoAdjusting.png")
autoAdjustingDisabledImage = PhotoImage(file = "./images/autoAdjusting_disabled.png")
adjustingGIF = Image.open("./images/adjusting.gif")
adjustedImage = PhotoImage(file = "./images/adjusted.png")
autoAdjustingButton = Button(
    image = autoAdjustingDisabledImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = autoAdjusting,
)
autoAdjustingButton.config(state = "disabled")

autoAdjustingGIFframes = []
for frame in range(adjustingGIF.n_frames):
    adjustingGIF.seek(frame)
    resized_frame = adjustingGIF.resize((170, 170))
    autoAdjustingGIFframes.append(ImageTk.PhotoImage(resized_frame))

autoStandingImage = PhotoImage(file = "./images/autoStanding.png")
autoStandingPressedImage = PhotoImage(file = "./images/autoStanding_pressed.png")
autoStandingDisabledImage = PhotoImage(file = "./images/autoStanding_disabled.png")
autoStandingButton = Button(
    image = autoStandingDisabledImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    anchor = "center",
    borderwidth=0,
    highlightthickness=0,
    command = autoStandingButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
autoStandingButton.config(state = "disabled")

autoSittingImage = PhotoImage(file = "./images/autoSitting.png")
autoSittingPressedImage = PhotoImage(file = "./images/autoSitting_pressed.png")
autoSittingDisabledImage = PhotoImage(file = "./images/autoSitting_disabled.png")
autoSittingButton = Button(
    image = autoSittingDisabledImage,
    bg = background,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = autoSittingButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
autoSittingButton.config(state = "disabled")

exitAutoImage = PhotoImage(file = "./images/exit.png")
exitAutoModeButton = Button(
    image = exitAutoImage,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = toggleAutoMode,
)

pauseButtonImage = PhotoImage(file = "./images/pause.png")
pauseButton = Button(
    image = pauseButtonImage,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = stop,
)
#TODO: pauseButton 애니메이션 추가하기
# pauseButton.bind(clicked, pauseButtonDidTap)
# pauseButton.bind(released, pauseButtonDidRelease)

infoLabelFont = font.Font(size = 20, weight = "bold")
infoLabel = Label(
    # text = f"{scanner.height} cm\n{scanner.weight} kg",
    font = infoLabelFont,
    justify = "right",
    background = background,
)

originalDimmedImage = Image.open("./images/dimmedView.png")
resizedDimmedImage = originalDimmedImage.resize((1024, 600), Image.LANCZOS)
dimmedImage = ImageTk.PhotoImage(resizedDimmedImage)
dimmedLabel = Label(
    image = dimmedImage,
)
dimmedLabel.bind(clicked, cancelScanQRMode)

scanLabel = Label(
    background = "black"
)

## MARK: -------- Window --------

setLayout()

window.title("ATLAS GUI")
window.geometry("1024x600")
window.configure(bg = background)
window.resizable(False, False)

scanner.start()
window.mainloop()

scanner.stop()
dcMotorPWM.stop()
linearPWM.stop()
GPIO.cleanup()
