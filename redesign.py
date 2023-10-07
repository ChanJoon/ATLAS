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

## MARK: ------------ GPIO ------------

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 24V dc motor
dcMotorPWM = 19
dcMotorDIR = 6

# 12V linear actuator
linearPWM = 9
linearDIR = 26

# step motor L
# stepMotorLeftEN = 25
stepMotorLeftSTEP = 7
stepMotorLeftDIR = 8

# step motor R
# stepMotorRightEN  = 16
stepMotorRightSTEP = 23
stepMotorRightDIR = 24

# limit switch
dcMotorTopLimitSwitch = 17 # for ball skrew T
dcMotorBottomLimitSwitch = 22 # for ball skrew B
linearTopLimitSwitch = 27 # for rear profile
linearBottomLimitSwitch = 10 # for rear profile

M1 = 30 # 24V dc motor pwm
M2 = 30 # 12V linear actuator pwm
steps = 3200 # step motor steps
delay = 0.00001 # step motor delay

# GPIO setup
GPIO.setup(dcMotorPWM, GPIO.OUT)
GPIO.setup(dcMotorDIR, GPIO.OUT)
GPIO.setup(linearPWM, GPIO.OUT)
GPIO.setup(linearDIR, GPIO.OUT)
# GPIO.setup(stepMotorLeftEN, GPIO.OUT)
GPIO.setup(stepMotorLeftSTEP, GPIO.OUT)
GPIO.setup(stepMotorLeftDIR, GPIO.OUT)
# GPIO.setup(stepMotorRightEN, GPIO.OUT)
GPIO.setup(stepMotorRightSTEP, GPIO.OUT)
GPIO.setup(stepMotorRightDIR, GPIO.OUT)

GPIO.setup(dcMotorBottomLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dcMotorTopLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(linearTopLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(linearBottomLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# PWM setup
dcMotorPWM = GPIO.PWM(dcMotorPWM, 100)
dcMotorPWM.start(0)
linearPWM = GPIO.PWM(linearPWM, 100)
linearPWM.start(0)

if M1 > 80:
    M1 = 80

if M2 > 30:
    M2 = 30


def moveStepMotor(stepMotorSTEP, stepMotorDIR, direction=GPIO.HIGH):
    count = 0
    step_delay = 0.001
    while True:
        GPIO.output(stepMotorDIR, direction)
            
        GPIO.output(stepMotorSTEP, GPIO.HIGH)
        time.sleep(step_delay)
        GPIO.output(stepMotorSTEP, GPIO.LOW)
        time.sleep(step_delay)
        count += 1

        if count == 500:
            # GPIO.output(DIR, not GPIO.input(DIR))
            count = 0
            time.sleep(1)    
            break
    
## MARK: -------- QR Code --------

class qrCodeReceiver:
    def __init__(self):
        self.data = None
        self.connect_server()
        
    def connect_server(self):
        while True:
            try:
                self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_server.bind(("localhost", 1234))
                self.socket_server.listen(1)
                self.connection, self.address = self.socket_server.accept()
                break
            except ConnectionRefusedError:
                print("Connection refused. Retrying in 1 second...")
                time.sleep(1)
            except TimeoutError:
                print("Connection timeout. Retrying in 1 second...")
                time.sleep(1)
            except Exception as e:
                print(f"Error occurred: {str(e)}")
                break

    def receive_data(self):
        while True:
            self.data = self.connection.recv(1024).decode()
            if self.data is not None and self.data != "":
                print("Received data:", self.data)
            
# qr_receiver = qrCodeReceiver()

## MARK: -------- QRCodeScan --------

enableQRScan = False

class QRCodeScanner:
    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.ret, self.frame = self.camera.read()
        self.running = True
        self.frame = None
        self.resized_frame = None
        self.name = ""
        self.height = ""
        self.t0_dcMotor = 0
        self.t1_linearPWM = 0
        self.t2_dcMotor = 0

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
                    self.parse(qr_code_data)
                    dimmedLabel.place_forget()
                    scanLabel.place_forget()
                    toggleAutoMode()
                    return
                
                scan_frame = Image.fromarray(resized_frame)
                scan_image = ImageTk.PhotoImage(scan_frame)
                if enableQRScan:
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
            elif key == "t0":
                self.t0 = int(value)
            elif key == "t1":
                self.t1 = int(value)
            elif key == "t2":
                self.t2 = int(value)

        print("Name:", self.name)
        print("Height:", self.height)
        infoLabel.config(text = f"{scanner.name}\n{scanner.height} cm")
        

scanner = QRCodeScanner()

## MARK: -------- AutoController --------

class AutoController():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # 24V dc motor
        self.dcMotorPWM = 19
        self.dcMotorDIR = 6

        # 12V linear actuator
        self.linearPWM = 9
        self.linearDIR = 26

        # step motor L
        self.stepMotorLeftEN = 25
        self.stepMotorLeftSTEP = 8
        self.stepMotorLeftDIR = 7

        # step motor R
        self.stepMotorRightEN  = 16
        self.stepMotorRightSTEP = 20
        self.stepMotorRightDIR = 21

        # limit switch
        self.dcMotorTopLimitSwitch = 17 # for ball skrew T
        self.dcMotorBottomLimitSwitch = 22 # for ball skrew B
        self.linearTopLimitSwitch = 27 # for rear profile
        self.linearBottomLimitSwitch = 10 # for rear profile

        self.M1 = 30 # 24V dc motor pwm
        self.M2 = 30 # 12V linear actuator pwm
        self.steps = 3200 # step motor steps
        self.delay = 0.00001 # step motor delay

        # GPIO setup
        GPIO.setup(self.dcMotorPWM, GPIO.OUT)
        GPIO.setup(self.dcMotorDIR, GPIO.OUT)
        GPIO.setup(self.linearPWM, GPIO.OUT)
        GPIO.setup(self.linearDIR, GPIO.OUT)
        GPIO.setup(self.stepMotorLeftEN, GPIO.OUT)
        GPIO.setup(self.stepMotorLeftSTEP, GPIO.OUT)
        GPIO.setup(self.stepMotorLeftDIR, GPIO.OUT)
        GPIO.setup(self.stepMotorRightEN, GPIO.OUT)
        GPIO.setup(self.stepMotorRightSTEP, GPIO.OUT)
        GPIO.setup(self.stepMotorRightDIR, GPIO.OUT)
        GPIO.setup(self.dcMotorBottomLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.dcMotorTopLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.linearTopLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.linearBottomLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        if self.M1 > 80:
            self.M1 = 80

        if self.M2 > 30:
            self.M2 = 30
    def initiate(self):
        global dcMotorPWM, linearPWM
        temp = False
        temp2 = False
        while True:
            # DownButton: 볼스크류 하강
            if GPIO.input(self.dcMotorBottomLimitSwitch) != 1:
                GPIO.output(self.dcMotorDIR, GPIO.HIGH)
                dcMotorPWM.ChangeDutyCycle(self.M1)
            # TiltDownButton: 액츄에이터 틸트다운
            if GPIO.input(self.linearBottomLimitSwitch) != 1:
                GPIO.output(self.linearDIR, GPIO.LOW)
                linearPWM.ChangeDutyCycle(self.M2)
            
            if GPIO.input(self.dcMotorBottomLimitSwitch) == 1:
                # print("dc motor bottom limit switch")
                dcMotorPWM.ChangeDutyCycle(0)
                temp = True
            if GPIO.input(self.linearBottomLimitSwitch) == 1:
                print("linear bottom limit switch")
                linearPWM.ChangeDutyCycle(0)
                temp2 = True
                
            if temp and temp2:
                break
            
            # 볼스크류, 액츄에이터 초기화 후 체스트 가이드 넓히기
            # WidenButton
            # TODO: 스텝모터 엔코더 제어로 업데이트
            # GPIO.output(stepMotorLeftEN, GPIO.LOW)
            # GPIO.output(stepMotorRightEN, GPIO.LOW)
            # GPIO.output(stepMotorLeftDIR, GPIO.LOW)
            # GPIO.output(stepMotorRightDIR, GPIO.HIGH)
            # for _ in range(steps):
            #     GPIO.output(stepMotorLeftSTEP, GPIO.HIGH)
            #     GPIO.output(stepMotorRightSTEP, GPIO.HIGH)
            #     time.sleep(delay)
            #     GPIO.output(stepMotorLeftSTEP, GPIO.LOW)
            #     GPIO.output(stepMotorRightSTEP, GPIO.LOW)
            #     time.sleep(delay)
    
    def setting(self, stepMotorEncoderValue, t0_dcMotor, t1_linearPWM, t2_dcMotor):
        global dcMotorPWM
        ### i. 초기조정
        # TODO: 스텝모터 엔코더 제어로 어깨 간격 맞게 들어오기
        # UpButton: 볼스크류 상승
        GPIO.output(self.dcMotorDIR, GPIO.LOW)
        dcMotorPWM.ChangeDutyCycle(self.M1)
        print("setting > dcMotorPWM activated")
        for _ in range(t0_dcMotor):
            time.sleep(1.0)
            if GPIO.input(self.dcMotorTopLimitSwitch) == 1:
                break
        dcMotorPWM.ChangeDutyCycle(0)
        print(time.time())
        print("setting > dcMotorPWM deactivated")
        
        autoAdjustingButton.configure(image = adjustedImage, state = "disabled")
        autoStandingButton.config(image = autoStandingImage, state = "normal")
        autoSittingButton.config(image = autoSittingImage, state = "disabled")
        print("setting done")
        
    def stand(self, stepMotorEncoderValue, t0_dcMotor, t1_linearPWM, t2_dcMotor):
        global linearPWM, dcMotorPWM
        ### ii. 기립
        # TiltUpButton: 액츄에이터 틸트업
        GPIO.output(self.linearDIR, GPIO.HIGH)
        linearPWM.ChangeDutyCycle(self.M2)
        print("standing > linearPWM activated")
        for _ in range(t1_linearPWM):
            time.sleep(1.0)
            if GPIO.input(self.linearTopLimitSwitch) == 1:
                break
        linearPWM.ChangeDutyCycle(0)
        print("standing > linearPWM deactivated")
        # UpButton: 볼스크류 상승
        t2_start = time.time()
        GPIO.output(self.dcMotorDIR, GPIO.LOW)
        dcMotorPWM.ChangeDutyCycle(M1)
        print("standing > dcMotorPWM activated")
        for _ in range(t2_dcMotor):
            time.sleep(1.0)
            if GPIO.input(self.dcMotorTopLimitSwitch) == 1:
                break
        dcMotorPWM.ChangeDutyCycle(0)
        print("standing > dcMotorPWM deactivated")
        self.t2 = int(time.time() - t2_start) # 볼스크류 상승한 시간
        print("t2: ", self.t2)
        
        autoStandingButton.config(image = autoStandingImage, state = "disabled")
        autoSittingButton.config(image = autoSittingImage, state = "normal")
        print("standing done")
        
    def sit(self, stepMotorEncoderValue, t0_dcMotor, t1_linearPWM, t2_dcMotor):
        global dcMotorPWM, linearPWM
        ### iii. 착석
        # DownButton: 볼스크류 하강(상승한 만큼)
        GPIO.output(self.dcMotorDIR, GPIO.HIGH)
        dcMotorPWM.ChangeDutyCycle(M1)
        print("sitting > dcMotorPWM activated")
        print("t2:: ", self.t2)
        for _ in range(self.t2):
            time.sleep(1.0)
            if GPIO.input(dcMotorBottomLimitSwitch) == 1:
                break
        dcMotorPWM.ChangeDutyCycle(0)
        print("sitting > dcMotorPWM deactivated")
        # TiltDownButton: 액츄에이터 틸트다운
        GPIO.output(self.linearDIR, GPIO.LOW)
        linearPWM.ChangeDutyCycle(M2)
        print("sitting > linearPWM activated")
        for _ in range(5):
            #TODO 하강 시간체크
            time.sleep(1.0)
            if GPIO.input(self.linearBottomLimitSwitch) == 1:
                linearPWM.ChangeDutyCycle(0)
                break
            if GPIO.input(self.linearTopLimitSwitch) == 1:
                linearPWM.ChangeDutyCycle(0)
                break
        linearPWM.ChangeDutyCycle(0)
        print("sitting > linearPWM deactivated")
        self.initiate()
        toggleAutoMode()
        print("sitting done")
        

autoControl = AutoController()
        
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

# Button Methods

def widenButtonDidTap(event):
    widenButton.config(image = widenPressedImage)
    print("widenButtonDidTap")
    moveStepMotor(stepMotorLeftSTEP, stepMotorLeftDIR, GPIO.HIGH)
    moveStepMotor(stepMotorRightSTEP, stepMotorRightDIR, GPIO.HIGH)

def widenButtonIsPressing():
    print("widenButtonIsPressing")

def widenButtonDidRelease(event):
    widenButton.config(image = widenImage)
    print("widenButtonDidRelease") 
       

def narrowButtonDidTap(event):
    narrowButton.config(image = narrowPressedImage)
    print("narrowButtonDidTap")
    moveStepMotor(stepMotorLeftSTEP, stepMotorLeftDIR, GPIO.LOW)
    moveStepMotor(stepMotorRightSTEP, stepMotorRightDIR, GPIO.LOW)

def narrowButtonIsPressing():
    print("narrowButtonIsPressing")

def narrowButtonDidRelease(event):
    narrowButton.config(image = narrowImage)
    print("narrowButtonDidRelease")

def upButtonDidTap(event):
    print("upButtonDidTap")
    upButton.config(image = upPressedImage)
    if GPIO.input(dcMotorTopLimitSwitch) != 1:
        GPIO.output(dcMotorDIR, GPIO.LOW)
        dcMotorPWM.ChangeDutyCycle(M1)
    elif GPIO.input(dcMotorTopLimitSwitch) == 1:
        dcMotorPWM.ChangeDutyCycle(0)

def upButtonIsPressing():
    print("upButtonIsPressing")
    if GPIO.input(dcMotorTopLimitSwitch) == 1:
        dcMotorPWM.ChangeDutyCycle(0)

def upButtonDidRelease(event):
    print("upButtonDidRelease")
    upButton.config(image = upImage)
    dcMotorPWM.ChangeDutyCycle(0)

def downButtonDidTap(event):
    print("downButtonDidTap")
    downButton.config(image = downPressedImage)
    if GPIO.input(dcMotorBottomLimitSwitch) != 1:
        GPIO.output(dcMotorDIR, GPIO.HIGH)
        dcMotorPWM.ChangeDutyCycle(M1)
    elif GPIO.input(dcMotorBottomLimitSwitch) == 1:
        print("dcMotorBottomLimitSwitch Pressed")
        dcMotorPWM.ChangeDutyCycle(0)

def downButtonIsPressing():
    print("downButtonIsPressing")
    if GPIO.input(dcMotorBottomLimitSwitch) == 1:
        print("dcMotorBottomLimitSwitch Pressed")
        dcMotorPWM.ChangeDutyCycle(0)

def downButtonDidRelease(event):
    print("downButtonDidRelease")
    downButton.config(image = downImage)
    dcMotorPWM.ChangeDutyCycle(0)

def tiltUpButtonDidTap(event):
    print("tiltUpButtonDidTap")
    tiltUpButton.config(image = tiltUpPressedImage)
    if GPIO.input(linearTopLimitSwitch) != 1:
        GPIO.output(linearDIR, GPIO.LOW)
        linearPWM.ChangeDutyCycle(M2)
    elif GPIO.input(linearTopLimitSwitch) == 1:
        print("linearTopLimitSwitch Pressed")
        linearPWM.ChangeDutyCycle(0)

def tiltUpButtonIsPressing():
    print("tiltUpButtonIsPressing")
    if GPIO.input(linearTopLimitSwitch) == 1:
        print("linearTopLimitSwitch Pressed")
        linearPWM.ChangeDutyCycle(0)

def tiltUpButtonDidRelease(event):
    print("tiltUpButtonDidRelease")
    tiltUpButton.config(image = tiltUpImage)
    linearPWM.ChangeDutyCycle(0)

def tiltDownButtonDidTap(event):
    print("tiltDownButtonDidTap")
    tiltDownButton.config(image = tiltDownPressedImage)
    if GPIO.input(linearBottomLimitSwitch) != 1:
        GPIO.output(linearDIR, GPIO.HIGH)
        linearPWM.ChangeDutyCycle(M2)
    elif GPIO.input(linearBottomLimitSwitch) == 1:
        print("linearBottomLimitSwitch Pressed")
        linearPWM.ChangeDutyCycle(0)

def tiltDownButtonIsPressing():
    print("tiltDownButtonIsPressing")
    if GPIO.input(linearBottomLimitSwitch) == 1:
        print("linearBottomLimitSwitch Pressed")
        linearPWM.ChangeDutyCycle(0)

def tiltDownButtonDidRelease(event):
    print("tiltDownButtonDidRelease")
    tiltDownButton.config(image = tiltDownImage)
    linearPWM.ChangeDutyCycle(0)

def scanQRButtonDidTap():
    global enableQRScan
    print("scanQRButtonDidTap")
    dimmedLabel.place(relx = 0.5, rely = 0.5, width = 1024, height = 600, anchor = "center")
    scanLabel.place(relx=0.5, rely=0.5, width=400, height=400, anchor="center")
    enableQRScan = True


def autoStandingButtonDidTap(event):
    print("autoStandingButtonDidTap")
    autoStandingButton.config(image = autoStandingPressedImage)
    autoControl.stand(0, scanner.t0, scanner.t1, scanner.t2)

def autoStandingButtonIsPressing():
    print("autoStandingButtonIsPressing")

def autoStandingButtonDidRelease(event):
    print("autoStandingButtonDidRelease")
    autoStandingButton.config(image = autoStandingImage)

def autoSittingButtonDidTap(event):
    print("autoSittingButtonDidTap")
    autoSittingButton.config(image = autoSittingPressedImage)
    autoControl.sit(0, scanner.t0, scanner.t1, scanner.t2)

def autoSittingButtonIsPressing():
    print("autoSittingButtonIsPressing")

def autoSittingButtonDidRelease(event):
    print("autoSittingButtonDidRelease")
    autoSittingButton.config(image = autoSittingImage)

# Layout Methods

def setLayout():
    window.grid_columnconfigure(3, weight=1)
    
    widenButton.grid(row = 0, column = 0, padx = 30, pady = 30)
    narrowButton.grid(row = 0, column = 1)
    armpitLabel.grid(row = 0, column = 2, padx = 30)
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

# UI Methods

def enableManualButtons():
    armpitLabel.config(image = armpitImage)
    updownLabel.config(image = updownImage)
    tiltLabel.config(image = tiltImage)
    widenButton.config(image = widenImage, state = "normal")
    widenButton.bind(clicked, widenButtonDidTap)
    widenButton.bind(released, widenButtonDidRelease)
    narrowButton.config(image = narrowImage, state = "normal")
    narrowButton.bind(clicked, narrowButtonDidTap)
    narrowButton.bind(released, narrowButtonDidRelease)
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
    armpitLabel.config(image = armpitDisabledImage)
    updownLabel.config(image = updownDisabledImage)
    tiltLabel.config(image = tiltDisabledImage)
    widenButton.config(image = widenDisabledImage, state = "disabled")
    widenButton.unbind(clicked)
    widenButton.unbind(released)
    narrowButton.config(image = narrowDisabledImage, state = "disabled")
    narrowButton.unbind(clicked)
    narrowButton.unbind(released)
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
        exitAutoModeButton.grid_forget()
        infoLabel.grid_forget()
        enableManualButtons()
        disableAutoButtons()
        dcMotorPWM.ChangeDutyCycle(0)
        linearPWM.ChangeDutyCycle(0)
    else:
        exitAutoModeButton.grid(row = 0, column = 5, padx = 30)
        infoLabel.grid(row = 1, column = 4, sticky = "e")
        disableManualButtons()
        enableAutoButtons()
        autoAdjustingButton.config(image=autoAdjustingImage, state='normal')
        autoStandingButton.config(image = autoStandingImage, state = "disabled")
        autoSittingButton.config(image = autoSittingImage, state = "disabled")
    isAutoMode = not isAutoMode

def cancelScanQRMode(event):
    dimmedLabel.place_forget()
    scanLabel.place_forget()
    
def autoAdjusting():
    animateAdjustingGIF(0)
    autoControl.setting(0, scanner.t0, scanner.t1, scanner.t2)


def animateAdjustingGIF(frameIndex):
    autoAdjustingButton.configure(image=frames[frameIndex])
    window.after(500, lambda: animateAdjustingGIF((frameIndex + 1) % len(frames)))


## MARK: -------- UI Components --------

spacer1 = Label(bg = background)
spacer2 = Label(bg = background)
spacer3 = Label(bg = background)

armpitImage = PhotoImage(file = "armpit.png")
armpitDisabledImage = PhotoImage(file = "armpit_disabled.png")
armpitLabel = Label(
    image = armpitImage,
    background = background
)

updownImage = PhotoImage(file = "updown.png")
updownDisabledImage = PhotoImage(file = "updown_disabled.png")
updownLabel = Label(
    image = updownImage,
    background = background
)

tiltImage = PhotoImage(file = "tilt.png")
tiltDisabledImage = PhotoImage(file = "tilt_disabled.png")
tiltLabel = Label(
    image = tiltImage,
    background = background
)

widenImage = PhotoImage(file = "widen.png")
widenPressedImage = PhotoImage(file = "widen_pressed.png")
widenDisabledImage = PhotoImage(file = "widen_disabled.png")
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

narrowImage = PhotoImage(file = "narrow.png")
narrowPressedImage = PhotoImage(file = "narrow_pressed.png")
narrowDisabledImage = PhotoImage(file = "narrow_disabled.png")
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

upImage = PhotoImage(file = "up.png")
upPressedImage = PhotoImage(file = "up_pressed.png")
upDisabledImage = PhotoImage(file = "up_disabled.png")
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

downImage = PhotoImage(file = "down.png")
downPressedImage = PhotoImage(file = "down_pressed.png")
downDisabledImage = PhotoImage(file = "down_disabled.png")
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

tiltUpImage = PhotoImage(file = "tiltUp.png")
tiltUpPressedImage = PhotoImage(file = "tiltUp_pressed.png")
tiltUpDisabledImage = PhotoImage(file = "tiltUp_disabled.png")
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

tiltDownImage = PhotoImage(file = "tiltDown.png")
tiltDownPressedImage = PhotoImage(file = "tiltDown_pressed.png")
tiltDownDisabledImage = PhotoImage(file = "tiltDown_disabled.png")
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

scanQRImage = PhotoImage(file = "scanQR.png")
scanQRButton = Button(
    image = scanQRImage,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = scanQRButtonDidTap,
)

autoAdjustingImage = PhotoImage(file = "autoAdjusting.png")
autoAdjustingDisabledImage = PhotoImage(file = "autoAdjusting_disabled.png")
adjustingGIF = Image.open("adjusting.gif")
adjustedImage = PhotoImage(file = "adjusted.png")
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

frames = []
for frame in range(adjustingGIF.n_frames):
    adjustingGIF.seek(frame)
    resized_frame = adjustingGIF.resize((170, 170))
    frames.append(ImageTk.PhotoImage(resized_frame))

autoStandingImage = PhotoImage(file = "autoStanding.png")
autoStandingPressedImage = PhotoImage(file = "autoStanding_pressed.png")
autoStandingDisabledImage = PhotoImage(file = "autoStanding_disabled.png")
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

autoSittingImage = PhotoImage(file = "autoSitting.png")
autoSittingPressedImage = PhotoImage(file = "autoSitting_pressed.png")
autoSittingDisabledImage = PhotoImage(file = "autoSitting_disabled.png")
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

exitAutoImage = PhotoImage(file = "exit.png")
exitAutoModeButton = Button(
    image = exitAutoImage,
    width = buttonSize,
    height = buttonSize,
    borderwidth=0,
    highlightthickness=0,
    command = toggleAutoMode,
)

infoLabelFont = font.Font(size = 20, weight = "bold")
infoLabel = Label(
    text = f"{scanner.name}\n{scanner.height} cm",
    font = infoLabelFont,
    justify = "right",
    background = background,
)

originalDimmedImage = Image.open("dimmedView.png")
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

window.title("STD GUI")
window.geometry("1024x600")
window.configure(bg = background)
window.resizable(False, False)

scanner.start()
window.mainloop()

scanner.stop()
dcMotorPWM.stop()
linearPWM.stop()
GPIO.cleanup()
