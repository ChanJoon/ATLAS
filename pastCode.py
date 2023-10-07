from tkinter import *
import tkinter.font as font
'''
import RPi.GPIO as GPIO
import time
import socket
import threading
'''

'''
class qrCodeReceiver:
    def __init__(self):
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_server.bind(("localhost", 1234))
        self.socket_server.listen(1)
        self.connection, self.address = self.socket_server.accept()
        
        self.data = None

    def receive_data(self):
        while True:
            self.data = self.connection.recv(1024).decode()
            if self.data is not None and self.data != "":
                print("Received data:", self.data)
            
qr_receiver = qrCodeReceiver()
'''

window = Tk()
'''
window.attributes("-fullscreen", True)
window.bind("<F11>", lambda event: window.attributes("-fullscreen", not window.attributes("-fullscreen")))
window.bind("<Escape>", lambda event: window.attributes("-fullscreen", False))
'''

'''
## MARK: ------------ GPIO ------------

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 24V dc motor
dcMotorPWM = 18
dcMotorDIR = 23

# 12V linear actuator
linearPWM = 24
linearDIR = 25

# step motor L
stepMotorLeftEN = 8
stepMotorLeftSTEP = 7
stepMotorLeftDIR = 1

# step motor R
stepMotorRightEN  = 16
stepMotorRightSTEP = 20
stepMotorRightDIR = 21

# limit switch
dcMotorBottomLimitSwitch = 17 # for ball skrew B
dcMotorTopLimitSwitch = 27 # for ball skrew T
linearTopLimitSwitch = 22 # for rear profile

M1 = 30 # 24V dc motor pwm
M2 = 30 # 12V linear actuator pwm
steps = 3200 # step motor steps
delay = 0.00001 # step motor delay

# GPIO setup
GPIO.setup(dcMotorPWM, GPIO.OUT)
GPIO.setup(dcMotorDIR, GPIO.OUT)
GPIO.setup(linearPWM, GPIO.OUT)
GPIO.setup(linearDIR, GPIO.OUT)
GPIO.setup(stepMotorLeftEN, GPIO.OUT)
GPIO.setup(stepMotorLeftSTEP, GPIO.OUT)
GPIO.setup(stepMotorLeftDIR, GPIO.OUT)
GPIO.setup(stepMotorRightEN, GPIO.OUT)
GPIO.setup(stepMotorRightSTEP, GPIO.OUT)
GPIO.setup(stepMotorRightDIR, GPIO.OUT)

GPIO.setup(dcMotorBottomLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(dcMotorTopLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(linearTopLimitSwitch, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# PWM setup
dcMotorPWM = GPIO.PWM(dcMotorPWM, 100)
dcMotorPWM.start(0)
linearPWM = GPIO.PWM(linearPWM, 100)
linearPWM.start(0)

if M1 > 80:
    M1 = 80

if M2 > 30:
    M2 = 30
'''

## MARK: ------------ UI ------------

# Variables

isAutoMode = False

clicked = "<Button-1>"
released = "<ButtonRelease-1>"

buttonSize = 160

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

def widenButtonIsPressing():
    print("widenButtonIsPressing")
    '''
    GPIO.output(stepMotorLeftEN, GPIO.LOW)
    GPIO.output(stepMotorRightEN, GPIO.LOW)
    GPIO.output(stepMotorLeftDIR, GPIO.LOW)
    GPIO.output(stepMotorRightDIR, GPIO.HIGH)
    for _ in range(steps):
        GPIO.output(stepMotorLeftSTEP, GPIO.HIGH)
        GPIO.output(stepMotorRightSTEP, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(stepMotorLeftSTEP, GPIO.LOW)
        GPIO.output(stepMotorRightSTEP, GPIO.LOW)
        time.sleep(delay)
        '''

def widenButtonDidRelease(event):
    widenButton.config(image = widenImage)
    print("widenButtonDidRelease")    

def narrowButtonDidTap(event):
    narrowButton.config(image = narrowPressedImage)
    print("narrowButtonDidTap")

def narrowButtonIsPressing():
    print("narrowButtonIsPressing")
    '''
    GPIO.output(stepMotorLeftEN, GPIO.LOW)
    GPIO.output(stepMotorRightEN, GPIO.LOW)
    GPIO.output(stepMotorLeftDIR, GPIO.HIGH)
    GPIO.output(stepMotorRightDIR, GPIO.LOW)
    for _ in range(steps):
        GPIO.output(stepMotorLeftSTEP, GPIO.HIGH)
        GPIO.output(stepMotorRightSTEP, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(stepMotorLeftSTEP, GPIO.LOW)
        GPIO.output(stepMotorRightSTEP, GPIO.LOW)
        time.sleep(delay)
        '''

def narrowButtonDidRelease(event):
    narrowButton.config(image = narrowImage)
    print("narrowButtonDidRelease")

def upButtonDidTap(event):
    print("upButtonDidTap")
    upButton.config(image = upPressedImage)
    '''
    GPIO.output(dcMotorDIR, GPIO.LOW)
    dcMotorPWM.ChangeDutyCycle(M1)
    '''

def upButtonIsPressing():
    print("upButtonIsPressing")
    '''
    if GPIO.input(dcMotorTopLimitSwitch) == 1:
        dcMotorPWM.ChangeDutyCycle(0)
        '''

def upButtonDidRelease(event):
    print("upButtonDidRelease")
    upButton.config(image = upImage)
    '''
    dcMotorPWM.ChangeDutyCycle(0)
    '''

def downButtonDidTap(event):
    print("downButtonDidTap")
    downButton.config(image = downPressedImage)
    '''
    GPIO.output(dcMotorDIR, GPIO.HIGH)
    dcMotorPWM.ChangeDutyCycle(M1)
    '''

def downButtonIsPressing():
    print("downButtonIsPressing")
    '''
    if GPIO.input(dcMotorBottomLimitSwitch) == 1:
        print("dcMotorBottomLimitSwitch Pressed")
        dcMotorPWM.ChangeDutyCycle(0)
    '''

def downButtonDidRelease(event):
    print("downButtonDidRelease")
    downButton.config(image = downImage)
    '''
    dcMotorPWM.ChangeDutyCycle(0)
    '''

def tiltUpButtonDidTap(event):
    print("tiltUpButtonDidTap")
    tiltUpButton.config(image = tiltUpPressedImage)
    '''
    GPIO.output(linearDIR, GPIO.LOW)
    linearPWM.ChangeDutyCycle(M2)
    '''

def tiltUpButtonIsPressing():
    print("tiltUpButtonIsPressing")
    '''
    if GPIO.input(linearTopLimitSwitch) == 1:
        print("linearTopLimitSwitch Pressed")
        linearPWM.ChangeDutyCycle(0)
        '''

def tiltUpButtonDidRelease(event):
    print("tiltUpButtonDidRelease")
    tiltUpButton.config(image = tiltUpImage)
    '''
    linearPWM.ChangeDutyCycle(0)
    '''

def tiltDownButtonDidTap(event):
    print("tiltDownButtonDidTap")
    tiltDownButton.config(image = tiltDownPressedImage)
    '''
    GPIO.output(linearDIR, GPIO.HIGH)
    linearPWM.ChangeDutyCycle(M2)
    '''

def tiltDownButtonIsPressing():
    print("tiltDownButtonIsPressing")

def tiltDownButtonDidRelease(event):
    print("tiltDownButtonDidRelease")
    tiltDownButton.config(image = tiltDownImage)
    '''
    linearPWM.ChangeDutyCycle(0)
    '''

def autoStandingButtonDidTap(event):
    print("autoStandingButtonDidTap")
    autoStandingButton.config(image = autoStandingPressedImage)

def autoStandingButtonIsPressing():
    print("autoStandingButtonIsPressing")

def autoStandingButtonDidRelease(event):
    print("autoStandingButtonDidRelease")
    autoStandingButton.config(image = autoStandingImage)

def autoSittingButtonDidTap(event):
    print("autoSittingButtonDidTap")
    autoSittingButton.config(image = autoSittingPressedImage)

def autoSittingButtonIsPressing():
    print("autoSittingButtonIsPressing")

def autoSittingButtonDidRelease(event):
    print("autoSittingButtonDidRelease")
    autoSittingButton.config(image = autoSittingImage)

# UI Methods

def forgetHandButtons():
    leftHandButton.place_forget()
    rightHandButton.place_forget()
    allHandsButton.place_forget()

def setLeftHandMode():
    forgetHandButtons()
    placeManualUI(
        leftFirstColumnX,
        leftSecondColumnX
    )
    allHandsButton.place(x = leftModeHandsButtonX1, y = handsButtonY, width = handsButtonSize, height = handsButtonSize)
    rightHandButton.place(x = leftModeHandsButtonX2, y = handsButtonY, width = handsButtonSize, height = handsButtonSize)

def setrightHandMode():
    forgetHandButtons()
    placeManualUI(
        rightFirstColumnX,
        rightSecondColumnX
    )
    leftHandButton.place(x = rightModeHandsButtonX1, y = handsButtonY, width = handsButtonSize, height = handsButtonSize)
    allHandsButton.place(x = rightModeHandsButtonX2, y = handsButtonY, width = handsButtonSize, height = handsButtonSize)

def setAllHandsMode():
    forgetHandButtons()
    placeManualUI(
        leftFirstColumnX,
        rightSecondColumnX
    )
    leftHandButton.place(x = allModeHandsButtonX1, y = handsButtonY, width = handsButtonSize, height = handsButtonSize)
    rightHandButton.place(x = allModeHandsButtonX2, y = handsButtonY, width = handsButtonSize, height = handsButtonSize)

def toggleAutoMode():
    global isAutoMode
    if isAutoMode:
        forgetAutoUI()
        setManualMode()
        qr_receiver.data = None
    else:
        forgetManualUI()
        setAutoMode()
    isAutoMode = not isAutoMode

def placeManualUI(controlButtonX1, controlButtonX2):
    armpitLabel.place(x = 432, y = 60.25)
    updownLabel.place(x = 455, y = 220)
    tiltLabel.place(x = 440, y = 410)
    
    widenButton.place(x = controlButtonX1, y = firstRowY, width = buttonSize, height = buttonSize)
    narrowButton.place(x = controlButtonX2, y = firstRowY, width = buttonSize, height = buttonSize)
    upButton.place(x = controlButtonX1, y = secondRowY, width = buttonSize, height = buttonSize)
    downButton.place(x = controlButtonX2, y = secondRowY, width = buttonSize, height = buttonSize)
    tiltUpButton.place(x = controlButtonX1, y = thirdRowY, width = buttonSize, height = buttonSize)
    tiltDownButton.place(x = controlButtonX2, y = thirdRowY, width = buttonSize, height = buttonSize)
    ## TODO: Remove
    autoModeButton.place(relx = 0.5, rely = 0.9, anchor = "center", width = 160, height = 20)

def forgetManualUI():
    armpitLabel.place_forget()
    updownLabel.place_forget()
    tiltLabel.place_forget()
    widenButton.place_forget()
    narrowButton.place_forget()
    upButton.place_forget()
    downButton.place_forget()
    tiltUpButton.place_forget()
    tiltDownButton.place_forget()
    forgetHandButtons()
    ## TODO: Remove
    autoModeButton.place_forget()

def placeAutoUI():
    autoStandingButton.place(x = 50, y = 125, width = 350, height = 350)
    autoSittingButton.place(x = 624, y = 125, width = 350, height = 350)
    exitAutoModeButton.place(x = 20, y = 20, width = 100, height = 100)
    autoStandingLabel.place(x = 185, y = 485)
    autoSittingLabel.place(x = 764, y = 485)
    nameLabel.place(relx = 0.5, rely = 0.45, anchor = "center")
    heightLabel.place(relx = 0.5, rely = 0.55, anchor = "center")

def forgetAutoUI():
    autoStandingButton.place_forget()
    autoSittingButton.place_forget()
    exitAutoModeButton.place_forget()
    autoStandingLabel.place_forget()
    autoSittingLabel.place_forget()
    nameLabel.place_forget()
    heightLabel.place_forget()

def setManualMode():
    forgetAutoUI()
    setAllHandsMode()

def setAutoMode():
    forgetManualUI()
    placeAutoUI()

def updateInfoLabels(name, height):
    nameLabel.config(text = name)
    heightLabel.config(text = height)
    nameLabel.place(relx = 0.5, rely = 0.45, anchor = "center")
    heightLabel.place(relx = 0.5, rely = 0.55, anchor = "center")

'''
def checkQRcode():
    global isAutoMode
    if qr_receiver.data:
        isAutoMode = True
    elif qr_receiver.data is None and qr_receiver.data == "":
        isAutoMode = False
        
    if isAutoMode:
        forgetManualUI()
        setAutoMode()
    else:
        forgetAutoUI()
        setManualMode()
    window.after(100, checkQRcode)
'''
    
# Labels

armpitImage = PhotoImage(file = "armpit.png")
armpitLabel = Label(
    image = armpitImage,
    background = background
)
armpitLabel.place(
    x = 432,
    y = 60.25,
)

updownImage = PhotoImage(file = "updown.png")
updownLabel = Label(
    image = updownImage,
    background = background
)
updownLabel.place(
    x = 455,
    y = 220,
)

tiltImage = PhotoImage(file = "tilt.png")
tiltLabel = Label(
    image = tiltImage,
    background = background
)
tiltLabel.place(
    x = 440,
    y = 410,
)

# Buttons

widenImage = PhotoImage(file = "armpit_widen.png")
widenPressedImage = PhotoImage(file = "armpit_widen_pressed.png")
widenButton = Button(
    image = widenImage,
    borderwidth=0,
    highlightthickness=0,
    command = widenButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval,
)
widenButton.place(
    x=leftFirstColumnX,
    y=firstRowY,
    width=buttonSize,
    height=buttonSize
)
widenButton.bind(clicked, widenButtonDidTap)
widenButton.bind(released, widenButtonDidRelease)

narrowImage = PhotoImage(file = "armpit_narrow.png")
narrowPressedImage = PhotoImage(file = "armpit_narrow_pressed.png")
narrowButton = Button(
    image = narrowImage,
    borderwidth=0,
    highlightthickness=0,
    command = narrowButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
narrowButton.place(
    x=rightSecondColumnX,
    y=firstRowY,
    width=buttonSize,
    height=buttonSize
)
narrowButton.bind(clicked, narrowButtonDidTap)
narrowButton.bind(released, narrowButtonDidRelease)

upImage = PhotoImage(file = "up.png")
upPressedImage = PhotoImage(file = "up_pressed.png")
upButton = Button(
    image = upImage,
    borderwidth=0,
    highlightthickness=0,
    command = upButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
upButton.place(
    x=leftFirstColumnX,
    y=secondRowY,
    width=buttonSize,
    height=buttonSize
)
upButton.bind(clicked, upButtonDidTap)
upButton.bind(released, upButtonDidRelease)

downImage = PhotoImage(file = "down.png")
downPressedImage = PhotoImage(file = "down_pressed.png")
downButton = Button(
    image = downImage,
    borderwidth=0,
    highlightthickness=0,
    command = downButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
downButton.place(
    x=rightSecondColumnX,
    y=secondRowY,
    width=buttonSize,
    height=buttonSize
)
downButton.bind(clicked, downButtonDidTap)
downButton.bind(released, downButtonDidRelease)

tiltUpImage = PhotoImage(file = "tiltUp.png")
tiltUpPressedImage = PhotoImage(file = "tiltUp_pressed.png")
tiltUpButton = Button(
    image = tiltUpImage,
    borderwidth=0,
    highlightthickness=0,
    command = tiltUpButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
tiltUpButton.place(
    x=leftFirstColumnX,
    y=thirdRowY,
    width=buttonSize,
    height=buttonSize
)
tiltUpButton.bind(clicked, tiltUpButtonDidTap)
tiltUpButton.bind(released, tiltUpButtonDidRelease)

tiltDownImage = PhotoImage(file = "tiltDown.png")
tiltDownPressedImage = PhotoImage(file = "tiltDown_pressed.png")
tiltDownButton = Button(
    image = tiltDownImage,
    borderwidth=0,
    highlightthickness=0,
    command = tiltDownButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
tiltDownButton.place(
    x=rightSecondColumnX,
    y=thirdRowY,
    width=buttonSize,
    height=buttonSize
)
tiltDownButton.bind(clicked, tiltDownButtonDidTap)
tiltDownButton.bind(released, tiltDownButtonDidRelease)

leftHandImage = PhotoImage(file = "leftHand.png")
leftHandPressedImage = PhotoImage(file = "leftHand_pressed.png")
leftHandButton = Button(
    image = leftHandImage,
    borderwidth=0,
    highlightthickness=0,
    command = setLeftHandMode
)
leftHandButton.place(
    x=allModeHandsButtonX1,
    y=handsButtonY,
    width=handsButtonSize,
    height=handsButtonSize
)

rightHandImage = PhotoImage(file = "rightHand.png")
rightHandPressedImage = PhotoImage(file = "rightHand_pressed.png")
rightHandButton = Button(
    image = rightHandImage,
    borderwidth=0,
    highlightthickness=0,
    command = setrightHandMode
)
rightHandButton.place(
    x=allModeHandsButtonX2,
    y=handsButtonY,
    width=handsButtonSize,
    height=handsButtonSize
)

# Hands UI

allHandsImage = PhotoImage(file = "allHands.png")
allHandsPressedImage = PhotoImage(file = "allHands_pressed.png")
allHandsButton = Button(
    image = allHandsImage,
    borderwidth=0,
    highlightthickness=0,
    command = setAllHandsMode
)

# Auto UI

autoStandingImage = PhotoImage(file = "autoStanding.png")
autoStandingPressedImage = PhotoImage(file = "autoStanding_pressed.png")
autoStandingButton = Button(
    image = autoStandingImage,
    borderwidth=0,
    highlightthickness=0,
    command = autoStandingButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
autoStandingButton.bind(clicked, autoStandingButtonDidTap)
autoStandingButton.bind(released, autoStandingButtonDidRelease)

autoSittingImage = PhotoImage(file = "autoSitting.png")
autoSittingPressedImage = PhotoImage(file = "autoSitting_pressed.png")
autoSittingButton = Button(
    image = autoSittingImage,
    borderwidth=0,
    highlightthickness=0,
    command = autoSittingButtonIsPressing,
    repeatdelay = repeatdelay,
    repeatinterval = repeatinterval
)
autoSittingButton.bind(clicked, autoSittingButtonDidTap)
autoSittingButton.bind(released, autoSittingButtonDidRelease)

exitImage = PhotoImage(file = "exit.png")
exitAutoModeButton = Button(
    image = exitImage,
    borderwidth=0,
    highlightthickness=0,
    command = toggleAutoMode,
)

autoLabelFont = font.Font(size=40)
autoStandingLabel = Label(
    text = "기립",
    font = autoLabelFont,
    background = background
)
autoSittingLabel = Label(
    text = "착석",
    font = autoLabelFont,
    background = background
)

infoLabelFont = font.Font(size = 40, weight = "bold")
nameLabel = Label(
    text = "\(name)",
    font = infoLabelFont,
    background = background
)
heightLabel = Label(
    text = "\(height)",
    font = infoLabelFont,
    background = background
)

## TODO: Remove
autoModeButton = Button(
    text = "Toggle Auto Mode",
    command = toggleAutoMode
)
autoModeButton.place(
    relx = 0.5,
    rely = 0.9,
    anchor = "center",
    width = 160,
    height = 20
)

# window
window.title("STD GUI")
window.geometry("1024x600")
window.configure(bg = background)
window.resizable(False, False)
'''
receive_thread = threading.Thread(target=qr_receiver.receive_data)
receive_thread.start()
checkQRcode()
'''
window.mainloop()
