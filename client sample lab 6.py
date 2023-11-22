# python 3 code
import math
import socket
from time import *
from pynput import keyboard

"""pynput: On Mac OSX, one of the following must be true:
* The process must run as root. OR
* Your application must be white listed under Enable access for assistive devices. Note that this might require that you package your application, since otherwise the entire Python installation must be white listed."""
import sys
import threading
import enum
import urllib.request
import cv2
import numpy
import copy

socketLock = threading.Lock()
imageLock = threading.Lock()

IP_ADDRESS = "192.168.1.103"  # SET THIS TO THE RASPBERRY PI's IP ADDRESS
RESIZE_SCALE = 2  # try a larger value if your computer is running slow.
ENABLE_ROBOT_CONNECTION = True #change to True to use the robot


# You should fill this in with your states
class States(enum.Enum):
    LISTEN = enum.auto()
    CENTER = enum.auto()
    ABOVE = enum.auto()
    BELOW = enum.auto()
    LEFT = enum.auto()
    RIGHT = enum.auto()
    BELOW_RIGHT = enum.auto()
    BELOW_LEFT = enum.auto()
    NO = enum.auto()
    DONE = enum.auto()
    SPIN = enum.auto()
    DRIVE = enum.auto()


class StateMachine(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)  # MUST call this to make sure we setup the thread correctly
        # CONFIGURATION PARAMETERS
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.CONTROLLER_PORT = 5001
        self.TIMEOUT = 10  # If its unable to connect after 10 seconds, give up.  Want this to be a while so robot can init.
        self.STATE = States.LISTEN
        self.RUNNING = True
        self.DIST = False
        self.video = ImageProc()
        # Start video
        self.video.start()

        self.threshX = 100
        self.threshY = 76.67

        self.direction = True
        self.numCones = 0
        self.begin = True

        self.initialX = -1
        self.initialY = -1
        self.newX = -1
        self.newY = -1

        self.robotAngle = -1
        self.clickAngle = -1

        self.counter = 0.0
        self.spinCounter = 0

        # default option in which the robot goes towards an object (colors must be set manually)
        self.option = 1

        # connect to the motorcontroller
        try:
            with socketLock:
                self.sock = socket.create_connection((self.IP_ADDRESS, self.CONTROLLER_PORT), self.TIMEOUT)
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Connected to RP")
        except Exception as e:
            print("ERROR with socket connection", e)
            sys.exit(0)

        # connect to the robot
        """ The i command will initialize the robot.  It enters the create into FULL mode which means it can drive off tables and over steps: be careful!"""
        if ENABLE_ROBOT_CONNECTION:
            with socketLock:
                self.sock.sendall("i /dev/ttyUSB0".encode())
                print("Sent command")
                result = self.sock.recv(128)
                print(result)
                if result.decode() != "i /dev/ttyUSB0":
                    self.RUNNING = False

        self.sensors = Sensing(self.sock)
        # Start getting data
        if ENABLE_ROBOT_CONNECTION:
            self.sensors.start()

        # Collect events until released
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

    def run(self):

        # BEGINNING OF THE CONTROL LOOP
        while (self.RUNNING):
            sleep(0.1)
            video = self.video
            if self.STATE == States.LISTEN:
                if video.wasClicked:
                    self.initialX = video.robotCenterX
                    self.initialY = video.robotCenterY
                    self.STATE = States.SPIN
                    initLength = abs(math.hypot(video.robotCenterY - video.centerY, video.robotCenterX - video.centerX))
            if self.STATE == States.SPIN:
                self.sock.sendall("a spin_left(50)".encode())
                self.sock.recv(128).decode()
                sleep(.5)
                self.sock.sendall("a spin_left(0)".encode())
                self.sock.recv(128).decode()

                length = abs(math.hypot(video.robotCenterY - video.centerY, video.robotCenterX - video.centerX))
                if (length < initLength):
                    initLength = length
                    self.counter = self.spinCounter
                self.spinCounter = self.spinCounter + 1

                if self.spinCounter >= 26:
                    self.sock.sendall("a spin_left(50)".encode())
                    self.sock.recv(128).decode()
                    sleep(self.counter/2)
                    self.STATE = States.DRIVE
            if self.STATE == States.DRIVE:
                self.sock.sendall("a drive_straight(50)".encode())
                self.sock.recv(128).decode()

                if (video.robotCenterX - 20 > video.centerX and video.robotCenterX + 20 < video.centerX):
                    if (video.robotCenterY - 20 > video.centerY and video.robotCenterY + 20 < video.centerY):
                        self.STATE = States.DONE
            if self.STATE == States.DONE:
                self.sock.sendall("a drive_straight(0)".encode())
                self.sock.recv(128).decode()
                sleep(50)



                """
                self.sock.sendall("a drive_straight(50)".encode())
                self.sock.recv(128).decode()
                sleep(1)
                self.sock.sendall("a drive_straight(0)".encode())
                self.sock.recv(128).decode()
                self.newX = video.robotCenterX
                self.newY = video.robotCenterY
                self.robotAngle = math.atan2((self.newY-self.initialY), (self.newX - self.initialX))
                self.clickAngle = math.atan2((video.centerY-self.newY), (video.centerX - self.newX))
                print(self.clickAngle)
                print(self.robotAngle)
                video.wasClicked = False
                """



        # END OF CONTROL LOOP

        # First stop any other threads talking to the robot
        self.sensors.RUNNING = False
        self.video.RUNNING = False

        sleep(1)  # Wait for threads to wrap up

        # Need to disconnect
        """ The c command stops the robot and disconnects.  The stop command will also reset the Create's mode to a battery safe PASSIVE.  It is very important to use this command!"""
        with socketLock:
            self.sock.sendall("c".encode())
            print(self.sock.recv(128))
            self.sock.close()

        # If the user didn't request to halt, we should stop listening anyways
        self.listener.stop()

        # self.sensors.join()
        # self.video.join()

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == 'q':
                # Stop listener
                self.RUNNING = False
                self.sensors.RUNNING = False
                self.video.RUNNING = False
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == '1':
                # Set it to option 1
                self.option = 1
                self.STATE = States.LISTEN
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == '2':
                # Set it to option 2
                self.option = 2
                self.direction = True
                self.STATE = States.LISTEN
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_press(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        try:
            print('alphanumeric key {0} pressed'.format(key.char))
            if key.char == '3':
                # Set it to option 3
                self.option = 3
                self.direction = True
                self.begin = True
                self.STATE = States.LISTEN
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def on_release(self, key):
        # WARNING: DO NOT attempt to use the socket directly from here
        print('{0} released'.format(key))
        if key == keyboard.Key.esc or key == keyboard.Key.ctrl:
            # Stop listener
            self.RUNNING = False
            self.sensors.RUNNING = False
            self.video.RUNNING = False
            return False


# END OF STATEMACHINE


class Sensing(threading.Thread):
    def __init__(self, socket):
        threading.Thread.__init__(self)  # MUST call this to make sure we setup the thread correctly
        self.RUNNING = True
        self.sock = socket

    def run(self):
        while self.RUNNING:
            sleep(1)
            # This is where I would get a sensor update
            # Store it in this class
            # You can change the polling frequency to optimize performance, don't forget to use socketLock
            with socketLock:
                self.sock.sendall("a distance".encode())
                print(self.sock.recv(128))


# END OF SENSING

class ImageProc(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)  # MUST call this to make sure we setup the thread correctly
        global IP_ADDRESS
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = 8081
        self.RUNNING = True
        self.latestImg = []
        self.feedback = []
        self.thresholds = {'low_hue': 186, 'high_hue': 278, 'low_sat': 113, 'high_sat': 242, 'low_val': 74,
                           'high_val': 133}
        self.dict = {"oCone": [166, 230, 144, 255, 0, 22], "gCone": [0, 289, 31, 241, 45, 66],
                     "yCone": [139, 360, 110, 227, 13, 42], "gBall": [108, 280, 0, 181, 63, 105],
                     "oBall": [121,360,125,255,0,16], "bluePaper": [186, 278, 113, 242, 74, 133]}
        self.centerY = -1
        self.centerX = -1
        self.backY = 0
        self.backX = 0
        self.radius = -1
        #ADDED
        self.cam = cv2.VideoCapture(0)
        self.robotLeft = -1
        self.robotTop = -1
        self.robotBottom = -1
        self.robotRight = -1
        self.wasClicked = False
        self.robotCenterX = -1
        self.robotCenterY = -1



    def run(self):
        url = "http://" + self.IP_ADDRESS + ":" + str(self.PORT)
        #stream = urllib.request.urlopen(url)
        #ADDED
        #retValue, image = self.cam.read()
        while (self.RUNNING):
            sleep(0.1)
            """bytes = b''
            while self.RUNNING:
                bytes += stream.read(8192)  # image size is about 40k bytes, so this loops about 5 times
                a = bytes.find(b'\xff\xd8')
                b = bytes.find(b'\xff\xd9')
                if a > b:
                    bytes = bytes[b + 2:]
                    continue
                if a != -1 and b != -1:
                    jpg = bytes[a:b + 2]
                    # bytes= bytes[b+2:]
                    # print("found image", a, b, len(bytes))
                    break
            img = cv2.imdecode(numpy.frombuffer(jpg, dtype=numpy.uint8), cv2.IMREAD_COLOR)
            # Resize to half size so that image processing is faster
            img = cv2.resize(img, ((int)(len(img[0]) / RESIZE_SCALE), (int)(len(img) / RESIZE_SCALE)))
            """
            retValue, img = self.cam.read()
            """if not img:
                sleep(1)
                continue"""
            with imageLock:
                self.latestImg = copy.deepcopy(img)  # Make a copy not a reference

            masked = self.doImgProc()  # pass by reference for all non-primitve types in Python

            # after image processing you can update here to see the new version
            with imageLock:
                self.feedback = copy.deepcopy(masked)

    def setThresh(self, name, value):
        self.thresholds[name] = value

    def doImgProc(self):
        low = (self.thresholds['low_val'], self.thresholds['low_sat'], self.thresholds['low_hue'])
        high = (self.thresholds['high_val'], self.thresholds['high_sat'], self.thresholds['high_hue'])
        # theMask = cv2.inRange(self.latestImg, low, high)

        # TODO: Work here
        hsvImage = cv2.cvtColor(self.latestImg, cv2.COLOR_BGR2HSV)
        hsvMask = cv2.inRange(hsvImage, low, high)
        updatedImage = cv2.bitwise_and(hsvImage, hsvImage, mask=hsvMask)
        colorImage = cv2.cvtColor(updatedImage, cv2.COLOR_HSV2RGB)
        bwImage = cv2.cvtColor(colorImage, cv2.COLOR_RGB2GRAY)
        kernel = numpy.ones((3, 3), numpy.uint8)
        erodedImage = cv2.erode(bwImage, kernel, iterations=1)
        dilatedImage = cv2.dilate(erodedImage, kernel, iterations=1)
        numlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(dilatedImage)
        try:
            self.robotLeft = int(stats[1, cv2.CC_STAT_LEFT])
            self.robotTop = int(stats[1, cv2.CC_STAT_TOP])
            self.robotBottom = int(stats[1, cv2.CC_STAT_TOP] + stats[1, cv2.CC_STAT_HEIGHT])
            self.robotRight = int(stats[1, cv2.CC_STAT_LEFT] + stats[1, cv2.CC_STAT_WIDTH])


            cv2.rectangle(self.latestImg,(self.robotLeft, self.robotTop),(self.robotRight, self.robotBottom),(255, 0, 255), 1)
            self.radius = int(stats[1, cv2.CC_STAT_WIDTH] / 2)

            self.robotCenterX = int(centroids[1][0])
            self.robotCenterY = int(centroids[1][1])
        except:
            self.robotTop = -1
            self.robotLeft = -1
            self.robotBottom = -1
            self.robotRight = -1
            self.robotCenterY = -1
            self.robotCenterX = -1

        self.backX = int(centroids[0][0])
        self.backY = int(centroids[0][1])
        with imageLock:
            if self.wasClicked:
                cv2.circle(self.latestImg, (self.centerX, self.centerY), 15, (255, 0, 255), 5)

        # test with width and height
        # self.backX = stats[0, cv2.CC_STAT_WIDTH]
        # self.backY = stats[0, cv2.CC_STAT_HEIGHT]

        # END TODO
        # return cv2.bitwise_and(self.latestImg, self.latestImg, mask=theMask)
        # return cv2.bitwise_and(hsvImage, hsvImage, mask=hsvMask)
        return dilatedImage
    def click(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.centerX = x
            self.centerY = y
            self.wasClicked = True


# END OF IMAGEPROC


if __name__ == "__main__":

    cv2.namedWindow("Create View", flags=cv2.WINDOW_AUTOSIZE)
    cv2.moveWindow("Create View", 21, 21)

    cv2.namedWindow('sliders')
    cv2.moveWindow('sliders', 680, 21)

    sm = StateMachine()
    sm.start()

    # Probably safer to do this on the main thread rather than in ImgProc init
    cv2.createTrackbar('low_hue', 'sliders', sm.video.thresholds['low_hue'], 360,
                       lambda x: sm.video.setThresh('low_hue', x))
    cv2.createTrackbar('high_hue', 'sliders', sm.video.thresholds['high_hue'], 360,
                       lambda x: sm.video.setThresh('high_hue', x))

    cv2.createTrackbar('low_sat', 'sliders', sm.video.thresholds['low_sat'], 255,
                       lambda x: sm.video.setThresh('low_sat', x))
    cv2.createTrackbar('high_sat', 'sliders', sm.video.thresholds['high_sat'], 255,
                       lambda x: sm.video.setThresh('high_sat', x))

    cv2.createTrackbar('low_val', 'sliders', sm.video.thresholds['low_val'], 255,
                       lambda x: sm.video.setThresh('low_val', x))
    cv2.createTrackbar('high_val', 'sliders', sm.video.thresholds['high_val'], 255,
                       lambda x: sm.video.setThresh('high_val', x))
    cv2.setMouseCallback("Create View", sm.video.click, sm.video)

    while (len(sm.video.latestImg) == 0) or (len(sm.video.feedback) == 0):
        sleep(1)

    while (sm.RUNNING):
        with imageLock:
            cv2.imshow("Create View", sm.video.latestImg)
            cv2.imshow("sliders", sm.video.feedback)
        cv2.waitKey(5)
    sm.video.cam.release()
    cv2.destroyAllWindows()

    sleep(1)

    # sm.join()