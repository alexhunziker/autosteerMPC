from picamera import PiCamera
import time

class Camera(object):
    def __init__(self):
        self.camera = PiCamera()
        self.camera.resolution = (600, 800)
        self.camera.framerate = 40
        # Camera measures exposure during this time
        time.sleep(2)

    def capture(self):
        return self.camera.capture()