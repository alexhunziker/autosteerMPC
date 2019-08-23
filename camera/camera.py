import threading
import time

from picamera import PiCamera
from picamera.array import PiRGBArray


class Camera(object):
    def __init__(self, resolution=(1280, 720), debug=False):
        self.camera = PiCamera()
        self.raw_capture = PiRGBArray(self.camera)
        self.camera.resolution = resolution
        self.camera.rotation = 180

        self.stop = False
        self.debug = debug

        # Camera measures exposure during this time
        time.sleep(2)

        measure_thread = threading.Thread(target=self.measure_loop)
        measure_thread.start()

    def measure_loop(self):
        while not self.stop:
            try:
                self.raw_capture = PiRGBArray(self.camera)
                self.camera.capture(self.raw_capture, format="bgr")
            except:
                print("WARN: Image capturing failed...")
                time.sleep(0.1)
                return None
            if self.debug:
                print(self.raw_capture.array)

    def stop(self):
        self.stop = True

    def retrieve_data(self):
        return self.raw_capture.array


if __name__ == "__main__":
    Camera(debug=True)
