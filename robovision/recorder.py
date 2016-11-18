import cv2
import logging
import os
from datetime import datetime
from threading import Thread, Event
from queue import Empty
from managed_threading import ManagedThread
from datetime import datetime
logger = logging.getLogger("recorder")

class Recorder(ManagedThread):
    def on_enabled(self, first_frame, *args):
        self.then = datetime.now()
        self.path = os.path.expanduser("~/bot-%s.avi") % datetime.now().strftime("%Y%m%d%H%M%S")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        height, width, bytes_per_pixel = first_frame.shape
        assert bytes_per_pixel == 4
        self.writer = cv2.VideoWriter(self.path, fourcc, 30, (width, height))
        logger.info("Recorder saving video to %s", self.path)

    def step(self, frame, *args):
        self.writer.write(frame)

    def on_disabled(self):
        self.writer.release()
        logger.info("Recorder saved %s of video to %to:", datetime.now()-self.then, self.path)

