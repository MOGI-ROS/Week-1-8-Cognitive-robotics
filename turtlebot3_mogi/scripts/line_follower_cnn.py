#!/usr/bin/env python3

from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from tensorflow.compat.v1 import InteractiveSession
from tensorflow.compat.v1 import ConfigProto
from tensorflow.keras import __version__ as keras_version
import tensorflow as tf

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import rospy
import rospkg
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np
import h5py
import time

# Set image size
image_size = 24

# Initialize Tensorflow session
config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

# Initialize ROS node and get CNN model path
rospy.init_node('line_follower')

rospack = rospkg.RosPack()
path = rospack.get_path('turtlebot3_mogi')
model_path = path + "/network_model/model.best.h5"

print("[INFO] Version:")
print("OpenCV version: %s" % cv2.__version__)
print("Tensorflow version: %s" % tf.__version__)
keras_version = str(keras_version).encode('utf8')
print("Keras version: %s" % keras_version)
print("CNN model: %s" % model_path)
f = h5py.File(model_path, mode='r')
model_version = f.attrs.get('keras_version')
print("Model's Keras version: %s" % model_version)

if model_version != keras_version:
    print('You are using Keras version ', keras_version, ', but the model was built using ', model_version)

# Finally load model:
model = load_model(model_path)

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.image = None

        # Initialize published Twist message
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.last_time = time.time()

    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while True:
            self.image = self.queue.get()

            # Process the current image
            mask = self.processImage(self.image)

            # Add processed images as small images on top of main image
            result = self.addSmallPictures(self.image, [mask])
            cv2.imshow("frame", result)

            # Check for 'q' key to exit
            k = cv2.waitKey(1) & 0xFF
            if k in [27, ord('q')]:
                # Stop every motion
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                pub.publish(self.cmd_vel)
                # Quit
                rospy.signal_shutdown('Quit')

    def processImage(self, img):

        image = cv2.resize(img, (image_size, image_size))
        image = img_to_array(image)
        image = np.array(image, dtype="float") / 255.0

        image = image.reshape(-1, image_size, image_size, 3)
        
        with tf.device('/gpu:0'):
            prediction = np.argmax(model(image, training=False))
                
        print("Prediction %d, elapsed time %.3f" % (prediction, time.time()-self.last_time))
        self.last_time = time.time()

        if prediction == 0: # Forward
            self.cmd_vel.angular.z = 0
            self.cmd_vel.linear.x = 0.1
        elif prediction == 1: # Left
            self.cmd_vel.angular.z = -0.2
            self.cmd_vel.linear.x = 0.05
        elif prediction == 2: # Right
            self.cmd_vel.angular.z = 0.2
            self.cmd_vel.linear.x = 0.05
        else: # Nothing
            self.cmd_vel.angular.z = 0.1
            self.cmd_vel.linear.x = 0.0

        # Publish cmd_vel
        pub.publish(self.cmd_vel)
        
        # Return processed frames
        return cv2.resize(img, (image_size, image_size))

    # Add small images to the top row of the main image
    def addSmallPictures(self, img, small_images, size=(160, 120)):
        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img

def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        #cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # in case of non-compressed image stream only
        cv2Img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)


queueSize = 1      
qMono = BufferQueue(queueSize)

bridge = CvBridge()

# Define your image topic
image_topic = "/camera/image/compressed"
# Set up your subscriber and define its callback
rospy.Subscriber(image_topic, CompressedImage, queueMonocular)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Start image processing thread
cvThreadHandle = cvThread(qMono)
cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

# Spin until Ctrl+C
rospy.spin()