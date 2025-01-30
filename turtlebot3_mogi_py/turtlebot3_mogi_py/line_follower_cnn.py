import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from tensorflow.compat.v1 import InteractiveSession
from tensorflow.compat.v1 import ConfigProto
from tensorflow.keras import __version__ as keras_version
import tensorflow as tf
import h5py
import zipfile
import json

import cv2
import numpy as np
import threading
import time

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')


        # Set image size
        self.image_size = 24

        # Initialize Tensorflow session
        self.config = ConfigProto()
        self.config.gpu_options.allow_growth = True
        self.session = InteractiveSession(config=self.config)

        model_path = "/home/david/ros2_ws/src/ROS2-lessons/Week-1-8-Cognitive-robotics/turtlebot3_mogi_py/network_model/model.best.keras"

        print("Tensorflow version: %s" % tf.__version__)
        keras_version_str = str(keras_version)
        print("Keras version: %s" % keras_version_str)
        print("CNN model: %s" % model_path)

        # Fetch the saved Keras version used to produce the file
        model_version = self.get_keras_version_from_keras_file(model_path)
        print("Model's Keras version:", model_version)

        if model_version != keras_version_str:
            print('You are using Keras version ', keras_version_str, ', but the model was built using ', model_version)
            exit()

        # Finally load model:
        self.model = load_model(model_path, custom_objects=None, compile=True, safe_mode=True)
        self.model.summary()

        # Define a new model to extract intermediate layer outputs
        self.activation_model_1 = tf.keras.models.Model(inputs=self.model.inputs, outputs=self.model.get_layer("conv2d_1").output)
        self.activation_model_2 = tf.keras.models.Model(inputs=self.model.inputs, outputs=self.model.get_layer("activation_1").output)

        self.last_time = time.time()

        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',  # Replace with your topic name
            self.image_callback,
            1  # Queue size of 1
        )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Variable to store the latest frame
        self.latest_frame = None
        self.frame_lock = threading.Lock()  # Lock to ensure thread safety
        
        # Flag to control the display loop
        self.running = True

        # Start a separate thread for spinning (to ensure image_callback keeps receiving new frames)
        self.spin_thread = threading.Thread(target=self.spin_thread_func)
        self.spin_thread.start()

    def spin_thread_func(self):
        """Separate thread function for rclpy spinning."""
        while rclpy.ok() and self.running:
            rclpy.spin_once(self, timeout_sec=0.05)

    def image_callback(self, msg):
        """Callback function to receive and store the latest frame."""
        # Convert ROS Image message to OpenCV format and store it
        with self.frame_lock:
            #self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def display_image(self):

        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while rclpy.ok():
            # Check if there is a new frame available
            if self.latest_frame is not None:

                # Process the current image
                grid_1, grid_2 = self.process_image(self.latest_frame)

                # Add processed images as small images on top of main image
                result = self.add_small_pictures(self.latest_frame, [grid_1, grid_2], size=(260, 125))

                # Show the latest frame
                cv2.imshow("frame", result)
                self.latest_frame = None  # Clear the frame after displaying

            # Check for quit key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.stop_robot()
                self.running = False
                break

        # Close OpenCV window after quitting
        cv2.destroyAllWindows()
        self.running = False

    def process_image(self, img):

        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        image = cv2.resize(img, (self.image_size, self.image_size))
        image = img_to_array(image)
        image = np.array(image, dtype="float") / 255.0

        image = image.reshape(-1, self.image_size, self.image_size, 3)
        
        with tf.device('/gpu:0'):
            prediction = np.argmax(self.model(image, training=False))

        # Get the feature maps for the input image
        activations_1 = self.activation_model_1.predict(image)
        activations_2 = self.activation_model_2.predict(image)

        grid_image_1 = self.visualize_feature_maps(activations_1, num_rows=5, num_cols=10, padding=1)
        grid_image_2 = self.visualize_feature_maps(activations_2, num_rows=5, num_cols=10, padding=1)

        print("Prediction %d, elapsed time %.3f" % (prediction, time.time()-self.last_time))
        self.last_time = time.time()

        if prediction == 0: # Forward
            msg.angular.z = 0.0
            msg.linear.x = 0.08
        elif prediction == 1: # Left
            msg.angular.z = -0.3
            msg.linear.x = 0.05
        elif prediction == 2: # Right
            msg.angular.z = 0.3
            msg.linear.x = 0.05
        else: # Nothing
            msg.angular.z = 0.2
            msg.linear.x = 0.0

        # Publish cmd_vel
        self.publisher.publish(msg)

        # Return processed frames
        return grid_image_1, grid_image_2

    # Function to visualize feature maps
    def visualize_feature_maps(self, activations, num_rows=2, num_cols=10, padding=2):
        # activations shape: (1, 24, 24, 20)
        activations = activations[0]  # Remove batch dimension

        # Parameters for the grid
        feature_map_size = activations.shape[1]  # Assuming feature maps are square (24x24)

        # Create an empty RGB image to hold the grid
        grid_width = num_cols * (feature_map_size + padding) - padding
        grid_height = num_rows * (feature_map_size + padding) - padding
        grid_image = np.zeros((grid_height, grid_width, 3), dtype=np.uint8)  # 3 channels for RGB

        # Loop through each feature map and place it in the grid as a heatmap
        for i in range(activations.shape[-1]):
            # Get the i-th feature map
            feature_map = activations[:, :, i]

            # Normalize the feature map to [0, 1]
            feature_map = (feature_map - feature_map.min()) / (feature_map.max() - feature_map.min())

            # Convert the feature map to a heatmap using a colormap
            heatmap = cv2.applyColorMap((feature_map * 255).astype(np.uint8), cv2.COLORMAP_TURBO)

            # Calculate the position in the grid
            row = i // num_cols
            col = i % num_cols
            x_start = col * (feature_map_size + padding)
            y_start = row * (feature_map_size + padding)
            x_end = x_start + feature_map_size
            y_end = y_start + feature_map_size

            # Place the heatmap in the grid
            grid_image[y_start:y_end, x_start:x_end, :] = heatmap

        return grid_image

    # Convert to RGB channels
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B

    # convert to HLS color space
    def convert2hls(self, img):
        hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        H = hls[:, :, 0]
        L = hls[:, :, 1]
        S = hls[:, :, 2]

        return H, L, S
    
    # apply a trapezoid polygon mask, size is hardcoded for 640x480px
    def apply_polygon_mask(self, img):
        mask = np.zeros_like(img)
        ignore_mask_color = 255
        imshape = img.shape
        vertices = np.array([[(0,imshape[0]),(200, 200), (440, 200), (imshape[1],imshape[0])]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_image = cv2.bitwise_and(img, mask)

        return masked_image, mask

    # Apply threshold and result a binary image
    def threshold_binary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255

    # Add small images to the top row of the main image
    def add_small_pictures(self, img, small_images, size=(160, 120)):

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

    # Helper to read the .keras file's metadata
    def get_keras_version_from_keras_file(self, path):
        with zipfile.ZipFile(path, 'r') as archive:
            # Look for metadata.json (exact filename may vary in future versions)
            if 'metadata.json' in archive.namelist():
                with archive.open('metadata.json') as f:
                    metadata = json.load(f)
                    return metadata.get('keras_version', 'Unknown')
            return 'Unknown'

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher.publish(msg)

    def stop(self):
        """Stop the node and the spin thread."""
        self.running = False
        self.spin_thread.join()

def main(args=None):

    print("OpenCV version: %s" % cv2.__version__)

    rclpy.init(args=args)
    node = ImageSubscriber()
    
    try:
        node.display_image()  # Run the display loop
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()  # Ensure the spin thread and node stop properly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()