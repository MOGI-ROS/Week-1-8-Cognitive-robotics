# import the necessary packages
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Activation, Flatten, Dense, Conv2D, MaxPooling2D
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.callbacks import ReduceLROnPlateau, ModelCheckpoint
from tensorflow.keras.utils import to_categorical
from tensorflow.keras import __version__ as keras_version
from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession
from tensorflow.random import set_seed
import tensorflow as tf
from sklearn.model_selection import train_test_split
from imutils import paths
import numpy as np
import random
import cv2
import os
import matplotlib.pyplot as plt
from numpy.random import seed

# Set image size
image_size = 24

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

# Fix every random seed to make the training reproducible
seed(1)
set_seed(2)
random.seed(42)

print("[INFO] Version:")
print("Tensorflow version: %s" % tf.__version__)
keras_version = str(keras_version).encode('utf8')
print("Keras version: %s" % keras_version)

def build_LeNet(width, height, depth, classes):
    # initialize the model
    model = Sequential()
    inputShape = (height, width, depth)

    # first set of CONV => RELU => POOL layers
    model.add(Conv2D(20, (5, 5), padding="same", input_shape=inputShape))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

    # second set of CONV => RELU => POOL layers
    model.add(Conv2D(50, (5, 5), padding="same"))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

    # first (and only) set of FC => RELU layers
    model.add(Flatten())
    model.add(Dense(500))
    model.add(Activation("relu"))

    # softmax classifier
    model.add(Dense(classes))
    model.add(Activation("softmax"))

    # return the constructed network architecture
    return model

    
dataset = '..//training_images'
# initialize the data and labels
print("[INFO] loading images and labels...")
data = []
labels = []
 
# grab the image paths and randomly shuffle them
imagePaths = sorted(list(paths.list_images(dataset)))
random.shuffle(imagePaths)
# loop over the input images
for imagePath in imagePaths:
    # load the image, pre-process it, and store it in the data list
    image = cv2.imread(imagePath)
    image = cv2.resize(image, (image_size, image_size))
    image = img_to_array(image)
    data.append(image)
    # extract the class label from the image path and update the
    # labels list
    label = imagePath.split(os.path.sep)[-2]
    print("Image: %s, Label: %s" % (imagePath, label))
    if label == 'forward':
        label = 0
    elif label == 'right':
        label = 1
    elif label == 'left':
        label = 2
    else:
        label = 3
    labels.append(label)
    
    
# scale the raw pixel intensities to the range [0, 1]
data = np.array(data, dtype="float") / 255.0
labels = np.array(labels)
 
# partition the data into training and testing splits using 75% of
# the data for training and the remaining 25% for testing
(trainX, testX, trainY, testY) = train_test_split(data, labels, test_size=0.25, random_state=42)# convert the labels from integers to vectors
trainY = to_categorical(trainY, num_classes=4)
testY = to_categorical(testY, num_classes=4)


# initialize the number of epochs to train for, initial learning rate,
# and batch size
EPOCHS  = 40
INIT_LR = 0.001
DECAY   = INIT_LR / EPOCHS
BS      = 32

# initialize the model
print("[INFO] compiling model...")
model = build_LeNet(width=image_size, height=image_size, depth=3, classes=4)
opt = Adam(learning_rate=INIT_LR, decay=DECAY)
model.compile(loss="binary_crossentropy", optimizer=opt, metrics=["accuracy"])
 
# print model summary
model.summary()

# checkpoint the best model
checkpoint_filepath = "..//network_model//model.best.h5"
checkpoint = ModelCheckpoint(checkpoint_filepath, monitor = 'val_loss', verbose=1, save_best_only=True, mode='min')

# set a learning rate annealer
reduce_lr = ReduceLROnPlateau(monitor='val_loss', patience=3, verbose=1, factor=0.5, min_lr=1e-6)

# callbacks
callbacks_list=[reduce_lr, checkpoint]

# train the network
print("[INFO] training network...")
history = model.fit(trainX, trainY, batch_size=BS, validation_data=(testX, testY), epochs=EPOCHS, callbacks=callbacks_list, verbose=1)
 
# save the model to disk
print("[INFO] serializing network...")
model.save("..//network_model//model.h5")

plt.xlabel('Epoch Number')
plt.ylabel("Loss / Accuracy Magnitude")
plt.plot(history.history['loss'], label="loss")
plt.plot(history.history['accuracy'], label="acc")
plt.plot(history.history['val_loss'], label="val_loss")
plt.plot(history.history['val_accuracy'], label="val_acc")
plt.legend()
plt.savefig('model_training')
plt.show()