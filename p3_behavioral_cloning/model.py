import os
import time
import pickle
import random

import pandas as pd
import numpy as np

import matplotlib.image as mpimg

from sklearn.model_selection import train_test_split

from keras.models import Model
from keras.layers import Input, Dense, Flatten, Dropout
from keras.layers.pooling import AveragePooling2D, MaxPooling2D
from keras.layers.normalization import BatchNormalization
from keras.layers.convolutional import Cropping2D, Convolution2D

def load_features(path, off_center=False, angle_correction=0.0, mirror_data=False,
                  zero_angle_tolerance=0.01, zero_angle_fraction=1.):
    '''
    off_center           - if True, then use images from left and right cameras
    angle_correction     - absolute amount of angle correction for left and right cameras
    mirror_data          - if True, then flip images and angles and add to dataset
    zero_angle_tolerance - threshold for angle to be considered as 'zero-angle'
    zero_angle_fraction  - fraction of 'zero-angle' examples to keep in dataset
    '''

    # read driving log to Pandas dataframe
    path_to_csv = os.path.join(path, 'driving_log.csv')
    with open(path_to_csv,'r') as f:
        data = pd.read_csv(f)

    # number of examples
    nb_examples = len(data)

    # initialize lists for images from center camera and angles
    X_center_all, y_center_all = [], []

    for i in range(nb_examples):
        # read path to center image
        center_image_path = os.path.join(path, data['center'][i].strip())
        # read image from center camera and write to array
        center_image = mpimg.imread(center_image_path)
        # add array to initialized list
        X_center_all.append(center_image)

        # read corresponding angle and add it to initialized list
        center_angle = data['steering'][i]
        y_center_all.append(center_angle)

    # convert lists to NumPy arrays
    X_center_all = np.array(X_center_all)
    y_center_all = np.array(y_center_all)

    # there are too many examples with angle value of 0 (or close to zero)
    # it leads to the car tending to move straight
    # to balance training set, I am going to drop some of the examples with angle close to zero

    # get indices of examples where steering angle is very small (or equals 0) and inverse set
    zero_angle_idxs = abs(y_center_all) < zero_angle_tolerance
    nonzero_angle_idxs = abs(y_center_all) >= zero_angle_tolerance

    # get all examples with steering angles close to zero
    X_center_zero = X_center_all[zero_angle_idxs]
    y_center_zero = y_center_all[zero_angle_idxs]

    # get all the other examples
    X_center_nonzero = X_center_all[nonzero_angle_idxs]
    y_center_nonzero = y_center_all[nonzero_angle_idxs]

    # calculate number of examles with zero steering angle to keep in final training dataset
    nb_zero_angle_examples = len(y_center_zero)
    nb_zero_angle_examples_to_keep = int(nb_zero_angle_examples * zero_angle_fraction)

    # take random sample of all examples with zero steering angle
    zero_angle_chosen_idxs = random.sample(range(nb_zero_angle_examples),nb_zero_angle_examples_to_keep)
    X_center_zero_chosen = X_center_zero[zero_angle_chosen_idxs]
    y_center_zero_chosen = y_center_zero[zero_angle_chosen_idxs]

    # concatenate all nonzero steering angle examples with chosen portion of zero steering angle examples
    X_center = np.concatenate((X_center_zero_chosen, X_center_nonzero), axis=0)
    y_center = np.concatenate((y_center_zero_chosen, y_center_nonzero), axis=0)

    # use images from left and right cameras
    if off_center:
        # initialize lists for images from side cameras
        X_left_all, y_left_all, X_right_all, y_right_all = [], [], [], []

        for i in range(nb_examples):
            # add image from the left camera to initialized list
            left_image_path = os.path.join(path, data['left'][i].strip())
            left_image = mpimg.imread(left_image_path)
            X_left_all.append(left_image)

            # correct steering angle as if the car was closer to the left side of the track than it was
            left_angle = data['steering'][i] + angle_correction
            y_left_all.append(left_angle)

            # add image from the right camera to initialized list
            right_image_path = os.path.join(path, data['right'][i].strip())
            right_image = mpimg.imread(right_image_path)
            X_right_all.append(right_image)

            # correct steering angle as if the car was closer to the right side of the track than it was
            right_angle = data['steering'][i] - angle_correction
            y_right_all.append(right_angle)


        # convert lists to NumPy arrays
        X_left_all = np.array(X_left_all)
        y_left_all = np.array(y_left_all)

        X_right_all = np.array(X_right_all)
        y_right_all = np.array(y_right_all)

        # get all the examples with steering angle close to zero
        X_left_zero = X_left_all[zero_angle_idxs]
        y_left_zero = y_left_all[zero_angle_idxs]

        X_right_zero = X_right_all[zero_angle_idxs]
        y_right_zero = y_right_all[zero_angle_idxs]

        # get all the other examples
        X_left_nonzero = X_left_all[nonzero_angle_idxs]
        y_left_nonzero = y_left_all[nonzero_angle_idxs]

        X_right_nonzero = X_right_all[nonzero_angle_idxs]
        y_right_nonzero = y_right_all[nonzero_angle_idxs]

        # take random samples using the same indices as for center camera.
        # without decreasing number of those angles there would be too much angles equal +-angle_correction
        # because for each image from center camera with zero steering angle
        # there are exactly two examples with corrected angles (from the left and the right camera).
        # for simplicity indices are the same
        X_left_zero_chosen = X_left_zero[zero_angle_chosen_idxs]
        y_left_zero_chosen = y_left_zero[zero_angle_chosen_idxs]

        X_right_zero_chosen = X_right_zero[zero_angle_chosen_idxs]
        y_right_zero_chosen = y_right_zero[zero_angle_chosen_idxs]

        # concatenate sets of images from the left and the right cameras respectively
        X_left = np.concatenate((X_left_zero_chosen, X_left_nonzero), axis=0)
        y_left = np.concatenate((y_left_zero_chosen, y_left_nonzero), axis=0)

        X_right = np.concatenate((X_right_zero_chosen, X_right_nonzero), axis=0)
        y_right = np.concatenate((y_right_zero_chosen, y_right_nonzero), axis=0)

        # concatenate images from all cameras if off_center is True
        # "actual" is in the sense "not mirrored"
        X_actual = np.concatenate((X_center, X_left, X_right), axis=0)
        y_actual = np.concatenate((y_center, y_left, y_right), axis=0)

    else:
        # use only images from center camera for training if off_center is False
        X_actual = X_center
        y_actual = y_center

    # mirror all images and steering angles to balance data and cheaply increase training data
    if mirror_data:
        X_mirrored = np.array([np.fliplr(image) for image in X_actual])
        y_mirrored = -y_actual

        X = np.concatenate((X_actual, X_mirrored), axis=0)
        y = np.concatenate((y_actual, y_mirrored), axis=0)
    else:
        X, y = X_actual, y_actual

    return X, y

# process Udacity data and load features and labels
# it is very memory expensive. With 32GB of RAM sometimes I got Memory error when prototopyng it in Jupyter
path = 'data'
X, y = load_features(path, off_center=True, angle_correction=0.25, mirror_data=True,
                     zero_angle_tolerance=0.01, zero_angle_fraction=0.20)

pickle.dump((X, y), open('data.p','wb'))


# X, y = pickle.load(open('data.p','rb'))

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.1, random_state=0)

input_shape = X.shape[1:]

# network architecture based on Nvidia's paper
# https://arxiv.org/pdf/1604.07316v1.pdf

inputs = Input(shape=input_shape)

x = BatchNormalization()(inputs)
x = Cropping2D(cropping=((80, 20), (0, 0)))(x)
x = AveragePooling2D(pool_size=(2, 2))(x)

x = Convolution2D(24, 5, 5)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

x = Convolution2D(36, 5, 5)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

x = Convolution2D(48, 3, 3)(x)
x = MaxPooling2D(pool_size=(2, 2))(x)

x = Flatten()(x)
x = Dropout(0.25)(x)

x = Dense(100, activation='elu')(x)
x = Dropout(0.25)(x)

x = Dense(50, activation='elu')(x)
x = Dropout(0.25)(x)

x = Dense(10, activation='elu')(x)
x = Dropout(0.25)(x)

predictions = Dense(1, activation='linear')(x)

model = Model(input=inputs, output=predictions)
# model.summary()

model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])

model.fit(X_train, y_train, batch_size=256, nb_epoch=7, verbose=1, validation_split=0.2)

model.save('model.h5')

print('Evaluating on the test data')
print(model.evaluate(X_test, y_test, batch_size=256, verbose=1))
