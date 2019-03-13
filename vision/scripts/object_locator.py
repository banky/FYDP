# Copyright Beach Cleaning Automated
# 
# This file is used to locate objects of interest in in view of 
# a connected kinect and determine how far they are

import colorsys
import os
from timeit import default_timer as timer

import numpy as np
from keras import backend as K
from keras.models import load_model
from keras.layers import Input
from PIL import Image, ImageFont, ImageDraw
import cv2

from .model import yolo_eval, yolo_body, tiny_yolo_body
from .utils import letterbox_image
from keras.utils import multi_gpu_model
import time

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RESOLUTION = 10 # Number of pixels per degree Kinect uses
FRAME_MAX_DEPTH = 2047
BASE_PATH = os.path.join(os.path.dirname(__file__), '../scripts/')

def depth_2_feet(depth):
    """ Returns the depth measured by the Kinect in meters """
    return 4945422 + (2.258474 - 4945422)/(1 + (depth/5031.15)**8.170714)

def depth_2_metres(depth):
    """ Converts a value in feet to metres """
    return 0.3048 * depth_2_feet(depth)

class Point():
    """ Point describes locations of objects of interest in video """
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Feature():
    """ Describes the location of interesting objects in our vicinity
    (both garbage and obstacles) Positions are described using angular
    notation (distance and angle from kinect reference frame) """

    def __init__ (self, dist, theta, is_garbage):
        self.dist = dist
        self.theta = theta
        self.is_garbage = is_garbage

class YOLO(object):
    _defaults = {
        #"model_path": 'model_data/yolo.h5',
        "model_path": BASE_PATH + 'model_data/trained_weights_stage_1.h5',
        "anchors_path": BASE_PATH + 'model_data/yolo_anchors.txt',
        "classes_path": BASE_PATH + 'model_data/google_classes.txt',
        "score" : 0.3,
        "iou" : 0.45,
        "model_image_size" : (416, 416),
        "gpu_num" : 1,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults) # set up default values
        self.__dict__.update(kwargs) # and update with user overrides
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()
        self.boxes, self.scores, self.classes = self.generate()

    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)

        with open(classes_path) as f:
            class_names = f.readlines()
        class_names = [c.strip() for c in class_names]
        return class_names

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        return np.array(anchors).reshape(-1, 2)

    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        num_anchors = len(self.anchors)
        num_classes = len(self.class_names)
        is_tiny_version = num_anchors==6 # default setting
        try:
            self.yolo_model = load_model(model_path, compile=False)
        except:
            self.yolo_model = tiny_yolo_body(Input(shape=(None,None,3)), num_anchors//2, num_classes) \
                if is_tiny_version else yolo_body(Input(shape=(None,None,3)), num_anchors//3, num_classes)
            self.yolo_model.load_weights(self.model_path) # make sure model, anchors and classes match
        else:
            assert self.yolo_model.layers[-1].output_shape[-1] == \
                num_anchors/len(self.yolo_model.output) * (num_classes + 5), \
                'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))
        np.random.seed(10101)  # Fixed seed for consistent colors across runs.
        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.
        np.random.seed(None)  # Reset seed to default.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2, ))
        if self.gpu_num>=2:
            self.yolo_model = multi_gpu_model(self.yolo_model, gpus=self.gpu_num)
        boxes, scores, classes = yolo_eval(self.yolo_model.output, self.anchors,
                len(self.class_names), self.input_image_shape,
                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):
        #start = timer()
        

        if self.model_image_size != (None, None):
            assert self.model_image_size[0]%32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1]%32 == 0, 'Multiples of 32 required'
            boxed_image = letterbox_image(image, tuple(reversed(self.model_image_size)))
        else:
            new_image_size = (image.width - (image.width % 32),
                              image.height - (image.height % 32))
            boxed_image = letterbox_image(image, new_image_size)
        image_data = np.array(boxed_image, dtype='float32')

        # print(image_data.shape)
        image_data /= 255.
        image_data = np.expand_dims(image_data, 0)  # Add batch dimension.

        start = time.time()

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        thickness = (image.size[0] + image.size[1]) // 300

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            draw = ImageDraw.Draw(image)
            label_size = draw.textsize(label)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.size[1], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.size[0], np.floor(right + 0.5).astype('int32'))
            # print(label, (left, top), (right, bottom))

            if top - label_size[1] >= 0:
                text_origin = np.array([left, top - label_size[1]])
            else:
                text_origin = np.array([left, top + 1])

            # My kingdom for a good redistributable image drawing library.
            for i in range(thickness):
                draw.rectangle(
                    [left + i, top + i, right - i, bottom - i],
                    outline=self.colors[c])
            draw.rectangle(
                [tuple(text_origin), tuple(text_origin + label_size)],
                fill=self.colors[c])
            # draw.text(text_origin, label, fill=(0, 0, 0))
            del draw

        #end = timer()
        # end = time.time()
        # print(end - start)
        return image, out_boxes, out_scores, out_classes

    def close_session(self):
        self.sess.close()

def update(yolo, frame):
    image = Image.fromarray(frame)
    img, out_boxes, out_scores, out_classes = yolo.detect_image(image)

    # For debug
    result = np.asarray(img)
    cv2.namedWindow("result", cv2.WINDOW_NORMAL)
    cv2.imshow("result", result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

    return out_boxes, out_scores, out_classes

def find_obstacles(yolo, out_boxes, out_scores, out_classes, depth):
    obstacles = []
    obstacle_types = ["box", "human", "leg", "chair", "tree"]

    for i, c in reversed(list(enumerate(out_classes))):
        if yolo.class_names[c] in obstacle_types:
            y_min, x_min, y_max, x_max = out_boxes[i]
            x = int((x_min + x_max) // 2)
            y = int((y_min + y_max) // 2)
            obstacle_loc = Point(x, y)

            theta = obstacle_loc.x - FRAME_WIDTH / 2
            theta = theta / FRAME_RESOLUTION
            # print(np.shape(depth))
            # print(obstacle_loc.x)
            # print(obstacle_loc.y)
            dist = depth[obstacle_loc.y][obstacle_loc.x]
            dist = depth_2_metres(dist)
            feature = Feature(dist, theta, False)

            # Only add feature if it is in an acceptable distance away
            if dist < FRAME_MAX_DEPTH:  
                obstacles.append(feature)

    return obstacles

def find_garbage(yolo, out_boxes, out_scores, out_classes, depth):
    """ Get the feature location of garbage in the frame """

    garbages = []
    garbage_types = ["bottle", "can"]

    for i, c in reversed(list(enumerate(out_classes))):
        if yolo.class_names[c] in garbage_types:
            y_min, x_min, y_max, x_max = out_boxes[i]
            x = int((x_min + x_max) // 2)
            y = int((y_min + y_max) // 2)
            garbage_loc = Point(x, y)

            theta = garbage_loc.x - FRAME_WIDTH / 2
            theta = theta / FRAME_RESOLUTION
            dist = depth[garbage_loc.y][garbage_loc.x]
            dist = depth_2_metres(dist)
            feature = Feature(dist, theta, True)

            # Only add feature if it is in an acceptable distance away
            if dist < FRAME_MAX_DEPTH:  
                garbages.append(feature)

    return garbages
