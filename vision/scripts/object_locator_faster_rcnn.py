# Copyright Beach Cleaning Automated
# 
# This file is used to locate objects of interest in in view of 
# a connected kinect and determine how far they are

import os

import numpy as np
import cv2
from . import frame_convert2
import freenect
from . import faster_rcnn
from .faster_rcnn import Config

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RESOLUTION = 10 # Number of pixels per degree Kinect uses
FRAME_MAX_DEPTH = 2047

def depth_2_feet(depth):
    """ Returns the depth measured by the Kinect in meters """
    return 4945422 + (2.258474 - 4945422)/(1 + (depth/5031.15)**8.170714)

def depth_2_metres(depth):
    """ Converts a value in feet to metres """
    return 0.3048 * depth_2_feet(depth)

def format_img_size(img, C):
    """ formats the image size based on config """
    img_min_side = float(C.im_size)
    (height,width,_) = img.shape
        
    if width <= height:
        ratio = img_min_side/width
        new_height = int(ratio * height)
        new_width = int(img_min_side)
    else:
        ratio = img_min_side/height
        new_width = int(ratio * width)
        new_height = int(img_min_side)
    img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
    return img, ratio

def format_img_channels(img, C):
    """ formats the image channels based on config """
    img = img[:, :, (2, 1, 0)]
    img = img.astype(np.float32)
    img[:, :, 0] -= C.img_channel_mean[0]
    img[:, :, 1] -= C.img_channel_mean[1]
    img[:, :, 2] -= C.img_channel_mean[2]
    img /= C.img_scaling_factor
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return img

def format_img(img, C):
    """ formats an image for model prediction based on config """
    img, ratio = format_img_size(img, C)
    img = format_img_channels(img, C)
    return img, ratio

# Method to transform the coordinates of the bounding box to its original size
def get_real_coordinates(ratio, x1, y1, x2, y2):

    real_x1 = int(round(x1 // ratio))
    real_y1 = int(round(y1 // ratio))
    real_x2 = int(round(x2 // ratio))
    real_y2 = int(round(y2 // ratio))

    return (real_x1, real_y1, real_x2 ,real_y2)

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

class ObjectLocator:
    """ Contains interface for performing object localization in a frame """

    def __init__(self):
        # ctx = freenect.init()
        # dev = freenect.open_device(ctx, freenect.num_devices(ctx) - 1)
        # freenect.set_tilt_degs(dev, 0)
        # freenect.close_device(dev)

        self.base_path = 'scripts'
        self.test_path = 'Dataset/annotation.txt' # Test data (annotation file)
        self.test_base_path = 'Dataset/test' # Directory to save the test images
        self.config_output_filename = os.path.join(self.base_path,
                                                               'model_vgg_config.pickle')
        self.depth = None
        self.video = None
        self.objects = []   # Objects in current frame

        with open(self.config_output_filename, 'rb') as f_in:
            self.config = faster_rcnn.pickle.load(f_in)

        # turn off any data augmentation at test time
        self.config.use_horizontal_flips = False
        self.config.use_vertical_flips = False
        self.config.rot_90 = False

        num_features = 512

        input_shape_img = (None, None, 3)
        input_shape_features = (None, None, num_features)

        img_input = faster_rcnn.Input(shape=input_shape_img)
        roi_input = faster_rcnn.Input(shape=(self.config.num_rois, 4))
        feature_map_input = faster_rcnn.Input(shape=input_shape_features)

        # define the base network (VGG here, can be Resnet50, Inception, etc)
        shared_layers = faster_rcnn.nn_base(img_input, trainable=True)

        # define the RPN, built on the base layers
        num_anchors = len(self.config.anchor_box_scales) * len(self.config.anchor_box_ratios)
        rpn_layers = faster_rcnn.rpn_layer(shared_layers, num_anchors)

        classifier = faster_rcnn.classifier_layer(feature_map_input, roi_input, self.config.num_rois,
                                                  nb_classes=len(self.config.class_mapping))

        self.model_rpn = faster_rcnn.Model(img_input, rpn_layers)
        self.model_classifier_only = faster_rcnn.Model([feature_map_input, roi_input], classifier)

        model_classifier = faster_rcnn.Model([feature_map_input, roi_input], classifier)

        self.config.model_path = os.path.join(self.base_path, self.config.model_path)
        print('Loading weights from {}'.format(self.config.model_path))
        self.model_rpn.load_weights(self.config.model_path, by_name=True)
        model_classifier.load_weights(self.config.model_path, by_name=True)

        self.model_rpn.compile(optimizer='sgd', loss='mse')
        model_classifier.compile(optimizer='sgd', loss='mse')

    def get_depth(self):
        """ Get's the current depth data from Kinect """
        self.depth = freenect.sync_get_depth()[0]

    def get_video(self):
        """ Get's the current video data from Kinect """
        self.video = frame_convert2.video_cv(freenect.sync_get_video()[0])

    def update(self):
        """ Gets a new frame of video and performs processing on it """

        # Switch key value for class mapping
        class_mapping = self.config.class_mapping
        class_mapping = {v: k for k, v in class_mapping.items()}
        print(class_mapping)
        class_to_color = {class_mapping[v]: np.random.randint(0, 255, 3) for v in class_mapping}

        # If the box classification value is less than this, we ignore this box
        bbox_threshold = 0.7

        # img = cv2.imread(self.base_path + '/images/IMG_2815.JPG') # Get image from kinect
        self.get_depth()
        self.get_video()
        img = self.video

        X, ratio = format_img(img, self.config)

        X = np.transpose(X, (0, 2, 3, 1))

        # get output layer Y1, Y2 from the RPN and the feature maps F
        # Y1: y_rpn_cls
        # Y2: y_rpn_regr
        [Y1, Y2, F] = self.model_rpn.predict(X)

        # Get bboxes by applying NMS 
        # R.shape = (300, 4)
        R = faster_rcnn.rpn_to_roi(Y1, Y2, self.config, faster_rcnn.K.image_dim_ordering(),
                                   overlap_thresh=0.7)

        # convert from (x1,y1,x2,y2) to (x,y,w,h)
        R[:, 2] -= R[:, 0]
        R[:, 3] -= R[:, 1]

        # apply the spatial pyramid pooling to the proposed regions
        bboxes = {}
        probs = {}
        self.objects = []

        for jk in range(R.shape[0]//self.config.num_rois + 1):
            ROIs = np.expand_dims(R[self.config.num_rois*jk:self.config.num_rois*(jk+1), :], axis=0)
            if ROIs.shape[1] == 0:
                break

            if jk == R.shape[0]//self.config.num_rois:
                #pad R
                curr_shape = ROIs.shape
                target_shape = (curr_shape[0], self.config.num_rois, curr_shape[2])
                ROIs_padded = np.zeros(target_shape).astype(ROIs.dtype)
                ROIs_padded[:, :curr_shape[1], :] = ROIs
                ROIs_padded[0, curr_shape[1]:, :] = ROIs[0, 0, :]
                ROIs = ROIs_padded

            [P_cls, P_regr] = self.model_classifier_only.predict([F, ROIs])

            # Calculate bboxes coordinates on resized image
            for ii in range(P_cls.shape[1]):
                # Ignore 'bg' class
                if np.max(P_cls[0, ii, :]) < bbox_threshold or np.argmax(P_cls[0, ii, :]) == (P_cls.shape[2] - 1):
                    continue

                cls_name = class_mapping[np.argmax(P_cls[0, ii, :])]

                if cls_name not in bboxes:
                    bboxes[cls_name] = []
                    probs[cls_name] = []

                (x, y, w, h) = ROIs[0, ii, :]

                cls_num = np.argmax(P_cls[0, ii, :])
                try:
                    (tx, ty, tw, th) = P_regr[0, ii, 4*cls_num:4*(cls_num+1)]
                    tx /= config.classifier_regr_std[0]
                    ty /= config.classifier_regr_std[1]
                    tw /= config.classifier_regr_std[2]
                    th /= config.classifier_regr_std[3]
                    x, y, w, h = apply_regr(x, y, w, h, tx, ty, tw, th)
                except:
                    pass
                bboxes[cls_name].append([self.config.rpn_stride*x, self.config.rpn_stride*y, 
                                         self.config.rpn_stride*(x+w), self.config.rpn_stride*(y+h)])
                probs[cls_name].append(np.max(P_cls[0, ii, :]))

            all_dets = []

            for key in bboxes:
                bbox = np.array(bboxes[key])
                ob = {} # Object identified

                new_boxes, new_probs = faster_rcnn.non_max_suppression_fast(bbox, np.array(probs[key]), overlap_thresh=0.2)
                for jk in range(new_boxes.shape[0]):
                    (x1, y1, x2, y2) = new_boxes[jk,:]

                    # Calculate real coordinates on original image
                    (real_x1, real_y1, real_x2, real_y2) = get_real_coordinates(ratio, x1, y1, x2, y2)
                    ob["x"] = (real_x1 + real_x2) // 2
                    ob["y"] = (real_y1 + real_y2) // 2
                    ob["class"] = key

                    self.objects.append(ob)

                    cv2.rectangle(img,(real_x1, real_y1), (real_x2, real_y2), (int(class_to_color[key][0]), int(class_to_color[key][1]), int(class_to_color[key][2])),4)

                    textLabel = '{}: {}'.format(key,int(100*new_probs[jk]))
                    all_dets.append((key,100*new_probs[jk]))

                    (retval,baseLine) = cv2.getTextSize(textLabel,cv2.FONT_HERSHEY_COMPLEX,1,1)
                    textOrg = (real_x1, real_y1-0)

                    cv2.rectangle(img, (textOrg[0] - 5, textOrg[1]+baseLine - 5), (textOrg[0]+retval[0] + 5, textOrg[1]-retval[1] - 5), (0, 0, 0), 1)
                    cv2.rectangle(img, (textOrg[0] - 15,textOrg[1]+baseLine - 15), (textOrg[0]+retval[0] + 15, textOrg[1]-retval[1] - 15), (255, 255, 255), -1)
                    cv2.putText(img, textLabel, textOrg, cv2.FONT_HERSHEY_DUPLEX, 3, (0, 0, 0), 2)

        # faster_rcnn.plt.figure(figsize=(10,10))
        # faster_rcnn.plt.grid()
        # faster_rcnn.plt.imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
        # faster_rcnn.plt.show()

    def find_obstacles(self):
        """ Get the feature location of obstacles in frame """

        obstacles = []
        obstacle_types = ["Box", "Human", "Human leg", "Chair", "Tree"]

        if not self.objects:  # No objects in frame
            return obstacles

        for ob in self.objects:
            if ob["class"] in obstacle_types:
                obstacle_loc = Point(ob["x"], ob["y"])

                theta = obstacle_loc.x - FRAME_WIDTH / 2
                theta = theta / FRAME_RESOLUTION
                # dist = self.depth[obstacle_loc.y][obstacle_loc.x]
                dist = 3
                dist = depth_2_metres(dist)
                feature = Feature(dist, theta, False)

                # Only add feature if it is in an acceptable distance away
                if dist < FRAME_MAX_DEPTH:  
                    obstacles.append(feature)

        return obstacles

    def find_garbage(self):
        """ Get the feature location of garbage in the frame """

        garbages = []
        garbage_types = ["Bottle", "Tin can"]

        if not self.objects:  # No objects in frame
            return garbages

        for ob in self.objects:
            if ob["class"] in garbage_types:
                garbage_loc = Point(ob["x"], ob["y"])

                theta = garbage_loc.x - FRAME_WIDTH / 2
                theta = theta / FRAME_RESOLUTION
                # dist = self.depth[garbage_loc.y][garbage_loc.x]
                dist = 3
                dist = depth_2_metres(dist)
                feature = Feature(dist, theta, False)

                # Only add feature if it is in an acceptable distance away
                if dist < FRAME_MAX_DEPTH:  
                    garbages.append(feature)

        return garbages