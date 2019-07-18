#!/usr/bin/env python3
#
# Last modification: 20 Dec. 2018
# Author: Franklin Fernandez

import numpy as np
import os
import six.moves.urllib as urllib
import sys
import tarfile
import tensorflow as tf
import zipfile
from pathlib import Path
from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import subprocess

width = 640
height = 480
WINDOW_NAME = 'Detection'

def open_camera(width, height):
 gst_str = ('nvcamerasrc ! '
               'video/x-raw(memory:NVMM), '
               'width=(int)2592, height=(int)1458, '
               'format=(string)I420, framerate=(fraction)30/1 ! '
               'nvvidconv ! '
               'video/x-raw, width=(int){}, height=(int){}, '
               'format=(string)BGRx ! '
               'videoconvert ! appsink').format(width, height)
 return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)

def open_window(width, height):
 cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
 cv2.resizeWindow(WINDOW_NAME, width, height)
 cv2.setWindowTitle(WINDOW_NAME, 'Camera Demo for Jetson TX2')


cap = open_camera(width, height)

sys.path.append("..")


# ## Object detection imports
from utils import label_map_util
from utils import visualization_utils as vis_util


# # Model preparation 

MODEL_NAME = 'ssd_mobilenet_v1_coco_11_06_2017'
MODEL_FILE = MODEL_NAME + '.tar.gz'
DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'
PATH_TO_LABELS = os.path.join('data', 'mscoco_label_map.pbtxt')

NUM_CLASSES = 90

opener = urllib.request.URLopener()
opener.retrieve(DOWNLOAD_BASE + MODEL_FILE, MODEL_FILE)
tar_file = tarfile.open(MODEL_FILE)
for file in tar_file.getmembers():
  file_name = os.path.basename(file.name)
  if 'frozen_inference_graph.pb' in file_name:
    tar_file.extract(file, os.getcwd())


# ## Load a (frozen) Tensorflow model into memory.

detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')


# ## Loading label map

label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map,   max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

def load_image_into_numpy_array(image):
  (im_width, im_height) = image.size
  return np.array(image.getdata()).reshape(
      (im_height, im_width, 3)).astype(np.uint8)


def main():
 with detection_graph.as_default():
   with tf.Session(graph=detection_graph) as sess:
     while True:
       ret, image_np = cap.read()
       image_np_expanded = np.expand_dims(image_np, axis=0)
       image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
       boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
       scores = detection_graph.get_tensor_by_name('detection_scores:0')
       classes = detection_graph.get_tensor_by_name('detection_classes:0')
       num_detections = detection_graph.get_tensor_by_name('num_detections:0')
      # Read desired object
       obj = Path('/home/nvidia/catkin_ws/src/project/src/object.fcf').read_text()
      # Actual detection.
       (boxes, scores, classes, num_detections) = sess.run(
          [boxes, scores, classes, num_detections],
          feed_dict={image_tensor: image_np_expanded})
      # Visualization of the results of a detection.
       image, bounds, ident = vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          category_index,
          use_normalized_coordinates=True,
          line_thickness=8,
          desired_class=obj)
       print(bounds)
       print(ident)
       f= open("/home/nvidia/catkin_ws/src/project/src/data.fcf","w")
       f.write(str(bounds[0])+","+str(bounds[1])+","+str(bounds[2])+","+str(bounds[3]))
       f.close()

       cv2.imshow('object detection', image)
       if cv2.waitKey(25) & 0xFF == ord('q'):
         cv2.destroyAllWindows()
         break

# Main loop
if __name__ == '__main__':
 try:
  main()
 except rospy.ROSInterruptException: pass
