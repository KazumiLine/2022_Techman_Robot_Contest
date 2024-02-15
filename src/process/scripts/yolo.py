#!/usr/bin/env python3
from glob import glob
from re import I
import string
from turtle import distance
import cv2
import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon
from std_srvs.srv import Empty, Trigger, Trigger_Response
# from vision_msg.msg import YoloBoundingBox
bridge = CvBridge()

square_image_rect_TL = (0, 0)
square_image_rect_BR = (0, 0)
line_image_rect_TL = (0, 0)
line_image_rect_BR = (0, 0)
roboteye_rect_click = [0, 0]
yolo_detect_rect = [[0, 0], [640, 480]]

hole_image_pos = np.zeros((19, 2), dtype=int)
hole_image_roboteye_pos = np.zeros((10, 2), dtype=int)

hole_status = np.zeros(19, dtype=int)

LOCKER_model_processing = False

status = "getrect"


def distance(P1, P2):
    return ((P1[0] - P2[0]) * (P1[0] - P2[0]) + (P1[1] - P2[1]) * (P1[1] - P2[1]))**0.5


def center(box):
    return [int(box[0] + box[2] / 2), int(box[1] + box[3] / 2)]


def handle_NG_detection_service(req, resp):
    resp.success = True
    resp.message = ''
    for s in hole_status[9:]:
        resp.message += str(s)
    return resp
    # ======================= #
    global LOCKER_model_processing
    LOCKER_model_processing = True

    img_roboteye = cv2.imread('/home/fire/Desktop/showimage.png')  #2592*1944
    # img_roboteye = cv2.resize(img_roboteye, (432, 324))
    # cv2.imshow('robot cam', img_roboteye)
    # print("Service called")

    img_detect = img_roboteye.copy()
    img_detect = cv2.resize(img_detect, (432, 324), interpolation=cv2.INTER_AREA)

    classIds, scores, boxes = model.detect(img_detect, confThreshold=0.6, nmsThreshold=0.4)

    ng_status = np.zeros(10, dtype=int)
    for (classId, score, box) in zip(classIds, scores, boxes):
        cv2.rectangle(img_detect, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(139, 38, 212), thickness=1)
        # cv2.circle(img_detect, center(box), 8, color=(139, 38, 212), thickness=1)
        cv2.putText(img_detect, str(classes[int(classId)]), (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(139, 38, 212), thickness=1)

        # 判斷hole的狀態  #利用找到的 YOLO位置 去尋找每一個 設定位置
        for i, pos in enumerate(hole_image_roboteye_pos):
            if (distance(center(box), pos) < 8):
                if (classId == 4):  #bad pin
                    ng_status[i] = 2
                break  #不會同時找到兩個 設定位置
        # >>end for loop for YOLO
    # draw hole pose
    for i, p in enumerate(hole_image_roboteye_pos):
        if (ng_status[i] == 2):
            cv2.circle(img_detect, p.astype('int32'), 4, (23, 34, 179), -1)
        else:
            cv2.circle(img_detect, p.astype('int32'), 4, (44, 222, 160), 1)
        cv2.putText(img_detect, str(i), (p[0], p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(44, 222, 160), thickness=1)

    # cv2.imshow('robot cam', img_detect)
    # #state to string
    ng_status_msg = ''
    for s in ng_status:
        ng_status_msg += str(s)

    LOCKER_model_processing = False
    resp.success = True
    resp.message = ng_status_msg
    return resp


def mouse_event_on_get_image(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        global yolo_detect_rect
        yolo_detect_rect[0] = [x, y]
        yolo_detect_rect[1] = [x + detect_width, y + detect_height]


def mouse_event_on_detect_image(event, x, y, flags, param):
    global status
    if event == cv2.EVENT_LBUTTONDOWN:
        if status == "getrect":
            global square_image_rect_TL
            square_image_rect_TL = (x, y)
        elif status == "getline":
            global line_image_rect_TL
            line_image_rect_TL = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        if status == "getrect":
            global square_image_rect_BR
            square_image_rect_BR = (x, y)

            hole_image_pos[0] = square_image_rect_TL
            hole_image_pos[3] = (square_image_rect_TL[0], (square_image_rect_BR[1] + square_image_rect_TL[1]) / 2)
            hole_image_pos[6] = (square_image_rect_TL[0], square_image_rect_BR[1])
            hole_image_pos[1] = ((square_image_rect_BR[0] + square_image_rect_TL[0]) / 2, square_image_rect_TL[1])
            hole_image_pos[4] = ((square_image_rect_BR[0] + square_image_rect_TL[0]) / 2, (square_image_rect_BR[1] + square_image_rect_TL[1]) / 2)
            hole_image_pos[7] = ((square_image_rect_BR[0] + square_image_rect_TL[0]) / 2, square_image_rect_BR[1])
            hole_image_pos[2] = (square_image_rect_BR[0], square_image_rect_TL[1])
            hole_image_pos[5] = (square_image_rect_BR[0], (square_image_rect_BR[1] + square_image_rect_TL[1]) / 2)
            hole_image_pos[8] = square_image_rect_BR
            status = "getline"
        elif status == "getline":
            global line_image_rect_BR
            line_image_rect_BR = (x, y)
            #Y line
            distance = line_image_rect_BR[1] - line_image_rect_TL[1]
            for i in range(10):
                index_line = i + 9
                hole_image_pos[index_line] = (line_image_rect_TL[0], line_image_rect_TL[1] + i * distance / 9)
            status = "getrect"


def mouse_event_on_roboteye_image(event, x, y, flags, param):
    if event == cv2.EVENT_RBUTTONDOWN:
        global roboteye_rect_click
        roboteye_rect_click = [x, y]
    elif event == cv2.EVENT_RBUTTONUP:
        roboteye_rect_release = [x, y]
        #Y line
        distance_x = roboteye_rect_release[0] - roboteye_rect_click[0]
        distance_y = roboteye_rect_release[1] - roboteye_rect_click[1]
        for i in range(10):
            hole_image_roboteye_pos[i] = (roboteye_rect_click[0] + i * distance_x / 9, roboteye_rect_click[1] + i * distance_y / 9)

class YoloService(Node):

    def __init__(self):
        super().__init__('yolo_node')
        self.service = self.create_service(Trigger, '/ng_detect', handle_NG_detection_service)
        self.yolo_image = self.create_publisher(Image, '/yolo/image', 1)
        self.pub_hole_status_line = self.create_publisher(String, '/hole_status', 1)
        self.sub_image = self.create_subscription(Image, '/camera/image', self.img_callback, 1)

        cv2.namedWindow('get_image')
        cv2.setMouseCallback('get_image', mouse_event_on_get_image)
        cv2.namedWindow('detect_image')
        cv2.setMouseCallback('detect_image', mouse_event_on_detect_image)
        # cv2.namedWindow('robot cam')
        # cv2.setMouseCallback('robot cam', mouse_event_on_roboteye_image)
        # cv2.destroyAllWindows()

        
    def img_callback(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr('bridge error {e}')

        ###YOLO###
        img_detect = cv_image[yolo_detect_rect[0][1]:yolo_detect_rect[1][1], yolo_detect_rect[0][0]:yolo_detect_rect[1][0], :].copy()
        img_detect = cv2.resize(img_detect, (detect_width, detect_height), interpolation=cv2.INTER_AREA)
        global LOCKER_model_processing
        try:
            if (LOCKER_model_processing == False):
                classIds, scores, boxes = model.detect(img_detect, confThreshold=0.6, nmsThreshold=0.4)
            else:
                print('locker locked')
                return
        except:
            print('client using model')
            return

        for (classId, score, box) in zip(classIds, scores, boxes):
            # cv2.rectangle(img_detect, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), color=(139, 38, 212), thickness=1)
            cv2.circle(img_detect, center(box), 8, color=(139, 38, 212), thickness=1)
            cv2.putText(img_detect, str(classes[int(classId)]), (box[0], box[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(139, 38, 212), thickness=1)

            #判斷hole的狀態  #利用找到的 YOLO位置 去尋找每一個 設定位置
            for i, pos in enumerate(hole_image_pos):
                if (distance(center(box), pos) < 8):
                    if (classId == 2):  #good pin
                        hole_status[i] = 1
                    elif (classId == 1):  #hole
                        hole_status[i] = 0
                    break  #不會同時找到兩個 設定位置
            #這裡如果都沒找到 那就維持上一個選項不會變，也就是找到 才更新
            #>>end for loop for YOLO

        # draw hole pose
        for i, p in enumerate(hole_image_pos):
            if (i <= 8):  # square
                if (hole_status[i] == 0):
                    cv2.circle(img_detect, p.astype('int32'), 4, (37, 240, 34), 1)
                elif (hole_status[i] == 1):
                    cv2.circle(img_detect, p.astype('int32'), 4, (37, 240, 34), -1)
                cv2.putText(img_detect, str(i), (p[0], p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(37, 240, 34), thickness=1)
            elif (i >= 9):  #line
                if (hole_status[i] == 0):
                    cv2.circle(img_detect, p.astype('int32'), 4, (44, 222, 160), 1)
                elif (hole_status[i] == 1):
                    cv2.circle(img_detect, p.astype('int32'), 4, (44, 222, 160), -1)
                index_line = i - 9
                cv2.putText(img_detect, str(index_line), (p[0] + 6, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(44, 222, 160), thickness=1)

        cv2.imshow('detect_image', img_detect)
        image_message = bridge.cv2_to_imgmsg(img_detect, "bgr8")
        self.yolo_image.publish(image_message)

        #state to string
        hole_status_msg = ''
        for s in hole_status:
            hole_status_msg += str(s)
        # pub status message
        self.pub_hole_status_line.publish(String(data=hole_status_msg))

        cv2.rectangle(cv_image, yolo_detect_rect[0], yolo_detect_rect[1], [50, 200, 200], 2)
        cv2.imshow('get_image', cv_image)

        cv2.waitKey(2)


#__Main__
if __name__ == "__main__":
    file_names = "yolov4-pin.names"
    file_cfg = "yolov4-pin.cfg"
    file_weights = "yolov4-pin-v2.weights"

    with open("./yolo/" + file_names, 'r') as f:
        classes = f.read().splitlines()

    net = cv2.dnn.readNetFromDarknet("./yolo/" + file_cfg, "./yolo/" + file_weights)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    model = cv2.dnn_DetectionModel(net)
    detect_width = 384
    detect_height = 288
    model.setInputParams(scale=1 / 255, size=(detect_width, detect_height), swapRB=True)

    
    rclpy.init()
    yolo_service = YoloService()
    rclpy.spin(yolo_service)
    rclpy.shutdown()
