import cv2
import mediapipe as mp
import imutils
import numpy as np
import json, os, sys, math
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
'''
沒有註解
import沒有放在一起
沒有標注程式開始點(Main)

重要的參數要放上面
善用下一行區分是在做什麼事情(also include 'if else')
參數命名沒問題, 但適合中型程式, 不要怕長, 大型程式怕的是不清楚。有些忘記大小寫getRect
有些數值一樣的，請變成變數，尤其是明顯是不同段落的
'''


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __str__(self):
        return 'Point(x: %d, y: %d)' % (self.x, self.y)
    def __repr__(self):
        return 'Point(x: %d, y: %d)' % (self.x, self.y)


def Left_index(points):
    minn = 0
    points.sort(key=lambda p: (p.x, p.y))
    count = {}
    removed = []
    for i in range(len(points)):
        if i >= len(points): break
        if points[i].x not in count: count[points[i].x] = 0
        count[points[i].x] += 1
        if count[points[i].x] > 2:
            removed.append(points[i-1])
    for i in removed:
        points.remove(i)
    for i in range(1, len(points)):
        if points[i].x < points[minn].x:
            minn = i
        elif points[i].x == points[minn].x:
            if points[i].y > points[minn].y:
                minn = i
    return minn


def orientation(p, q, r):
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2


def convexHull(points):
    res = []
    if len(points) < 3:
        return res
    l = Left_index(points)
    n = len(points)
    hull = []
    p = l
    q = 0
    while (True):
        hull.append(p)
        q = (p + 1) % n
        for i in range(n):
            d = orientation(points[p], points[i], points[q])
            if (d == 2):
                q = i
        p = q
        if (p == l):
            break
    for each in hull:
        res.append((points[each].x, points[each].y, 1))
    return res


#___Main___
rclpy.init()
node = rclpy.create_node('hand_detector')

pub_hand = node.create_publisher(String, 'hand_detector', 1)
pub_image = node.create_publisher(Image, '/camera/image', 1)
pub_hand_image = node.create_publisher(Image, '/hand/image', 1)

#v4l2-ctl --list-devices
cap = cv2.VideoCapture(int(sys.argv[1]))
status = "init"
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
points = []
bridge = CvBridge()

while rclpy.ok():
    ret, image = cap.read()
    #image: 1280x720
    if not ret:
        print("Ignoring empty camera frame.")
        continue
    image = cv2.flip(image, 1)

    image_message = bridge.cv2_to_imgmsg(image, "bgr8")
    pub_image.publish(image_message)

    image = image[120:-120, 320:-320, :]  #new image: 640x480

    if status == "init":
        gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        thresh_img = cv2.threshold(gray_img, 150, 255, cv2.THRESH_BINARY)[1]
        cnts = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        mask = np.zeros(gray_img.shape, dtype=np.uint8)
        getrect = False

        #找方形
        for c in cnts:
            M = cv2.moments(c)
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)

            if len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                #找到算阿(效率問題
                # pts1 = np.float32([approx[0][0], approx[1][0], approx[2][0], approx[3][0]])
                # pts2 = np.float32([[300, 0], [0, 0], [0, 300], [300, 300]])

                #找到的座標是不是常常換來換去, 旋轉出現問題！！
                if x != 0 and y != 0 and w > 100 and h > 100:
                    pts1 = np.float32([approx[0][0], approx[1][0], approx[2][0], approx[3][0]])
                    if approx[1][0][1] < approx[3][0][1]:  #another case
                        pts1 = np.float32([approx[1][0], approx[2][0], approx[3][0], approx[0][0]])
                    pts2 = np.float32([[0, 0], [0, 300], [300, 300], [300, 0]])
                    trans = cv2.getPerspectiveTransform(pts1, pts2)
                    cv2.drawContours(image, [c], -1, (255, 0, 0), 3)
                    cv2.fillConvexPoly(mask, c, (255, 255, 255))
                    gray_img = cv2.bitwise_and(gray_img, gray_img, mask=mask)
                    mask = np.ones(gray_img.shape, dtype=np.uint8) * np.min(gray_img)
                    cv2.fillConvexPoly(mask, c, (0, 0, 0))
                    gray_img = cv2.bitwise_or(mask, gray_img)
                    getrect = True
                    cv2.putText(image, '0', (pts1[0]).astype('int32'), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(37, 240, 34), thickness=1)
                    cv2.putText(image, '1', (pts1[1]).astype('int32'), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(37, 240, 34), thickness=1)
                    cv2.putText(image, '2', (pts1[2]).astype('int32'), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(37, 240, 34), thickness=1)
                    cv2.putText(image, '3', (pts1[3]).astype('int32'), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color=(37, 240, 34), thickness=1)
        #ISSUE: 超過兩個怎麼辦？？

        #Draw hole in rect
        # if getrect:
        #     blur = cv2.GaussianBlur(gray_img, (5, 5), 0)
        #     canny = cv2.Canny(blur, 150, 200)
        #     cv2.imshow('canny', canny)
        #     (cnts, _) = cv2.findContours(canny, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #     cv2.drawContours(gray_img, cnts, -1, (0,255,0), 3)
        #     points = []
        #     for cnt in cnts:
        #         hull = cv2.convexHull(cnt)
        #         if len(hull) > 4 :
        #             points.append((sum(hull)[0] / len(hull)).astype('int32'))
        #             cv2.circle(image, (sum(hull)[0] / len(hull)).astype('int32'), 5, (255, 255, 0), -1)
        cv2.putText(image, '(q)Quit; (r)Detect Hand', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow('frame', image)

    elif status == "hand":
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        # mask = np.zeros(image.shape[:2], dtype=np.uint8)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                myhand = []
                for landmark in hand_landmarks.landmark:
                    if landmark.x < 0:
                        landmark.x = 0
                    if landmark.y < 0:
                        landmark.y = 0
                    myhand.append(Point(int(landmark.x * image.shape[1]), int(landmark.y * image.shape[0])))
                # myhand.append(Point(5*myhand[0].x-4*myhand[5].x, 5*myhand[0].y-4*myhand[5].y))
                res = np.array(convexHull(myhand), dtype=np.int32)
                #視角轉換
                dst = cv2.warpPerspective(image, trans, (300, 300))
                dst = cv2.rotate(dst, cv2.ROTATE_90_COUNTERCLOCKWISE)
                #res transform
                max_x = 0
                min_x = 300
                canny = cv2.Canny(dst[-15:-5, 20:-20], 30, 100)
                cv2.imshow('canny', canny)
                for i in range(260):
                    if np.sum(canny[:,i]) > 0:
                        min_x = i
                        break
                for i in range(260):
                    if np.sum(canny[:,259-i]) > 0:
                        max_x = 259-i
                        break
                for i, p in enumerate(res):
                    res[i] = np.dot(trans, res[i]).astype(np.int32)
                    if res[i][0] < 0:
                        res[i][0] = 0
                    elif res[i][0] > 300:
                        res[i][0] = 300
                    if res[i][1] < 0:
                        res[i][1] = 0
                    elif res[i][1] > 300:
                        res[i][1] = 300
                    res[i][0], res[i][1] = res[i][1], 300-res[i][0]
                    # if res[i][0] > max_x:
                    #     max_x = res[i][0]
                    # if res[i][0] < min_x:
                    #     min_x = res[i][0]
                res = res[:, :2].astype(np.int32)
                if max_x > 50 and min_x < 300:
                    res = np.concatenate([res, np.array([[max_x, 300], [min_x, 300]])])
                for i, p in enumerate(res):
                    if res[i][0] < 0:
                        res[i][0] = 0
                    elif res[i][0] > 300:
                        res[i][0] = 300
                    if res[i][1] < 0:
                        res[i][1] = 0
                    elif res[i][1] > 300:
                        res[i][1] = 300
                res = np.array(convexHull([Point(x[0], x[1]) for x in res]), dtype=np.int32)
                res = res[:, :2].astype(np.float64)
                center = sum(res)/len(res)
                for i in range(len(res)):
                    vec = res[i] - center
                    res[i] += vec / math.sqrt(np.linalg.norm(vec) + 0.0001)*5

                res = res.astype(np.int32)

                #draw hand range
                image_hand_poly = np.zeros(dst.shape, dtype=dst.dtype)
                cv2.fillPoly(image_hand_poly, [res], (50, 0, 100))
                dst = cv2.addWeighted(dst, 1, image_hand_poly, 0.5, 1.0)

                data = json.dumps({"convex": res.tolist()})
                pub_hand.publish(String(data=data))
                break
        else:
            dst = cv2.warpPerspective(image, trans, (300, 300))
            dst = cv2.rotate(dst, cv2.ROTATE_90_COUNTERCLOCKWISE)
    
        image_message = bridge.cv2_to_imgmsg(dst, "bgr8")
        pub_hand_image.publish(image_message)

        cv2.imshow('frame', dst)

        # for point in points:
        #     if mask[point[1]][point[0]] != 0:
        #         cv2.circle(image, point, 5, (255, 0, 255), -1)
        #     else:
        #         cv2.circle(image, point, 5, (255, 100, 100), -1)

    # cv2.putText(image, '(q)Quit; (i)Detect Rect; (r)Detect Hand', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    # cv2.imshow('frame', image)  #如果怎樣的情況都會show，那就合併即可

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('r'):
        status = "hand"
    elif key == ord('i'):
        status = "init"

cap.release()
cv2.destroyAllWindows()