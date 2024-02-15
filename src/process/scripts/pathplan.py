'''
180,0,90 vertical.
'''

from ast import Del
from operator import truediv
import string

from cv2 import ROTATE_90_CLOCKWISE
import cv2
import numpy as np
import random, time, math, json, re, threading, sys
import rclpy
from tm_msgs.srv import *
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty, Trigger
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge, CvBridgeError


frameSize = (300, 300, 3)
handpos = np.array([random.randint(100, frameSize[0] - 100), random.randint(100, frameSize[1] - 100)], dtype=np.int32)

unit_vector = np.array([0, 1], dtype=np.int32)
robotPos = np.array([0, 300])  #image base
robot_target = np.array([0, 0])
handshape = np.array([[76, -28], [38, 50], [-40, 54], [-72, -24], [-2, -52]])
status = "pause"

FLAG_realMove = True

####座標相關####
# 1. 到image(0,300)_in real的座標, 給定original_robot_pos
# 2. 依序給定points_in_robot_pos裡面的座標軸
place_points = np.array([(75, 225), (75, 150), (75, 75), (150, 225), (150, 150), (150, 75), (225, 225), (225, 150), (225, 75)], dtype=np.int32)  #square hole #image base
place_points_in_robot_pos = np.ones((len(place_points), 2), dtype=np.float64)  #robot base # unit: M
reached_place_points = np.zeros(len(place_points), dtype=np.bool_)

pick_points = np.array([(20, -25),(40, -25),(60, -25),(80, -25), (100, -25),(120, -25),(140, -25),(160, -25),(180, -25),(200, -25)], dtype=np.int32)
pick_points_in_robot_pos = np.ones((len(pick_points), 2), dtype=np.float64) #robot base # unit: M
reached_pick_points = np.ones(len(pick_points), dtype=np.bool_)
pick_point_index = 0  #可以夾的，第一順位 index

##座標轉換##
original_image_pos = [0, 300]
original_robot_pos = [0.400, -0.150]  #Unit: M
robot_z_high = 0.15  #unit: M
robot_z_down = 0.1  #unit: M

###parameters###
hand_safe_distance = 5  #pixel

bridge = CvBridge()
rclpy.init()
node = rclpy.create_node('pathplan_node')
pub_image = node.create_publisher(Image, '/pathplan/image', 1)
futures = []
joint_position = [0, 0, 0, 0, 0, 0]
tool_pose = [0, 0, 0, 0, 0, 0, 0]


def pol2cart(r,theta):
    z = float(r) * np.exp(1j * float(theta))
    x, y = z.real, z.imag
    return x, y

def cart2pol(x, y):
    z = float(x) + float(y) * 1j
    r,theta = np.abs(z), np.angle(z)
    return r,theta

def Delay(sec):
    for _ in range(5 * sec):
        UpdateImage()
        # cv2.waitKey(200)

def tm_send_script_client(cmd: str):
    global FLAG_realMove, futures
    if "JPP" in cmd:
        data = re.findall(r"[-+]?(?:\d*\.*\d+)", cmd)
        data[0] = str(float(data[0])+45)
        cmd = 'PTP("JPP",'+','.join(data)+',false)'
    elif "CPP" in cmd:
        data = re.findall(r"[-+]?(?:\d*\.*\d+)", cmd)
        r, theta = cart2pol(data[0], data[1])
        newx, newy = pol2cart(r, theta+math.pi/4)
        data[0], data[1] = str(newx), str(newy)
        cmd = 'PTP("CPP",'+','.join(data)+',false)' 
    elif "SetContinueVLine" in cmd:
        data = re.findall(r"[-+]?(?:\d*\.*\d+)", cmd)
        if data[0] != '0' or data[1] != '0':
            r, theta = cart2pol(data[0], data[1])
            newx, newy = pol2cart(r, theta+math.pi/4)
            data[0], data[1] = str(newx), str(newy)
            cmd = 'SetContinueVLine('+','.join(data)+')' 
    if FLAG_realMove == True:
        tm_send_script = node.create_client(SendScript, '/send_script')
        while not tm_send_script.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('tm_send_script_client not available, waiting again...')
        future = tm_send_script.call_async(SendScript.Request(id="demo", script=cmd))
        rclpy.spin_until_future_complete(node, future)
        if "JPP" in cmd:
            check_arm("JPP", data)
        elif "CPP" in cmd:
            check_arm("CPP", data)
        # return future.result()
    else:
        print(cmd)


def tm_send_gripper_client(grisp: bool):
    global FLAG_realMove, futures
    if FLAG_realMove == True:
        tm_send_io = node.create_client(SetIO, '/set_io')
        while not tm_send_io.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('tm_send_io not available, waiting again...')
        if grisp:
            future = tm_send_io.call_async(SetIO.Request(module=SetIO_Request.MODULE_ENDEFFECTOR, type=SetIO_Request.TYPE_DIGITAL_OUT, pin=0, state=1.0))
        else:
            future = tm_send_io.call_async(SetIO.Request(module=SetIO_Request.MODULE_ENDEFFECTOR, type=SetIO_Request.TYPE_DIGITAL_OUT, pin=0, state=0.0))
        rclpy.spin_until_future_complete(node, future)
        return future.result()
    else:
        print(grisp)


def ng_detect_client():
    global futures
    trigger_request = node.create_client(Trigger, '/ng_detect')
    while not trigger_request.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('trigger_request not available, waiting again...')
    future = trigger_request.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def set_points_to_robot_points():
    global place_points, place_points_in_robot_pos, pick_points, pick_points_in_robot_pos
    for i, p in enumerate(place_points):
        place_points_in_robot_pos[i] = Image2Robot(place_points[i])
    for i, p in enumerate(pick_points):
        pick_points_in_robot_pos[i] = Image2Robot(pick_points[i])

def Robot2Image(robot_point):
    global original_image_pos, original_robot_pos
    # print("robot: (%f, %f)" % (robot_point[0], robot_point[1]))
    #robot_point => Unit: M
    #image (0, 300) = Robot arm (x, y)  ##1 pixel = 1mm
    #image x_axis = robot x_axis
    #image y_axis = robot -y_axis
    return np.array((original_image_pos[0] + (robot_point[0] - original_robot_pos[0]) * 1000, (original_image_pos[1] - (robot_point[1] - original_robot_pos[1]) * 1000)), dtype=np.int32)


def Image2Robot(image_point):
    global original_image_pos, original_robot_pos
    #image (0, 300) = Robot arm (x, y)  ##1 pixel = 1mm
    #image x_axis = robot x_axis
    #image y_axis = robot -y_axis
    return np.array(((original_robot_pos[0] * 1000 + (image_point[0] - original_image_pos[0])) / 1000, (original_robot_pos[1] * 1000 - (image_point[1] - original_image_pos[1])) / 1000), dtype=np.float64)  #Unit M


set_points_to_robot_points()

def FixCoordinate_vector(vector):  #我不知道怎麼算的...只能先這樣修改
    # switch x y
    # return np.array([unit_vector[1], unit_vector[0]])
    return np.array([vector[0], -vector[1]])


def intersect(p1, p2, p3, p4):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4
    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if denom == 0:
        return None
    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    if ua < 0 or ua > 1:
        return None
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom
    if ub < 0 or ub > 1:
        return None
    x = x1 + ua * (x2 - x1)
    y = y1 + ua * (y2 - y1)
    return (x, y)


def get_safe_dst(com, extraLen):
    hand = handshape + handpos
    for i in range(len(hand)):
        inter = intersect(hand[i - 1], hand[i], handpos, com)
        if inter != None:
            return np.linalg.norm(handpos - inter) + extraLen
    return 100


def is_inside(polygon, point):
    length = len(polygon) - 1
    dy2 = point[1] - polygon[0][1]
    intersections = 0
    ii = 0
    jj = 1
    while ii < length:
        dy = dy2
        dy2 = point[1] - polygon[jj][1]
        if dy * dy2 <= 0.0 and (point[0] >= polygon[ii][0] or point[0] >= polygon[jj][0]):
            if dy < 0 or dy2 < 0:
                F = dy * (polygon[jj][0] - polygon[ii][0]) / (dy - dy2) + polygon[ii][0]
                if point[0] > F:
                    intersections += 1
                elif point[0] == F:
                    return 2
            elif dy2 == 0 and (point[0] == polygon[jj][0] or (dy == 0 and (point[0] - polygon[ii][0]) * (point[0] - polygon[jj][0]) <= 0)):
                return 2
        ii = jj
        jj += 1
    return intersections & 1


def wtf_unit_vec(vec):
    _unit_vector = vec / (math.sqrt(np.linalg.norm(vec) + 0.0001) * 2)
    dir = ((vec > 0) * 2 - np.ones(2)).astype(np.int32)
    _unit_vector = np.ceil(abs(_unit_vector)) * dir
    _unit_vector = _unit_vector.astype(np.int32)
    return _unit_vector


def check_collision():  #命名怪怪的耶
    # collision = np.zeros(len(points), dtype=np.bool)
    for i, p in enumerate(place_points):
        if np.linalg.norm(robotPos - p) < 8:
            # collision[i] = True
            return i
    # return collision
    return -1


def handle_joint_states(msg):
    global joint_position
    joint_position = msg.position


def handle_tool_pose(msg):
    global tool_pose
    tool_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

def check_arm(ttype: str, data):
    not_stoped = True
    while not_stoped:
        rclpy.spin_once(node, timeout_sec=1)
        time.sleep(0.2)
        if ttype == "JPP":
            for idx, deg in enumerate(joint_position):
                if deg/math.pi*180-float(data[idx])>1:
                    break
            else:
                not_stoped = False
        elif ttype == "CPP":
            print("checking_arm", [pos*1000 for pos in tool_pose[:3]], data[:3])
            for idx, pos in enumerate(tool_pose[:2]):
                if pos*1000-float(data[idx])>10:
                    break
            else:
                not_stoped = False
    print("checked_arm")
    time.sleep(1)
                

# ============================================================================================== #
# 夾取函數 index 0-8 為盤面 9-18 為小盤PIN
def grasp_pin(index: int, is_test=False) -> bool:
    global reached_pick_points
    set_points_to_robot_points()
    #if 9-18 grip only
    #0-8 place only
    if (index <= 8):
        print("place in index %d" % index)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_down * 1000))
        if not is_test: tm_send_gripper_client(False)
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (place_points_in_robot_pos[index][0] * 1000, place_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        Delay(5)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
    elif (index >= 9):
        index = index - 9
        print("pick in index %d" % (index))
        #set to empty in reached
        reached_pick_points[index] = False
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("StopContinueVmode()")
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (pick_points_in_robot_pos[index][0] * 1000, pick_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (pick_points_in_robot_pos[index][0] * 1000, pick_points_in_robot_pos[index][1] * 1000, robot_z_down * 1000))
        if not is_test: tm_send_gripper_client(True)
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (pick_points_in_robot_pos[index][0] * 1000, pick_points_in_robot_pos[index][1] * 1000, robot_z_high * 1000))
        Delay(5)
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
        tm_send_script_client("ContinueVLine(300, 500)")
        tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
    return True


# 選擇放置目標
def robot_place(targets):
    global reached_place_points, status

    collision = check_collision()
    if collision >= 0:
        if not reached_place_points[collision]:  #無意義的判斷state == "place/put", 唯一入口在main的判斷裡面
            # reached[collision] = True
            if grasp_pin(collision):  ##if中的判斷式 盡量不要寫功能型的function, if在閱讀上就是判斷,這樣會感覺重點是有沒有成功夾取, 而不是夾取這個行為
                status = "pick"

    target = np.array([0, 0])
    minDst = np.Infinity
    found = False
    while not found:
        rclpy.spin_once(node, timeout_sec=1)
        for i, p in enumerate(targets):
            safeDst = get_safe_dst(p, hand_safe_distance)
            if reached_place_points[i]:
                continue
            elif is_inside(handshape + handpos, p):
                continue
            elif np.linalg.norm(handpos - p) < safeDst:  #is_inside(handshape + handpos, p):
                continue
            elif np.linalg.norm(robotPos - p) < minDst:
                target = p
                minDst = np.linalg.norm(robotPos - p)
                found = True

    # if (found == False):  #沒找到就結束
    #     status = "end"

    bot_move(target)


# 路徑規劃
def bot_move(target):
    global robotPos, unit_vector, robot_target
    robot_target = target
    safeDst = get_safe_dst(robotPos, hand_safe_distance)
    v = []
    # 防邊緣超出
    # v.append(np.array((botpos[0],0)))
    # v.append(np.array((botpos[0]-frameSize[0],0)))
    # v.append(np.array((0,botpos[0])))
    # v.append(np.array((0,botpos[0]-frameSize[1])))
    v.append(robotPos - handpos)
    sv = np.array((0, 0), dtype=np.float32)
    for x in v:
        x_len = np.linalg.norm(x)
        x.astype(np.float32)
        if x_len < safeDst:
            sv += (x / x_len) * (safeDst - x_len)

    if np.linalg.norm(sv) > 1:  #如果overlap手的話
        # hand = handpos + handshape
        # if is_inside(hand, botpos):
        # direction = sv
        direction = np.array((0, -10))  #圖片方向不一樣阿，明明就是向左！？
        # else:
        # up, down = get_tangents_vec(hand, botpos)
        # # = np.dot((botpos-hand)[tangents[0]], (target-botpos)), np.dot((botpos-hand)[tangents[1]], (target-botpos))
        # # print(up, down)
        # # if up > down and abs(up-down)>abs(up+down)/10:
        # # direction = sv + (botpos-hand)[tangents[0]]/2
        # # else:
        # # direction = sv + (hand[up] - hand[down]) / 10
        # if hand[up][1] < hand[down][1]:
        # direction = sv + hand[up] / 10
        # else:
        # direction = sv + hand[down] / 10
    else:  #沒有overlap任何東西, 正常走
        direction = target - robotPos + sv

    unit_vector = wtf_unit_vec(direction)  #direction: vector of P1 to P2
    global FLAG_realMove
    if FLAG_realMove == True:
        r, theta = cart2pol(tool_pose[0], tool_pose[1])
        # TM指令區 #
        robotPos = Robot2Image(pol2cart(r, theta-math.pi/4))  #np.array(((trans[0] - 0.3) * 1000, (trans[1] + 0.15) * 1000), dtype=np.int32)
        # print("Vline: (%f, %f)" % (float(unit_vector[0]) / 200, float(unit_vector[1]) / 200))
        unit_vector = FixCoordinate_vector(unit_vector)
        # print(robotPos)
        tm_send_script_client("SetContinueVLine(%f, %f, 0, 0, 0, 0)" % (float(unit_vector[0]) / 200, float(unit_vector[1]) / 200))
    elif FLAG_realMove == False:
        robotPos += unit_vector  #simulate
        print("Vline: (%f, %f)" % (unit_vector[0], unit_vector[1]))
        time.sleep(0.1)


# 回原點取卯
def robot_pick(index: int):
    global status
    target = np.array([0, 0])  #給予初始值 避免全部都拿完了,target會找不到
    for i, p in enumerate(reached_pick_points):
        if (p == True):
            target = pick_points[i]
            pick_point_index = i
            break

    if np.linalg.norm(robotPos - target) < 8:
        if grasp_pin(9 + pick_point_index):
            status = "place"

    bot_move(target)


#abandon
def get_tangents_vec(a, p):

    def sqDist(p1, p2):
        return ((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

    def orientation(a, b, c):
        res = ((b[1] - a[1]) * (c[0] - b[0]) - (c[1] - b[1]) * (b[0] - a[0]))
        if (res == 0):
            return 0
        if (res > 0):
            return 1
        return -1

    ind = 0
    n = len(a)
    for i in range(1, n):
        if (sqDist(p, a[i]) < sqDist(p, a[ind])):
            ind = i
    up = ind
    while (orientation(p, a[up], a[(up + 1) % n]) >= 0):
        up = (up + 1) % n
    low = ind
    while (orientation(p, a[low], a[(n + low - 1) % n]) <= 0):
        low = (n + low - 1) % n
    return up, low


# 接收手部位置與圖像
def handle_hand_detector(data):
    global handpos, handshape
    # print(data.data)
    data = json.loads(data.data)
    hand = np.array(data["convex"], dtype=np.int32)
    handpos = np.array([np.sum(hand[:, 0]) / len(hand), np.sum(hand[:, 1]) / len(hand)], dtype=np.int32)
    handshape = hand - handpos


# 接收洞口更新資訊
def handle_hole_status(msg):
    global reached_place_points, reached_pick_points
    # 0100000101111112101
    # global pick_point_index
    if len(msg.data) != 19:
        return
    for i, ch in enumerate(msg.data[:9]):
        if ch == "0":
            reached_place_points[i] = False
        else:
            reached_place_points[i] = True

    #拿的部份, 有我就選, 不管順序
    for i, ch in enumerate(msg.data[9:]):
        if ch == "1":
            reached_pick_points[i] = True
        else:
            reached_pick_points[i] = False


# 鼠標控制(測試用)
def mouse_move(event, x, y, flags, param):
    global handpos
    handpos = np.array((x, y), dtype=np.int32)


# 繪製圖面
def UpdateImage():
    global status
    global FLAG_realMove
    global unit_vector
    hand = handshape + handpos
    img_process = np.ones(frameSize, dtype=np.uint8) * 100
    img_process_sub = np.ones((50, 300, 3), dtype=np.uint8) * 150 #宣告頭頂一塊空間
    cv2.rectangle(img_process_sub, (0, 0), (220, 50), (100, 100, 100), -1) #把小板的位置畫出來
    cv2.fillPoly(img_process, [hand], (100, 0, 200))

    # if status == "pick":
    #     if FLAG_realMove:
    #         cv2.circle(img_process, robotPos, 10, (181, 24, 37), 3)
    #     else:
    #         cv2.circle(img_process, robotPos, 10, (174, 214, 51), 3)
    # elif status == "place":
    #     if FLAG_realMove:
    #         cv2.circle(img_process, robotPos, 10, (181, 24, 37), -1)
    #     else:
    #         cv2.circle(img_process, robotPos, 10, (174, 214, 51), -1)

    # draw 9 square point
    for i, p in enumerate(place_points):
        if reached_place_points[i]:
            cv2.circle(img_process, p, 5, (100, 0, 100), -1)
            cv2.putText(img_process, str(i), (p[0], p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color=(100, 0, 100), thickness=1)
        else:
            cv2.circle(img_process, p, 5, (0, 200, 200), -1)
            cv2.putText(img_process, str(i), (p[0], p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.3, color=(0, 200, 200), thickness=1)
            
    # 繪製小板的十個點
    for i, p in enumerate(pick_points):
        if reached_pick_points[i] == 1:
            cv2.circle(img_process_sub, p-np.array([0, -50]), 6, (150, 0, 150), -1)
        elif reached_pick_points[i] == 2:
            cv2.circle(img_process_sub, p-np.array([0, -50]), 6, (0, 0, 240), -1)
        else:
            cv2.circle(img_process_sub, p-np.array([0, -50]), 6, (0, 200, 200), -1)

    #沒什麼意義複製,暫時先用來區分 畫一些無意義的文字
    img_show = np.concatenate((img_process_sub, img_process), axis=0)

    # 高端繪圖法 一行解千愁
    cv2.circle(img_show, robotPos-np.array([0, -50]), 10, (181, 24, 37) if FLAG_realMove else (174, 214, 51), -1 if status == "place" else 3)
    
    if (FLAG_realMove == True):
        unit_vector = FixCoordinate_vector(unit_vector)
    cv2.line(img_show, robotPos-np.array([0, -50]), robotPos-np.array([0, -50]) + unit_vector * 20, (162, 33, 176), 3)
    cv2.line(img_show, robotPos-np.array([0, -50]), robot_target-np.array([0, -50]), (69, 8, 13), 1)

    cv2.putText(img_show, 'Real(z)' if FLAG_realMove else 'Simulate(z)', (10, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (181, 24, 37) if FLAG_realMove else (174, 214, 51), 1, cv2.LINE_AA)
    cv2.putText(img_show, status, (250, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 82, 200), 1, cv2.LINE_AA)
    
    # cv2.imshow("map", img_show)
    image_message = bridge.cv2_to_imgmsg(img_show, "bgr8")
    pub_image.publish(image_message)

def handle_commands(req):
    global status, reached_pick_points, FLAG_realMove
    key = req.data
    node.get_logger().info(req.data)
    if key == 'init':
        status = "init"
    elif key == 'pause':
        status = "pause"
    elif key == 'start':
        status = 'init'
    elif key == 'end':
        status = "end"
    elif key == 'detect':
        status = 'detect'
    elif key == 'remove_ng_pin':
        status = 'remove_ng_pin'
    elif key == 'mode_sim':
        FLAG_realMove = False
    elif key == 'mode_real':
        FLAG_realMove = True
    elif key == 'm':
        tm_send_gripper_client(True)
    elif key == 'n':
        tm_send_gripper_client(False)
    elif key == '1':  # goto 1
        print('Robot go to index 0 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[0][0] * 1000, place_points_in_robot_pos[0][1] * 1000, robot_z_high * 1000))
    elif key == '2':  # goto 2
        print('Robot go to index 1 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[1][0] * 1000, place_points_in_robot_pos[1][1] * 1000, robot_z_high * 1000))
    elif key == '3':  # goto 3
        print('Robot go to index 2 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[2][0] * 1000, place_points_in_robot_pos[2][1] * 1000, robot_z_high * 1000))
    elif key == '4':  # goto 4
        print('Robot go to index 3 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[3][0] * 1000, place_points_in_robot_pos[3][1] * 1000, robot_z_high * 1000))
    elif key == '5':  # goto 5
        print('Robot go to index 4 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[4][0] * 1000, place_points_in_robot_pos[4][1] * 1000, robot_z_high * 1000))
    elif key == '6':  # goto 6
        print('Robot go to index 5 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[5][0] * 1000, place_points_in_robot_pos[5][1] * 1000, robot_z_high * 1000))
    elif key == '7':  # goto 7
        print('Robot go to index 6 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[6][0] * 1000, place_points_in_robot_pos[6][1] * 1000, robot_z_high * 1000))
    elif key == '8':  # goto 8
        print('Robot go to index 7 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[7][0] * 1000, place_points_in_robot_pos[7][1] * 1000, robot_z_high * 1000))
    elif key == '9':  # goto 9
        print('Robot go to index 8 pos')
        tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,50,200,0,false)' % (place_points_in_robot_pos[8][0] * 1000, place_points_in_robot_pos[8][1] * 1000, robot_z_high * 1000))
    elif key == '0':  # goto outside
        print('Robot go to OUTSIDE pos')
        tm_send_script_client('PTP("JPP",0.0,-44.1,115.1,135.0,-90.0,180.0,80,200,0,false)')  #out pos#TM5-700


# ============================================================================================== #

# ROS1實裝 #
#___Main___
if __name__ == "__main__":
    set_points_to_robot_points()
    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            tm_send_gripper_client(True)
            time.sleep(1)
            grasp_pin(int(sys.argv[1]), is_test=True)
        else:
            tm_send_script_client('PTP("JPP",0.0,-24.1,115.1,135.0,-90.0,180.0,80,200,0,false)') 
        time.sleep(5)
        rclpy.shutdown()
        sys.exit()

    node.create_subscription(String, '/hand_detector', handle_hand_detector, 1)
    node.create_subscription(String, '/hole_status', handle_hole_status, 1)
    node.create_subscription(String, '/pathplan/command', handle_commands, 1)
    node.create_subscription(JointState, '/joint_states', handle_joint_states, 1)
    node.create_subscription(PoseStamped, '/tool_pose', handle_tool_pose, 1)

    # ============================================================================================== #
    # cv2.namedWindow('map')
    # cv2.namedWindow('robot cam')
    # cv2.setMouseCallback('map', mouse_move)

    while rclpy.ok():
        rclpy.spin_once(node)
        if status == "init":  #robot back to initial point
            print('Initial')
            tm_send_gripper_client(False)
            tm_send_script_client("StopAndClearBuffer(0)")
            # PTP("JPP",0.0,-44.1,115.1,    135.0,-90.0,180.0   ,80,200,0,false)
            tm_send_script_client('PTP("JPP",0.0,-24.1,115.1,135.0,-90.0,180.0,80,200,0,false)')  # higher pos #TM5-700
            Delay(1)
            tm_send_script_client('PTP("JPP",0.0,-14.1,115.1,125.0,-90.0,180.0,80,200,0,false)')  #ready pos #TM5-700
            Delay(1)
            tm_send_script_client('PTP("CPP",%f,%f,%f,135.0,0,135.0,35,200,0,false)' % (original_robot_pos[0] * 1000, original_robot_pos[1] * 1000, robot_z_high * 1000))
            Delay(2)
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            tm_send_script_client("ContinueVLine(300, 500)")
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            status = "pick"
        elif status == "detect":
            #find NG pin
            tm_send_script_client('PTP("JPP",0.0,-20.0,90.0,125.0,-90.0,180.0,80,200,0,false)')  #camera pos #TM5-700
            tm_send_script_client('ScriptExit(0)')
            Delay(5)
            ng_detect_client()
            tm_send_script_client('PTP("JPP",0.0,-44.1,115.1,135.0,-90.0,180.0,80,200,0,false)')  # out pos #TM5-700
            tm_send_script_client('PTP("JPP",0.0,-24.1,115.1,135.0,-90.0,180.0,80,200,0,false)')  # higher pos #TM5-700
            status = "pause"
        elif status == "remove_ng_pin":
            status = "pause"
        elif status == "pause":
            pass
        elif status == "pick":
            # print('Pick')
            robot_pick(0)
        elif status == "place":
            # print('Place')
            robot_place(place_points)
        elif status == "end":
            print('End')
            tm_send_script_client("SetContinueVLine(0, 0, 0, 0, 0, 0)")
            tm_send_script_client("StopContinueVmode()")
            tm_send_script_client('PTP("JPP",0.0,-14.1,115.1,125.0,-90.0,180.0,80,200,0,false)')  #TM5-700
            status = "pause"

        ###draw image###
        UpdateImage()

    ###node exit###
    tm_send_script_client("StopContinueVmode()")
    cv2.destroyAllWindows()
