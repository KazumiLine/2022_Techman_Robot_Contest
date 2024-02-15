#!/usr/bin/env python3

from __future__ import print_function

import sys
import rclpy
from tm_msgs.srv import *
import geometry_msgs.msg
import numpy as np
from std_msgs.msg import String, Int32MultiArray, Float64MultiArray

rclpy.init()
node = rclpy.create_node('demo_send_script')
node.get_logger().info('Created node')

def tm_send_script_client(cmd):
    tm_send_script = node.create_client(SendScript, '/send_script')
    while not tm_send_script.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('tm_send_script_client not available, waiting again...')
    future = tm_send_script.call_async(SendScript.Request(id="demo", script=cmd))
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def tm_send_gripper_client(grisp: bool):
    '''
    SetIORequest_()
    : module(0)
    , type(0)
    , pin(0)
    , state(0.0)  {
    }
    '''
    tm_send_io = node.create_client(SetIO, '/set_io')
    while not tm_send_io.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('tm_send_io not available, waiting again...')
    if grisp:
        future = tm_send_io.call_async(SetIO.Request(module=SetIO_Request.MODULE_ENDEFFECTOR, type=SetIO_Request.TYPE_DIGITAL_OUT, pin=0, state=1.0))
    else:
        future = tm_send_io.call_async(SetIO.Request(module=SetIO_Request.MODULE_ENDEFFECTOR, type=SetIO_Request.TYPE_DIGITAL_OUT, pin=0, state=0.0))
    rclpy.spin_until_future_complete(node, future)
    return future.result()


if __name__ == "__main__":

    # print(tm_send_script_client(node, "PTP(\"CPP\",610,0,400,0,180,-90,100,200,0,false)"))
    # print(tm_send_gripper_client(False))
    print(tm_send_script_client(input()))

    # rclpy.spin(node)
