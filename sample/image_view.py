# -*- coding: utf-8 -*-
from __future__ import unicode_literals, print_function
import os

import numpy as np
try:
    import cv2 as cv
    _cv_imported = True
except (ImportError):
    import matplotlib.pyplot as plt
    _cv_imported = False

import rospy
from sensor_msgs.msg import Image


class ImageView(object):
    """
    rosから送られてきた画像を表示するクラス

    Parameters
    ----------
    image_topic_name: str,
        受信する画像のトピック名
    """
    def __init__(self, image_topic_name=None):
        # importできたモジュールに応じてコールバック関数を変える
        if _cv_imported:
            image_callback_func = self.imageCBbyCv
        else:
            image_callback_func = self.imageCBbyMatplotlib
        self.sub = rospy.Subscriber("hsrb/head_rgbd_sensor/rgb/image_color",
                                     Image,
                                     image_callback_func)

    def imageCBbyCv(self, msg):
        # 受信データをnumpy-arrayに変換
        img = np.fromstring(msg.data, dtype=np.byte).reshape((msg.height, msg.width, 3))
        # データをunsigned intにキャスト
        img = img.astype(np.uint8)

        # show
        cv.imshow("image", img)
        cv.waitKey(10)

    def imageCBbyMatplotlib(self, msg):
        # 受信データをnumpy-arrayに変換
        img = np.fromstring(msg.data, dtype=np.byte).reshape((msg.height, msg.width, 3))
        # データをunsigned intにキャスト
        img = img.astype(np.uint8)
        # BGR2RGB
        img = img[:, :, ::-1]

        # plot
        plt.imshow(img)
        plt.pause(0.01)


if __name__ == "__main__":
    os.environ["ROS_MASTER_URI"] = "http://192.168.1.70:11311"
    os.environ["ROS_IP"] = "192.168.1.130"

    rospy.init_node("joe_image_view")
    view = ImageView()
    rospy.spin()