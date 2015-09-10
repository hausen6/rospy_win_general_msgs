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
    def __init__(self, image_topic_name):
        # importできたモジュールに応じてコールバック関数を変える
        if _cv_imported:
            image_callback_func = self.imageCBbyCv
        else:
            image_callback_func = self.imageCBbyMatplotlib
        self.sub = rospy.Subscriber(image_topic_name,
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
    from argparse import ArgumentParser
    parser = ArgumentParser()

    parser.add_argument("topic_name",
                        type=str,
                        help="受信する画像のトピック名")

    # 環境変数(rosのip周りの設定をチェック)
    ros_master = False
    ros_ip = False
    if "ROS_MASTER_URI" in os.environ:
        ros_master = True
    if "ROS_IP" in os.environ:
        ros_ip = True
    # 環境変数がセットされていない場合は引数から設定
    if not ros_master:
        parser.add_argument("ros_master_uri",
                            type=str,
                            help="ROS_MASTER_URI (接続先IP)")
    if not ros_ip:
        parser.add_argument("ros_ip",
                            type=str,
                            help="ROS_IP (自分のIP)")

    args = parser.parse_args()

    # 環境変数に値をセット
    if not ros_master:
        os.environ["ROS_MASTER_URI"] = args.ros_master_uri
    if not ros_ip:
        os.environ["ROS_IP"] = args.ros_ip

    rospy.init_node("joe_image_view")
    view = ImageView(args.topic_name)
    rospy.spin()