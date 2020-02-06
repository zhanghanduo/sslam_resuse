#!/usr/bin/env python2

import os
import sys
import rospkg
sys.path.append('.')

from calc2 import vh, vw, vss
# from utils import *

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from obstacle_msgs.msg import _Image_desc
from obstacle_msgs.msg import _Keypoint
import numpy as np
import tensorflow as tf
import rospy

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2

'''
CalcNet to extract key points with their descriptors.
'''


def kp_descriptor(tensor_):
    b, h, w, c = tensor_.shape
    assert b == 1
    ky = []
    kx = []
    theta_full = []
    response_full = []
    n = 4
    for i in range(n):
        for j in range(n):
            _h = h // n
            _w = w // n
            _t = tensor_[0, i * _h:(i + 1) * _h, j * _w:(j + 1) * _w]
            ky_, kx_ = np.unravel_index(np.argmax(
                _t.reshape(-1, c), axis=0), (_h, _w))
            ky.append(ky_ * (i + 1))
            kx.append(kx_ * (j + 1))

            _t = np.pad(_t, ((1, 1), (1, 1), (0, 0)), 'constant')

            for k in range(len(ky_)):
                _ky = ky_[k] + 1  # +1 for pad
                _kx = kx_[k] + 1
                _y = _t[_ky + 1, _kx, k] - _t[_ky - 1, _kx, k]
                _x = _t[_ky, _kx + 1, k] - _t[_ky, _kx - 1, k]
                theta_full.append(np.arctan2(_y, _x))
                response_full.append(_t[ky_[k], kx_[k], k])

    ky = np.concatenate(ky, axis=0)[..., np.newaxis]
    kx = np.concatenate(kx, axis=0)[..., np.newaxis]
    kp_full = np.concatenate((ky, kx), axis=1)
    kp = np.unique(kp_full, axis=0)
    # Keep unique kp with max activation
    mapping = {}
    for kp_i in kp:
        mapping[kp_i.tostring()] = np.where(kp_full == kp_i)[0]

    theta = np.empty((len(kp)), dtype=np.float32)
    response = np.empty((len(kp)), dtype=np.float32)
    for i in range(len(kp)):
        kp_i = kp
        inds = mapping[kp[i].tostring()]
        r = -np.inf
        t = -1
        for j in inds:
            if response_full[j] > r:
                r = response_full[j]
                t = theta_full[j]
        response[i] = r
        theta[i] = t
    ky = kp[:, 0]
    kx = kp[:, 1]

    ky = np.minimum(np.maximum(1, ky), h - 2)
    kx = np.minimum(np.maximum(1, kx), w - 2)

    kp_d = []
    kp = []
    pi = np.pi

    # tensor = np.pad(tensor, ((0,0),(1,1),(1,1),(0,0)), 'constant')

    for i in range(len(ky)):
        ky_i = ky[i] + 1
        kx_i = kx[i] + 1

        kp.append(cv2.KeyPoint(float(kx[i]), float(ky[i]),
                               _size=1.0, _response=10000 * np.log(1 + np.exp(response[i])), _angle=theta[i]))
        t = theta[i]
        od = [tensor_[:, ky_i - 1, kx_i - 1],
              tensor_[:, ky_i - 1, kx_i],
              tensor_[:, ky_i - 1, kx_i + 1],
              tensor_[:, ky_i, kx_i - 1],
              tensor_[:, ky_i, kx_i + 1],
              tensor_[:, ky_i + 1, kx_i - 1],
              tensor_[:, ky_i + 1, kx_i],
              tensor_[:, ky_i + 1, kx_i + 1]]

        _d = np.concatenate(tuple(od),
                            axis=0).reshape(1, -1, c)

        d = _d - tensor_[:, ky_i, kx_i]
        d = d.reshape(-1, c)
        kp_d.append(d.reshape(1, -1))
    kp_d = np.concatenate(kp_d, axis=0)
    return kp, kp_d


class ImageDescriptor():
    '''
    To publish the extracted image descriptors.
    '''

    def __init__(self):
        '''
        node initialization
        '''
        if rospy.has_param('~image_topic'):
            self.image_topic = rospy.get_param('~image_topic', '/left/image_rect')
        print(self.image_topic)

        if rospy.has_param('~output_topic'):
            self.output_topic = rospy.get_param('~output_topic', '/img_desc')

        self._cv_bridge = CvBridge()
        self._session = tf.Session()

        self.loops = []
        self.loop_count = 0
        self.last_loop_id = -1
        self.skipped = False

        self.images = tf.compat.v1.placeholder(tf.float32, [None, vh, vw, 3])
        ret = vss(self.images, False, True,
                  ret_mu=False, ret_c_centers=False,
                  ret_c5=True)
        self.descriptor = ret[0]
        self.c5 = ret[1]

        pkg_path = rospkg.RosPack()
        current_pkg = pkg_path.get_path('sslam')
        model_path = current_pkg + '/scripts/loopDetect/model'
        saver = tf.compat.v1.train.Saver()
        ckpt = tf.compat.v1.train.get_checkpoint_state(model_path)
        cpath = ckpt.model_checkpoint_path

        print("loading model: ", cpath)
        saver.restore(self._session, cpath)

        # init = tf.global_variables_initializer()
        # self._session.run(init)
        # self.graph = tf.get_default_graph()
        self._sub = rospy.Subscriber(self.image_topic, Image, self.callback, queue_size=10)
        self._pub = rospy.Publisher(self.output_topic, _Image_desc.Image_desc, queue_size=1)

    def callback(self, image_msg):
        '''call back funcion, which will send the image and category of each pixel'''
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "rgb8")
        h_ori, w_ori = cv_image.shape[:2]

        # with self.graph.as_default():
        #     probs = self.pspnet.model.predict(self.img_proc(cv_image))[0]

        im = cv2.resize(cv_image, (vw, vh))

        # with self._session as sess:
        if len(im.shape) == 2:
            # Grayscale
            im = np.repeat(im[..., np.newaxis], 3, axis=-1)
        if len(im.shape) == 3:
            im = im[np.newaxis, ...]

        descr, c5 = self._session.run(
            [self.descriptor, self.c5],
            feed_dict={self.images: im})

        kps, kp_d = kp_descriptor(c5)

        # rospy.loginfo("running")

        output = _Image_desc.Image_desc()
        output.header = image_msg.header
        output.keypoints = []

        # pts = cv2.KeyPoint_convert(kps)

        for kp_ in kps:
            keypoint_ = _Keypoint.Keypoint()
            keypoint_.pt.x = kp_.pt[0]
            keypoint_.pt.y = kp_.pt[1]
            keypoint_.size = kp_.size
            keypoint_.angle = kp_.angle
            keypoint_.class_id = kp_.class_id
            keypoint_.octave = kp_.octave
            keypoint_.response = kp_.response
            # keypoint_ = kp_
            output.keypoints.append(keypoint_)

        output.keypoint_descriptors = self._cv_bridge.cv2_to_imgmsg(kp_d)

        output.image_descriptor = self._cv_bridge.cv2_to_imgmsg(descr)

        # h_ = output.image_descriptor.height
        # w_ = output.image_descriptor.width
        # print("image desc shape: {0} | {1}".format(h_, w_))

        # print("keypoint shape: {0}".format(len(output.keypoints)))
        # print("keypoint type: {0}".format(type(output.keypoints)))

        self._pub.publish(output)


if __name__ == '__main__':
    rospy.init_node('loop_node')
    ImageDescriptor()
    rospy.spin()
