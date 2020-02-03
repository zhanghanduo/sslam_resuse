from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import math
import numpy as np
import os
import tensorflow as tf
import tensorflow.contrib.slim as slim
from tensorflow.python.client import device_lib

from calc2 import vh, vw, vss

import cv2

from dataset.coco_classes import calc_classes, calc_class_names

gtdata = None
preddata = None
imdata = None
recdata = None

N_CLASSES = len(calc_classes.keys())


class CALC2(object):
    def __init__(self, sess):
        self.sess = sess
        self.images = tf.placeholder(tf.float32, [None, vh, vw, 3])
        ret = vss(self.images, False, True,
                  ret_mu=False, ret_c_centers=False,
                  ret_c5=True)
        self.descriptor = ret[0]
        self.c5 = ret[1]

        saver = tf.train.Saver()
        ckpt = tf.train.get_checkpoint_state("model")
        cpath = ckpt.model_checkpoint_path

        print("loading model: ", cpath)
        saver.restore(self.sess, cpath)

    def run(self, images):

        if len(images.shape) == 2:
            # Grayscale
            images = np.repeat(images[..., np.newaxis], 3, axis=-1)
        if len(images.shape) == 3:
            images = images[np.newaxis, ...]

        descr, c5 = self.sess.run(
            [self.descriptor, self.c5],
            feed_dict={self.images: images})
        return descr, c5


def kp_descriptor(tensor):
    b, h, w, c = tensor.shape
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
            _t = tensor[0, i * _h:(i + 1) * _h, j * _w:(j + 1) * _w]
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
        od = [tensor[:, ky_i - 1, kx_i - 1],
              tensor[:, ky_i - 1, kx_i],
              tensor[:, ky_i - 1, kx_i + 1],
              tensor[:, ky_i, kx_i - 1],
              tensor[:, ky_i, kx_i + 1],
              tensor[:, ky_i + 1, kx_i - 1],
              tensor[:, ky_i + 1, kx_i],
              tensor[:, ky_i + 1, kx_i + 1]]

        _d = np.concatenate(tuple(od),
                            axis=0).reshape(1, -1, c)

        d = _d - tensor[:, ky_i, kx_i]
        d = d.reshape(-1, c)
        kp_d.append(d.reshape(1, -1))
    kp_d = np.concatenate(kp_d, axis=0)
    return kp, kp_d


def num_gpus():
    local_device_protos = device_lib.list_local_devices()
    return len([x.name for x in local_device_protos if x.device_type == 'GPU'])