#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Code copied from:
# [hand-gesture-recognition-mediapipe]
# URL: [https://github.com/Kazuhito00/hand-gesture-recognition-using-mediapipe]
# License: [Apache v2 license.]

import numpy as np
import tensorflow as tf
from timingdecorator.timeit import timeit


class KeyPointClassifier(object):
    def __init__(
        self,
        model_path='model/static/keypoint_classifier.tflite',
        num_threads=1,
    ):
        self.interpreter = tf.lite.Interpreter(model_path=model_path,
                                               num_threads=num_threads)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()


    def __call__(
        self,
        landmark_list,
    ):
        input_details_tensor_index = self.input_details[0]['index']
        self.interpreter.set_tensor(
            input_details_tensor_index,
            np.array([landmark_list], dtype=np.float32))
        self.interpreter.invoke()

        output_details_tensor_index = self.output_details[0]['index']

        result = self.interpreter.get_tensor(output_details_tensor_index)

        result_probs = np.squeeze(result)
        result_index = np.argmax(result_probs)
        confidence_score = result_probs[result_index]

        return result_index, confidence_score
