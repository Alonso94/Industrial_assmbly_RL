from collections import OrderedDict
import numpy as np
import tensorflow as tf
import tensorflow_probability as tfp
from flatten_dict import flatten
from tensorflow.contrib.checkpoint import Checkpointable

class RL_algo(Checkpointable):
    # general train and evaluate methods

    def __init__(self):
        self.num_epochs=10000
        self.epoch_length=500

        self.epoch=0
        self.timestep=0
        self.num_trainsteps=0

        self.build()


