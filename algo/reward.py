import abc
import tensorflow as tf
from gpflow import Parameter as Param
import numpy as np

from tensorflow import linalg as tfl
from gpflow.config import default_float
float_type = default_float()


class Reward(tf.Module):
    def __init__(self):
        super().__init__()

    @abc.abstractmethod
    def compute_reward(self, m, s):
        raise NotImplementedError


class ExponentialReward(Reward):
    def __init__(self, state_dim, W=None, t=None):
        Reward.__init__(self)
        self.state_dim = state_dim
        if W is not None:
            self.W = Param(np.reshape(W, (state_dim, state_dim)), trainable=False)
        else:
            self.W = Param(np.eye(state_dim), trainable=False)
        if t is not None:
            self.t = Param(np.reshape(t, (1, state_dim)), trainable=False)
        else:
            self.t = Param(np.zeros((1, state_dim)), trainable=False)

    def compute_reward(self, m, s):
        '''
        Reward function, calculating mean and variance of rewards, given
        mean and variance of state distribution, along with the target State
        and a weight matrix.
        Input m : [1, k]
        Input s : [k, k]

        Output M : [1, 1]
        Output S  : [1, 1]
        '''
        # for robot arm
        m=m[:,:3]
        s=s[:3,:3]

        SW = s @ self.W

        iSpW = tf.transpose(
                tfl.solve( (tf.eye(self.state_dim, dtype=float_type) + SW),
                tf.transpose(self.W), adjoint=True))

        muR = tf.exp(-(m-self.t) @  iSpW @ tf.transpose(m-self.t)/2) / \
                tf.sqrt( tfl.det(tf.eye(self.state_dim, dtype=float_type) + SW) )

        i2SpW = tf.transpose(
                tfl.solve( (tf.eye(self.state_dim, dtype=float_type) + 2*SW),
                tf.transpose(self.W), adjoint=True))

        r2 =  tf.exp(-(m-self.t) @ i2SpW @ tf.transpose(m-self.t)) / \
                tf.sqrt( tfl.det(tf.eye(self.state_dim, dtype=float_type) + 2*SW) )

        sR = r2 - muR @ muR
        muR.set_shape([1, 1])
        sR.set_shape([1, 1])
        return muR, sR