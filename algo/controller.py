import tensorflow as tf
import numpy as np
import gpflow

from tensorflow import linalg as tfl
from algo.mgpr import MGPR
from gpflow.config import default_float
float_type = default_float()


def squash_sin(m, s, max_action=None):
    '''
    Squashing function, passing the controls mean and variance
    through a sinus, as in gSin.m. The output is in [-max_action, max_action].
    IN: mean (m) and variance(s) of the control input, max_action
    OUT: mean (M) variance (S) and input-output (C) covariance of the squashed
         control input
    '''
    k = tf.shape(m)[1]
    if max_action is None:
        max_action = tf.ones((1,k), dtype=float_type)  #squashes in [-1,1] by default
    else:
        max_action = max_action * tf.ones((1,k), dtype=float_type)

    M = max_action * tf.exp(-tfl.diag_part(s) / 2) * tf.sin(m)

    lq = -(tfl.diag_part(s)[:, None] + tfl.diag_part(s)[None, :]) / 2
    q = tf.exp(lq)
    S = (tf.exp(lq + s) - q) * tf.cos(tf.transpose(m) - m) \
        - (tf.exp(lq - s) - q) * tf.cos(tf.transpose(m) + m)
    S = max_action * tf.transpose(max_action) * S / 2

    C = max_action * tfl.diag( tf.exp(-tfl.diag_part(s)/2) * tf.cos(m))
    return M, S, tf.reshape(C,shape=[k,k])

class FakeGPR(tf.Module):
    def __init__(self, X, Y, kernel):
        tf.Module.__init__(self)
        self.X = gpflow.Parameter(X)
        self.Y = gpflow.Parameter(Y)
        self.kern = kernel
        self.likelihood = gpflow.likelihoods.Gaussian()

class RbfController(MGPR):
    '''
    An RBF Controller implemented as a deterministic GP
    See Deisenroth et al 2015: Gaussian Processes for Data-Efficient Learning in Robotics and Control
    Section 5.3.2.
    '''
    def __init__(self, state_dim, control_dim, num_basis_functions, max_action=None):
        MGPR.__init__(self,
            np.random.randn(num_basis_functions, state_dim),
            0.1*np.random.randn(num_basis_functions, control_dim)
        )
        for model in self.models:
            model.kern.variance = 1.0
            model.kern.variance.trainable = False
            self.max_action = max_action

    def create_models(self, X, Y):
        #TODO
        self.models = gpflow.utilities.
        param_dict = gpflow.utilities.leaf_components(self.models)
        gpflow.utilities.multiple_assign(self.models, param_dict)
        for i in range(self.num_outputs):
            kern = gpflow.kernels.RBF(ard=True)
            self.models.append(FakeGPR(X, Y[:, i:i+1], kern))

    def compute_action(self, m, s, squash=True):
        '''
        RBF Controller. See Deisenroth's Thesis Section
        IN: mean (m) and variance (s) of the state
        OUT: mean (M) and variance (S) of the action
        '''
        iK, beta = self.calculate_factorizations()
        M, S, V = self.predict_given_factorizations(m, s, 0.0 * iK, beta)
        S = S - tfl.diag(self.variance - 1e-6)
        if squash:
            M, S, V2 = squash_sin(M, S, self.max_action)
            V = V @ V2
        return M, S, V