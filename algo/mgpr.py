import tensorflow as tf
import gpflow
import numpy as np

from tensorflow import linalg as tfl
from gpflow.config import default_float
float_type = default_float()

class MGPR(tf.Module):
    def __init__(self, X, Y, name=None):
        super(MGPR, self).__init__(name)

        self.num_outputs = Y.shape[1]
        self.num_dims = X.shape[1]
        self.num_datapoints = X.shape[0]

        self.create_models(X, Y)
        self.optimizers = []

    def create_models(self, X, Y):
        self.models = []
        for i in range(self.num_outputs):
            #kern = gpflow.kernels.RBF(input_dim=X.shape[1], ARD=True)
            kern=gpflow.kernels.RBF()
            #kern.lengthscales.prior = gpflow.priors.Gamma(1,10) # priors have to be included before
            #kern.variance.prior = gpflow.priors.Gamma(1.5,2)    # before the model gets compiled
            kern.lengthscale.prior=gpflow.probability_distributions.Gaussian(1,10)
            kern.variance.prior=gpflow.probability_distributions.Gaussian(1.5,2)
            self.models.append(gpflow.models.GPR(X, Y[:, i:i+1], kern))
            self.models[i].clear()
            self.models[i].compile()

    def set_XY(self, X, Y):
        for i in range(len(self.models)):
            self.models[i].X = X
            self.models[i].Y = Y[:, i:i+1]

    def optimize(self, restarts=0):
        if len(self.optimizers) == 0:  # This is the first call to optimize();
            for model in self.models:
                # Create an gpflow.train.ScipyOptimizer object for every model embedded in mgpr
                #optimizer = gpflow.train.ScipyOptimizer(method='L-BFGS-B')
                optimizer=gpflow.optimizers.Scipy()
                optimizer.minimize(model)
                self.optimizers.append(optimizer)
                restarts -= 1

        for model, optimizer in zip(self.models, self.optimizers):
            session = optimizer._model.enquire_session(None)
            best_parameters = model.read_values(session=session)
            best_likelihood = model.compute_log_likelihood()
            for restart in range(restarts):
                #randomize(model)
                optimizer._optimizer.minimize(session=session,
                            feed_dict=optimizer._gen_feed_dict(optimizer._model, None),
                            step_callback=None)
                likelihood = model.compute_log_likelihood()
                if likelihood > best_likelihood:
                    best_parameters = model.read_values(session=session)
                    best_likelihood = likelihood
            model.assign(best_parameters)

    def predict_on_noisy_inputs(self, m, s):
        iK, beta = self.calculate_factorizations()
        return self.predict_given_factorizations(m, s, iK, beta)

    def calculate_factorizations(self):
        K = self.K(self.X)
        batched_eye = tf.eye(tf.shape(self.X)[0], batch_shape=[self.num_outputs], dtype=float_type)
        L = tfl.cholesky(K + self.noise[:, None, None]*batched_eye)
        iK = tfl.cholesky_solve(L, batched_eye)
        Y_ = tf.transpose(self.Y)[:, :, None]
        # Why do we transpose Y? Maybe we need to change the definition of self.Y() or beta?
        beta = tfl.cholesky_solve(L, Y_)[:, :, 0]
        return iK, beta

    def predict_given_factorizations(self, m, s, iK, beta):
        """
        Approximate GP regression at noisy inputs via moment matching
        IN: mean (m) (row vector) and (s) variance of the state
        OUT: mean (M) (row vector), variance (S) of the action
             and inv(s)*input-ouputcovariance
        """

        s = tf.tile(s[None, None, :, :], [self.num_outputs, self.num_outputs, 1, 1])
        inp = tf.tile(self.centralized_input(m)[None, :, :], [self.num_outputs, 1, 1])

        # Calculate M and V: mean and inv(s) times input-output covariance
        iL = tfl.diag(1/self.lengthscales)
        iN = inp @ iL
        B = iL @ s[0, ...] @ iL + tf.eye(self.num_dims, dtype=float_type)

        # Redefine iN as in^T and t --> t^T
        # B is symmetric so its the same
        t = tf.transpose(
                tfl.solve(B, tf.transpose(iN), adjoint=True),
            )

        lb = tf.exp(-tf.reduce_sum(iN * t, -1)/2) * beta
        tiL = t @ iL
        c = self.variance / tf.sqrt(tf.linalg.det(B))

        M = (tf.reduce_sum(lb, -1) * c)[:, None]
        V = tf.matmul(tiL, lb[:, :, None], adjoint_a=True)[..., 0] * c[:, None]

        # Calculate S: Predictive Covariance
        R = s @ tfl.diag(
                1/tf.square(self.lengthscales[None, :, :]) +
                1/tf.square(self.lengthscales[:, None, :])
            ) + tf.eye(self.num_dims, dtype=float_type)

        X = inp[None, :, :, :]/tf.square(self.lengthscales[:, None, None, :])
        X2 = -inp[:, None, :, :]/tf.square(self.lengthscales[None, :, None, :])
        Q = tfl.solve(R, s)/2
        Xs = tf.reduce_sum(X @ Q * X, -1)
        X2s = tf.reduce_sum(X2 @ Q * X2, -1)
        maha = -2 * tf.matmul(X @ Q, X2, adjoint_b=True) + \
            Xs[:, :, :, None] + X2s[:, :, None, :]
        #
        k = tf.math.log(self.variance)[:, None] - \
            tf.reduce_sum(tf.square(iN), -1)/2
        L = tf.exp(k[:, None, :, None] + k[None, :, None, :] + maha)
        S = (tf.tile(beta[:, None, None, :], [1, self.num_outputs, 1, 1])
                @ L @
                tf.tile(beta[None, :, :, None], [self.num_outputs, 1, 1, 1])
            )[:, :, 0, 0]

        diagL = tf.transpose(tf.linalg.diag_part(tf.transpose(L)))
        S = S - tfl.diag(tf.reduce_sum(tf.multiply(iK, diagL), [1, 2]))
        S = S / tf.sqrt(tf.linalg.det(R))
        S = S + tfl.diag(self.variance)
        S = S - M @ tf.transpose(M)

        return tf.transpose(M), S, tf.transpose(V)

    def centralized_input(self, m):
        return self.X - m

    def K(self, X1, X2=None):
        return tf.stack(
            [model.kern.K(X1, X2) for model in self.models]
        )

    @property
    def Y(self):
        return tf.concat(
            [model.Y.parameter_tensor for model in self.models],
            axis = 1
        )

    @property
    def X(self):
        return self.models[0].X.parameter_tensor

    @property
    def lengthscales(self):
        return tf.stack(
            [model.kern.lengthscales.constrained_tensor for model in self.models]
        )

    @property
    def variance(self):
        return tf.stack(
            [model.kern.variance.constrained_tensor for model in self.models]
        )

    @property
    def noise(self):
        return tf.stack(
            [model.likelihood.variance.constrained_tensor for model in self.models]
        )