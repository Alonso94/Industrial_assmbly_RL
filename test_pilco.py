import numpy as np
import gym
import tensorflow as tf
import gpflow
from gpflow import autoflow
from gym.wrappers import Monitor

from algo.pilco import PILCO

from algo.controller import RbfController
from algo.reward import ExponentialReward
import time
import matplotlib
import matplotlib.pyplot as plt

from env.env_real_pilco import rozum_real

float_type = gpflow.settings.dtypes.float_type

np.random.seed(0)
target = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
reward = ExponentialReward(9, t=target, W=weights)

i=0
def rollout(env,T, random=False,trial=0):
    start=time.time()
    X = []
    Y = []
    x=env.reset()
    s=x.copy()
    tt=[]
    env.render()
    rewards=[]
    for t in range(T):
        if random:
            u = np.random.rand(7)*0.6-0.3
            new_x,s2,_,done,new_target= env.random_step(u)
            X.append(np.hstack((s, u)))
            Y.append(s2 - s)
            s=s2.copy()
        else:
            u = pilco.compute_action(x[None, :])[0, :]
            new_x,_,done,new_target= env.step(u)
        # if new_target:
        #     continue
        tt.append(t)
        distance=np.linalg.norm(new_x[:3]-env.target[:3])
        if distance<0.1:
            weights = np.diag([1.0, 1.0, 1.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
            reward.update_weights(weights)
        if distance<0.2:
            weights = np.diag([1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            reward.update_weights(weights)
        rewards.append(distance)
        env.render()
        X.append(np.hstack((x, u)))
        Y.append(new_x-x)
        x=new_x.copy()
        # if np.linalg.norm(new_x[:3]-env.target) <0.05:
        #     break
        if done:
            break
    plt.plot(tt, rewards)
    plt.title("distance to goal - Trial %d" %trial)
    plt.xlabel("t")
    plt.ylabel("d")
    plt.savefig("dist%d.png"%trial)
    plt.show()
    end=time.time()
    print ("time on real robot= %.1f s"%(end-start))
    return np.stack(X),np.stack(Y),end-start

@autoflow((float_type,[None, None]), (float_type,[None, None]))
def predict_one_step_wrapper(mgpr, m, s):
    return mgpr.predict_on_noisy_inputs(m, s)


@autoflow((float_type,[None, None]), (float_type,[None, None]), (np.int32, []))
def predict_trajectory_wrapper(pilco, m, s, horizon):
    return pilco.predict(m, s, horizon)


@autoflow((float_type,[None, None]), (float_type,[None, None]))
def compute_action_wrapper(pilco, m, s):
    return pilco.controller.compute_action(m, s)


@autoflow((float_type, [None, None]), (float_type, [None, None]))
def reward_wrapper(reward, m, s):
    return reward.compute_reward(m, s)

def plot(pilco,X,Y,T,trial):
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 6))
    axes[0].set_title('One step prediction - Trial#%d' % trial)
    axes[2].set_xlabel('t')
    axes[1].set_ylabel('x')
    for i, m in enumerate(pilco.mgpr.models):
        y_pred_test, var_pred_test = m.predict_y(X)
        axes[i].plot(range(len(y_pred_test)), y_pred_test, Y[:, i])
        axes[i].fill_between(range(len(y_pred_test)),
                         y_pred_test[:, 0] - 2 * np.sqrt(var_pred_test[:, 0]),
                         y_pred_test[:, 0] + 2 * np.sqrt(var_pred_test[:, 0]), alpha=0.3)
        if i==2: break

    plt.savefig("onep%d.png" % trial)
    plt.show()
    m_p = np.zeros((T, state_dim))
    S_p = np.zeros((T, state_dim, state_dim))
    m_init = X[0:1, 0:state_dim]
    S_init = np.diag(np.ones(state_dim) * 0.1)
    for h in range(T):
        m_h, S_h, _ = predict_trajectory_wrapper(pilco, m_init, S_init, h)
        m_p[h, :], S_p[h, :, :] = m_h[:], S_h[:, :]

    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(12, 6))
    axes[0].set_title('Multi-step prediction - Trial#%d' % trial)
    axes[2].set_xlabel('t')
    axes[1].set_ylabel('x')
    for i in range(state_dim):
        axes[i].plot(range(T - 1), m_p[0:T - 1, i], X[1:T, i])  # can't use Y_new because it stores differences (Dx)
        axes[i].fill_between(range(T - 1),
                         m_p[0:T - 1, i] - 2 * np.sqrt(S_p[0:T - 1, i, i]),
                         m_p[0:T - 1, i] + 2 * np.sqrt(S_p[0:T - 1, i, i]), alpha=0.2)
        if i == 2: break

    plt.savefig("multistep%d.png" % trial)
    plt.show()

with tf.Session() as sess:
    p_start=time.time()
    env = rozum_real()
    T=25
    num_basis_functions = 50
    max_action = 0.3
    time_on_real_robot = 0
    X,Y,t=rollout(env,25,random=True,trial=0)
    time_on_real_robot += t
    state_dim = Y.shape[1]
    control_dim = X.shape[1] - Y.shape[1]
    controller = RbfController(state_dim,control_dim, num_basis_functions, max_action)
    pilco=PILCO(X,Y,controller=controller,reward=reward)
    plot(pilco,X,Y,T,0)
    n=10
    t_model=0
    t_policy=0
    for i in range(1,n):
        env.reset()
        t1 = time.time()
        pilco.optimize_models()
        t2 = time.time()
        t_model+=t2-t1
        print("model optimization done!")
        pilco.optimize_policy()
        t3 = time.time()
        t_policy+=t3-t2
        print("policy optimization done!")
        X_,Y_,t=rollout(env,T,trial=i)
        time_on_real_robot += t
        plot(pilco,X_,Y_,T,i)
        X=np.concatenate((X,X_[:T,:]),axis=0)
        Y=np.concatenate((Y,Y_[:T,:]),axis=0)
        X=X[3*T:]
        Y = Y[3 * T:]
        pilco.mgpr.set_XY(X,Y)
    print("t_robot= %.2f s" %time_on_real_robot)
    print("t_model= %.2f s" %t_model)
    print("t_policy= %.2f s" %t_policy)
    print("program running time = %d s" %(time.time()-p_start))