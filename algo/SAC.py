from collections import OrderedDict
import numpy as np
import tensorflow as tf
print(tf.__version__)

import tensorflow_probability as tfp
import random


# Just disables the warning, doesn't enable AVX/FMA
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

gpus= tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
    print("Name: ",gpu.name,"Type",gpu.device_type)

class ReplyBuffer:
    def __init__(self,capacity):
        self.capacity=capacity
        self.buffer=[tuple() for _ in range(self.capacity)]
        self.position=0

    def append(self,tuple):
        self.position+=1
        self.buffer[self.position%self.capacity]=tuple

    def sample(self,batch_size):
        inds=random.sample(self.buffer,batch_size)
        states, actions, next_states, rewards, dones= map(np.stack,zip(*inds))
        return states, actions, next_states, rewards, dones

    def __len__(self):
        if self.position>self.capacity:
            return self.capacity
        return self.position

class Policy:
    def __init__(self,input_shape,output_dims,hidden_units=128):
        self.input_shape=input_shape
        self.hidden_units=hidden_units
        self.output_dims=output_dims
        # layers definition
        input=tf.keras.layers.Dense(units=self.input_shape,input_shape=[self.input_shape])
        h1=tf.keras.layers.Dense(units=self.hidden_units,activation=tf.nn.relu)
        h2=tf.keras.layers.Dense(units=self.hidden_units,activation=tf.nn.relu)
        mean=tf.keras.layers.Dense(units=self.output_dims,activation=)


class SoftActorCritic:
    def __init__(self,env,reply_buffer_size,batch_size,num_epochs,num_updates):
        self.env = env
        self.reply_buffer=ReplyBuffer(reply_buffer_size)
        self.batch_size=batch_size
        self.num_epochs=num_epochs
        self.num_updates=num_updates
        self.input_shape=env.input_shape
        self.output_dims=env.output_shape
        self.policy=PolicyNetwork(self.input_shape,self.output_dims)

    def train(self):
        state=self.env.reset()
        for i in range(self.num_epochs):
            action= self.policy.model(state)
            state,action,next_state,reward,done=self.env.step(action)
            self.reply_buffer.append((state,action,next_state,reward,done))
            if done:
                state=self.env.reset()
            if start_update:
                for j in range(self.num_updates):
                    states,actions,next_states,rewards,dones=self.reply_buffer.sample(self.batch_size)
                    compute_target_q(rewards,next_states,dones)
                    compute_target_v(states)
                    update_q(batch)
                    update_v(states)
                    update_policy(states)
                    update_target_v_network()