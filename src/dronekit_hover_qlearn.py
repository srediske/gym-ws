#!/usr/bin/env python
import gym
from gym import wrappers
import gym_ros
import time
import numpy as np
import pandas
import time
import rospy
import qlearn
import liveplot


def render():
    render_skip = 0  # Skip first X episodes.
    render_interval = 50  # Show render Every Y episodes.
    render_episodes = 10  # Show Z episodes every rendering.

    if (x % render_interval == 0) and (x != 0) and (x > render_skip):
        env.render()
    elif ((x - render_episodes) % render_interval == 0) and (x != 0) and (x > render_skip) and (render_episodes < x):
        env.render(close=True)


def build_state(features):
    return int("".join(map(lambda feature: str(int(feature)), features)))


def to_bin(value, bins):
    return np.digitize(x=[value], bins=bins)[0]


if __name__ == '__main__':

    # Create the Gym environment
    env = gym.make('dronekit_Hover-v0')
    rospy.loginfo("Gym environment done")  # added 23.06.19 SR

    # Set the logging system
    outdir = '/tmp/gazebo_gym_experiments'
    env = gym.wrappers.Monitor(env, outdir, force=True)
    plotter = liveplot.LivePlot(outdir)

    last_time_steps = np.ndarray([0])
    max_number_of_steps = 1000

    latitude_bins = pandas.cut([-1.6, 1.6], bins=10, retbins=True)[1][1:-1]
    longitude_bins = pandas.cut([-1.6, 1.6], bins=10, retbins=True)[1][1:-1]

    qlearn = qlearn.QLearn(actions=range(env.action_space.n),
                           alpha=0.1, gamma=0.9, epsilon=0.9)

    initial_epsilon = qlearn.epsilon

    epsilon_discount = 0.999

    start_time = time.time()
    total_episodes = 10000
    highest_reward = 0

    for x in range(total_episodes):
        done = False

        cumulated_reward = 0

        observation = env.reset()
        latitude, longitude = observation

        state = build_state([to_bin(latitude, latitude_bins),
                             to_bin(longitude, longitude_bins)])

        if qlearn.epsilon > 0.05:
            qlearn.epsilon *= epsilon_discount

        # state = ''.join(map(str, observation))

        # render() #defined above, not env.render()

        for i in range(max_number_of_steps):

            # Pick an action based on the current state
            action = qlearn.chooseAction(state)

            # Execute the action and get feedback
            observation, reward, done, info = env.step(action)
            cumulated_reward += reward

            if highest_reward < cumulated_reward:
                highest_reward = cumulated_reward

            latitude, longitude = observation
            nextState = build_state([to_bin(latitude, latitude_bins),
                                     to_bin(longitude, longitude_bins)])

            qlearn.learn(state, action, reward, nextState)
            env._flush(force=True)  # test

            if not done:
                state = nextState
            else:
                last_time_steps = np.append(last_time_steps, [int(i + 1)])
                break

        if x % 100 == 0:
            plotter.plot(env)

        m, s = divmod(int(time.time() - start_time), 60)
        h, m = divmod(m, 60)
        print ("EP: " + str(x + 1) + " - [alpha: " + str(round(qlearn.alpha, 2)) + " - gamma: " + str(
            round(qlearn.gamma, 2)) + " - epsilon: " + str(round(qlearn.epsilon, 2)) + "] - Reward: " + str(
            cumulated_reward) + "     Time: %d:%02d:%02d" % (h, m, s))

    # Github table content
    print ("\n|" + str(total_episodes) + "|" + str(qlearn.alpha) + "|" + str(qlearn.gamma) + "|" + str(
        initial_epsilon) + "*" + str(epsilon_discount) + "|" + str(highest_reward) + "| PICTURE |")

    Lts = last_time_steps.tolist()
    Lts.sort()

    # print("Parameters: a="+str)
    print("Overall score: {:0.2f}".format(last_time_steps.mean()))
    print("Best 100 score: {:0.2f}".format(reduce(lambda x, y: x + y, Lts[-100:]) / len(Lts[-100:])))

    env.close()
