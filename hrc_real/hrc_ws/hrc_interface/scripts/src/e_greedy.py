#!/usr/bin/env python

import numpy as np
import json
import os
import random

class EGreedy:
    def __init__(self, tasks, actions, epsilon=0.1, learning_rate=0.2, save_file="try.json"):
        self.tasks = tasks  # Tasks: [3, 4, 5, 6]
        self.actions = actions  # Actions: [7, 8, 9, 10]
        self.epsilon = epsilon  # Exploration probability
        self.learning_rate = learning_rate  # Learning rate
        self.save_file = save_file
        if os.path.exists(self.save_file):
            self.task_action_probabilities = self.load_probabilities()
        else:
            self.task_action_probabilities = {task: np.ones(len(actions)) / len(actions) for task in tasks}

    def choose_action(self, task):
        if random.random() < self.epsilon:
            return random.choice(self.actions)
        else:
            probs = self.task_action_probabilities[task]
            return np.random.choice(self.actions, p=probs)

    def update_probabilities(self, task, action, reward):
        current_probs = self.task_action_probabilities[task]
        action_index = self.actions.index(action)
        if reward == 1:
            increase = self.learning_rate * (1 - current_probs[action_index])
            current_probs[action_index] += increase
        else:
            decrease = self.learning_rate * current_probs[action_index]
            current_probs[action_index] -= decrease

        total_adjustment = decrease if reward == 0 else -increase
        redistribute = total_adjustment / (len(self.actions) - 1)
        for i in range(len(current_probs)):
            if i != action_index:
                current_probs[i] += redistribute
        current_probs = np.clip(current_probs, 0, 1)
        current_probs /= np.sum(current_probs)
        self.task_action_probabilities[task] = current_probs
        self.save_probabilities()

    def save_probabilities(self):
        data = {str(task): probs.tolist() for task, probs in self.task_action_probabilities.items()}
        with open(self.save_file, "w") as f:
            json.dump(data, f)

    def load_probabilities(self):
        try:
            with open(self.save_file, "r") as f:
                data = json.load(f)
            return {int(task): np.array(probs) for task, probs in data.items()}
        except:
            return {task: np.ones(len(self.actions)) / len(self.actions) for task in self.tasks}