import numpy as np
import random
import json
import os
import sys
import signal

class ReinforcementLearning:
    def __init__(self, tasks, actions, epsilon=0.1, learning_rate=0.2, save_file="rl_data.json"):
        self.tasks = tasks  # Tasks: [1, 2, 3]
        self.actions = actions  # Actions: ['A', 'B', 'C']
        self.epsilon = epsilon  # Exploration probability
        self.learning_rate = learning_rate  # Learning rate
        self.save_file = save_file

        # Load saved probabilities or initialize uniform probabilities
        if os.path.exists(self.save_file):
            self.task_action_probabilities = self.load_probabilities()
        else:
            self.task_action_probabilities = {task: np.ones(len(actions)) / len(actions) for task in tasks}

    def choose_action(self, task):
        """
        Choose an action using ε-greedy strategy.
        """
        if random.random() < self.epsilon:
            return random.choice(self.actions)  # Explore
        else:
            probs = self.task_action_probabilities[task]
            return np.random.choice(self.actions, p=probs)  # Exploit

    def update_probabilities(self, task, action, reward):
        """
        Update the action probabilities for a task based on feedback.
        Positive feedback boosts the chosen action's probability.
        Negative feedback penalizes the chosen action's probability.
        """
        current_probs = self.task_action_probabilities[task]
        action_index = self.actions.index(action)

        if reward == 1:  # Positive feedback
            increase = self.learning_rate * (1 - current_probs[action_index])
            current_probs[action_index] += increase
        else:  # Negative feedback
            decrease = self.learning_rate * current_probs[action_index]
            current_probs[action_index] -= decrease

        # Redistribute the remaining probability mass among other actions
        total_adjustment = decrease if reward == 0 else -increase
        redistribute = total_adjustment / (len(self.actions) - 1)
        for i in range(len(current_probs)):
            if i != action_index:
                current_probs[i] += redistribute

        # Ensure probabilities remain valid (clamped to [0, 1] and normalized)
        current_probs = np.clip(current_probs, 0, 1)
        current_probs /= np.sum(current_probs)
        self.task_action_probabilities[task] = current_probs

        # Save updated probabilities to file
        self.save_probabilities()

    def save_probabilities(self):
        """
        Save task-action probabilities to a JSON file.
        Convert numpy arrays to lists for JSON serialization.
        """
        data = {task: probs.tolist() for task, probs in self.task_action_probabilities.items()}
        with open(self.save_file, "w") as f:
            json.dump(data, f)

    def load_probabilities(self):
        """
        Load task-action probabilities from a JSON file.
        Convert lists back to numpy arrays after loading.
        """
        try:
            with open(self.save_file, "r") as f:
                data = json.load(f)
            return {int(task): np.array(probs) for task, probs in data.items()}
        except (json.JSONDecodeError, FileNotFoundError):
            print(f"Warning: Could not load probabilities from {self.save_file}. Initializing default probabilities.")
            return {task: np.ones(len(self.actions)) / len(self.actions) for task in self.tasks}

    def print_probabilities(self):
        """
        Print the current probabilities for all tasks.
        """
        print("\nCurrent Task-Action Probabilities:")
        for task, probs in self.task_action_probabilities.items():
            probs_str = ", ".join(f"{action}: {prob:.2f}" for action, prob in zip(self.actions, probs))
            print(f"Task {task} -> {probs_str}")

def exit_gracefully(signum, frame):
    print("\nProgram interrupted by user. Saving state and exiting...")
    rl_model.save_probabilities()
    sys.exit(0)

# Set the signal handler
signal.signal(signal.SIGINT, exit_gracefully)

# Instantiate the model
tasks = [1, 2, 3]
actions = ['A', 'B', 'C']
rl_model = ReinforcementLearning(tasks, actions)

# Main loop
print("Enter a task number (1, 2, 3) and provide feedback (1 for correct, 0 for incorrect):")
rl_model.print_probabilities()
while True:
    try:
        task_input = int(input("\nEnter a task number (1, 2, 3): "))
        if task_input not in tasks:
            print("Invalid task number. Please enter 1, 2, or 3.")
            continue
    except ValueError:
        print("Please enter a valid number.")
        continue

    while True:
        chosen_action = rl_model.choose_action(task_input)
        print(f"Chosen action for task {task_input}: {chosen_action}")

        try:
            feedback = int(input("Enter feedback (1 for correct, 0 for incorrect): "))
            if feedback not in [0, 1]:
                print("Invalid feedback. Please enter 0 or 1.")
                continue
        except ValueError:
            print("Please enter a valid number.")
            continue

        rl_model.update_probabilities(task_input, chosen_action, feedback)
        rl_model.print_probabilities()

        if feedback == 1:
            print(f"Task {task_input} completed. Action {chosen_action} was correct!")
            break
        else:
            print(f"Action {chosen_action} was incorrect for task {task_input}. Retrying...")