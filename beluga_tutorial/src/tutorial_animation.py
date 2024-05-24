# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import yaml
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def read_yaml_file(filename):
    current_dir = os.getcwd()
    file_path = os.path.join(current_dir, filename)
    if os.path.exists(file_path):
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            return data
    else:
        print(f"File '{filename}' not found in directory: {current_dir}")
        return None


def plot_stages(yaml_data, axs, index):
    # Plot particles
    particles = yaml_data['simulation_records'][index]["particles"]
    for j, stage in enumerate(['current', 'propagate', 'reweight', 'resample']):
        states = particles[stage]['states']
        # weights = particles[stage]['weights']
        ax = axs[j]
        ax.clear()  # Clear previous plot on this axis
        ax.hist(
            states,
            bins=np.arange(min(states), max(states) + 0.5, 0.5),
            color='skyblue',
            edgecolor='black',
        )
        ax.set_title(f"{stage} - Sim Cycle {index+1}")
        ax.set_xlabel("States")
        ax.set_ylabel("NP")
        ax.set_xlim(-5, 100)
        ax.set_xticks(np.arange(-5, 101, 5))  # Set ticks at multiples of 5

    # Plot for landmark map
    landmark_map = yaml_data["landmark_map"]
    ax_landmark = axs[4]
    ax_landmark.clear()

    # Plot bars for landmark map data
    for x in range(101):  # Iterate over values from 0 to 100
        if x in landmark_map:
            color = 'red'
        else:
            color = 'blue'
        ax_landmark.bar(x, 1, color=color)  # Plot bars of height 1

    ax_landmark.set_title("Landmark Map")
    ax_landmark.set_xlabel("States")
    ax_landmark.set_xlim(-5, 100)
    ax_landmark.set_ylim(0, 2)
    ax_landmark.set_xticks(np.arange(-5, 101, 5))  # Set ticks at multiples of 5

    # Plot ground truth
    ground_truth = yaml_data['simulation_records'][index]["ground_truth"]
    ax_ground_truth = axs[5]
    ax_ground_truth.clear()  # Clear previous plot on this axis
    ax_ground_truth.bar(ground_truth, 1, color='green')  # Plot bars of height 1
    ax_ground_truth.set_title(f"Ground Truth: {ground_truth}")
    ax_ground_truth.set_xlabel("States")
    ax_ground_truth.set_xlim(-5, 100)
    ax_ground_truth.set_ylim(0, 2)
    ax_ground_truth.set_xticks(np.arange(-5, 101, 5))  # Set ticks at multiples of 5

    # Plot mean and standard deviation
    mean = "{:.3f}".format(yaml_data['simulation_records'][index]["estimation"]["mean"])
    sd = "{:.3f}".format(yaml_data['simulation_records'][index]["estimation"]["sd"])
    plt.text(
        0.5,
        0.5,
        f"Mean: {mean}\nSD: {sd}",
        ha='center',
        va='center',
        transform=axs[5].transAxes,
        bbox=dict(facecolor='white', alpha=0.5),
    )

    plt.tight_layout()
    plt.draw()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:", sys.argv[0], "<bag_file_path>")
        sys.exit(1)
    bag_file_path = sys.argv[1]
    yaml_data = read_yaml_file(bag_file_path)

    if yaml_data:
        fig, axs = plt.subplots(6, 1)
        num_frames = len(yaml_data['simulation_records'])

        def update(frame):
            # Plot the stages for the current frame
            plot_stages(yaml_data, axs, frame)

        # Create the animation
        ani = animation.FuncAnimation(
            fig, update, frames=num_frames, blit=False, interval=250, repeat=False
        )

        plt.show()
