import os
import yaml
import matplotlib.pyplot as plt
import matplotlib.animation as animation

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
    particles = yaml_data['simulation_dataset'][index]["particles"]
    for j, stage in enumerate(['current', 'propagate', 'reweight', 'resample']):
        states = particles[stage]['states']
        weights = particles[stage]['weights']
        ax = axs[j]
        ax.clear()  # Clear previous plot on this axis
        # ax.plot(states, weights, 'o')
        ax.bar(states, weights, width=0.03,edgecolor='None',color='k',align='center')
        ax.set_title(f"{stage} - Sim Cycle {index+1}")
        ax.set_xlabel("States")
        ax.set_ylabel("Weights")
        # ax.legend()
        ax.set_xlim(-5, 100)
        if stage != "reweight":
            ax.set_ylim(0, 2)
        else:
            ax.set_ylim(0,0.02)
    
    # Plot for landmark map
    landmark_map = yaml_data["landamrk_map"]
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
    ax_landmark.set_ylabel("Height (1)")
    ax_landmark.set_xlim(-5, 100)
    ax_landmark.set_ylim(0, 2)

    # Plot ground truth
    ground_truth = yaml_data['simulation_dataset'][index]["ground_truth"]
    ax_ground_truth = axs[5]
    ax_ground_truth.clear()  # Clear previous plot on this axis
    ax_ground_truth.bar(ground_truth, 1, color='green')  # Plot bars of height 1
    ax_ground_truth.set_title(f"Ground Truth: {ground_truth}")
    ax_ground_truth.set_xlabel("States")
    ax_ground_truth.set_ylabel("Height (1)")
    ax_ground_truth.set_xlim(-5, 100)
    ax_ground_truth.set_ylim(0, 2)
    
    plt.tight_layout()
    plt.draw()

if __name__ == "__main__":
    filename = './src/beluga/beluga_tutorial/bags/bag.yaml'
    yaml_data = read_yaml_file(filename)
    
    if yaml_data:
        fig, axs = plt.subplots(6, 1)
        num_frames = len(yaml_data['simulation_dataset'])

        def update(frame):
            # Plot the stages for the current frame
            plot_stages(yaml_data, axs, frame)

        # Create the animation
        ani = animation.FuncAnimation(fig, update, frames=num_frames, blit=False, interval=100, repeat=False)

        plt.show()