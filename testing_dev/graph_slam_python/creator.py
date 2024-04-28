import numpy as np
import math
import matplotlib.pyplot as plt
import numpy as np
import json
import argparse

# Initialize variables
image_path = "map1.png"  # Replace with your image path
poses = []  # List to store robot poses (x, y, theta)
landmarks = []  # List to store landmarks (x, y)
edges = []  # List to store edges (pose_id, landmark_id)
edges_landmark = []  # List to store landmark edges (pose_id, landmark_id)
init_pose = (0, 0, 0)  # Initial robot pose

sigma_x = 0.0001 # Standard deviation for x measurements
sigma_y = 0.0001 # Standard deviation for y measurements
sigma_theta = 0.0001  # Standard deviation for theta measurements
covariance_matrix = np.diag([sigma_x**2, sigma_y**2, sigma_theta**2])
information_matrix = np.linalg.inv(covariance_matrix)
last_change = []

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
    try:
        v1_u = v1 / np.linalg.norm(v1)
        v2_u = v2 / np.linalg.norm(v2)
        return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)), True

    except Exception as e:
        print(e)
        return 0, False

def compute_angle(prev_pose, current_pose):
    # Vector from previous pose to current pose
    v = np.array([current_pose[0] - prev_pose[0], current_pose[1] - prev_pose[1]])
    # Get angle between the x-axis and the vector v
    angle, succ = angle_between([1, 0], v)
    # Adjust sign of the angle
    if v[1] < 0:
        angle = -angle
    return angle, succ

def on_click(event):
    global poses, landmarks, init_pose, plt, last_change
    x, y = event.xdata, event.ydata  # Get the coordinates of the click

    if event.button == 1:  # Left mouse button
        # Mark robot pose
        angle = 0
        if len(poses) == 0:
            # First pose, assume angle of 0
            init_pose = (x, y, 0)
            poses.append((0, 0, 0))
        else:
            # Compute angle based on previous pose
            prev_pose = poses[-1]
            curr_pos = (x - init_pose[0], y - init_pose[1])
            angle, succ = compute_angle(prev_pose, curr_pos)
            if succ and not np.isnan(angle):
                poses.append((curr_pos[0], curr_pos[1], angle))
                last_change.append("pose")

        vector_length = 10  # Adjust the length of the vector as needed
        end_x = x + vector_length * math.cos(angle)
        end_y = y + vector_length * math.sin(angle)
        plt.plot([x, end_x], [y, end_y], 'r-')  # Draw a red line
        plt.draw()

    elif event.button == 3:  # Right mouse button
        # Mark landmark in global coordinates and associate it with the latest pose
        if len(poses) > 0:
            landmark_global = (x - init_pose[0], y - init_pose[1])
            pose_index = len(poses) - 1  # Index of the latest pose
            landmarks.append((landmark_global, pose_index))
            plt.plot(x, y, 'bo')  # Draw a blue dot
            plt.draw()
            last_change.append("landmark")


def compute_edges():
    global edges,landmarks, edges_landmark
    edges.clear()  # Clear existing edges
    loop_closure_threshold = 50
    landmark_closure_threshold = 20

    # Add edges between consecutive poses
    for i in range(1, len(poses)):
        dx = poses[i][0] - poses[i - 1][0]
        dy = poses[i][1] - poses[i - 1][1]
        dtheta = poses[i][2] - poses[i - 1][2]
        edges.append((i - 1, i, dx, dy, dtheta, information_matrix))

    # Detect and add loop closure edges
    for i in range(len(poses)):
        for j in range(i + 1, len(poses)):
            distance = np.hypot(poses[j][0] - poses[i][0], poses[j][1] - poses[i][1])
            if distance < loop_closure_threshold:
                dx = poses[j][0] - poses[i][0]
                dy = poses[j][1] - poses[i][1]
                dtheta = poses[j][2] - poses[i][2]
                edges.append((i, j, dx, dy, dtheta, information_matrix))

    edges_landmark.clear()  # Clear existing landmark edges
    # Add edges for landmarks
    for i, lmrk in enumerate(landmarks):
        landmark, pose_index = lmrk
        dx = landmark[0] - poses[pose_index][0]
        dy = landmark[1] - poses[pose_index][1]
        dtheta = 0  # Typically, there is no orientation information for landmarks in 2D SLAM
        edges_landmark.append((pose_index, len(poses) + i, dx, dy, dtheta, information_matrix))

    #Detect and add loop closure edges for landmarks
    for i, (landmark1, pose_index1) in enumerate(landmarks):
        for j, (landmark2, pose_index2) in enumerate(landmarks):
            if i >= j:  # Avoid duplicate comparisons and self-comparison
                continue
            distance = np.hypot(landmark2[0] - landmark1[0], landmark2[1] - landmark1[1])
            if distance < landmark_closure_threshold:
                # Add an edge between the poses that observed the landmarks
                dx = poses[pose_index2][0] - poses[pose_index1][0]
                dy = poses[pose_index2][1] - poses[pose_index1][1]
                dtheta = poses[pose_index2][2] - poses[pose_index1][2]
                edges.append((pose_index1, pose_index2, dx, dy, dtheta, information_matrix))


def save_to_g2o(filename):
    pose_uncertainty = 0.6
    landmark_uncertainty = 0.8
    with open(filename, "w") as f:
        for i, pose in enumerate(poses):
            f.write(f"VERTEX_SE2 {i} {pose[0]} {pose[1]} {pose[2]}\n")

        for j, (landmark, pose_id) in enumerate(landmarks):
            f.write(f"VERTEX_XY {len(poses) + j} {landmark[0]} {landmark[1]}\n")
        
        
        for edge in edges:
            f.write(f"EDGE_SE2 {edge[0]} {edge[1]} {edge[2]} {edge[3]} {edge[4]} "
                    f"9999 999 0.000000 9999 0.000000 9999\n")
                    # f"{edge[5][0,0]} {edge[5][0,1]} {edge[5][0,2]} "
                    # f"{edge[5][1,1]} {edge[5][1,2]} {edge[5][2,2]}\n")
        for edge in edges_landmark:
            f.write(f"EDGE_SE2_XY {edge[0]} {edge[1]} {edge[2]} {edge[3]} "
                    f"99999 0 99999 \n")
            # if len(edge) == 5:  # Pose-pose edge
            #     f.write(f"EDGE_SE2 {edge[0]} {edge[1]} {edge[2]} {edge[3]} {edge[4]} 11.111271 -0.249667 0.000000 399.999840 0.000000 2496.793089\n")
            # elif len(edge) == 6:  # Pose-landmark edge
            #     f.write(f"EDGE_SE2 {edge[0]} {edge[1]} {edge[2]} {edge[3]} {edge[4]} {landmark_uncertainty} 0 0 {landmark_uncertainty} 0 {landmark_uncertainty}\n")
def on_key(event):
    #If press u removes the last pose from the list
    if event.key == 'u':
        
        if last_change[-1] =="pose" and len(poses) > 0:
            poses.pop()
            last_change.pop()
        elif last_change[-1] == "landmark" and len(landmarks) > 0:
            landmarks.pop()
            last_change.pop()

        plt.cla()
        plt.imshow(plt.imread(image_path), origin='lower')
            
        for pose in poses:
            x, y, theta = pose
            x = x + init_pose[0]
            y = y + init_pose[1]
            vector_length = 10
            x_end = x + vector_length * math.cos(theta)
            y_end = y + vector_length * math.sin(theta)
            plt.plot([x, x_end], [y, y_end], 'r-')
        for lmrk in landmarks:
            x, y = lmrk[0]
            x = x + init_pose[0]
            y = y + init_pose[1]
            plt.plot(x, y, 'bo')
        plt.draw()

def store_arrays_to_json(poses, landmarks, filename='output_log.json'):
    data = {'poses': poses, 'landmarks': landmarks}
    with open(filename, 'w') as f:
        json.dump(data, f)

def load_arrays_from_json(filename='output_log.json'):
    with open(filename, 'r') as f:
        data = json.load(f)
    poses = data.get('poses', [])
    landmarks = data.get('landmarks', [])
    return poses, landmarks

def main():
    argparser = argparse.ArgumentParser(description="Graph SLAM creator")
    argparser.add_argument("--load", help="Load data from JSON file")
    args = argparser.parse_args()
    if args.load:
        global poses, landmarks
        poses, landmarks = load_arrays_from_json(args.load)
        print(landmarks[0])
        print("Data loaded from", args.load)
    else:
        image = plt.imread(image_path)
        fig, ax = plt.subplots()
        ax.imshow(image, origin='lower')
        fig.canvas.mpl_connect('button_press_event', on_click)
        #Connect keyboard event
        fig.canvas.mpl_connect('key_press_event', on_key)
        plt.show()

    compute_edges()
    save_to_g2o("output.g2o")
    print("Edges computed and saved to output.g2o")
    store_arrays_to_json(poses, landmarks)

if __name__ == "__main__":
    main()
