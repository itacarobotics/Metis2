import numpy as np
import matplotlib.pyplot as plt

# Delta Robot Parameters
L1 = 5  # Length of the upper arms
L2 = 10  # Length of the lower arms
base_radius = 4  # Radius of the base
end_effector_radius = 2  # Radius of the end effector

# Function to calculate the position of the end effector from given angles
def delta_kinematics(thetas):
    # Coordinates of the base joints
    base_joints = [
        [base_radius * np.cos(np.deg2rad(0)), base_radius * np.sin(np.deg2rad(0))],
        [base_radius * np.cos(np.deg2rad(120)), base_radius * np.sin(np.deg2rad(120))],
        [base_radius * np.cos(np.deg2rad(240)), base_radius * np.sin(np.deg2rad(240))]
    ]
    
    # Calculate end effector joint positions based on the angles of the upper arms
    end_effector_joints = []
    
    for i, theta in enumerate(thetas):
        # Base joint position
        x_b, y_b = base_joints[i]
        
        # Calculate the position of the elbow (where upper and lower arms connect)
        x_e = x_b + L1 * np.cos(np.deg2rad(theta))
        y_e = y_b + L1 * np.sin(np.deg2rad(theta))
        
        # Append to the list of end effector joints
        end_effector_joints.append([x_e, y_e])
    
    return np.array(base_joints), np.array(end_effector_joints)

# Function to plot the delta robot
def plot_delta_robot(thetas):
    base_joints, end_effector_joints = delta_kinematics(thetas)
    
    # Plot the base and arms
    fig, ax = plt.subplots()
    
    # Plot the base triangle
    base_triangle = np.vstack([base_joints, base_joints[0]])  # Close the loop
    ax.plot(base_triangle[:, 0], base_triangle[:, 1], 'bo-', label='Base')
    
    # Plot the end effector triangle
    effector_triangle = np.vstack([end_effector_joints, end_effector_joints[0]])  # Close the loop
    ax.plot(effector_triangle[:, 0], effector_triangle[:, 1], 'ro-', label='End Effector')
    
    # Plot the arms (upper arms)
    for i in range(3):
        ax.plot([base_joints[i][0], end_effector_joints[i][0]],
                [base_joints[i][1], end_effector_joints[i][1]], 'g-', label='Upper Arm' if i == 0 else "")
    
    # Labels and limits
    ax.set_aspect('equal')
    ax.set_title('Delta Robot')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    plt.grid(True)
    plt.xlim(-15, 15)
    plt.ylim(-15, 15)
    plt.show()

# Test with specific angles for the upper arms
thetas = [30, 45, 60]  # Angles in degrees
plot_delta_robot(thetas)
