import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import robot

# Delta Robot Parameters
R1 = robot.BASE_RADIUS
L1 = robot.BICEPS_LEN
L2 = robot.FOREARM_LEN
R2 = robot.EE_RADIUS


def compute_angle_YZ(x, y, z):

    d = R1 - y - R2
    theta = abs(np.atan2(z, d))

    a = np.sqrt(L2*L2 - x*x)
    c = np.sqrt(d*d + z*z)

    k = (L1*L1 + c*c - a*a) / (2 * L1 * c)
    if k > 1:
        print(f"k: {k}")
        return None
    
    alpha = abs(np.acos(k))

    q_i = (theta + alpha) - np.pi

    return q_i



def inverse_geometry(x, y, z):
    
    x1 = x
    y1 = y
    z1 = z
    q1 = compute_angle_YZ(x1, y1, z1)
    if q1 is None:
        return None
    
    x2 = x*np.cos(2*np.pi/3) - y*np.sin(2*np.pi/3)
    y2 = x*np.sin(2*np.pi/3) + y*np.cos(2*np.pi/3)
    z2 = z
    q2 = compute_angle_YZ(x2, y2, z2)
    if q2 is None:
        return None
    
    x3 = x*np.cos(-2*np.pi/3) - y*np.sin(-2*np.pi/3)
    y3 = x*np.sin(-2*np.pi/3) + y*np.cos(-2*np.pi/3)
    z3 = z
    q3 = compute_angle_YZ(x3, y3, z3)
    if q3 is None:
        return None

    return np.array([q1, q2, q3])




# Function to calculate the position of the end effector from given angles
def forward_geometry(q, x):
    # Coordinates of the base joints
    bs_joints = [
        [R1 * np.sin(np.deg2rad(0)),    R1 * np.cos(np.deg2rad(0)),     0],
        [R1 * np.sin(np.deg2rad(120)),  R1 * np.cos(np.deg2rad(120)),   0],
        [R1 * np.sin(np.deg2rad(240)),  R1 * np.cos(np.deg2rad(240)),   0]
    ]

    el_joint1 = bs_joints[0] + np.array([0 , L1 * np.cos(q[0]), L1 * np.sin(q[0])])
    el_joint2 = bs_joints[1] + np.array([L1 * np.cos(q[1]) * np.sin(np.deg2rad(120)), L1 * np.cos(q[1]) * np.cos(np.deg2rad(120)), L1 * np.sin(q[1])])
    el_joint3 = bs_joints[2] + np.array([L1 * np.cos(q[2]) * np.sin(np.deg2rad(240)), L1 * np.cos(q[2]) * np.cos(np.deg2rad(240)), L1 * np.sin(q[2])])
    el_joints = np.array([el_joint1, el_joint2, el_joint3])

    ee_joint1 = [x[0] + R2 * np.sin(np.deg2rad(0)),    x[1] + R2 * np.cos(np.deg2rad(0)),   x[2]]
    ee_joint2 = [x[0] + R2 * np.sin(np.deg2rad(120)),  x[1] + R2 * np.cos(np.deg2rad(120)), x[2]]
    ee_joint3 = [x[0] + R2 * np.sin(np.deg2rad(240)),  x[1] + R2 * np.cos(np.deg2rad(240)), x[2]]
    ee_joints = np.array([ee_joint1, ee_joint2, ee_joint3])
    
    return np.array(bs_joints), el_joints, ee_joints




# Function to plot the delta robot in 3D
def plot_delta_robot_3d(x_ee, y_ee, z_ee):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    

    # Calculate inverse kinematics for the given end effector position
    q = inverse_geometry(x_ee, y_ee, z_ee)
    if q is None:
        print("[ERROR!] Solution not found.")
        return None
    
    print(q)
    
    # Calculate the positions of the elbow joints
    base_joints, elbow_joints ,end_effector_joints = forward_geometry(q, np.array([x_ee, y_ee, z_ee]))
    
    # Plot the base joints
    ax.scatter(base_joints[:, 0], base_joints[:, 1], base_joints[:, 2], c='b', label='Base Joints')
    


    # Plot the elbow joints
    ax.scatter(elbow_joints[:, 0], elbow_joints[:, 1], elbow_joints[:, 2], c='g', label='Ebow Joints')

    # Plot the end effector joints
    ax.scatter(end_effector_joints[:, 0], end_effector_joints[:, 1], end_effector_joints[:, 2], c='r', label='End Effector')


    # Plot the arms (upper arms)
    for i in range(3):
        # Plot upper arm
        ax.plot([base_joints[i][0], elbow_joints[i][0]], 
                [base_joints[i][1], elbow_joints[i][1]], 
                [base_joints[i][2], elbow_joints[i][2]], 'g-', label='Upper Arm' if i == 0 else "")
        
        # Plot lower arm
        ax.plot([elbow_joints[i][0], end_effector_joints[i][0]], 
                [elbow_joints[i][1], end_effector_joints[i][1]], 
                [elbow_joints[i][2], end_effector_joints[i][2]], 'g-', label='Upper Arm' if i == 0 else "")
        

    print(np.linalg.norm(base_joints[0] - elbow_joints[0]))
    print(np.linalg.norm(base_joints[1] - elbow_joints[1]))
    print(np.linalg.norm(base_joints[2] - elbow_joints[2]))

    print(np.linalg.norm(-end_effector_joints[0] + elbow_joints[0]))
    print(np.linalg.norm(-end_effector_joints[1] + elbow_joints[1]))
    print(np.linalg.norm(-end_effector_joints[2] + elbow_joints[2]))

    print(elbow_joints)
    print(end_effector_joints)

    # Set plot limits and labels
    ax.set_xlim([-100, 100])
    ax.set_ylim([-100, 100])
    ax.set_zlim([-200, 15])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Delta Robot 3D Visualization')
    ax.legend()
    plt.show()









def main():
    
    # Test with specific end-effector positions
    x_ee, y_ee, z_ee = 50, 0, -139  # Desired end-effector position (x, y, z)
    plot_delta_robot_3d(x_ee, y_ee, z_ee)

    return


if __name__ == "__main__":
    main()