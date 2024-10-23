import numpy as np
import math

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from fg import forward_geometry



def main():

    dens = 25
    q_min = -np.pi/4
    q_max = np.pi/3

    q_instance = np.linspace(q_min, q_max, dens)
    ws_points = np.empty([dens**3, 3])

    i = 0
    for q1 in q_instance:
        for q2 in q_instance:
            for q3 in q_instance:
                
                x, y, z = forward_geometry(q1, q2, q3)
                if x is None:
                    print("[ERROR!] Invalid position")
                    return

                ws_points[i, :] = np.array([x, y, z])
                i += 1
            
    
    ws_highest_z = max(ws_points[:,2])
    ws_height = ws_highest_z - min(ws_points[:,2])
    ws_dim = 2 * max(ws_points[:,0])


    print(f"diameter: {ws_dim}\theight: {ws_height}\thighest z: {ws_highest_z}")


    # 3D Plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot workspace points
    ax.scatter(ws_points[:, 0], ws_points[:, 1], ws_points[:, 2], s=1, c='b', marker='o')

    # Parameters for the cylinder
    radius = 0.5 * ws_dim
    height = np.linspace(ws_highest_z-ws_height, ws_highest_z, 100)  # Adjust the height as needed
    theta = np.linspace(0, 2 * np.pi, 100)
    theta, height = np.meshgrid(theta, height)

    # Cylinder coordinates
    x_cylinder = radius * np.cos(theta)
    y_cylinder = radius * np.sin(theta)
    z_cylinder = height

    # Plot the cylinder
    ax.plot_surface(x_cylinder, y_cylinder, z_cylinder, color='g', alpha=0.3, rstride=5, cstride=5)


    # Set equal scaling for all axes
    ax.set_aspect("equal")

    # Labels
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    plt.title('3D Plot of Workspace Points with Cylinder')
    plt.show()

    return




if __name__ == "__main__":
    main()