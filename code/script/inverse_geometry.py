import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Delta Robot Parameters
R1 = 60  # Radius of the base
L1 = 80  # Length of the upper arms
L2 = 120  # Length of the lower arms
R2 = 20  # Radius of the end effector


'''
int DeltaKinematics::delta_calcAngleYZ(double *Angle, double x0, double y0, double z0)
{
  double y1 = -0.5 * tan30 * BassTri;  // f/2 * tan(30 deg)
      y0 -= 0.5 * tan30 * PlatformTri;  // shift center to edge

  // z = a + b*y
  double aV = (x0*x0 + y0*y0 + z0*z0 +ArmLength*ArmLength - RodLength*RodLength - y1*y1)/(2.0*z0);
  double bV = (y1-y0)/z0;

  // discriminant
  double dV = -(aV+bV*y1)*(aV+bV*y1)+ArmLength*(bV*bV*ArmLength+ArmLength); 
  if (dV < 0)
  {
    return non_existing_povar_error; // non-existing povar.  return error, theta
  }

  double yj = (y1 - aV*bV - sqrt(dV))/(bV*bV + 1); // choosing outer povar
  double zj = aV + bV*yj;
  *Angle = atan2(-zj,(y1 - yj)) * 180.0/pi;

  return no_error;  // return error, theta
}
'''
def compute_angle_YZ(x, y, z):
    y1 = -0.5 * np.tan(np.pi/6) * R2
    y -= 0.5 * np.tan(np.pi/6) * R1

    aV = (x*x + y*y + z*z +L1*L1 - L2*L2 - y1*y1)/(2.0*z)
    bV = (y1-y)/z

    dV = -(aV+bV*y1)*(aV+bV*y1)+L1*(bV*bV*L1+L1)
    if dV < 0:
        return None
    
    yj = (y1 - aV*bV - np.sqrt(dV))/(bV*bV + 1)
    zj = aV + bV*yj

    q_i = np.atan2(-zj,(y1 - yj))
    return q_i


'''
int DeltaKinematics::inverse(double x0, double y0, double z0) 
{
  a = 0;
  b = 0;
  c = 0;
  int error = delta_calcAngleYZ(&a, x0, y0, z0);
  if(error != no_error)
    return error;
  error = delta_calcAngleYZ(&b, x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0);
  if(error != no_error)
    return error;
  error = delta_calcAngleYZ(&c, x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0);

  return error;
}
'''
def inverse_geometry(x, y, z):
    
    x1 = x
    y1 = y
    z1 = z
    q1 = compute_angle_YZ(x1, y1, z1)
    if q1 is None:
        return None
    
    x2 = x*np.cos(2*np.pi/3) + y*np.sin(2*np.pi/3)
    y2 = y*np.cos(2*np.pi/3) - x*np.sin(2*np.pi/3)
    z2 = z
    q2 = compute_angle_YZ(x2, y2, z2)
    if q2 is None:
        return None
    
    x3 = x*np.cos(2*np.pi/3) - y*np.sin(2*np.pi/3)
    y3 = y*np.cos(2*np.pi/3) + x*np.sin(2*np.pi/3)
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

    el_joint1 = bs_joints[0] + np.array([0 , L1 * np.cos(-q[0]), L1 * np.sin(-q[0])])
    el_joint2 = bs_joints[1] + np.array([L1 * np.cos(-q[1]) * np.sin(np.deg2rad(120)), L1 * np.cos(-q[1]) * np.cos(np.deg2rad(120)), L1 * np.sin(-q[1])])
    el_joint3 = bs_joints[2] + np.array([L1 * np.cos(-q[2]) * np.sin(np.deg2rad(240)), L1 * np.cos(-q[2]) * np.cos(np.deg2rad(240)), L1 * np.sin(-q[2])])
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
    x_ee, y_ee, z_ee = -40, 30, -180  # Desired end-effector position (x, y, z)
    plot_delta_robot_3d(x_ee, y_ee, z_ee)

    return


if __name__ == "__main__":
    main()