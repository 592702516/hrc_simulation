import numpy as np

Joint_Names = [

]

# format is theta(joint angles), a, d, alpha
dh_parameters = [
    (0, 0, 0.1625,(np.pi/2)),
    (0, -0.425, 0, 0),
    (0, -0.3922, 0, 0),
    (0, 0, 0.1333, (np.pi/2)),
    (0, 0, 0.0997, (-np.pi/2)),
    (0, 0, 0.0996, 0)
]

class UR5EJacobian:
    def __init__(self):
        print("Initialised Kinematics")

    # calculate jacobian up to a distance from a join
    def jacobian(self, joint_angles, joint = 5, link_distance = 0.0996):

        # need to get transform of each link and joint first
        transformations = self.forward_kinematics(joint_angles, joint, link_distance)
        # print(transformations)
        T_end = transformations[-1]

        J = np.zeros((6, 6))

        # The z-axis of the base frame
        z = np.array([0, 0, 1])
        # The position vector of the end effector
        p_end = T_end[:3, 3]
        # print(joint)
        for i in range(joint+1):
            # print(i)
            T = transformations[i]
            p = T[:3, 3]
            z = T[:3, 2]

            # Linear velocity part (cross product of z-axis and (p_end - p))
            J[:3, i] = np.cross(z, p_end - p)
            # Angular velocity part (z-axis of the i-th joint)
            J[3:, i] = z
        return J

    

    def getTransform(self, theta, a,d ,alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    # Will calculate the jacobian matrix to the default end effector position, else will calculate it up to 
    # the specified joint and link length
    def forward_kinematics(self, joint_angles, joint = 5, link_distance = 0.0996):
        T = np.eye(4)
        transformations = [T]
        # only calculates transform up to the specficed joint
        for i, (_,a, d, alpha) in enumerate(dh_parameters[:joint+1]):
            # print(a, d, alpha, i)
            theta = joint_angles[i]

            # checks if it is the spefied joint 
            if i==joint:
                # print("Modifying last transformation")
                if(a != 0):
                    T = np.dot(T,self.getTransform(theta, -link_distance, d, alpha))  # Matrix multiplication
                else:
                    T = np.dot(T,self.getTransform(theta, a, link_distance, alpha))  # Matrix multiplication
                transformations.append(T)
                break
            else:
                 T = np.dot(T,self.getTransform(theta, a, d, alpha))  # Matrix multiplication
                 transformations.append(T)
            
        return transformations



    