# kinematics for Delta axis robot arm

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class DeltaArm:
    """
    DeltaArm - A class designed to represent a delta axis robot arm

    SerialArms have frames 0 to n defined, with frame 0 located at the first joint and aligned with the robot body
    frame, and frame n located at the end of link n.

    Delta arms have a triangle base three vertical axes, and three arms to the end effector

    """

    def __init__(self, length: float, width: float, height: float):
        """
        arm = SerialArm(l, w, h)
        :param length: length of arm between motors and end effector
        :param width: width of base triangle
        :param height: height of base
        """
        self.l = length
        self.w = width
        self.h = height

        # Define the coordinates of the three vertices of the base triangle
        altitude = self.w/np.sqrt(3)
        self.base_A = np.array([0, altitude, 0])
        self.base_B = np.array([altitude * np.cos(np.pi/6),
                                -altitude * np.sin(np.pi/6),
                                0])
        self.base_C = np.array([-altitude * np.cos(np.pi/6),
                                -altitude * np.sin(np.pi/6),
                                0])
        
        self.base_vertices = [self.base_A, self.base_B, self.base_C]

        # test if the robot link length is too short
        if altitude >= self.l:
            print('Link length l is too short, make it longer to generate a valid arm.')
            return None

        # TODO test if the robot is too short

    # Forward Kinematics: Take joint positions, and return position of end effector
    def fk(self):
        pass

    # Inverse Kinematics: Take ee position, and return joint positions
    def ik(self, pos):
        # find position of ee
        x, y, z = pos
        q = [] # list of joint positions
        # position of joint
        for vertex in self.base_vertices:
            xj, yj, _ = vertex
            zj = z + np.sqrt(self.l**2 - x**2 + 2*x*xj - xj**2 - y**2 + 2*y*yj - yj**2)
            q.append(zj)
        return q

    # plot the arm on a graph
    def plotArm(self, pos=None, q=None, baselabels=True):
        """
        arm.plotArm(baselabels=True)
        Description: 
            Plots the delta arm with the given parameters

        Parameters:
            pos - position of end effector
            q - position of joints
            baselabels - Select if you want to label the base points of the delta arm

        Returns:
            Nothing, just plots the arm and launches the matplotlib 3d viewer
        """

        # find the position of joints
        if type(pos) == np.ndarray:
            q = self.ik(pos)

        # Create a figure and 3D axis
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, projection='3d')

        # Plot base triangle
        base_triangle = Poly3DCollection([self.base_vertices], color='gray', linewidths=1, edgecolors='r', alpha=0.5) # alpha adjusts transparency
        ax.add_collection3d(base_triangle)
        ax.add_collection3d(self.__make_top_triangle())

        # plot position of end effector
        ax.scatter(*pos, s=100, color='blue')

        for i in range(len(self.base_vertices)):
            # plot vertical rods
            ax.plot(*self.__make_vertical_rods(self.base_vertices[i]), color='k')

            # plot link lengths and actuators
            if type(pos) == np.ndarray:
                ax.plot(*self.__make_link_lengths(self.base_vertices[i], q[i], pos), color='red')
                act_pos = np.array([self.base_vertices[i][0], self.base_vertices[i][1], q[i]])
                ax.scatter(*act_pos, s=100, color='purple')
                

        # label the base vertices of the triangle
        if baselabels==True:
            ax.text(*self.base_A, 'A', color='black', fontsize=10)
            ax.text(*self.base_B, 'B', color='black', fontsize=10)
            ax.text(*self.base_C, 'C', color='black', fontsize=10)

        # Set the limits for the plot
        ax.set_xlim([-self.w, self.w])
        ax.set_ylim([-self.w, self.w])
        ax.set_zlim([0, self.h * 1.1])

        # Labels and title
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title(f'Delta Arm Viewer')

        plt.show()

    def __make_link_lengths(self, base_point: np.ndarray, q: int, pos: np.ndarray):
        """
        Creates a line to represent the link lengths.

        Parameters: 
        :param base_point: base point to plot the end effector length on
        :param q: q value for joint
        :param pos: position of end effector
        """
        p0 = np.array([base_point[0], base_point[1], q])

        link_vector = np.array([pos[0] - p0[0],
                                pos[1] - p0[1],
                                pos[2] - p0[2]])
        
        link_vector = link_vector/np.linalg.norm(link_vector)
        print('link vector=', link_vector)

        x = [p0[0], (p0[0] + link_vector[0]*self.l)]
        y = [p0[1], (p0[1] + link_vector[1]*self.l)]
        z = [p0[2], (p0[2] + link_vector[2]*self.l)]

        # x = [base_point[0], pos[0]]
        # y = [base_point[1], pos[1]]
        # z = [q, pos[2]]
        link_len = np.sqrt(
            (x[0] - x[1])**2 + 
            (y[0] - y[1])**2 +
            (z[0] - z[1])**2
        )
        print('Length:', link_len)
        return x, y, z


    # create vertical rods that the actuators slide along
    def __make_vertical_rods(self, point):
        """
        Creates a vertical line to represent the vertical rods.

        Parameters: point: point where to plot the lines
        """
        x = [point[0], point[0]] # same x and y position
        y = [point[1], point[1]] 
        z = [point[2], point[2] + self.h] 

        return x, y, z


    # create the top triangle object to plot (private function)
    def __make_top_triangle(self):
        # add the height to the base vectors
        A_top = self.base_A + np.array([0, 0, self.h])
        B_top = self.base_B + np.array([0, 0, self.h])
        C_top = self.base_C + np.array([0, 0, self.h])

        # create and return triangle object
        top_vertices = [A_top, B_top, C_top]
        top_triangle = Poly3DCollection([top_vertices], color='gray', linewidths=1, edgecolors='r', alpha=0.5)
        return top_triangle


# for testing the class
if __name__ == "__main__":
    
    pos = np.array([0, 0, 0])
    arm = DeltaArm(3.5, 4, 10) # l, w, h
    print('pos =', arm.ik(pos))
    arm.plotArm(pos=pos)