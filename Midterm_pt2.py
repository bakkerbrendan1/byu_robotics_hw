# %% [markdown]
# # Midterm 2024
# * Copy this file to your homework workspace to have access to your other kinematic and visualization functions

# %%
# To test your setup, after defining the robot arm as described below, (but nothing else)
# you can run this file directly to make sure it is plotting the arm, obstacle, and goal 
# as expected. 

import kinematics as kin  #this is your kinematics file that you've been developing all along
from visualization import VizScene #this is the visualization file you've been using for homework
import time
import numpy as np


# Define your kinematics and an "arm" variable here using DH parameters so they
# are global variables that are available in your function below:

dh = [[0, 1.5, 0, -np.pi/2],
      [0, 0, 1.5, 0],
      [0, 0, 0, np.pi/2],
      [0, 1.5, 0, np.pi/2],
      [np.pi/2, 0, 0, -np.pi/2],
      [-np.pi, 1.5, 0, 0]]
arm = kin.SerialArm(dh)


# let's also plot robot to make sure it matches what we think it should
# (this will look mostly like the pictures on part 1 if your DH parameters
# are correct)
# viz_check = VizScene()
# viz_check.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n, draw_frames=True)
# # viz_check.update(qs = [[0, 0, 0, 0]])
# viz_check.hold()

# %%
#####################
# changes to the code are on line 55. my code adds error to the tip if it gets close to the obstacle
#####################


def compute_robot_path(q_init, goal, obst_location, obst_radius):
      # this can be similar to your IK solution for HW 6, but will require modifications
      # to make sure you avoid the obstacle as well.
      K = np.eye(3)
      kd = 0.001

      def get_error(q, index=None):
            if index==None:
                  index = len(q)

            cur_position = arm.fk(q, index=index)[0:3, 3]
            e = goal - cur_position

            # add error to the arm if the tip is too close
            if np.linalg.norm(cur_position - obst_location) - obst_radius < 0.5:
                  e = e + cur_position-obst_location - obst_rad

            return e

      def q_dot(q):
            # get top 3 rows of jacobian
            J = arm.jacob(q)[0:3, :]
            e = get_error(q)
            J_dag = J.T @ np.linalg.inv(J @ J.T + (kd**2) * np.eye(3))
            qdot = J_dag @ K @ e
            return qdot

      ####################################################
      # Repulsive field stuff I couldn't quite get to work
      ####################################################

      # repulsive field, eqn (12.14)
      def grad_Ur(q, index):
            kr = 0.001
            gamma = 5 # integer equal to or larger than 2
            range = 0.5 # range of influence
            joint_pos = arm.fk(q, index=index)[0:3, 3]

            # distance to obstacle
            eta = np.linalg.norm(joint_pos - obst_location) - obst_radius
            eta_vec = (joint_pos - obst_location) / eta

            if eta <= range:
                  # Ur_i = kr/(eta**2) * (1/eta - 1/range)**(gamma-1) * np.gradient(eta_vec)
                  Ur_i = kr/gamma * (1/eta - 1/range)**gamma
            else:
                  Ur_i = 0

            return Ur_i
      
      # attractive vector field eqn (12.13)
      def grad_Ua(q, index):
            ka = 0.01 # tuning parameter
            error = get_error(q, index=index)
            Ua_i = -0.5 * ka * np.linalg.norm(error)

            # Ua_i = ka * error / np.linalg.norm(error)
            return Ua_i
      
      def f_t(q):

            output = np.array([grad_Ua(q, i) + grad_Ur(q, i) for i in range(len(q))])

            return output
      ###################
      #  end of unused repulsive field stuff
      ###################


      # "q_s" is an empty list that we'll use to return our solutions so they can be plotted.
      # Unlike with IK where all we are interested in is the last joint angle, here we are
      # interested in the intermediate angles as they define a path that avoids the obstacle. 
      q_s = []
      q_s.append(q_init)

      count = 0
      error = get_error(q_s[-1])

      # print(0.01 * f_t(q_s[-1] ))
      while np.linalg.norm(error) > 1e-4 and count < 100: # set your stopping criteria here
            # TODO fill this out with your path planning method

            # Uncomment one of the lines below to test the different methods
            q = q_s[-1] + q_dot(q_s[-1])                         # method 1 (error of end point)
            # q = q_s[-1] + 10 * f_t(q_s[-1])                      # method 2 (potential energy field)
            error = get_error(q)
            count += 1

            q_s.append(q)

      return q_s

arm.fk(np.zeros(6))

# %%
if __name__ == "__main__":

      # if your function works, this code should show the goal, the obstacle, and your robot moving towards the goal.
      # Please remember that to test your function, I will change the values below to see if the algorithm still works.
      q_0 = [0, 0, 0, 0, 0, 0]
      q_1 = [np.pi, 0, np.pi, np.pi, 0, 0]

      # q_3: position of arm at the desired position
      # q_3 = [ 2.37656414e+00, -4.17077102e-01,  3.52665195e-01,  1.66443208e+00, -2.61315287e-01,  1.23047099e-16]
      # arm.fk(q_3)[0:3, 3]

      # goal = [0, 2, 4] # original goal
      goal = [-2, 3, 2]
      obst_position = [0, 3, 2]
      obst_rad = 1.0

      q_ik_slns = compute_robot_path(q_0, goal, obst_position, obst_rad)


      # if you just want to check if you have your code set up and arm defined correctly, you can uncomment the next three lines 
      # and run this file using either vs code or the terminal (and running "python3 midterm_2024.py"). None of the next three 
      # lines are needed for your solution, they may just help you check your visualization before you get going. It will just 
      # display 100 different random sets of joint angles as well as the goal and obstacle.

      # import numpy as np
      # q_ik_slns = np.random.uniform(size=(100,4))
      # q_ik_slns = q_ik_slns.tolist()


      # depending on how you store q_ik_slns inside your function, you may need to change this for loop
      # definition. However if you store q as I've done above, this should work directly.
      viz = VizScene()
      viz.add_arm(arm, joint_colors=[np.array([0.95, 0.13, 0.13, 1])]*arm.n)
      # viz.add_marker(arm.fk(), radius=0.1)
      viz.add_marker(goal, radius=0.1)
      viz.add_obstacle(obst_position, rad=obst_rad)
      for q in q_ik_slns:
            viz.update(qs=[q])

            # if your step in q is very small, you can shrink this time, or remove it completely to speed up your animation
            time.sleep(0.05)
      # print(q_ik_slns[-1])
      viz.hold()

# %%
