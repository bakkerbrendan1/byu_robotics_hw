#%%[markdown]

# # Solution for HW 07
# ## For Problems 1-2:
#%%
import kinematics_key_hw07 as kin
from visualization import ArmPlayer
import numpy as np
np.set_printoptions(precision=4)

# these DH parameters are based on solutions from HW 3, if you
# pick a different set that still describes the robots accurately,
# that's fine.
a_len = 0.5
d_len = 0.35

dh_part_a = [[0, d_len, 0., np.pi/2.0],
            [0, 0, a_len, 0], 
            [0, 0, a_len, 0]]

dh_part_b = [[0, d_len, 0., -np.pi/2.0],
            [0, 0, a_len, 0], 
            [np.pi/2.0, 0, 0, np.pi/2.0], 
            [np.pi/2.0, d_len*2, 0, -np.pi/2.0],
            [0, 0, 0, np.pi/2],
            [0, d_len*2, 0, 0]]

# make robot arm (assuming all joints are revolute)
arm_part_a = kin.SerialArm(dh_part_a)
arm_part_b = kin.SerialArm(dh_part_b, 
                           tip=kin.se3(R=kin.roty(-np.pi/2)))

arm_gui = ArmPlayer(arm_part_b)

#%%[markdown]
# ## For Problem 3
# ### part i):

# %%
dh = [[0, 0, 1.0, -np.pi/2.0],
      [0, 0, 1.0, 0]]

jt_types = ['r', 'r']

arm = kin.SerialArm(dh, jt=jt_types)
q = [0, 0]

from numpy.linalg import inv
T_2_in_0 = arm.fk(q)
T_0_in_2 = inv(T_2_in_0)
T_2_in_1 = arm.fk(q, index=[1,2])
T_1_in_2 = inv(T_2_in_1)

z_0_in_2= T_0_in_2[0:3,2]
z_1_in_2 = T_1_in_2[0:3,2]

o_0_in_2 = T_0_in_2[0:3,3]
o_1_in_2 = T_1_in_2[0:3,3]
o_2_in_2 = np.array([0.0, 0.0, 0.0])

J_at_2_in_2 = np.zeros((6,2))
J_at_2_in_2[0:3, 0] = np.cross(z_0_in_2, (o_2_in_2 - o_0_in_2))
J_at_2_in_2[3:, 0] = z_0_in_2
J_at_2_in_2[0:3, 1] = np.cross(z_1_in_2, (o_2_in_2 - o_1_in_2))
J_at_2_in_2[3:, 1] = z_1_in_2


# %% [markdown] 
# $J_2^2$ from direct calculation:

# %%
print(J_at_2_in_2)



# %% [markdown] 
# # part ii)

# %%
J_at_2_in_0 = arm.jacob(q)
T_2_in_0 = arm.fk(q)
R_0_in_2 = T_2_in_0[0:3,0:3].T

Z = np.zeros((6,6))
Z[0:3,0:3] = R_0_in_2
Z[3:6, 3:6] = R_0_in_2

J_at_2_in_2 = Z @ J_at_2_in_0

# %% [markdown] 
# $J_2^2 =  Z_0^2 J_2^0$ (from rotation):

# %%
print(J_at_2_in_2)


# %% [markdown]
# # Problem 4:

# Given the DH parameters for a Puma 560 robot, and the offset from the robot tip to a tool frame, find the 
# jacobian at the tool, and in the tool frame. 


# %%
dh = [[0, 0,        0,      np.pi/2.0],
      [0, 0,        0.4318, 0], 
      [0, 0.15,     0.02,   -np.pi/2.0], 
      [0, 0.4318,   0,      np.pi/2.0],
      [0, 0,        0,      -np.pi/2.0],
      [0, 0.4,      0,      0]]

jt_types = ['r']*6

# making the 2 DoF arm
arm = kin.SerialArm(dh, jt=jt_types)

# defining joint angles
q = [0]*6

# from problem definition
T_tool_in_6 = kin.se3(R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]), p = np.array([0, 0, 0.2]))

# using FK to get rotation from 0 to 6 
T_6_in_0 = arm.fk(q)

# finding rotation to describe transfrom to take Jacobian in frame zero, to the tool frame 
# (indices should cancel to give J a "tool" superscript)
R_0_in_6 = T_6_in_0[0:3,0:3].T
R_6_in_tool = T_tool_in_6[0:3,0:3].T

# finding the vector from frame 6 to tool, buit in the tool frame
p_from_6_to_tool_in_frame_6 = T_tool_in_6[0:3,3]

Z_0_in_6_from_6_to_6 = arm.Z_shift(R=R_0_in_6)
Z_6_in_tool_from_6_to_tool = arm.Z_shift(R=R_6_in_tool, p=p_from_6_to_tool_in_frame_6, p_frame='i')

J_at_6_in_frame_0 = arm.jacob(q)

# shifting the Jacobian, in two steps because it's easier this way.
J_at_tool_in_tool_frame = Z_6_in_tool_from_6_to_tool @ Z_0_in_6_from_6_to_6 @ J_at_6_in_frame_0

print('Shifted Jacobian is:')
print(J_at_tool_in_tool_frame)

# %%
