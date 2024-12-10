import transforms as tr
import transforms_symbolic as st
import kinematics as kin
from dynamics import SerialArmDyn
import kinematics_symbolic as kins
import time

import numpy as np
import sympy as sm
from sympy import UnevaluatedExpr as uneval
from IPython.display import display, Math
from sympy.physics.vector.printing import vlatex

from visualization import VizScene 
from visualization import ArmPlayer

np.set_printoptions(precision=4, suppress=True)


d = 1
dh = [[0, d, 0, -np.pi/2],
      [-np.pi/2, d, 0, -np.pi/2],
      [0, d, 0, 0]]
jt = ['p']*3

arm = SerialArmDyn(dh, jt=jt)

viz = VizScene()
viz.add_arm(arm)

viz.hold()

viz.close_viz()