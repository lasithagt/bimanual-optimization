import sympy
import sympybotics
from sympy import symbols, diff
from sympybotics._compatibility_ import exec_
a1, a2, a3, a4, tool = symbols('a1 a2 a3 a4 tool')
d1, d2, d3, d4 = symbols('d1 d2 d3 d4')
# This is for a robot with specific alpha angles
# rbtdef = sympybotics.RobotDef('7DOF Robot', [('pi/2', a1, d1, 'q'),  ( '-pi/2', a2, d2, 'q'), ('-pi/2', a3, d3, 'q'), ( 'pi/2', a4, d4, 'q'), ( 'pi/2', 0, 0, 'q'), ( '-pi/2', 0, 0, 'q'), ( '0', 0, 0, 'q')], dh_convention='standard')
rbtdef = sympybotics.RobotDef('7DOF Robot', [('pi/2', a1, d1, 'q'),  ( '-pi/2', a2, d2, 'q'), ('-pi/2', a3, d3, 'q'), ( '0', a4, d4, 'q-pi/2'), ( 'pi/2', 0, 0, 'q'), ( '-pi/2', 0, 0, 'q-pi/2'), ( '0', 0, tool, 'q')], dh_convention='standard')

rbtdef.frictionmodel = {'Coulomb', 'viscous'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])

# generate dynamics of the robot
rbt 	   = sympybotics.RobotDynCode(rbtdef, verbose=True)
dyn_params = rbtdef.dynparms()

print(rbtdef.dynparms())

q_test = [0, 0, 0, 0, 0, 0, 0]
subs_ = dict(zip(rbtdef.q, q_test))

jac = rbt.kin.J[-1]
fk  = rbt.geo.T[-1]

dyn  = rbt.M_code
grav = rbt.g_code;

# l = locals()
fk_list = ([],fk)
diff_fk_a1 = diff(fk, a1)
diff_fk_a2 = diff(fk, a2)
diff_fk_a3 = diff(fk, a3)
diff_fk_a4 = diff(fk, a4)

diff_fk_d1 = diff(fk, d1)
diff_fk_d2 = diff(fk, d2)
diff_fk_d3 = diff(fk, d3)
diff_fk_d4 = diff(fk, d4)

diff_fk_a1_list = ([], diff_fk_a1)
diff_fk_a2_list = ([], diff_fk_a2)
diff_fk_a3_list = ([], diff_fk_a3)
diff_fk_a4_list = ([], diff_fk_a4)

diff_fk_d1_list = ([], diff_fk_d1)
diff_fk_d2_list = ([], diff_fk_d2)
diff_fk_d3_list = ([], diff_fk_d3)
diff_fk_d4_list = ([], diff_fk_d4)

jac = ([],jac)

FK_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', fk_list, 'FK', 'FK_kuka', rbtdef)

FK_a1_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_a1_list, 'FK_a1', 'FK_kuka_a1', rbtdef)
FK_a2_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_a2_list, 'FK_a2', 'FK_kuka_a2', rbtdef)
FK_a3_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_a3_list, 'FK_a3', 'FK_kuka_a3', rbtdef)
FK_a4_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_a4_list, 'FK_a4', 'FK_kuka_a4', rbtdef)

FK_d1_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_d1_list, 'FK_d1', 'FK_kuka_d1', rbtdef)
FK_d2_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_d2_list, 'FK_d2', 'FK_kuka_d2', rbtdef)
FK_d3_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_d3_list, 'FK_d3', 'FK_kuka_d3', rbtdef)
FK_d4_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', diff_fk_d4_list, 'FK_d4', 'FK_kuka_d4', rbtdef)

Jac_func_def  = sympybotics.robotcodegen.robot_code_to_func('matlab', jac, 'Jac', 'Jac', rbtdef)

M_func_def   = sympybotics.robotcodegen.robot_code_to_func('matlab', rbt.M_code, 'M', 'M_kuka', rbtdef)
C_func_def   = sympybotics.robotcodegen.robot_code_to_func('matlab', rbt.C_code, 'C', 'C_kuka', rbtdef)
G_func_def   = sympybotics.robotcodegen.robot_code_to_func('matlab', rbt.g_code, 'G', 'G_kuka', rbtdef)
INV_func_def = sympybotics.robotcodegen.robot_code_to_func('matlab', rbt.invdyn_code, 'INV', 'INV_kuka', rbtdef)


# writing matrices to a text file
file = open('rbt_params.txt','w') 
file.write(FK_a1_func_def)
file.write(FK_a2_func_def)
file.write(FK_a3_func_def)
file.write(FK_a4_func_def)

file.write(FK_d1_func_def)
file.write(FK_d2_func_def)
file.write(FK_d3_func_def)
file.write(FK_d4_func_def)

file.write(Jac_func_def)

file.write(FK_func_def) 
file.write(M_func_def) 
file.write(C_func_def)
file.write(G_func_def)
file.write(INV_func_def) 
file.close()

