import sympy
import sympybotics
from sympy import symbols, diff
from sympybotics._compatibility_ import exec_

a1 = 0; a2 = 0; a3 = 0; a4 = 0; a5 = 0; a6 = 0; a7 = 0
d1 = 0.2025; d2 = 0; d3 = 0.42; d4 = 0; d5 = 0.4; d6 = 0; d7 = 0.126
# ( '0', a3, d3, 'q'), ( '0', a4, d4, 'q'), ( '0', a5, d5, 'q'),( '0', a6, d6, 'q'),( '0', a7, d7, 'q')

# Define the robot model in DH convention
rbtdef = sympybotics.RobotDef('KUKA14', [('pi/2', a1, d1, 'q'),  ('-pi/2', a2, d2, 'q'), ( '-pi/2', a3, d3, 'q'), ( 'pi/2', a4, d4, 'q'), ( 'pi/2', a5, d5, 'q'),( '-pi/2', a6, d6, 'q'),( '0', a7, d7, 'q')], dh_convention='standard')
rbtdef.frictionmodel = {'Coulomb', 'viscous'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])

# generate dynamics of the robot
rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)
dyn_params = rbtdef.dynparms()
print(rbtdef.dynparms())

q_test = [0,0,0,0]
subs_ = dict(zip(rbtdef.q, q_test))

jac = rbt.kin.J[-1]
jac_2 = jac.row([0,1])
jac_null = jac_2.nullspace()


fk = rbt.geo.T[-1]
fk_list = ([],fk)
FK_func_def  = sympybotics.robotcodegen.robot_code_to_func('python', fk_list, 'FK', 'FK_kuka', rbtdef)

M_func_def   = sympybotics.robotcodegen.robot_code_to_func('python', rbt.M_code, 'M', 'M_kuka', rbtdef)

C_func_def   = sympybotics.robotcodegen.robot_code_to_func('python', rbt.C_code, 'C', 'C_kuka', rbtdef)

G_func_def   = sympybotics.robotcodegen.robot_code_to_func('python', rbt.g_code, 'G', 'G_kuka', rbtdef)

INV_func_def = sympybotics.robotcodegen.robot_code_to_func('python', rbt.invdyn_code, 'INV', 'INV_kuka', rbtdef)
# print(rbt.M_code)
# print(M_func_def)
# FK_func_def = sympybotics.robotcodegen.robot_code_to_func('python', rbt.geo.T[-1], 'FK', 'FK_kuka', rbtdef)

# FK_kuka = l['FK_kuka']

# writing matrices to a text file
file = open('kuka_M.txt','w') 
file.write(FK_func_def) 

file.write(M_func_def) 

file.write(C_func_def)
file.write(G_func_def)
file.write(INV_func_def) 

file.close()

# print(fk)
dyn = rbt.M_code
grav = rbt.g_code;


# subs_dyn = dyn[1].subs(dyn[0])
# subs_dyn = subs_dyn.subs(dyn[0])
# subs_dyn = subs_dyn.subs(dyn[0])
# subs_dyn = subs_dyn.subs(dyn[0])
# subs_dyn = subs_dyn.subs(dyn[0])
# diff_M_l1x = diff(subs_dyn, dyn_params[20])

# print(diff_M_l1x)
# print(subs_dyn)
# print(dyn[1])
# print(dyn[0])
# print(fk)
# print(jac)
# print(grav)
# diff_fk = diff(fk, l1)
# print(diff_fk)

# fk_subs = diff_fk.subs(subs_)
# print(fk_subs)




