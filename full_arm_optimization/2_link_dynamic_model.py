import sympy
import sympybotics
from sympy import symbols, diff
from sympybotics._compatibility_ import exec_
l1,l2,l3 = symbols('l1 l2 l3 l4')

rbtdef = sympybotics.RobotDef('4-link Robot', [('0', l1, 0, 'q'),  ( '0', l2, 0, 'q'), ('0', l3, 0, 'q'),  ( '0', l4, 0, 'q')], dh_convention='standard')
rbtdef.frictionmodel = {'Coulomb', 'viscous'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
rbtdef.gravityacc = sympy.Matrix([0.0, 0.0, -9.81])

# generate dynamics of the robot
rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)
dyn_params = rbtdef.dynparms()
print(rbtdef.dynparms())

q_test = [0,0]
subs_ = dict(zip(rbtdef.q, q_test))

jac = rbt.kin.J[-1]
jac_2 = jac.row([0,1])
jac_null = jac_2.nullspace()


fk = rbt.geo.T[-1]
# print(fk)
dyn = rbt.M_code
grav = rbt.g_code;

l = locals()
M_func_def = sympybotics.robotcodegen.robot_code_to_func('python', rbt.M_code, 'M', 'M_2link', rbtdef)
exec_(M_func_def, globals(), l)
M_2link = l['M_2link']

subs_dyn = dyn[1].subs(dyn[0])
subs_dyn = subs_dyn.subs(dyn[0])
subs_dyn = subs_dyn.subs(dyn[0])
subs_dyn = subs_dyn.subs(dyn[0])
subs_dyn = subs_dyn.subs(dyn[0])
diff_M_l1x = diff(subs_dyn, dyn_params[20])

# print(diff_M_l1x)
# print(subs_dyn)
# print(dyn[0])
# print(fk)
# print(jac)
print(grav)
diff_fk = diff(fk, l1)
# print(diff_fk)

fk_subs = diff_fk.subs(subs_)
# print(fk_subs)




