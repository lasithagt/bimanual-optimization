input_ = load('robot.mat');
input  = input_.input;
x = [2,0,0,0,input.link_variables];
% define the dual arm robot
alpha  = [pi/2 -pi/2 -pi/2 0 pi/2 -pi/2 0];
offset = [0,0,0,-pi/2,0,-pi/2,0];
d      = [x(5) x(6) x(7) 0 0 0 0];
a      = [0 0 0 x(8) 0 0 0];
tool   = input.tool;

L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
L7 = Link('d', tool, 'a',    0, 'alpha', alpha(7),'offset', offset(7));

m             = 200;
q_min         = input.q_min;
q_max         = input.q_max;

pd  = des_poses; 

% Display the current manipulator.

T_base_1 = SE3.rpy(rotm2eul(input.T_R(1:3,1:3),'XYZ'));
T_base_1.t = input.T_R(1:3,end);
T_base_2 = SE3.rpy(rotm2eul(input.T_L(1:3,1:3),'XYZ'));
T_base_2.t = input.T_L(1:3,end);


mini_chain_1 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_L','base', T_base_1);
mini_chain_2 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_R','base', T_base_2);


q_test = zeros(2,7);
q_test = [    1.5708   -1.5708    0.4064    0.9334   -0.1112   -0.0467   -1.1149;
   -1.5708    1.4605   -1.5702   -0.5005    0.1203   -0.2421   -0.2430];


FK(x,q_test)

input.T_L * mini_chain_1.A(1:7, q_test(1,:)).T
input.T_R * mini_chain_2.A(1:7, q_test(2,:)).T