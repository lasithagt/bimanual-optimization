close all;
clear;

%% Manipulator properties
NDOFs = 4;
% mass  = [0.66393 ,0.34366 ,0.10602 ,0.04566 ];
mass  = [1.2 ,0.54366 ,0.30602 ,0.50566 ];

CoM   = [0.00    0.00     0.00000 0;...
         0.00    -0.08003 0.002    0.09265;...
         0.0382  -0.04000 0.01211 0.0];

% This is the URDF convention
I(:,:,1) = [0.00189 0.00001 0.00011;
0.00001 0.00295 0.00007;
0.00011 0.00007 0.0256];

I(:,:,2) = [0.00303 0.00000 -0.00036;
0.00000	0.00311	0.00000;
-0.00036 0.00000 0.00014];
    
I(:,:,3) = [0.00033	0.00000	0.00000;
0.00000	0.00011	0.00010;
0.00000	0.00010	0.00026];

I(:,:,4) = [0.00052	0.00000	0.00000;
0.00000	0.00052	0.00000;
0.00000	0.00000	0.00000];


robot.m     = mass;
robot.mC    = CoM;
robot.I     = I;
robot.NDOFs = NDOFs;
robot.grav  = roty(pi/2)*[0 0 -9.8]';
phi         = [0.2; 0.0; 0.0; 0.0; 0.24; 0.0; 0.0; 0.0;0.3;0.0;0.0;0.0;0.0;0.2];

alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];
d     = phi([1 3 5 7 9 11 13]);
a     = phi([2 4 6 8 10 12 14]);

L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1));        
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset',0);
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset',0);
L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset',0);
L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset',0);
L7 = Link('d', d(7), 'a', a(7), 'alpha', alpha(7),'offset',0);

% Display the current manipulator.
l = 0.05;
T_base_1 = SE3.Ry(pi/2);
T_base_2 = SE3.Ry(-pi/2)*SE3.Rz(-pi);
T_base_1.t = [l;-0.6;0];
T_base_2.t = [-l;-0.6;0];


mini_chain_1 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_R','base', T_base_1);
mini_chain_2 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_L','base', T_base_2);

% Transform the coordinates
% Conversion to the coordinate system.
% --------------------------------------------------------------
for i = 1 : NDOFs
    T_1   = mini_chain_1.A(i,[0 0 0 0 0 0 0]);
    T_2   = mini_chain_2.A(i,[0 0 0 0 0 0 0]);
    p     = T_1.t;  
    R     = T_1.R;
    CoM(:,i) = R'*(CoM(:,i) - p);
end

CoM(:,2) = CoM(:,2) + [0 0 0.1245]';
% CoM(:,4) = CoM(:,4) + [0 0 0.1545]';
% CoM(2:4,6) = CoM(2:4,6) + [0 0 0.081]';
% ---------------------------------------------------------------


%% Compute the trajectory
q0 = [0 0 0 0 0 0 0];
qf = [pi/2 pi/4 pi/3 pi/4 0 0 0];

% q0 = [0 pi/8 0 0 0 0 0]; 
% % qf = [pi/2 pi/4 pi/3 -pi/2];
% qf = [pi/2 pi/4 pi/3 -pi/4 0 0 0];
m  = 2;

tf = 0.3;
t = 0:0.01:tf;
[q,qd,qdd] = jtraj(q0, qf, t);

%% Compute Torque
T_1 = TorqueCompute(robot, mini_chain_1, q, qd, qdd, T_base_1);
T_2 = TorqueCompute(robot, mini_chain_2, q, qd, qdd, T_base_2);


%% Compute the motor characterization (Torque is dependant on the speed of the motor but acceleration cannot be gaurenteeded)
stall_torque(1)   = 8;
free_run_speed(1) = 70*0.104719755;

stall_torque(2)   = 3;
free_run_speed(2) = 80*0.104719755;

stall_torque(3)   = 3;
free_run_speed(3) = 80*0.104719755;

stall_torque(4)   = 3;
free_run_speed(4) = 80*0.104719755;

if any(qd > free_run_speed(1))
    warning('Motor Cannot Hold the Speed')
end

M1_torque_lim(:,1) = interp1(linspace(0,free_run_speed(1),100), linspace(stall_torque(1),0,100), abs(qd(:,1)));
M1_torque_lim(:,2) = interp1(linspace(0,free_run_speed(2),100), linspace(stall_torque(2),0,100), abs(qd(:,2)));
M1_torque_lim(:,3) = interp1(linspace(0,free_run_speed(3),100), linspace(stall_torque(3),0,100), abs(qd(:,3)));
M1_torque_lim(:,4) = interp1(linspace(0,free_run_speed(4),100), linspace(stall_torque(4),0,100), abs(qd(:,4)));

%% Plot the graphs
figure(1)
for i=1:4
    subplot(2,2,i)
    plot(t,(T_1(i,:)))
    hold on
    plot(t,M1_torque_lim(:,i),'r--')
    hold on
    plot(t,-M1_torque_lim(:,i),'r--')
    title(['Joint ',num2str(i)])
    xlabel('Time(s)')
    legend('Actual Profile', 'Max Torque') 
end

% Plot the after and previous
figure(2)
% set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
% hold on
for i = 1:size(q,1)
    mini_chain_1.plot(q(i,:), 'noshadow','workspace',[-1 1 -1 1 -1 1],'noarrow', 'view',[0 60],'tile1color',[10 1 1],'delay',0.0001)
    hold on
    q(i,1) = -q(i,1);
    q(i,3) = -q(i,3);
    q(i,5) = -q(i,5);
    q(i,7) = -q(i,7);
    mini_chain_2.plot(q(i,:), 'noshadow','workspace',[-1 1 -1 1 -1 1],'noarrow', 'view',[0 60],'tile1color',[10 1 1],'delay',0.0001)
end


% [T,A] = manip_chain.fkine([0 pi/4 pi/3 -pi/2]);
% hold on
% for i=1:4
%   trplot(A(i), 'frame', num2str(i))
% end
% manip_res.vellipse(q, options)