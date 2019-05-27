close all;
clear;

%% Manipulator properties
NDofs = 4;
% mass  = [0.66393 ,0.34366 ,0.10602 ,0.04566 ];
mass  = [1.2 ,0.54366 ,0.10602 ,0.10566 ];
CoM   = [0.00075  0.01109 0.00000 0;...
         0.02922 -0.00003 0.03581 0;...
         0.00382 -0.08500 0.02211 -0.09265];

I(:,:,1) = [0.00189 0.00001 0.00011;
0.00001 0.00295 0.00007;
0.00011 0.00007 0.00256];

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
robot.NDOFs = NDofs;
robot.grav  = roty(pi/2)*[0 0 -9.8]';
phi         = [0.2; 0.0; 0.0; 0.0; 239.35/1000; 0.0; 0.; 0.3];

alpha = [-pi/2 pi/2 -pi/2 0];
d     = phi([1 3 5 7]);
a     = phi([2 4 6 8]);

% Transform the coordinates
% Conversion to the coordinate system.
% --------------------------------------------------------------
% for i = 1 : robot.NDOFs
%     T   = obj.getTransform([0 0 0 0 0 0 0],i,i+1);
%     p   = T(1:3, end);  
%     R   = T(1:3,1:3);
%     robot.mC(:,i) = R'*(robot.mC(:,i) - p);
% end
% 
% robot.mC(:,2) = robot.mC(:,2) + [0 0 0.2045]';
% % ---------------------------------------------------------------

L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1));        
L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset',0);
L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));
L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset',0);

% Display the current manipulator.
T_base = SE3.Ry(pi/2);
% T_base.t = [0;2;0];

manip_chain = SerialLink([L1 L2 L3 L4], 'name', '4DOF robot','base', T_base);

%% Compute the trajectory
q0 = [0 pi/8 0 0]; 
% qf = [pi/2 pi/4 pi/3 -pi/2];
qf = [pi/2 pi/4 pi/3 -pi/4];
m  = 2;

tf = 0.1;
t = 0:0.01:tf;
[q,qd,qdd] = jtraj(q0, qf, t);

%% Compute Torque
T = TorqueCompute(robot, manip_chain, q, qd, qdd, T_base);

%% Compute the motor characterization (Torque is dependant on the speed of the motor but acceleration cannot be gaurenteeded)
stall_torque(1)   = 200*0.0070615518333333;
free_run_speed(1) = 150*0.104719755;

stall_torque(2)   = 220*0.0070615518333333;
free_run_speed(2) = 100*0.104719755;

stall_torque(3)   = 200*0.0070615518333333;
free_run_speed(3) = 150*0.104719755;

stall_torque(4)   = 200*0.0070615518333333;
free_run_speed(4) = 150*0.104719755;

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
    plot(t,(T(i,:)))
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
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
manip_chain.plot(q, 'noshadow','workspace',[-0.6 0.6 -0.6 0.6 -1.3 1.3],'noarrow', 'view',[-130 50],'tile1color',[10 1 1],'delay',0.0001)

% [T,A] = manip_chain.fkine([0 pi/4 pi/3 -pi/2]);
% hold on
% for i=1:4
%   trplot(A(i), 'frame', num2str(i))
% end
% manip_res.vellipse(q, options)