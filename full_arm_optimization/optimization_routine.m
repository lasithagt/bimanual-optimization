clear;
close all;

%% This script implements the implementation in 
% "Joint Optimization of Robot Design and Motion Parameters using the
% Implicit Function Theorem" RSS 2017

global L_1xx L_1xy L_1xz L_1yy L_1yz L_1zz l_1x l_1y l_1z m_1 fv_1 fc_1 L_2xx L_2xy L_2xz L_2yy L_2yz L_2zz l_2x l_2y l_2z m_2 fv_2 fc_2...
        L_3xx L_3xy L_3xz L_3yy L_3yz L_3zz l_3x l_3y l_3z m_3 fv_3 fc_3 L_4xx L_4xy L_4xz L_4yy L_4yz L_4zz l_4x l_4y l_4z m_4 fv_4 fc_4
global theta;
global des_poses

% Positions to be reached. (position and orientation)
L_1zz = 0.1; l_1x = 0.1;  m_1 = 1; L_2zz = 0.2; l_2x = 0.1; l_2y = 0.2; m_2 = 2;
L_3zz = 0.1; l_3x = 0.1;  l_3y = 0; m_3 = 1; L_4zz = 0.2; l_4x = 0.2; l_4y = 0.1; m_4 = 2;


%% Data Structure Generation
m       = 1;              % Number of discrete points along trajectory
n_links = 4;              % Number of revolute joints
% pa      = zeros(n_links, m, max_iter);    % EE location after FK

des_poses = trajectory_poses(m);
pd        = des_poses;           % Trajectory points

q_min = [-pi/2,-pi/2, -pi/2,-pi/2];
q_max = [pi/2, pi/2, pi/2, pi/2];

input.m       = m;
input.n_links = n_links;
input.pd      = pd;
input.q_min   = q_min;
input.q_max   = q_max;

%% Solve the kinematic problem to satisfy the reachability. These lq would
% allow to be on the fk manifold. (position + orientation)


func = @(X)cost_function_kin(X, input);
func_vec = @(X)cost_function_vec(X, input);
init_a = [0.2 0.2 0.2 0.2];
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');

% % Constraints for physical feasibility 
a_min = [0.1 0.1 0.1 0.1];
a_max = [2 2 2 2];
A = [eye(n_links);-eye(n_links)];
b = [a_max';a_min'];
optim_a = fmincon(func, init_a, A, b,[],[],[],[],[],options);

t_init = [0.5;0.5;0.5;0.5];

lq = [theta(:)', optim_a];
J  = J_(lq);
v  = pinv(J(1:2,1:4)')*t_init;

% x  = [lq v'];

%% Get the partials from implicit constraints
% for i = 1:m
[M_mat_u, M_mat_uu] = M_u(lq);  % Dyanmic Equation
[fk_u, fk_uu]       = FK_u(lq); % End effector point (pose)

fk_uu_pos   = fk_uu;
ind_p       = [];
% end


%% Matrix Computation (formulation as a quadratic problem
% Matrix with partials

% A_fwk = squeeze(2.*tmprod(fk_uu, func_vec(lq)', 1)) + 2.*tmprod(fk_u', fk_u', 2);
A_fwk = fk_u;

A     = A_fwk;
b     = zeros(8,1);% zeros(8,1);

% Using quadprogramming to get the increment with joint limits
% quadprog
H    = eye(10); 
f    = [0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.1];

Aeq = -[1 0 0 0 0 0 0 0 zeros(1,2);-1 0 0 0 0 0 0 0 zeros(1,2);0 1 0 0 0 0 0 0 zeros(1,2);0 -1 0 0 0 0 0 0 zeros(1,2);0 0 1 0 0 0 0 0 zeros(1,2);0 0 -1 0 0 0 0 0 zeros(1,2);0 0 0 1 0 0 0 0 zeros(1,2);0 0 0 -1 0 0 0 0 zeros(1,2);...
        0 0 0 0 1 0 0 0 zeros(1,2);0 0 0 0 -1 0 0 0 zeros(1,2);0 0 0 0 0 1 0 0 zeros(1,2);0 0 0 0 0 -1 0 0 zeros(1,2);0 0 0 0 0 0 1 0 zeros(1,2);0 0 0 0 0 0 -1 0 zeros(1,2);0 0 0 0 0 0 0 1 zeros(1,2);0 0 0 0 0 0 0 -1 zeros(1,2)];
beq = -[(-pi/2)-lq(1);-((pi/2)-lq(1));(-pi/2)-lq(2);-((pi/2)-lq(2));(-pi/2)-lq(3);-((pi/2)-lq(3));(-pi/2)-lq(4);-((pi/2)-lq(4));...
        0.5-lq(5);-(2-lq(5));0.5-lq(6);-(2-lq(6)); 0.5-lq(7);-(2-lq(7));0.5-lq(8);-(2-lq(8))];
thresh = 0.001;
x      = [lq';v];

% Run this infinity loop. To be on the manifold.

while (1)
    [fk_u,fk_uu]  = FK_u(x(1:8));
%     [M_u,  M_uu]  = M_u(x(1:8));
    [M,g] = manip_grad(x(1:8),x(9:end));
    A   = [fk_u zeros(7,2);g];

    beq = -[(-pi/2)-x(1);-((pi/2)-x(1));(-pi/2)-x(2);-((pi/2)-x(2));(-pi/2)-x(3);-((pi/2)-x(3));(-pi/2)-x(4);-((pi/2)-x(4));...
            0.5-x(5);-(2-x(5));0.5-x(6);-(2-x(6)); 0.5-x(7);-(2-x(7));0.5-x(8);-(2-x(8))];    
     
    % Quadratic Programming Problem
    [dx, fval] = quadprog(H, f, Aeq, beq, A, b);
    x  = x + dx;
    A*dx;
    x(9:end)'*M*x(9:end)
    c  = cost_function(x, input);
    
    if (c > thresh)
        break;
    end
    clf(gcf)
    L1 = Link('d', 0, 'a', x(5), 'alpha', 0);        
    L2 = Link('d', 0, 'a', x(6), 'alpha', 0);
    L3 = Link('d', 0, 'a', x(7), 'alpha', 0);
    L4 = Link('d', 0, 'a', x(8), 'alpha', 0);
    
    optim_arm = SerialLink([L1 L2 L3 L4], 'name', 'Single Manipulator');
    optim_arm.fellipse(x(1:4)','2d')
    optim_arm.plot(x(1:4)','workspace', 4*[-0.3 1.5 -1 1 -1 1.5], 'noshadow','noarrow', 'view',[-90 90], 'tile1color',[0.9 0.9 0.9],'delay',0.01);
    
    hold on
    plot(2,1,'r*')
    
end


%% Plotting

% Use transformations to get rid of joint limits

% Run in a loop while it converges


