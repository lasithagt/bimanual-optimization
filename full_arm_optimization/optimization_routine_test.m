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
L_1zz = 0.1; l_1x = 0.1;  m_1 = 3; L_2zz = 0.2; l_2x = 0.1; l_2y = 0.2; m_2 = 1;
L_3zz = 0.1; l_3x = 0.1;  l_3y = 0; m_3 = 0.2; L_4zz = 0.2; l_4x = 0.2; l_4y = 0.1; m_4 = 0.3;


%% Data Structure Generation
m       = 10;              % Number of discrete points along trajectory
n_links = 4;              % Number of revolute joints
% pa      = zeros(n_links, m, max_iter);    % EE location after FK

[des_poses, qdd_des] = trajectory_poses(m);
xx = linspace(0.9, 2.2, m);

% for i = 1:m
%     [yy(i), ~]       = arc_traj(xx(i));
%     rotm             = axang2rotm([0 0 1 atan2(norm(1),norm(2))]);
%     p_               = [yy(i) xx(i) 0];
%     des_poses(:,:,i) = [rotm p_';0 0 0 1];
% end

pd = des_poses;           % Trajectory points

q_min = (3/2).*[-pi/2,-pi/2, -pi/2,-pi/2];
q_max = (3/2).*[pi/2, pi/2, pi/2, pi/2];

input.m       = m;
input.n_links = n_links;
input.pd      = pd;
input.q_min   = q_min;
input.q_max   = q_max;

%% Solve the kinematic problem to satisfy the reachability. These lq would
% allow to be on the fk manifold. (position + orientation)

func     = @(X)cost_function_kin(X, input);
func_vec = @(X,pd)cost_function_vec(X,pd, input);

init_a   = [0.9759 0.5000 0.9153 0.5387];
a_min    = [0.5 0.5 0.5 0.5];
a_max    = [3 3 3 3];

% [lq, lq_vec] = optim_initial_morphology(init_a, a_min, a_max, input, func);
% cost_    = SA_cost([1.3421 0.7856 0.5190 0.5019], a_min, a_max, input, func);
SA_cost_ = @(x)SA_cost(x,a_min, a_max, input, func);
optim_a  = simulannealbnd(SA_cost_,init_a,a_min,a_max);

% optim_a = [1.2998 0.7458 0.5 0.5];
% optim_a = [1.3022 0.7163 0.5326 0.5895];
% cost_   = SA_cost(optim_a, a_min, a_max, input, func);
[lq, lq_vec] = optim_initial_morphology(optim_a, a_min, a_max, input, func);
% Constraints for physical feasibility 
fk_u      = zeros(7,8,m);
fk_u_full = [];
temp_fk_a = [];

%% Get the partials from implicit constraints

A     = [];
b     = [];

% Using quadprogramming to get the increment with joint limits
H    = eye(3 + 4 + (4*m)); 
f1   = [0 0 0 0];
f    = [];

for i = 1:m
    f  = horzcat(f, f1);
end

f = [f 0.0 0.0 0.0 0.0 0 0 0];
x = [lq';2;1;0];
% This is just for fwk constraint
Aeq = -[eye(3+4+(4*m));-eye(3+4+(4*m))];
q_u = (3/2).*[pi/2 pi/2 pi/2 pi/2]; q_l = -(3/2).*[pi/2 pi/2 pi/2 pi/2];

x_cons_u = [repmat(q_u,1,m) 2 2 2 2 2 2 2];
x_cons_l = [repmat(q_l,1,m) 0.5 0.5 0.5 0.5 -2 -2 -2];

beq      = -[x_cons_l'-x;-x_cons_u'+x];

thresh   = 10;
% x        = [lq' ;M_(lq)*[1 1 1 1]'];

% Run this infinity loop. To be on the manifold.
while (1)
    
    temp_fk_a = []; temp_manip_a = []; b_manip_full = [];
    fk_u_full = [];
    manip_u_full = [];
    
    for i = 1:m
        [temp_fk, temp_fk_uu] = FK_u(lq_vec(i,:));
        [A_manip, b_manip]    = manip_grad([x(4*(i-1)+1:4*(i-1)+4) x(end-6:end-3)]);
        temp_fk_a             = [temp_fk_a; temp_fk(:,5:end)];
        temp_manip_a          = [temp_manip_a; A_manip(:,5:end)];
        b_manip_full          = [b_manip_full; b_manip];
        
        manip_u_full = blkdiag(manip_u_full, A_manip(:,1:4));
        fk_u_full    = blkdiag(fk_u_full, temp_fk(:,1:4));
        
        % [M_u_,  M_uu_]    = M_u(x(1:8));
        % dyn_grad          = dynamic_grad(M_u_, [1 1 1 1], [1 1 1 1]);
        
    end
    temp         = [-0.*eye(3);zeros(4,3)];
    fk_u_full    = [fk_u_full temp_fk_a repmat(temp,m,1)];
    manip_u_full = [manip_u_full temp_manip_a zeros(3*m,3)];
    
    % restructure this again
    % A = [fk_u zeros(7,4);A_manip zeros(3,4);dyn_grad];
    A = [fk_u_full; manip_u_full];
    b = [zeros(7*m,1); b_manip_full]; 
    
    % b = [zeros(7,1);b_manip;zeros(4,1)];
    % t = func_vec(x(end-6:end-3)',x(end-3:end));
    % b(1:7*m) = 0.01 * t;

    beq = -[x_cons_l'-x;-x_cons_u'+x]; 
     
    % Quadratic Programming Problem
    options       = optimoptions('quadprog','Display','off');
    [dx, fval, s] = quadprog(H, f, Aeq, beq, A, b,[],[],[],options);
    x             = x + dx;
    c             = cost_function(x, input);
    
    clf(gcf)
    L1 = Link('d', 0, 'a', x(end-6), 'alpha', 0);        
    L2 = Link('d', 0, 'a', x(end-5), 'alpha', 0);
    L3 = Link('d', 0, 'a', x(end-4), 'alpha', 0);
    L4 = Link('d', 0, 'a', x(end-3), 'alpha', 0);
    L5 = Link('d', 0, 'a', 0.5, 'alpha', 0);

    % Plot many cofigurations evolutions
    optim_arm = SerialLink([L1 L2 L3 L4]);
    optim_arm_tool = SerialLink([L1 L2 L3 L4 L5]);
    
    for i = 1:m
        optim_arm.fellipse([x(4*(2-1)+1:4*(2-1)+4)'],'2d')
        optim_arm_tool.plot([x(4*(2-1)+1:4*(2-1)+4)' pi/4],'workspace', 4*[-0.3 1.5 -1 1 -1 1.5], 'noshadow','noarrow', 'view',[-90 90], 'tile1color',[0.9 0.9 0.9],'delay',0.01);

        hold on
%         plot(pd(1,end,1),pd(2,end,1),'k*')
    end
    
    if (c > thresh)
        break;
    end
    
end


%% Cost for SA to find the global design
function cost_ = cost_SA(lq_vec, input)
    m = input.m;
    cost_ = 0;
    for i = 1:m
        [~, ~, cost_diff] = manip_grad(lq_vec(i,:));
        cost_ = cost_ + sum(cost_diff.^2);
    end
end

%% Gives a optimum value for link length
function [lq, lq_vec] = optim_initial_morphology(init_a, a_min, a_max, input, func)
    global theta
    m = input.m; n_links = input.n_links;
    options  = optimoptions('fmincon','Display','iter','Algorithm','sqp');

    % Constraints for physical feasibility 
    A = [eye(n_links);-eye(n_links)];
    b = [a_max';-a_min'];
    optim_a = fmincon(func, init_a, A, b,[],[],[],[],[],options);

    lq = [theta(:)', optim_a ];
    lq_vec = [theta' repmat(optim_a,m,1)];
end

%% Uses simulated annealing to get a better estimate of the inital morphology
function cost_ = SA_cost(init_a, a_min, a_max, input, func_kin)

    [~, lq_vec] = optim_initial_morphology(init_a, a_min, a_max, input, func_kin);
    lq_vec(1, end-3:end)
    cost_       = cost_SA(lq_vec, input);
end





