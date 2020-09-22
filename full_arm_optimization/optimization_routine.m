% clear;
close all;
% clear all;

%% This script implements the implementation in 
% "Joint Optimization of Robot Design and Motion Parameters using the
% Implicit Function Theorem" RSS 2017

% global L_1xx L_1xy L_1xz L_1yy L_1yz L_1zz l_1x l_1y l_1z m_1 fv_1 fc_1 L_2xx L_2xy L_2xz L_2yy L_2yz L_2zz l_2x l_2y l_2z m_2 fv_2 fc_2...
%         L_3xx L_3xy L_3xz L_3yy L_3yz L_3zz l_3x l_3y l_3z m_3 fv_3 fc_3 L_4xx L_4xy L_4xz L_4yy L_4yz L_4zz l_4x l_4y l_4z m_4 fv_4 fc_4

global theta;
global input

% Positions to be reached. (position and orientation)
% L_1zz = 0.1; l_1x = 0.1;  m_1 = 1; L_2zz = 0.2; l_2x = 0.1; l_2y = 0.2; m_2 = 2;
% L_3zz = 0.1; l_3x = 0.1;  l_3y = 0; m_3 = 1; L_4zz = 0.2; l_4x = 0.2; l_4y = 0.1; m_4 = 2;


%% Data Structure Generation

m       = 50;      % Number of discrete points along trajectory
n_links = 7;      % Number of revolute joints
d_var   = 8;
n_arms  = 2;

% get desired trajectory poses to optimize for
data_file = 'Data/name_writing_sticky_note.mat';

% data_file = 'Data/name_writing.mat';
des_poses = trajectory_pose_read(data_file, m);

data_file = {'Data/path_tracking_new_pp.mat', 'Data/soldering_new_pp.mat', 'Data/suturing_new_pp.mat'};
% data_file = { 'Data/path_tracking_new_pp.mat'};

des_poses = [];
% linearly connect end and start points of each data file
for i = 1:length(data_file)
    des_poses_temp = trajectory_pose_read(data_file{i}, m);
    
    first_point = des_poses_temp(:,:,1,:);
    
    % joint these two points by linear interpolation
    if (i > 1)
        for j = 1:2
            traj = CartesianTrajectory(first_point(:,:,:,j), last_point(:,:,:,j), 1, 10, 3);
            for k = 1:length(traj)
                transition(:,:,k,j) = traj{k};
            end
        end
    des_poses = cat(3, des_poses, transition);
    end
    
    last_point = des_poses_temp(:,:,end,:);
   
    des_poses = cat(3, des_poses, des_poses_temp);
end
% create dummy data points for testing
% des_poses = zeros(4,4,1,2);

m  = size(des_poses,3);


% create dummy data points for testing
% des_poses = zeros(4,4,1,2);

pd = des_poses;       % Trajectory points

% define joint limits
q_min = [-pi,-pi/3, -pi/2,-pi,-3*pi/4, -3*pi/4,-3*pi/4];
q_max = [ pi, pi/3,  pi/2, pi, 3*pi/4, 3*pi/4, 3*pi/4];

input.m       = m;
input.n_links = n_links;
input.n_arms  = n_arms;
input.pd      = pd;
input.q_min   = q_min;
input.q_max   = q_max;

T_L = eye(4);
T_R = eye(4);

T_R(1:3,1:3) = roty(-pi/2)*rotx(-pi/3);
T_R(1:3,end) = [3;-12;2];
T_L(1:3,1:3) = roty(pi/2)*rotx(-pi/3);
T_L(1:3,end) = [-3;-12;2];
% T_R(1:3,end) = [0.5;-12;3];
% T_L(1:3,end) = [-0.5;-12;3];

input.T_L = T_L;
input.T_R = T_R;

input.link_variables = [0.0395 6.0000 0.0076 1.5603];
input.tool = 0;

% joint positions for the given desired positions
input.curr_q = zeros(n_links, m, n_arms);

%% Solve the kinematic problem to satisfy the reachability. These lq would
% allow to be on the fk manifold. (position + orientation)

func         = @(X)cost_function_dual(X, input);
func_INVJAC  = @(X)cost_function_dual_INVJAC(X);
func_INVSE3  = @(X)cost_function_dual_INVSE3(X);
func_INVSE3_MR  = @(X)cost_function_dual_INVSE3_MR(X);
func_vec     = @(X)cost_function_vec(X, input);


%% Constraints for physical feasibility 

init_a   = [2.4354  -10.2294    0.7530    1.4584    1.4269         0    4.1412    3.9466]; 
init_a   = [2.4354  -10.2294    pi/2    0*pi/2    1.4269         0    4.1412    3.9466]; 
init_a   = [2.9980 -5.0025 1.5708 pi/3 0.0461 0.5000 6.0000 5.9964];
init_a   = [2.5371    6.5860    1.5708    0.7953    2.5173    0.5000    4.0102    3.0349];

% % Constraints for physical feasibility 
a_min    = [0 -10 pi/2 0.0 0 0.5 0.5 0.5];
a_max    = [4 10 pi/2 pi/2 6 0.5 6 6];
A        = [eye(d_var); -eye(d_var)];
b        = [a_max'; a_min'];
% global DebugLevel;
% DebugLevel = 1;

% use fmincon or sa to solve to get a prior solution for the design

%% Constrained Optimization
% options_cp  = optimoptions('fmincon','Display','iter','Algorithm','sqp');
% optim_X  = fmincon(func_INVSE3, init_a, A, b,[],[],[],[],[],options_cp)

%% Simulated Annealing
% define a function in SA to generate samples
optim = 0;
rng default
options_sa  = optimoptions('simulannealbnd','Display','iter','TemperatureFcn','temperatureboltz', ...
    'InitialTemperature',500,'MaxIterations',100);

if (optim==1)
    optim_X     = simulannealbnd(func_INVSE3_MR,init_a,a_min,a_max,options_sa);
else
    optim_X     = init_a;
end

save('robot.mat','input');
% input = update_input_struct(optim_X, input);
% options_sa  = optimoptions('simulannealbnd','Display','iter','TemperatureFcn','temperatureboltz','InitialTemperature',100,'MaxIterations',200);
% optim_X     = simulannealbnd(func_INVJAC,init_a,a_min,a_max,options_sa)

% %% Genetic Algorithm
% rng default % For reproducibility
% options_ga = optimoptions('ga','ConstraintTolerance',1e-6,'PlotFcn', @gaplotbestf);
% optim_X    = ga(func_INVJAC, 8, [], [], [], [], a_min, a_max, [], options_ga);

% cost_function_dual_INVJAC(optim_X)

save('Data/robot_temp.mat','input');

% animate the results.
input = update_input_struct(optim_X, input);
plot_animate(optim_X, input, data_file{1});

% optimization for dexterity while maintaining the positional constraints. 
t_init = [0.5;0.5;0.5;0.5];
lq     = [theta(:)', optim_X];







