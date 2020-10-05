% clear;
close all;
% clear all;

%% This script implements the implementation in 
global input


%% Data Structure Generation
m       = 500;     % Number of discrete points along trajectory
n_links = 7;       % Number of revolute joints
d_var   = 8;
n_arms  = 2;

% get desired trajectory poses to optimize for
% data_file = {'Data/path_tracking_new_pp.mat', 'Data/soldering_new_pp.mat', 'Data/suturing_new_pp.mat'};
data_file = { 'Data/path_tracking_new_pp.mat'};

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
m  = size(des_poses,3);
close all
plot_trajectory(des_poses, 2, m) 

pd = des_poses;       % Trajectory points

% define joint limits
q_min = [-3*pi/4,3*pi/4, -pi/2,-3*pi/4,-3*pi/4, -3*pi/4,-3*pi/4];
q_max = [3*pi/4, 5*pi/4,  pi/2, 3*pi/4, 3*pi/4, 3*pi/4, 3*pi/4];

input.m       = m;
input.n_links = n_links;
input.n_arms  = n_arms;
input.pd      = pd;
input.q_min   = q_min;
input.q_max   = q_max;
guess_init = {[pi/5;pi; pi/2; pi/3; pi/3; 0; 0], -[pi/5;pi; pi/2; pi/3; pi/3; 0; 0]};

input.guess_init = guess_init;

T_L           = eye(4);
T_L(1:3, 1:3) = roty(-pi/2);
T_L(1:3, end) = [-3.2876, -7.4806, 3.5388];

T_R           = eye(4);
T_R(1:3, 1:3) = [1 0 0;0 -1 0;0 0 1] * T_L(1:3,1:3);
T_R(1:3, end) = [3.2876 ;-7.4806; 3.5388];

input.T_L = T_L;
input.T_R = T_R;

input.link_variables = [0.0395 6.0000 0.0076 1.5603];
input.tool = 0;

% joint positions for the given desired positions
input.curr_q = zeros(n_links, m, n_arms);

%% Solve the kinematic problem to satisfy the reachability. These lq would
% allow to be on the fk manifold. (position + orientation)
func           = @(X)cost_function_dual(X, input);
func_vec       = @(X)cost_function_vec(X, input);

w = [0 0 1 0 1 0 0 1 0];
q = zeros(1,9);


init_a = [0.3525    0.8470    0.9581,...
         -0.4208    0.1685   -0.8550 ,...
          0.4513    0.3423    0.8871,...
         -1.7160    1.7745    0.7434,...
          1.7637   -1.7036    3.2626,...
         -0.6435    0.8234    6.5100,...
          5.7270   -5.5765    6.7165];

% init_a = [0, 0, 1,...
%           0, 1, 0,...
%           0, 0, 1,...
%           0, 0, 10*0.2025,...
%           0, 0, 10*(0.2025+0.42),...
%           0, 0, 10*(0.2025+0.42+0.4),...
%           3.2876, -7.4806, 3.5388];


%% Constraints for physical feasibility 
c_min_1 = [-2 -2 0.5];
c_min_2 = [-2 -2 3];
c_min_3 = [-2 -2 6];

c_max_1 = [2 2 3];
c_max_2 = [2 2 6];
c_max_3 = [2 2 8];

a_min    = [-ones(1,9) c_min_1 c_min_2 c_min_3 1 -12 0];
a_max    = [ones(1,9) c_max_1 c_max_2 c_max_3 10 -3 10];

% A        = [eye(d_var); -eye(d_var)];
% b        = [a_max'; a_min'];
% use fmincon or sa to solve to get a prior solution for the design
% % % parameters
% options_cp  = optimoptions('fmincon','Display','iter','Algorithm','sqp');
% optim_X  = fmincon(func_INVSE3, init_a, A, b,[],[],[],[],[],options_cp)
% define a function in SA to generate samples

global history iteration
history = [];
iteration = [];
optim   = 1;
if (optim == 1)
    rng default
    hybridopts = optimoptions('patternsearch','Display','iter');
    options_sa  = optimoptions('simulannealbnd', 'OutputFcn', @myoutput, 'Display','iter', ...
    'TemperatureFcn','temperatureboltz','InitialTemperature',500,'MaxIterations',100, 'HybridFcn',{@patternsearch,hybridopts});

    [optim_X, fval]  = simulannealbnd(func_INVSE3, init_a, a_min, a_max, options_sa);

else
    optim_X = init_a;
    F = cost_function_dual_INVSE3(optim_X)
end

save('robot.mat','input');
input = update_input_struct(optim_X, input);

% animate the results.
plot_animate(optim_X, input, data_file{1});


function [stop,options,optchanged] = myoutput(options, state, flag) 
global history iteration
optchanged = false;
  stop = false;
    iteration = [iteration; state.iteration];
    history = [history; state.bestx];
end

