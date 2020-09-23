% clear;
close all;
% clear all;

%% This script implements the implementation in 

global theta;
global input


%% Data Structure Generation
m       = 500;     % Number of discrete points along trajectory
n_links = 7;       % Number of revolute joints
d_var   = 8;
n_arms  = 2;

% get desired trajectory poses to optimize for
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
m  = size(des_poses,3);
close all
plot_trajectory(des_poses, 2, m) 

pd = des_poses;       % Trajectory points

% define joint limits
q_min = [-pi,-pi, -pi/2,-pi,-3*pi/4, -3*pi/4,-3*pi/4];
q_max = [ pi, pi,  pi/2, pi, 3*pi/4, 3*pi/4, 3*pi/4];

input.m       = m;
input.n_links = n_links;
input.n_arms  = n_arms;
input.pd      = pd;
input.q_min   = q_min;
input.q_max   = q_max;

T_L = eye(4);
T_R = eye(4);

T_R(1:3,1:3) = roty(-pi/2)*rotx(-0*pi/4);
T_R(1:3,end) = [10;-12;2];
T_L(1:3,1:3) = roty(pi/2)*rotx(-0*pi/4);
T_L(1:3,end) = [-10;-12;2];

input.T_L = T_L;
input.T_R = T_R;

input.link_variables = [0.0395 6.0000 0.0076 1.5603];
input.tool = 0;

% joint positions for the given desired positions
input.curr_q = zeros(n_links, m, n_arms);

%% Solve the kinematic problem to satisfy the reachability. These lq would
% allow to be on the fk manifold. (position + orientation)
func           = @(X)cost_function_dual(X, input);
func_INVJAC    = @(X)cost_function_dual_INVJAC(X);
func_INVSE3    = @(X)cost_function_dual_INVSE3(X);
% func_INVSE3_ga = @(X,input)cost_function_dual_INVSE3_ga(X,input);

func_vec       = @(X)cost_function_vec(X, input);

w = [0 0 1 0 1 0 0 1 0];
q = zeros(1,9);

init_a =  [-0.590711803586636, 0.849801739227642, 0.098539770795573,...
           -0.394528718057018, -0.313611137080781, -0.983279256066311,...
            0.526447520746619, -3.769026340884774, -0.845121971642268,...
            2.784223288127604, 2.042301014735812,-2.668835207631538,...
            -2.468384146280601, -2.320930273772749, 1.358513157641732,...
            2.379801527475640, 1.867896703390760, 2.406623292597008,...
            10,-12,2];


%% Constraints for physical feasibility 
a_min    = [-ones(1,9) -5*ones(1,9) 3 -12 0];
a_max    = [ones(1,9) 5*ones(1,9) 10 -6 5];

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
optim   = 0;
if (optim==1)
    rng default
    hybridopts = optimoptions('fmincon','Display','iter','Algorithm','interior-point');
    options_sa  = optimoptions('simulannealbnd', 'OutputFcn', @myoutput, 'Display','iter', ...
    'TemperatureFcn','temperatureboltz','InitialTemperature',100,'MaxIterations',5);

    [optim_X, fval]  = simulannealbnd(func_INVSE3, init_a, a_min, a_max, options_sa);

else
    optim_X = init_a;
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

