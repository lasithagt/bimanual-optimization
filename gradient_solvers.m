%% Gradient Based Methods for Optimization Routine for a 2DOF or 3DOF Robotic
% Manipulator
%   - This script determines the optimal set of DH parameters given a
%   trajectory discretized into several points.
%   - The objective function evaluates the weighted combination of the ability
%   for the manipulator to reach all the points along the trajectory and
%   the dexterity of the manipulator at each point along the trajectory
%   - The design vector (X or Xi) is a 2x1 (2 joint/2D) or 6x1 (3 joint/3D)
%   column vector.
clc; clear; close all;
global theta f2 dex

%% Data Structure Generation
fun         = @objectiveFunction;
max_iter    = 100;                             % max iterations for the optimization routine
iter        = 0;
fxn_calls   = 0;
m           = 20;                               % number of discrete points along trajectory
n_links     = 3;                               % number of revolute joints
pd          = zeros(n_links, m);              % trajectory points
X           = zeros(2*n_links, max_iter);     % time history of design vector (3D) (a and d)
alpha       = zeros(n_links, 1);              % vector of alphas if not included in design vector
% theta       = zeros(n_links, m, max_iter);    % joint angles throughout optimization routine
pa          = zeros(n_links, m, max_iter);    % EE location after FK
err_ls      = zeros(1, max_iter);              % Least squared error throughout the process
f1          = zeros(1, max_iter);              % sum of least square error each iteration
dex         = zeros(m, max_iter);              % dexterities evaluated at each pt
f2          = zeros(1, max_iter);              % std dev of dexterities each iteration
F           = zeros(1, max_iter);              % overall objective function throughout process

%% Initial Guesses
% Set Weights
w1          = 0.6;                             % weight/penalty on least squared error
w2          = 0.2;                             % weight/penalty on dexterity
w3          = 0.2;
%% Path to track, gives desired points
pd          = generateTrajectory_plot(m);

%% Input to optimizationRoutine Function
% 3D/3 Revolute Joints (Colume vector)
X(:,1)      = [0.05, 0.3, 0.05, 0.2, 0.05, 0.15]';
% X(:,1)      = [0.7786    0.1311    0.0009    0.8237    0.0010    0.5824];

input.Xi    = X(:,1);
alpha       = [pi/2, 0, 0]';
input.alpha = alpha;
input.q_min = [0 -2*pi/3 -2*pi/3]; 
input.q_max = [2*pi 2*pi/3 2*pi/3]; 
input.w1    = w1;
input.w2    = w2;
input.w3    = w3;
input.m     = (m-1)*5;
input.n_links = n_links;
input.pd    = pd;


%% Initial Tasks
func = @(X)objectiveFunction(X, input);

%% Solver (SQP matlab)
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
% % Constraints for physical feasibility 
da_min = [0 0.1 0 0.1 0 0.1];
da_max = [1 1 1 1 1 1];
A = [eye(2*n_links);-eye(2*n_links)];
b = [da_max';da_min'];
optim_X = fmincon(func, X(:,1)', A, b,[],[],[],[],[],options)

q_ref   = theta
f2
dex


%% Plotting
% before optimization
% generate initial trajectory
% q_ref_init = [];
% for i = 1:m
%     q_ref = IK(Xi(:,1), alpha, pd, initial_q, cons_l, cons_u)
% end
% 
% figure(1)
% plotResults(X(:,1), alpha, q_ref_init, x_d)

% after optimization

plotResults(optim_X, alpha, q_ref(:,2:end)', pd)
