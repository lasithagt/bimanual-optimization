clear;  close all

%% Import EM data
addpath('../')

%% Load and process EM tracker data
m         = 50;
des_poses = trajectory_pose_read('soldering_new_pp.mat', m);
pd        = des_poses(:,:,:,:);
pd(1:3,end,:,:) = pd(1:3,end,:,:)./100;


T_d_R = pd(:,:,:,2);
T_d_L = pd(:,:,:,1);

%% change the global reference frame.
T_ck          = [0.6691,0,0.7431,0;0,-1,0,-0.0893;0.7431, 0, -0.6691, 1.3571;0 0 0 1];
T_ck(1:3,1:3) = rotx(pi);
T_ck(1:3,end) = [0 -0.0893 0.9]';


for i = 1:m
    T_d_R(:,:,i) = T_ck * T_d_R(:,:,i);
    T_d_L(:,:,i) = T_ck * T_d_L(:,:,i);
end

%% Manipulator morphology (can be changed, with optimized . w and q)
w = {normalize([0.9998   -0.9834    0.3016],'norm'), normalize([-0.5236   -0.8948   -0.8543],'norm')', normalize([0.9248   -0.4488    0.9948],'norm')'};
q = {[1.9994 1.9895 -1.9846]/100,   [-1.4524   -1.9511    1.6301]/100,   [1.8253    0.2947    0.3526]/100};

T_L = eye(4); T_R = eye(4);

T_R(1:3,1:3) = roty(-pi/2)*rotx(-0*pi/4);
T_R(1:3,end) = [-3;-6.0107;0.9967]./100;

T_L(1:3,1:3) = roty(pi/2)*rotx(-0*pi/4);
T_L(1:3,end) = [3;-6.0107;0.9967]./100;

T_B = {T_L, T_R};

%% Manipulator twists

[ M_K, Slist_K, M_R, Slist_R, M_L, Slist_L] = dual_arm_twists_custom_update(w, q, T_B);
% [ M_K, Slist_K, M_R, Slist_R, M_L, Slist_L] = dual_arm_twists();

%% Compute the weights matrix
global W


%% pre-initialize mini arms
% T = zeros(4,4,2); T(:,:,1) = input.T_L; T(:,:,2) = input.T_R;
[temp_t]                            = IK_SE3(Slist_R, M_R, pd(:,:,:,2)); 
theta_R                             = temp_t;
pa(:,:,:,2)                         = FK_SE3(M_R, Slist_R, theta_R); 
V_se3(pd(:,:,:,2),pa(:,:,:,2))

[temp_t]                            = IK_SE3(Slist_L, M_L, pd(:,:,:,1)); 
theta_L                             = temp_t;
pa(:,:,:,1)                         = FK_SE3(M_L, Slist_L, theta_L); 
V_se3(pd(:,:,:,1),pa(:,:,:,1))

%% Current configuration
theta_mini_L = theta_L(:,1);
theta_mini_R = theta_R(:,1);
theta_KUKA   = [pi/4 pi/4 0 pi/2 0 pi/4 -pi/4]';
q            = [theta_KUKA;theta_mini_L;theta_mini_R];

%% Command script
eomg = 0.001;
ev   = 0.001;

% the delta that needs to move
T_delta_R    = RpToTrans(eye(3),[0.0,0.0,0.0]');
T_delta_L    = RpToTrans(eye(3),[0.01,0.0,0.0]');

% current positions
FK_K         = FKinSpace(M_K, Slist_K, theta_KUKA);
FK_R         = FK_K * FKinSpace(M_R, Slist_R, theta_mini_R);
FK_L         = FK_K * FKinSpace(M_L, Slist_L, theta_mini_L);

T_R          = FK_R * T_delta_R;
T_L          = FK_L * T_delta_L;

% find the global position with kuka abled.
W_KUKA = 1*eye(7);
W_L    = 10*eye(7);
W_R    = 10*eye(7);
W      = blkdiag(W_KUKA,W_L,W_R);


% [theta0,L,R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_R, T_L, q, eomg, ev);
[theta0,L,R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R(:,:,1), T_d_L(:,:,1), q, eomg, ev);
q            = theta0;

% redefine the matrix weights to be higher
W_KUKA = 1*eye(7);
W_L    = 0.1*eye(7);
W_R    = 0.1*eye(7);
W      = blkdiag(W_KUKA,W_L,W_R);


% compute the spatial velocity from velocity
% v_L = [0,0,0]'; v_R = [0,0,0]';
% w_L = [0,0,0]'; w_R = [0,0,0]';
% [V_L, V_R] = computeSpatialVel(v_L, w_L, v_R, w_R, FK_R, FK_L);

fk_actual_K = zeros(4,4,m);
fk_actual_L = zeros(4,4,m);
fk_actual_R = zeros(4,4,m);

q_mem = zeros(21,m);
q_mem(:,1) = theta0;

for i = 1:length(T_d_R)

    [thetalist,L,R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R(:,:,i), T_d_L(:,:,i), q, eomg, ev);
    
    %% evaluate the result
    theta_KUKA = thetalist(1:7);
    theta_L    = thetalist(8:14);
    theta_R    = thetalist(15:21);

    FK_K       = FKinSpace(M_K, Slist_K, theta_KUKA);
    FK_R       = FK_K * FKinSpace(M_R, Slist_R, theta_R);
    FK_L       = FK_K * FKinSpace(M_L, Slist_L, theta_L);
    
    fk_actual_K(:,:,i)  = FK_K;
    fk_actual_L(:,:,i)  = FK_L; 
    fk_actual_R(:,:,i)  = FK_R; 

    q_mem(:,i)   = thetalist;
    % compute performance
    %     J = computeJacobian(thetalist);
    %     JTJ = J'*J;
    %     E = eig(JTJ);
    %     d(i) = min(E);
    %     D(i) = max(E);
    
end


%% Quiver plot
figure(1)
for i = 1:m
    r_R = fk_actual_R(1:3,1:3,i) * [0 0 -1]';
    plot3(fk_actual_R(1,end,i), fk_actual_R(2,end,i), fk_actual_R(3,end,i),'b*');
    hold on
    quiver3(fk_actual_R(1,end,i), fk_actual_R(2,end,i), fk_actual_R(3,end,i), r_R(1), r_R(2), r_R(3),  'Color', 'b', 'LineWidth',2,'MaxHeadSize',0.1, 'AutoScaleFactor', 0.01);
    hold on;
    r_L = fk_actual_L(1:3,1:3,i) * [0 0 -1]';
    plot3(fk_actual_L(1,end,i), fk_actual_L(2,end,i), fk_actual_L(3,end,i),'k*');
    hold on
    quiver3(fk_actual_L(1,end,i), fk_actual_L(2,end,i), fk_actual_L(3,end,i), r_L(1), r_L(2), r_L(3),  'Color', 'k', 'LineWidth',2,'MaxHeadSize',0.1, 'AutoScaleFactor', 0.01);
    title('Obtained trajectory')
end

figure(2)
for i = 1:m
    r_R = T_d_R(1:3,1:3,i) * [0 0 -1]';
    plot3(T_d_R(1,end,i), T_d_R(2,end,i), T_d_R(3,end,i),'b*');
    hold on
    quiver3(T_d_R(1,end,i), T_d_R(2,end,i), T_d_R(3,end,i), r_R(1), r_R(2), r_R(3),  'Color', 'b', 'LineWidth',2,'MaxHeadSize',0.1, 'AutoScaleFactor', 0.01);
    hold on;
    r_L = T_d_L(1:3,1:3,i) * [0 0 -1]';
    plot3(T_d_L(1,end,i), T_d_L(2,end,i), T_d_L(3,end,i),'k*');
    hold on
    quiver3(T_d_L(1,end,i), T_d_L(2,end,i), T_d_L(3,end,i), r_L(1), r_L(2), r_L(3),  'Color', 'k', 'LineWidth',2,'MaxHeadSize',0.1, 'AutoScaleFactor', 0.01);
    title('Desired trajectory')
end


%% future extensions
% function [V_L, V_R] = computeSpatialVel(v_L, w_L, v_R, w_R, g_R, g_L)
%     Rdot_L = cross(repmat(w_L,1,3), g_L(1:3,1:3)); Rdot_R = cross(repmat(w_R,1,3), g_R(1:3,1:3)); 
%     %     Rdot_K = cross(repmat(w_K,1,3), g_K(1:3,1:3));
%     %     gdot_K = [[Rdot_K;0 0 0],[v_K;0]];
%     gdot_L = [[Rdot_L;0 0 0],[v_L;0]];
% end


    
