clear; clc;
%% Import EM data
load('path_tracking.mat')
%% Process EM data
euler{1} = em_data_adj(4:end, :, 1);
euler{2} = em_data_adj(4:end, :, 2);
xyz{1} = em_data_adj(1:3, :, 1)./100;
xyz{2} = em_data_adj(1:3, :, 2)./100;
orientations = cell(1,2);
orientations{1} = zeros(3,3,length(em_data_adj));
orientations{2} = zeros(3,3,length(em_data_adj));

for i = 1:length(euler{1})
   for c = 1:2
       orientations{c}(:,:,i) = eye(3);%eul2rotm(euler{c}(:,i)', 'ZYX');
   end
end

T_d_R = zeros(4,4,length(euler{1}));
T_d_L = zeros(4,4,length(euler{1}));
for i = 1:length(orientations{1})
   T_d_R(:,:,i) = RpToTrans(orientations{1}(:,:,i), xyz{1}(:,i));
   T_d_L(:,:,i) = RpToTrans(orientations{2}(:,:,i), xyz{2}(:,i));
end
%% Manipulator twists
[ M_K, Slist_K, M_R, Slist_R, M_L, Slist_L] = dual_arm_twists();
%% Compute the weights matrix
global W
W_KUKA = 10*eye(7);
W_L    = eye(7);
W_R    = eye(7);
W      = blkdiag(W_KUKA,W_L,W_R);

% J = [J_K_s Adjoint(g_K)*Jst_L zeros(6,7);J_K_s zeros(6,7) Adjoint(g_K)*Jst_R];
%% Current configuration
theta_mini_L = 0*[0 0 pi/2 -2*pi/3 0 0 0]';
theta_mini_R = 0*[-pi 0 -pi/2 -pi/3 0 0 0]';
theta_KUKA   = [-pi/2 pi/4 0 pi/4 0 0 -pi/2]';
q = [theta_KUKA;theta_mini_R;theta_mini_L];

%% Command script
eomg = 0.001;
ev = 0.0001;
% the delta that needs to move
T_delta_R    = RpToTrans(eye(3),[0.0,0.0,0.05]');
T_delta_L    = RpToTrans(eye(3),[0.03,0.0,0.05]');

% current positions
FKinSpace(M_R, Slist_R, theta_mini_R);
FK_K         = FKinSpace(M_K, Slist_K, theta_KUKA);
FK_R         = FK_K * FKinSpace(M_R, Slist_R, theta_mini_R);
FK_L         = FK_K * FKinSpace(M_L, Slist_L, theta_mini_L);

% compute the spatial velocity from velocity
v_L = [0,0,0]'; v_R = [0,0,0]';
w_L = [0,0,0]'; w_R = [0,0,0]';
% [V_L, V_R] = computeSpatialVel(v_L, w_L, v_R, w_R, FK_R, FK_L);


err_L = zeros(4,4,length(T_d_R));
err_R = zeros(4,4,length(T_d_R));
x_right = zeros(1,length(T_d_R));
x_left = zeros(1,length(T_d_R));
y_right = zeros(1,length(T_d_R));
y_left = zeros(1,length(T_d_R));
for i = 1:length(T_d_R)

    
    [thetalist,L,R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R(:,:,i), T_d_L(:,:,i), q, eomg, ev);

    %% evaluate the result
    theta_KUKA = thetalist(1:7);
    theta_L    = thetalist(8:14);
    theta_R    = thetalist(15:21);

    FK_K         = FKinSpace(M_K, Slist_K, theta_KUKA);
    FK_R         = FK_K * FKinSpace(M_R, Slist_R, theta_R);
    FK_L         = FK_K * FKinSpace(M_L, Slist_L, theta_L);

    err_R(:,:,i) = T_d_R(:,:,i) - FK_R;
    err_L(:,:,i) = T_d_L(:,:,i) - FK_L;
    q = thetalist;
    x_right(i) = FK_R(1,4);
    y_right(i) = FK_R(2,4);
    x_left(i) = FK_L(1,4);
    y_left(i) = FK_L(2,4);
end
%% Graph

plot(x_right, y_right, x_left, y_left);
title('Actual')
legend('Right', 'Left')
figure()
plot(squeeze(T_d_R(1,4,:)), squeeze(T_d_R(2,4,:)),squeeze(T_d_L(1,4,:)), squeeze(T_d_L(2,4,:)))
title('Desired')
legend('Right', 'Left')
%% inverse differential kinematics for the dual arm
function [thetalist,err_val_n_L,err_val_n_R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R, T_d_L, thetalist0, eomg, ev)
    global W
    % differential kinematics
    thetalist = thetalist0;
    i = 0;
    maxiterations = 200;
    
    theta_mini_R = thetalist0(15:21);
    theta_mini_L = thetalist0(8:14);
    theta_KUKA   = thetalist0(1:7);

    FK_K  = FKinSpace(M_K, Slist_K, theta_KUKA);
    Tsb_L = FK_K * FKinSpace(M_L, Slist_L, theta_mini_L);
    Tsb_R = FK_K * FKinSpace(M_R, Slist_R, theta_mini_R);
    
    Vs_L = Adjoint(Tsb_L) * se3ToVec(MatrixLog6(TransInv(Tsb_L) * T_d_L));
    Vs_R = Adjoint(Tsb_R) * se3ToVec(MatrixLog6(TransInv(Tsb_R) * T_d_R));
    Vs   = [Vs_L;Vs_R];
    
    err_L = norm(Vs_L(1: 3)) > eomg || norm(Vs_L(4: 6)) > ev;
    err_R = norm(Vs_R(1: 3)) > eomg || norm(Vs_R(4: 6)) > ev;
    
    err_val_n_L = norm(Vs_L(4: 6)); err_val_n_R = norm(Vs_R(4: 6));
    err_val_L   = err_val_n_L;
    err_val_R   = err_val_n_R;
    
    alpha =0.08;
    while err_L && err_R && i < maxiterations
        J    = computeJacobian(thetalist);
        Jinv = computeJacInv(W, J);
        
        thetalist = thetalist + alpha * Jinv * Vs;
        
        theta_mini_R = thetalist(15:21);
        theta_mini_L = thetalist(8:14);
        theta_KUKA   = thetalist(1:7);

        FK_K  = FKinSpace(M_K, Slist_K, theta_KUKA);
        Tsb_L = FK_K * FKinSpace(M_L, Slist_L, theta_mini_L);
        Tsb_R = FK_K * FKinSpace(M_R, Slist_R, theta_mini_R);

        Vs_L = Adjoint(Tsb_L) * se3ToVec(MatrixLog6(TransInv(Tsb_L) * T_d_L));
        Vs_R = Adjoint(Tsb_R) * se3ToVec(MatrixLog6(TransInv(Tsb_R) * T_d_R));
        
        Vs = [Vs_L;Vs_R];
        
        err_L = norm(Vs_L(1: 3)) > eomg || norm(Vs_L(4: 6)) > ev;
        err_R = norm(Vs_R(1: 3)) > eomg || norm(Vs_R(4: 6)) > ev;
        
        err_val_n_L = norm(Vs_L(4: 6)); err_val_n_R = norm(Vs_R(4: 6));
        
        if (err_val_n_L > err_val_L) && (err_val_n_R > err_val_R)
            break;
%             alpha = alpha * 0.1;
        end
        err_val_L = err_val_n_L; err_val_R = err_val_n_R;
        err_val_R;
        
        i = i + 1;
    end
end

function Jinv = computeJacInv(W, J)
    Jinv = W\J'/(J/W*J');
end

function J = computeJacobian(q)
    theta_mini_L = q(8:14);
    theta_mini_R = q(15:end);
    theta_KUKA   = q(1:7);
    [g_K, ~, ~, Jst_R, Jst_L, J_K_s, ~, ~,~] = dual_arm_manipulator_kinematics(theta_mini_L, theta_mini_R, theta_KUKA);
    J = [J_K_s Adjoint(g_K)*Jst_L zeros(6,7);J_K_s zeros(6,7) Adjoint(g_K)*Jst_R];
end

function [V_L, V_R] = computeSpatialVel(v_L, w_L, v_R, w_R, g_R, g_L)
    Rdot_L = cross(repmat(w_L,1,3), g_L(1:3,1:3)); Rdot_R = cross(repmat(w_R,1,3), g_R(1:3,1:3)); 
    %     Rdot_K = cross(repmat(w_K,1,3), g_K(1:3,1:3));
    %     gdot_K = [[Rdot_K;0 0 0],[v_K;0]];
    gdot_L = [[Rdot_L;0 0 0],[v_L;0]];
end


    
