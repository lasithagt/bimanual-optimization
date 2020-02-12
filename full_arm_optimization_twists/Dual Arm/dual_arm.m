clc; clear;


%% manipulator twists
[ M_K, Slist_K, M_R, Slist_R, M_L, Slist_L] = dual_arm_twists();

%% get dual arm twists
% [g_K, g_L, g_R, Jst_R, Jst_L, J_K_s, J_K_b, M_K, Slist_K] = dual_arm_manipulator_kinematics(theta_mini_L, theta_mini_R, theta_KUKA);


%% compute the weights matrix
global W
W_KUKA = 10*eye(7);
W_L    = eye(7);
W_R    = eye(7);
W      = blkdiag(W_KUKA,W_L,W_R);

% J = [J_K_s Adjoint(g_K)*Jst_L zeros(6,7);J_K_s zeros(6,7) Adjoint(g_K)*Jst_R];

%% current configuration
theta_mini_L = 0*[0 0 pi/2 -2*pi/3 0 0 0]';
theta_mini_R = 0*[-pi 0 -pi/2 -pi/3 0 0 0]';
theta_KUKA   = [-pi/2 pi/4 0 pi/4 0 0 -pi/2]';
q = [theta_KUKA;theta_mini_R;theta_mini_L];

%% compute the joint velocities
J            = computeJacobian(q);
% t = W\J'/(J/W*J')*[se3ToVec(V_R);se3ToVec(V_L)];

%% main script
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
[V_L, V_R] = computeSpatialVel(v_L, w_L, v_R, w_R, FK_R, FK_L);

% here, compute the final pose after moving the amount defined by
% T_delta_R and T_delta_L
T_d_R      = FK_R * T_delta_R;
T_d_L      = FK_L * T_delta_L;


% [thetalist, success] = IKinSpace(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R, T_d_L, q, eomg, ev);
[thetalist,L,R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R, T_d_L, q, eomg, ev);

%% evaluate the result
theta_KUKA = thetalist(1:7);
theta_L    = thetalist(8:14);
theta_R    = thetalist(15:21);

FK_K         = FKinSpace(M_K, Slist_K, theta_KUKA);
FK_R         = FK_K * FKinSpace(M_R, Slist_R, theta_R);
FK_L         = FK_K * FKinSpace(M_L, Slist_L, theta_L);

err_L        = T_d_R - FK_R
err_R        = T_d_L - FK_L

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
    gdot_R = [[Rdot_R;0 0 0],[v_R;0]];

    V_L = gdot_L / g_L;
    V_R = gdot_R / g_R;
    %     V_K = gdot_K * inv(g_K);
end