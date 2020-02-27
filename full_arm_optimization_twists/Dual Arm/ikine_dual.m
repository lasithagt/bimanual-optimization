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
    
    kp = {M_K, Slist_K, M_R, Slist_R, M_L, Slist_L};

    Vs_L = Adjoint(Tsb_L) * se3ToVec(MatrixLog6(TransInv(Tsb_L) * T_d_L));
    Vs_R = Adjoint(Tsb_R) * se3ToVec(MatrixLog6(TransInv(Tsb_R) * T_d_R));
    Vs   = [Vs_L;Vs_R];
    
    err_L = norm(Vs_L(1: 3)) > eomg || norm(Vs_L(4: 6)) > ev;
    err_R = norm(Vs_R(1: 3)) > eomg || norm(Vs_R(4: 6)) > ev;
    
    err_val_n_L = norm(Vs_L(4: 6)); err_val_n_R = norm(Vs_R(4: 6));
    err_val_L   = err_val_n_L;
    err_val_R   = err_val_n_R;
    
    alpha =1;
    while (err_L || err_R) && i < maxiterations
        J    = computeJacobian(kp, thetalist);
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
        
        i = i + 1;
    end
end

function Jinv = computeJacInv(W, J)
    Jinv = W\J'/(J/W*J');
end

function J = computeJacobian(kp, q)
    theta_mini_L = q(8:14);
    theta_mini_R = q(15:end);
    theta_KUKA   = q(1:7);
    %     [g_K, ~, ~, Jst_R, Jst_L, J_K_s, ~, ~,~] = dual_arm_manipulator_kinematics(theta_mini_L, theta_mini_R, theta_KUKA);
    [g_K, ~, ~, Jst_R, Jst_L, J_K_s] = dual_arm_manipulator_kinematics_custom(kp, theta_mini_R, theta_mini_L, theta_KUKA);

    J = [J_K_s Adjoint(g_K)*Jst_L zeros(6,7);J_K_s zeros(6,7) Adjoint(g_K)*Jst_R];
end