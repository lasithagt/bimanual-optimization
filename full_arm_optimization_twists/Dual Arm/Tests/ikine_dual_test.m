%% inverse differential kinematics for the dual arm
function [thetalist,err_val_n_L,err_val_n_R] = ikine_dual_test(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R, T_d_L, thetalist0, eomg, ev)
    global W T_B dual_kinematic_params kuka_weight
    % differential kinematics
    thetalist = thetalist0;
    i = 0;
    maxiterations = 500;
    
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
    
    
    alpha = 0.08;
    while (err_L || err_R) && i < maxiterations
        J    = computeJacobian(dual_kinematic_params, thetalist);
        Jinv = computeJacInv(W, J);
         
        % null space projection 
        null_space_proj = null_space_optimization(thetalist, T_B, dual_kinematic_params);
%         n_v             = (eye(21) - J'*Jinv') * null_space_proj';

%         J(:,1:7)        = J(:,1:7) * W;
        n_v             = null(J) * null(J)' * null_space_proj';

        thetalist       = thetalist + alpha * Jinv * Vs + 1.0*n_v;
        thetalist = thetalist + alpha * Jinv * Vs;
        
        theta_mini_R = thetalist(15:21);
        theta_mini_L = thetalist(8:14);
        theta_KUKA   = thetalist(1:7);

        FK_K  = FKinSpace(M_K, Slist_K, theta_KUKA);
        Tsb_L = FK_K * FKinSpace(M_L, Slist_L, theta_mini_L);
        Tsb_R = FK_K * FKinSpace(M_R, Slist_R, theta_mini_R);

        Vs_L = Adjoint(Tsb_L) * se3ToVec(MatrixLog6(TransInv(Tsb_L) * T_d_L));
        Vs_R = Adjoint(Tsb_R) * se3ToVec(MatrixLog6(TransInv(Tsb_R) * T_d_R));
        
        Vs = [Vs_L; Vs_R];
        
        err_L = norm(Vs_L(1: 3)) > eomg || norm(Vs_L(4: 6)) > ev;
        err_R = norm(Vs_R(1: 3)) > eomg || norm(Vs_R(4: 6)) > ev;
        
        err_val_n_L = norm(Vs_L(4: 6)); err_val_n_R = norm(Vs_R(4: 6));
        
        if (err_val_n_L > err_val_L) && (err_val_n_R > err_val_R)
            break;
            %  alpha = alpha * 0.1;
        end
        err_val_L = err_val_n_L; 
        err_val_R = err_val_n_R;
        
        i = i + 1;
    end
end

function Jinv = computeJacInv(W, J)
%     J(1:6,1:7) = W * J(1:6,1:7);
%     J(7:end,1:7) = W * J(7:end,1:7);  
    Jinv = W\J'/(J/W*J');
end

function J = computeJacobian(kp, q)
    global kuka_weight
    theta_mini_L = q(8:14);
    theta_mini_R = q(15:end);
    theta_KUKA   = q(1:7);
    [g_K, ~, ~, Jst_L, Jst_R, J_K_s] = dual_arm_manipulator_kinematics_custom(kp, theta_mini_L, theta_mini_R, theta_KUKA);
    J = [J_K_s*kuka_weight Adjoint(g_K)*Jst_L zeros(6,7);J_K_s*kuka_weight zeros(6,7) Adjoint(g_K)*Jst_R];
end

