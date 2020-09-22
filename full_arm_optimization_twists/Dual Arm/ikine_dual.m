%% inverse differential kinematics for the dual arm
function [thetalist,err_val_n_L,err_val_n_R] = ikine_dual(Slist_K, M_K, M_R, Slist_R, M_L, Slist_L, T_d_R, T_d_L, thetalist0, eomg, ev)
    global W T_B
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
    
    alpha = 1;
    while (err_L || err_R) && i < maxiterations
        J    = computeJacobian(kp, thetalist);
        Jinv = computeJacInv(W, J);
         
        % null space projection 
        null_space_proj = potential_field(thetalist, T_B, T_d_R, T_d_L, kp);
%         n_v             = (eye(21) - J'*Jinv')*null_space_proj';
% size(null(J))
        J(:,1:7)        = J(:,1:7) * W;
        n_v             = null(J) * null(J)' * null_space_proj';

        thetalist       = thetalist + alpha * Jinv * Vs + 1.0*n_v;
        
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
            % break;
            alpha = alpha * 0.9;
        end
        err_val_L = err_val_n_L; err_val_R = err_val_n_R;
        
        i = i + 1;
    end
end

function Jinv = computeJacInv(W, J)
    J(1:6,1:7) = W*J(1:6,1:7);
    J(7:end,1:7) = W*J(7:end,1:7);
    
    % Jinv = W\J'/(J/W*J');
    Jinv = J'/(J*J');
end

function J = computeJacobian(kp, q)
    theta_mini_L = q(8:14);
    theta_mini_R = q(15:end);
    theta_KUKA   = q(1:7);
    %     [g_K, ~, ~, Jst_R, Jst_L, J_K_s, ~, ~,~] = dual_arm_manipulator_kinematics(theta_mini_L, theta_mini_R, theta_KUKA);
    [g_K, ~, ~, Jst_R, Jst_L, J_K_s] = dual_arm_manipulator_kinematics_custom(kp, theta_mini_R, theta_mini_L, theta_KUKA);

    J = [J_K_s Adjoint(g_K)*Jst_L zeros(6,7);J_K_s zeros(6,7) Adjoint(g_K)*Jst_R];
end

%% null space projection
function dcdq = potential_field(q, T_B, T_d_R, T_d_L, kp)
    % q_lim_grad = exp((q - input.q_min')) + exp((q - input.q_max'));
    % d = d + finite_difference(f,q) * ones(7,1)*0.1;
    % cost = @(q)d - det(J(q_init)*J(q_init)');
    
    dcdq_1 = finite_difference(@obj_func, q);
    % dcdq_2 = q_lim_grad';
    
    dcdq   = dcdq_1; % + 0*dcdq_2;

    function J = finite_difference(fun, q)
        if nargin < 3
            h = 2^-17;
        end
        n       = numel(q);
        fun_0   = fun(q);
        x_d     = repmat(q,1,n) + 1*h*eye(n);
        J       = zeros(1,n);
        for j = 1:n
            J(j) = (fun(x_d(:,j)) - fun_0) / h;
        end
    end

    function cost = obj_func(q)
                
        theta_mini_R = q(15:21);
        theta_mini_L = q(8:14);
        theta_KUKA   = q(1:7);
        
        FK_K  = FKinSpace(kp{1}, kp{2}, theta_KUKA);
        Tsb_L = FK_K * FKinSpace(kp{5}, kp{6}, theta_mini_L);
        T_L = FKinSpace(kp{5}, kp{6}, theta_mini_L);
        Tsb_R = FK_K * FKinSpace(kp{3}, kp{4}, theta_mini_R);
        T_R = FKinSpace(kp{3}, kp{4}, theta_mini_R);
        
%         cost = sum((Tsb_L(1:3,end)-FK_K(1:3,end)-T_B{1}(1:3,end)).^2) + ...
%             sum((Tsb_R(1:3,end)-FK_K(1:3,end)-T_B{2}(1:3,end)).^2);
        cost = sum((T_L(1:3,end)-T_B{1}(1:3,end)).^2) + ...
            sum((T_R(1:3,end)-T_B{2}(1:3,end)).^2);
    end
end