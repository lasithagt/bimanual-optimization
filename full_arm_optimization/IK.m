% Solves inverse kinematics of the whole dual arm
function [theta,f] = IK(a, pose_des, initial_q)

    % IK This function numerically calculates the solution to the inverse
    % kinematics problem.
    %   Xi: column vector of DH parameters (d and a)
    %   A: column vector of alphas
    %   pd: desired EE point in the workspace
    
    % Constraints in Ax < b
    global input
    
    cons_l = input.q_min; cons_u = input.q_max; 
    
    A = [eye(input.n_arms * input.n_links); -eye(input.n_arms * input.n_links)];
    b = [cons_u(1);cons_u(2);cons_u(3);cons_u(4);cons_u(5);cons_u(6);cons_u(7);...
        cons_u(1);cons_u(2);cons_u(3);cons_u(4);cons_u(5);cons_u(6);cons_u(7);...
        -cons_l(1);-cons_l(2);-cons_l(3);-cons_l(4);-cons_l(5);-cons_l(6);-cons_l(7);...
        -cons_l(1);-cons_l(2);-cons_l(3);-cons_l(4);-cons_l(5);-cons_l(6);-cons_l(7)];
    
    A_ = [eye(input.n_links); -eye(input.n_links)];
    % TODO: Matlab Robotics Systems toolbox to get the ik solver.
    % Optimization routine to get IK
    options     = optimoptions('fmincon','Algorithm','interior-point', 'Display','off');
    options_sa  = optimoptions('simulannealbnd','Display','off','TemperatureFcn','temperatureboltz','InitialTemperature',200,'MaxIterations',2000);
    f = 0;
    
    for k = 1:input.n_arms
        IK_cost = @(Q)IK_cost_(Q,k);
        % [temp, ~] = simulannealbnd(IK_cost,initial_q(7*(k-1) + 1:7*k),cons_l,cons_u,options_sa);
        [temp, f_] = fmincon(IK_cost, initial_q(7*(k-1) + 1:7*k), A_, [b(1:7) b(15:21)], [], [], [], [], [], options);
        f = f + f_;
        theta_optim(7*(k-1) + 1:7*k) = temp;
    end
    
    theta = theta_optim;
    
    theta = reshape(theta,[],input.n_arms);
    
    function cost = IK_cost_(theta,i)
        if (i==1)
            theta = [theta zeros(1,7)];
        else
            theta = [zeros(1,7) theta];
        end
        temp_   = FK(a, reshape(theta,[],input.n_arms)');
        cost = sum((temp_(1:3,end,i) - pose_des(1:3,end,i)).^2) + ...
            sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pose_des(1:3,1:3,i))), rotm2quat(temp_(1:3,1:3,i)))).^2);
    end
end

