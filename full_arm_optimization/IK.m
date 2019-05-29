% Solves inverse kinematics of the whole dual arm
function [theta,f] = IK(a, pose_des, initial_q, cons_l, cons_u, input)

    % IK This function numerically calculates the solution to the inverse
    % kinematics problem.
    %   Xi: column vector of DH parameters (d and a)
    %   A: column vector of alphas
    %   pd: desired EE point in the workspace
    
    % Constraints in Ax < b
    A = [eye(input.n_arms * input.n_links); -eye(input.n_arms * input.n_links)];
    b = [cons_u(1);cons_u(2);cons_u(3);cons_u(4);cons_u(5);cons_u(6);cons_u(7);...
        cons_u(1);cons_u(2);cons_u(3);cons_u(4);cons_u(5);cons_u(6);cons_u(7);...
        -cons_l(1);-cons_l(2);-cons_l(3);-cons_l(4);-cons_l(5);-cons_l(6);-cons_l(7);...
        -cons_l(1);-cons_l(2);-cons_l(3);-cons_l(4);-cons_l(5);-cons_l(6);-cons_l(7)];
    

    % TODO: Matlab Robotics Systems toolbox to get the ik solver.
    % Optimization routine to get IK
    options          = optimoptions('fmincon','Algorithm','interior-point', 'Display','off');
    [theta_optim, f] = fmincon(@IK_cost, initial_q, A, b, [], [], [], [], [], options);
    theta            = theta_optim;
    
    theta = reshape(theta,input.n_arms,[]);
    function cost = IK_cost(theta)
        temp_   = FK(a, reshape(theta,input.n_arms,[]));
        cost = 0;
        for i = 1:input.n_arms
            cost = cost + sum((temp_(1:3,end,i) - pose_des(1:3,end,i)).^2) + ...
                sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pose_des(1:3,1:3,i))), rotm2quat(temp_(1:3,1:3,i)))).^2);
        end
    end
end

