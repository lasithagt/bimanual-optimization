function theta = IK(a, pose_des, initial_q, cons_l, cons_u)

    % IK This function numerically calculates the solution to the inverse
    % kinematics problem.
    %   Xi: column vector of DH parameters (d and a)
    %   A: column vector of alphas
    %   pd: desired EE point in the workspace
    
    % Constraints in Ax < b
    A = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1;-1 0 0 0;0 -1 0 0;0 0 -1 0;0 0 0 -1];
    b = [cons_u(1);cons_u(2);cons_u(3);cons_u(4);-cons_l(1);-cons_l(2);-cons_l(3);-cons_l(4)];
    
    % Number of links --> 3
    % initial_q = zeros(1,3);

    % Optimization routine to get IK
    options = optimoptions('fmincon','Algorithm','sqp', 'Display','off');
    [theta_optim, f] = fmincon(@IK_cost, initial_q, A, b, [], [], [], [], [], options);
    theta = theta_optim;
    function cost = IK_cost(theta)
        lq = [theta' a];
        temp_   = FK(lq);
                
        cost    = sum((temp_(1:3,end) - pose_des(1:3,end)).^2) + ...
            sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pose_des(1:3,1:3))), rotm2quat(temp_(1:3,1:3)))).^2);
    end
end

