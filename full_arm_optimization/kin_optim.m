function [x, err_kin] = IK(initial_x, cons_l, cons_u)

    % IK This function numerically calculates the solution to the inverse
    % kinematics problem.
    %   Xi: column vector of DH parameters (d and a)
    %   A: column vector of alphas
    %   pd: desired EE point in the workspace

    global n_samples
    global n_dof
    % Constraints in Ax < b, 2 links
    A_ = eye(n_samples*n_dof + n_dof);
    A = [A_;-A_];
    b = [repmat(cons_u(1:end-n_dof)',n_samples,1);cons_u(end-n_dof+1:end)';-repmat(cons_l(1:end-n_dof)',n_samples,1);-cons_l(end-n_dof+1:end)'];    
    b
    % Number of links --> 3
    % initial_q = zeros(1,3);

    % Optimization routine to get IK
    options = optimoptions('fmincon','Algorithm','interior-point', 'Display','off');
    [x_optim, err_kin] = fmincon(@cost_function_kin, initial_x, A, b, [], [], [], [], [], options);

    x = x_optim;
end

