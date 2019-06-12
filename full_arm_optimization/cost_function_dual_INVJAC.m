function F = cost_function_dual_INVJAC(x)
    
    % OPTIMIZATIONROUTINE This function evaluates the overall objective function (F) for a
    % specific set of DH parameters

    global theta
    global input
    % Read the parameters from the input structure
    
    m       = input.m;       % Number of samples
    n_links = input.n_links; % Number of links
    pd      = input.pd;
    n_arms  = input.n_arms;  % Number of arms
    
    % Constraints for theta
%     q_min   = input.q_min; 
%     q_max   = input.q_max; 

    % Allocate array space
    theta   = zeros(input.n_arms * n_links, m);
    pa      = zeros(4,4,m,input.n_arms);       
    err_pos = zeros(1,m);
    f_val   = 0;
    
    % update the input structure before feeding to IK
    update_input_struct(x);
    
    % parameters for INV JAC solver
    vel_d = [0 0 0 0 0 0]';
    
    % solve for the init q using cons opt.
    initial_q   = [0 0 0 0 0 0 0 0 0 0 0 0 0 0];
    temp_init   = IK(x, pd(:,:,1,:), initial_q);
    theta(:,1)  = reshape(temp_init,[],1);
    pa(:,:,1,:) = FK(x, temp_init');
    temp_t      = temp_init';
    
    for j = 1:m-1
       temp_t =  IK_invJac(x, temp_t, pd(:,:,1+j,:), vel_d);

       theta(:,j+1)  = reshape(temp_t',[],1);
       pa(:,:,j,:)   = FK(x, temp_t);
        
        for k = 1:n_arms
            % Error on Position and Orientation 
            err_pos(:,j,k)  = sum((pd(1:3,end,j,k) - pa(1:3,end,j,k)).^2) + ...
                            sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,j,k))), rotm2quat(pa(1:3,1:3,j,k)))).^2);
            f_val           = f_val + err_pos(:,j);
        end
       
    end 
    F = f_val; 
    
end
