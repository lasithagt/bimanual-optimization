function F = cost_function_dual(a)
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
    % q_min   = input.q_min; 
    % q_max   = input.q_max; 

    % Allocate array space
    theta   = zeros(input.n_arms * n_links, m);
    pa      = zeros(4,4,m,input.n_arms);       
    % err_pos = zeros(n_links,1,n_arms);
    f_val   = 0; % dex = 0;
    
    
    % update the input structure before feeding to IK
    input = update_input_struct(x, input);
    T_L = input.T_L;
    T_R = input.T_R;
    
    % solve for the init q using cons opt.
    T = zeros(4,4,2); T(:,:,1) = T_L; T(:,:,2) = T_R;
    %     w = {x(1:3),x(4:6),x(7:9)};
    %     q = {x(10:12),x(13:15),x(16:18)};
    
    for k = 1:n_arms
        robot                               = DH_robot(x, name, T(:,:,k));
        [Slist, M]                          = FK_DH(robot, );                                  
        [temp_t]                            = IK_INV_DH(robot, pd(:,:,:,k));
        theta(n_links*(k-1)+1:n_links*k,:)  = temp_t;
        pa(:,:,:,k)                         = FK_DH(robot, theta(n_links*(k-1)+1:n_links*k,:)); 
           
        V       = V_se3(pd,pa);
        err_rel = sum(sum((theta(:,1:end-1) - theta(:,2:end)).^2,2));
%         err_joint_limits = 
        %         dex     = dex(JacobianSpace(Slist, theta(n_links*(k-1)+1:n_links*k,:));
        f_val   = f_val + norm(V) + 0.1*err_rel;  
    end
       
    F = f_val;
    
end