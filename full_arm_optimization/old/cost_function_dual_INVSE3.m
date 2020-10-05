function F = cost_function_dual_INVSE3(x)
    
    % OPTIMIZATIONROUTINE This function evaluates the overall objective function (F) for a
    % specific set of DH parameters
    global rb
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
    err_pos = zeros(n_links,1,n_arms);
    f_val       = 0; dex = 0;
    
    
    % update the input structure before feeding to IK
    input = update_input_struct(x, input);
    T_L = input.T_L;
    T_R = input.T_R;
    
    % solve for the init q using cons opt.
    T = zeros(4,4,2); T(:,:,1) = T_L; T(:,:,2) = T_R;
    for k = 1:n_arms
        rb                                  = FK_exp(x(5:end), T(:,:,k));
        [temp_t]                            = IK_SE3(rb, pd(:,:,:,k));
        theta(n_links*(k-1)+1:n_links*k,:)  = temp_t;
        pa(:,:,:,k)                         = fkine(rb, theta(n_links*(k-1)+1:n_links*k,:));
         

        % Error on Position and Orientation 

        %         for i = 1:m
        %             F = F + twistcoords(twistlog(fkine(rb, temp_t(:,i)))) - twistcoords(twistlog(pd(:,:,i,k)));
        %         end
        % 
        %         sum((pd(1:3,end,:,k) - pa(1:3,end,:,k)).^2,3)

        err_rel         = sum(sum((theta(:,1:end-1) - theta(:,2:end)).^2,2));
        for i = 1:m
            dex         = dex + det(sjacob(rb,theta(1:7,i))*sjacob(rb,theta(1:7,i))');
        end
        
        err_pos(:,:,k)  = [sum((pd(1:3,end,:,k) - pa(1:3,end,:,k)).^2, 3);...
            sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,:,k))), rotm2quat(pa(1:3,1:3,:,k)))).^2)'];
        
        f_val           = f_val + sum(sum(err_pos(:,:),1)) + 0.1*err_rel;
    end
       
    F = f_val;
    
end
