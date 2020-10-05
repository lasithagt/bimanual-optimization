function F = cost_function_dual_INVSE3(x)
    
    % OPTIMIZATION ROUTINE This function evaluates the overall objective function (F) for a
    % specific set of DH parameters
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
    w = {x(1:3),x(4:6),x(7:9)};
    
    for i=1:3
        w{i} = w{i} ./ norm(w{i});
    end
    q = {x(10:12), x(13:15), x(16:18)};
    
    % Transformations
    L           = input.T_L; 
    L(1:3, 1:3) = eye(3);
    R           = input.T_R;
    R(1:3, 1:3) = rotz(pi);
    T_LR        = {L, R};
    
    T(1:3,4,1) = [0 0 0]';
    T(1:3,4,2) = [0 0 0]';
      
    for k = 1:n_arms
        [Slist, ~]                          = manipulator_exp(w, q, T(:,:,k));                                  
  
        Slist(:,8)                          = zeros(6, 1);
        DH_L                                = POE2DH(Slist);
        t_tool                              = DH ([DH_L(end,1:2),0,0], 'std');
        b_tool                              = DH ([DH_L(1,:),0,0], 'std');
        DH_L(end-1,:)                       = DH_L(end-1,:) * 0;
        M                                   = fkDH(DH_L(2:end-1,:), b_tool, t_tool, zeros(7,1), 'RRRRRRR','std');
        
        [temp_t]                            = IK_SE3(Slist(:,1:end-1), M, T_LR{k}, pd(:,:,:,k), input.guess_init{k});
        theta(n_links*(k-1)+1:n_links*k,:)  = temp_t;
        pa(:,:,:,k)                         = FK_SE3(Slist, M, T_LR{k}, theta(n_links*(k-1)+1:n_links*k,:)); 
              
        V                                   = V_se3(pd, pa);
        err_rel                             = sum(sum((theta(:,1:end-1) - theta(:,2:end)).^2,2));
        f_val                               = f_val + norm(V) + 0.1 * err_rel;  
        
%         for j = 1:m
%             T_E = T_LR{k} * fkDH(DH_L(2:4,:), b_tool, t_tool, , 'RRR','std');
%             if abs(T_E(1,end)) < abs(T_LR{k}(1,end))
%                 f_val = f_val + norm(abs(T_E(1,end)) - abs(T_LR{k}(1,end)));
%             end
%         end
    end
       
    F = f_val;
end
