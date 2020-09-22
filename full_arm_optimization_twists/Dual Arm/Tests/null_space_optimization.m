%% null space projection. H function
function dcdq = null_space_optimization(q, T_B, kp)
    q_lim_grad = -(q - zeros(21,1))/(2*pi);
    % d = d + finite_difference(f,q) * ones(7,1)*0.1;
    % cost = @(q)d - det(J(q_init)*J(q_init)');
    
    dcdq_1 = -finite_difference(@obj_func, q);
    dcdq_2 = q_lim_grad';
    dcdq   = 0.05 * dcdq_1 + 0.1 * dcdq_2;
    % dcdq   = dcdq_1; % + 0*dcdq_2;

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
        theta_mini_L = q(8:14);        
        theta_mini_R = q(15:21);
        theta_KUKA   = q(1:7);
        
        FK_K  = FKinSpace(kp{1}, kp{2}, theta_KUKA);
        % Tsb_L = FK_K * FKinSpace(kp{5}, kp{6}, theta_mini_L);
        % Tsb_R = FK_K * FKinSpace(kp{3}, kp{4}, theta_mini_R);
        
        % T_L   = FKinSpace(kp{5}, kp{6}, theta_mini_L);
        % T_R   = FKinSpace(kp{3}, kp{4}, theta_mini_R);
        
        % cost = sum((Tsb_L(1:3,end)-FK_K(1:3,end)-T_B{1}(1:3,end)).^2) + ...
        %             sum((Tsb_R(1:3,end)-FK_K(1:3,end)-T_B{2}(1:3,end)).^2);
        % cost = sum((T_L(1:3,end)-T_B{1}(1:3,end)).^2) + sum((T_R(1:3,end)-T_B{2}(1:3,end)).^2);
        cost = norm(FK_K(1:3,1:3) - [1 0 0;0 -1 0;0 0 -1]);
    end
end