
function [x,fval] = sqp(func, h_func, g_func, cost_function, initial_x, grad_F)
    
    %% Function and constrint functions

    % g_func = @(x)x(1);
    n_var = numel(initial_x);

    %%
    % Initial starting points
    x = initial_x;

    % Initial numerical hessian and gradient of the objective function
    % gf = gradient_(func, 1,2,initial_x);
    if (nargin > 5)
        [gf, fk_uu] = grad_F(x);
    else
        gf = gradient_(func, 1,2,initial_x);
    end
    % hf = hessian_(g_func,1,2,initial_x);

    lambda_g  = zeros();
    lambda_h  = zeros();
    H = eye(n_var); % initial Hessian of the func
    L_tilda_l = @(x,g,h) func(x) + g' * g_func(x) + h' * h_func(x);
    L_tilda   = @(x)L_tilda_l(x,lambda_g,lambda_h);
    
    % Use the numerical gradient for the lagrange problem
    L_delta   = gradient_(L_tilda, 1,2,x);
    p = 1;
    i = 0;
    x_v(:,1) = x;
    
    while (sum(L_delta.^2) > 0.0001 && (sum(p.^2) > 1e-6))
        i = i + 1;

        % Numerical gradient of constraints.
        [gf, fk_uu] = FK_u(x);
         % gf = gradient_(func, 1,2,x);
        gc = gradient_(g_func, 1,2,x);

        hc = []; h = [];

        % Compute p such that variables safisfy to be on the manifold delta z =
        % [f_z]^(-1) delta y
        p = sqp_subproblem(func, g_func, x, lambda, gf, H, gc, g_func(x), hc, h);

        % compute graient of lagrangian before update

        L_tilda   = @(x)L_tilda_l(x,lambda);
        g_lag_p   = gradient_(L_tilda, 1, 2, x);

        % compute lambda i+1
        lambda = gc \ (-gf-hf*p);

        % Increment x
        x = x + p;
        x_v(:,i+1) = x;
        % compute graient of lagrangian after update
        L_tilda = @(x)L_tilda_l(x, lambda_g, lambda_h);
        
        g_lag_n   = gradient_(L_tilda, 1, 2, x);

        % compute the new hessian
        % compute Q
        Q = g_lag_n - g_lag_p;

        if (p'*Q >= 0.2*p'*H*p)
            theta = 1;
        else
            theta = (0.8*p'*H*p) / (p'*H*p - p'*Q);
        end

        gamma = theta*Q + (1-theta)*H*p;
        H = H - (H*(p*p')*H)/(p'*H*p) + (gamma*gamma')/(p'*p);

    %     lambda_ = lambda;
    %     lambda_(lambda_<=0) = 0;
    %     
        L_tilda = @(x)L_tilda_l(x,lambda);
        L_delta  = gradient_(L_tilda, 1,2,x);

    end
    x = x(:,end)
    x_arr{1} = x_v;
    toc

    %% Solve the sqp subproblem. (To find p)
    function p = sqp_subproblem(func, g_func, x_, lambda, grad_f, hess_f, grad_g, g, grad_h, h)
        % Solve for p using interior point penalty function in matlab. Try to
        % do without matlab fmincon
        p = fmincon(@(p)grad_f'*p + (1/2)*p'*hess_f*p, ones(2,1), grad_g', -g, grad_h, h);

        a = fminunc(@(a)func(x_+a*p) + max(0,g_func(x_+a*p)), 0);
        p = a*p;
    end

end