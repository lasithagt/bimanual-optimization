function manifold_optimization(x, theta, desired_poses, limits)

    % theta -> nx7
    % n_waypoints -> number of points to compute
    n_waypoints = size(theta, 2);
    
    %% Get the partials from implicit constraints
    [FK_u_full,FK_u_stack, ~, ~] = FK_partials(x, theta, n_waypoints, false);

    %% Matrix Computation (formulation as a quadratic problem)
    % Matrix with partials
    A_fwk = FK_u_full;
    W     = blkdiag(eye(n_waypoints*size(theta, 1)), 1*eye(size(x, 1)));
    
    des_stack  = desired_poses(:);
    xqi_stack  = [theta(:); x];
    
    % define the dual arm robot
    alpha  = [pi/2 -pi/2 -pi/2 0 pi/2 -pi/2 0];
    offset = [0,0,0,-pi/2,0,-pi/2,0];
    
    while (1)
        
        d      = [x(1) x(2) x(3) 0 0 0 0];
        a      = [0 0 0 x(4) 0 0 0];
        % tool   = input.tool;

        L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
        L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
        L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
        L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
        L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
        L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
        L7 = Link('d', 0, 'a',    0, 'alpha', alpha(7),'offset', offset(7));

        optim_arm = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'manipulator');
        
        % clf(gcf)
        for i=1:n_waypoints
            optim_arm.plot(theta(:,i)','workspace', 10*[-0.3 1.5 -1 1 -1 1.5], 'noshadow','noarrow', 'view',[-136 24], 'tile1color',[0.9 0.9 0.9],'delay',0.01);
            hold on
        end
        
        Jinv = W\A_fwk'/(A_fwk/W*A_fwk');
        
        current_poses = zeros(6, size(theta,2));
        for i = 1:size(theta, 2)
            current_poses(:,i) = FK_analytical([theta(:,i);x]);
        end
        
        err  = -(current_poses(:) - des_stack);
        
        % redundancy resolution
        null_space_proj = redundancy_resolution(FK_u_stack, xqi_stack, limits);
        dx   = Jinv * 1 * err + 0.001 * null(A_fwk) * null(A_fwk)' * null_space_proj;
        
        xqi_stack   = xqi_stack + dx;
        x           = xqi_stack(end-3:end);
        theta       = xqi_stack(1:end-4);
        theta       = reshape(theta,[], n_waypoints);
        
        [FK_u_full, FK_u_stack, ~, ~] = FK_partials(x, theta, n_waypoints, false);
        A_fwk = FK_u_full;
        

    end
    
    function dcdq = redundancy_resolution(jac_stack, theta, limits)
       
        % dexterity
        dex_grad = dexterity_grad(jac_stack, theta);
        
        % joint limits
        q_lim_grad = exp((theta - limits(:,1))) + exp((theta - limits(:,2)));
        % d = d + finite_difference(f,q) * ones(7,1)*0.1;
        % cost = @(q)d - det(J(q_init)*J(q_init)');

        dcdq_1 = finite_difference(@obj_func, theta);
        dcdq_2 = q_lim_grad;

        dcdq   = dcdq_2; % + 0*dcdq_2;

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
            cost = 0;
            
        end
        
        
        function dex = dexterity_grad(jac_stack, theta)
            for k = 1:n_waypoints
                grad = det(jac_stack{k}'*jac_stack{k}) * (jac_stack)
            end
        end
    end
    
 
end