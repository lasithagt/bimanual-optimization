function plot_animate(x, input)
    
    close all;
    clf(gcf)
    
    % define the dual arm robot
    alpha = [pi/2 -pi/2 -pi/2 pi/2 pi/2 -pi/2 0];
    d     = [x(5:end) 0 0 0];
    a     = [0 0 0 0 0 0 0];

    L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1));        
    L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2));
    L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3));
    L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4));
    L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5));
    L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6));
    L7 = Link('d', d(7), 'a', a(7), 'alpha', alpha(7));
    
    m             = input.m;
    q_min         = input.q_min;
    q_max         = input.q_max;
    pose_d        = input.pd; 
    
    % Display the current manipulator.
    l = x(1);
    T_base_1 = SE3.Ry(pi/2);
    T_base_2 = SE3.Ry(-pi/2)*SE3.Rz(-pi);
    T_base_1.t = [l;0;0];
    T_base_2.t = [-l;0;0];


    mini_chain_1 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_R','base', T_base_1);
    mini_chain_2 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_L','base', T_base_2);
    
    theta   = zeros(input.n_arms * input.n_links, m);

    for j = 1:m
        
        % To maintain the continuity. Use the previous theta values.
        if (j > 1)
            initial_q   = theta(:,j-1);
        else
            initial_q   = theta(:,j);
        end

        temp        = IK(x, pose_d(:,:,j,:), initial_q, q_min, q_max, input);
        theta(:,m)  = reshape(temp',[],1);

    end 

    for i = 1:m        
        optim_arm_tool.vellipse([theta(:,i)'], '2d')
%         hold on
        optim_arm_tool.plot([theta(:,i)'], 'workspace', 4*[-0.3 1.5 -0.1 1 -1 1.5], 'noshadow','noarrow', 'view',[-90 90], 'tile1color',2*[0.5 0.5 0.5],'delay',0.1);
%         hold on
        plot(xx,yy,'r-')
    end

    % Plot the after and previous
    figure(1)
    % set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);
    % hold on
    for i = 1:size(q,1)
        mini_chain_1.plot(q(i,:), 'noshadow','workspace',[-1 1 -1 1 -1 1],'noarrow', 'view',[0 60],'tile1color',[10 1 1],'delay',0.0001)
        hold on
        % q(i,1) = -q(i,1);
        % q(i,3) = -q(i,3);
        % q(i,5) = -q(i,5);
        % q(i,7) = -q(i,7);
        q(i,:) = -q(i,:);
        mini_chain_2.plot(q(i,:), 'noshadow','workspace',[-1 1 -1 1 -1 1],'noarrow', 'view',[0 60],'tile1color',[10 1 1],'delay',0.0001)
    end

end


