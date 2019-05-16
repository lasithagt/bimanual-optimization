function plot_animate(a, input)
    
    close all;
    clf(gcf)
    L1 = Link('d', 0, 'a', a(1), 'alpha', 0);        
    L2 = Link('d', 0, 'a', a(2), 'alpha', 0);
    L3 = Link('d', 0, 'a', a(3), 'alpha', 0);
    L4 = Link('d', 0, 'a', a(4), 'alpha', 0);
    L5 = Link('d', 0, 'a', 0.5, 'alpha', 0);
    
    % Plot many cofigurations evolutions
    optim_arm      = SerialLink([L1 L2 L3 L4]);
    optim_arm_tool = SerialLink([L1 L2 L3 L4 L5]);
    m              = 100; %input.m;
    cons_l         = (2/2).*[-pi/2,-pi/2, -pi/2,-pi/2,-pi/4];
    cons_u         = (2/2).*[pi/2,pi/2, pi/2,pi/2,pi/4];

    xx = linspace(1.1, 2.0, m);
    init_q = [pi/3 pi/3 -pi/3 -pi/3 -pi/3];
    for i = 1:m
        [yy(i), norm]  = arc_traj(xx(i));
        rotm        = axang2rotm([0 0 1 atan2(norm(2),norm(1))]);
        p_          = [xx(i) yy(i) 0];
        p           = [rotm p_';0 0 0 1];
        if (i==1)
            init_q = init_q;
        else
            init_q = theta(:,i-1)';
        end
        theta(:,i)  = IK_tool([a;0.5], p,init_q, cons_l, cons_u);
    end

    for i = 1:m        
        optim_arm_tool.vellipse([theta(:,i)'], '2d')
%         hold on
        optim_arm_tool.plot([theta(:,i)'], 'workspace', 4*[-0.3 1.5 -0.1 1 -1 1.5], 'noshadow','noarrow', 'view',[-90 90], 'tile1color',2*[0.5 0.5 0.5],'delay',0.1);
%         hold on
        plot(xx,yy,'r-')
    end

end


