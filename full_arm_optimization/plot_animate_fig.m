function plot_animate(x, input, data_file)
    
    close all;
    clf(gcf)
    
    % define the dual arm robot
    alpha  = [pi/2 -pi/2 -pi/2 0 pi/2 -pi/2 0];
    offset = [0,0,0,-pi/2,0,-pi/2,0];
    d      = [0*x(5) 0*x(8) x(7) 0 0 0];
    a      = [0 0 0 x(8) 0 0 0];
    tool   = input.tool;

    L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
    L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
    L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
    L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
    L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
    L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
    L7 = Link('d', tool, 'a',    0, 'alpha', alpha(7),'offset', offset(7));
    
    m             = 120;
    q_min         = input.q_min;
    q_max         = input.q_max;
    des_poses     = trajectory_pose_read(data_file, m);

    pd  = des_poses(:,:,:,:);

%     m   = 120;
%     pd(1:3,1:3,1,1) = rotx(pi)*pd(1:3,1:3,1,1);
%     pd(1:3,1:3,1,2) = rotx(pi)*pd(1:3,1:3,1,2);
    % Display the current manipulator.

    T_base_1 = SE3(input.T_L);
    T_base_2 = SE3(input.T_R);

    mini_chain_1 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_R','base', T_base_1);
    mini_chain_2 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', 'robot_L','base', T_base_2);
    
    theta   = zeros(input.n_arms * input.n_links, m);

    anim_pos = [];
        

    
%     err_v = [];
    T = zeros(4,4,2); T(:,:,1) = input.T_L; T(:,:,2) = input.T_R;
    for k = 1:input.n_arms
        rb                                  = FK_exp(x(5:end), T(:,:,k));
        [temp_t]                            = IK_SE3(rb, pd(:,:,:,k));
        theta(input.n_links*(k-1)+1:input.n_links*k,:)  = temp_t;
        pa(:,:,:,k)                         = fkine(rb, theta(input.n_links*(k-1)+1:input.n_links*k,:));
    end
    
    for j = 1:m
%        [temp_t, err] =  IK_invJac(x, temp_t, pd(:,:,1+j,:), vel_d);
              
%        err_v(j) = err;
%        theta(:,j+1)  =  reshape(temp_t',[],1);
%        pa(:,:,j+1,:) =  FK(x, temp_t);
%        anim_pos(:,:,j+1,1) = input.T_L*mini_chain_1.A(1:7,temp_t(1,:)).T;
%        anim_pos(:,:,j+1,2) = input.T_R*mini_chain_2.A(1:7,temp_t(2,:)).T;
       anim_pos(:,:,j,1) = input.T_L*mini_chain_1.A(1:7,theta(1:7,j)).T;
       anim_pos(:,:,j,2) = input.T_R*mini_chain_2.A(1:7,theta(8:end,j)).T;
    end 
    
    
    
    q_L = theta(1:7,:);
    q_R = theta(8:end,:);
    
    % Plot both robots together
    fig = figure(1);
    ws = [-8 8 -18 5 -4 7];
    myVideo = VideoWriter('myfile.avi');
    myVideo.FrameRate = 15;  % Default 30
    myVideo.Quality = 50;    % Default 75
    open(myVideo);
    for i = 1:m
        %  optim_arm_tool.vellipse([theta(:,i)'], '2d')
        mini_chain_2.plot(q_R(:,i)', 'noshadow','workspace',ws,'noarrow', 'view',[-66 70],'tile1color',[10 1 1],'delay',0.01,'jointdiam',1)
        hold on
        plot3(pa(1,end,i,2), pa(2,end,i,2), pa(3,end,i,2),'b.-');
        hold on
        
        mini_chain_1.plot(q_L(:,i)', 'noshadow','workspace',ws,'noarrow', 'view',[-66 70],'tile1color',[10 1 1],'delay',0.01,'jointdiam',1)
 
        hold on
        plot3(pa(1,end,i,1), pa(2,end,i,1), pa(3,end,i,1),'k.-');
        
        writeVideo(myVideo, getframe(fig))    
    end
    close(myVideo);

end


