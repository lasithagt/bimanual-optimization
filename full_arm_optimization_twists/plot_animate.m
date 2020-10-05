function plot_animate(x, input, data_file)
    close all;
    clf(gcf)
    
    % define twists
    w = {x(1:3),x(4:6),x(7:9)};
    for i =1:3
       w{i} = w{i} ./ norm(w{i});
    end
    
    q           = {x(10:12),x(13:15),x(16:18)};
    
    %% manipulator right
    temp = input.T_R;
    t = temp;
    t(1:3, 4) = [0 0 0]';
    [Slist_R, ~]  = manipulator_exp(w, q, t);
    Slist_R(:,8)  = 0 * Slist_R(:,7);

    DH_R_         = POE2DH(Slist_R);
    DH_R          = DH_R_(2:end-1,:);

    alpha_R  = DH_R(:,3);
    offset_R = DH_R(:,1);
    d_R      = DH_R(:,2);
    a_R      = DH_R(:,4);


    L1_R = Link('d', d_R(1), 'a', a_R(1), 'alpha', alpha_R(1),'offset', offset_R(1));        
    L2_R = Link('d', d_R(2), 'a', a_R(2), 'alpha', alpha_R(2),'offset', offset_R(2));
    L3_R = Link('d', d_R(3), 'a', a_R(3), 'alpha', alpha_R(3),'offset', offset_R(3));
    L4_R = Link('d', d_R(4), 'a', a_R(4), 'alpha', alpha_R(4),'offset', offset_R(4));
    L5_R = Link('d', d_R(5), 'a', a_R(5), 'alpha', alpha_R(5),'offset', offset_R(5));
    L6_R = Link('d', d_R(6), 'a', a_R(6), 'alpha', alpha_R(6),'offset', offset_R(6));
    L7_R = Link('d', 0 * d_R(7), 'a', 0 *a_R(7), 'alpha', 0 * alpha_R(7),'offset', 0 * offset_R(7));
    L8_R = Link('d', DH_R_(end,2), 'a', 0, 'alpha', 0,'offset', DH_R_(end,1));

    %% manipulator left
    temp = input.T_L;
    t    = temp;
    t(1:3, 4)      = [0 0 0]';
    [Slist_L, ~]   = manipulator_exp(w, q, t);

    Slist_L(4:6,8) = 0*Slist_L(4:6,7);

    % Slist_K(:,8)  = 0*Slist_K(:,7);

    % DH_L         = POE2DH(Slist_K);
    % POE_puma     = DH2POE(DH_L(2:end-1,:), eye(4),eye(4),'RRRRRRR','std');
    DH_L_        = POE2DH(Slist_L);
    DH_L         = DH_L_(2:end-1,:);

    alpha_L  = DH_L(:,3);
    offset_L = DH_L(:,1);
    d_L      = DH_L(:,2);
    a_L      = DH_L(:,4);


    L1_L = Link('d', d_L(1), 'a', a_L(1), 'alpha', alpha_L(1),'offset', offset_L(1));        
    L2_L = Link('d', d_L(2), 'a', a_L(2), 'alpha', alpha_L(2),'offset', offset_L(2));
    L3_L = Link('d', d_L(3), 'a', a_L(3), 'alpha', alpha_L(3),'offset', offset_L(3));
    L4_L = Link('d', d_L(4), 'a', a_L(4), 'alpha', alpha_L(4),'offset', offset_L(4));
    L5_L = Link('d', d_L(5), 'a', a_L(5), 'alpha', alpha_L(5),'offset', offset_L(5));
    L6_L = Link('d', d_L(6), 'a', a_L(6), 'alpha', alpha_L(6),'offset', offset_L(6));
    L7_L = Link('d', 0 * d_L(7), 'a', 0 * a_L(7), 'alpha', 0 * alpha_L(7),'offset', 0 * offset_L(7));
    L8_L = Link('d', DH_L_(end,2), 'a', 0, 'alpha',0,'offset', DH_L_(end,1));


    
    %% dual manipulator specifications
    m             = 500; % number of data points.
    input.m       = m;

    des_poses     = trajectory_pose_read(data_file, m);
    pd            = des_poses(:,:,:,:);
  
    
    L           = input.T_L; 
    L(1:3, 1:3) = eye(3);
    R           = input.T_R;
    R(1:3, 1:3) = rotz(pi);
    T_LR        = {L, R};
    
    t_tool_L = DH ([DH_L_(end,1:2),0,0], 'std');
    b_tool_L = DH ([DH_L_(1,:),0,0], 'std');

    t_tool_R = DH ([DH_R_(end,1:2),0,0], 'std');
    b_tool_R = DH ([DH_R_(1,:),0,0], 'std');
    
    mini_chain_L = SerialLink([L1_L L2_L L3_L L4_L L5_L L6_L L7_L L8_L], 'name', 'robot_L','base', L * b_tool_L);
    mini_chain_R = SerialLink([L1_R L2_R L3_R L4_R L5_R L6_R L7_R L8_R], 'name', 'robot_R','base', R * b_tool_R);

    theta   = zeros(input.n_arms * input.n_links, m);
    anim_pos = zeros(4, 4, m, input.n_arms);
        
    %% compute the fwk of the current using exponentials
    T = zeros(4,4,2); T(:,:,1) = input.T_L; T(:,:,2) = input.T_R;
    T(1:3,4,1) = [0 0 0]';
    T(1:3,4,2) = [0 0 0]';
    pa = zeros(4, 4, m, input.n_arms);
    
    for k = 1:input.n_arms
%         [Slist, ~]                          = manipulator_exp(w, q, T(:,:,k));
%         
%         Slist(:,8)                          = zeros(6, 1);
%         DH_L                                = POE2DH(Slist);
%         t_tool                              = DH ([DH_L(end,1:2),0,0], 'std');
%         b_tool                              = DH ([DH_L(1,:),0,0], 'std');
%         M                                   = fkDH(DH_L(2:end-1,:), b_tool, t_tool, zeros(7,1), 'RRRRRRR','std');
% 
%         [temp_t]                            = IK_SE3(Slist(:,1:end-1), M, T_LR{k}, pd(:,:,:,3-k), input.guess_init{k}); 
%         theta(input.n_links * (k - 1) + 1:input.n_links * k, :)  = temp_t;
%         % pa(:, :, :, 3 - k)                  = FK_SE3(Slist, M, T_LR{k}, real(theta(input.n_links * (k - 1)+1:input.n_links*k,:))); 
%         pa(:, :, :, k)                  = FK_SE3(Slist, M, T_LR{k}, real(theta(input.n_links * (k - 1)+1:input.n_links*k,:))); 
%     
%         
        [Slist, ~]                          = manipulator_exp(w, q, T(:,:,k));                                  
  
        Slist(:,8)                          = zeros(6, 1);
        DH_L                                = POE2DH(Slist);
        t_tool                              = DH ([DH_L(end,1:2),0,0], 'std');
        b_tool                              = DH ([DH_L(1,:),0,0], 'std');
        DH_L(end-1,:)                       = DH_L(end-1,:) * 0;
        M                                   = fkDH(DH_L(2:end-1,:), b_tool, t_tool, zeros(7,1), 'RRRRRRR','std');
        
        [temp_t]                            = IK_SE3(Slist(:,1:end-1), M, T_LR{k}, pd(:,:,:,k), input.guess_init{k});
        theta(input.n_links*(k-1)+1:input.n_links*k,:)  = temp_t;
        pa(:,:,:,k)                         = FK_SE3(Slist, M, T_LR{k}, theta(input.n_links*(k-1)+1:input.n_links*k,:)); 
    end
    
    close all
    figure(1)
    plot_trajectory_data(pa, 2, size(pa,3))
    figure(2)
    plot_trajectory_data(pd, 2, size(pd,3))
    
   V_se3(pd(:,:,:,1), pa(:,:,:,1))
    V_se3(pd(:,:,:,2), pa(:,:,:,2))
    
    %% Test if the visulizing model and the analytical model agrees
    for j = 1:m
       anim_pos(:,:,j, 1) = L * b_tool_L * mini_chain_L.A(1:7, theta(1:7,j)).T * t_tool_L;
       anim_pos(:,:,j, 2) = R * b_tool_R * mini_chain_R.A(1:7, theta(8:end,j)).T * t_tool_R;
    end 
    
    q_L = theta(1:7,:);
    q_R = theta(8:end,:);
    
    % for testing
%     q = [pi/4 pi/3 pi/5 0 0 0 0]';
%     q_L = q;
%     q_R = -q;
    
    %% Plot both robots together
    fig = figure(3);
    ws = 1.5 * [-20 20 -18 10 -10 20];
    myVideo = VideoWriter('myfile_test.avi');
    myVideo.FrameRate = 15;  % Default 30
    myVideo.Quality = 50;    % Default 75
    open(myVideo);
    q_ = {'r','b'};
    
%     m = 1;
    for i = 1:m
        mini_chain_R.plot([q_R(:,i);0]', 'noshadow','workspace',ws, 'view',[-165 45],'tile1color',[10 1 1],'delay',0.01,'jointdiam',1.3,'scale',0.7,'wrist','jointcolor',[0.5 0.5 0.5],'linkcolor',[0.7 0.0 0.0])
        hold on
        mini_chain_L.plot([q_L(:,i);0]', 'noshadow','workspace',ws, 'view',[-165 45],'tile1color',[10 1 1],'delay',0.01,'jointdiam',1.3,'scale',0.7,'wrist','jointcolor',[0.5 0.5 0.5],'linkcolor',[0.7 0.0 0.0])
        hold on
        %         quiver3(pa(1,end,i,2), pa(2,end,i,2), pa(3,end,i,2), ...
        %             R_2(1),R_2(2),R_2(3),q_{2}, 'Color', 'k', 'LineWidth',2,'MaxHeadSize',0.7, 'AutoScaleFactor', 1)
        %         
        plot3(pa(1,end,i,1), pa(2,end,i,1), pa(3,end,i,1),'k*');
        hold on 
        plot3(pa(1,end,i,2), pa(2,end,i,2), pa(3,end,i,2),'b*');
        hold on
        %         quiver3(pa(1,end,i,1), pa(2,end,i,1), pa(3,end,i,1), ...
        %             R_1(1),R_1(2),R_1(3),q_{1},  'Color', 'r', 'LineWidth',2,'MaxHeadSize',0.7, 'AutoScaleFactor', 1)
        axis(ws)
        writeVideo(myVideo, getframe(fig))    
    end
    close(myVideo);

end


