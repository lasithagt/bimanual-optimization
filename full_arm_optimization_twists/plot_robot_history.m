function plot_robot_history(history,input,data_file)
    close all;
    m = 1;
    des_poses     = trajectory_pose_read(data_file, m);
    figure(1);
    for i = 1:20:size(history, 1)
        i = 103
        x = history(i,:);
        
        w = {x(1:3),x(4:6),x(7:9)};
        for k =1:3
           w{k} = w{k} ./ norm(w{k});
        end

        q           = {x(10:12),x(13:15),x(16:18)};
        [Slist, ~]  = manipulator_exp(w, q, eye(4));
        Slist(:,8)  = Slist(:,7);

        DH_         = POE2DH(Slist);
        POE_puma    = DH2POE(DH_(2:end,:), eye(4),eye(4),'RRRRRRRR','std');

        DH          = DH_(2:end-1,:);

        alpha  = DH(:,3);
        offset = DH(:,1);
        d      = DH(:,2);
        a      = DH(:,4);

        tool   = 0;

        % clear mini_chain_1 L1 L2 L3 L4 L5 L6 L7
        L1 = Link('d', d(1), 'a', a(1), 'alpha', alpha(1),'offset', offset(1));        
        L2 = Link('d', d(2), 'a', a(2), 'alpha', alpha(2),'offset', offset(2));
        L3 = Link('d', d(3), 'a', a(3), 'alpha', alpha(3),'offset', offset(3));
        L4 = Link('d', d(4), 'a', a(4), 'alpha', alpha(4),'offset', offset(4));
        L5 = Link('d', d(5), 'a', a(5), 'alpha', alpha(5),'offset', offset(5));
        L6 = Link('d', d(6), 'a', a(6), 'alpha', alpha(6),'offset', offset(6));
        L7 = Link('d', 0*tool, 'a', a(7)*0, 'alpha', alpha(7)*0,'offset', offset(7)*0);
        
        TL = input.T_L;
        TR = input.T_R;
        
        TR(1,end) = -x(end-2);
        TL(1,end) = x(end-2);
        
        TL(2:3,end) = x(end-1:end);
        TR(2:3,end) = x(end-1:end);
        
        T_base_1 = SE3(TL);
        T_base_2 = SE3(TR);

        r_name = strcat('robot_R_',num2str(i));
        l_name = strcat('robot_L_',num2str(i));
        
        mini_chain_1 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', r_name,'base', T_base_1);
        mini_chain_2 = SerialLink([L1 L2 L3 L4 L5 L6 L7], 'name', l_name,'base', T_base_2);

        
        input.m       = 1;
        q_min         = input.q_min;
        q_max         = input.q_max;
        
        pd            = des_poses(:,:,:,:);

        theta   = zeros(input.n_arms * input.n_links, m);

        T = zeros(4,4,2); T(:,:,1) = input.T_L; T(:,:,2) = input.T_R;
        for k = 1:2
            [Slist, M]                          = manipulator_exp(w, q, T(:,:,k));   

            [temp_t]                            = IK_SE3(Slist, M, pd(:,:,:,3-k)); 
            theta(input.n_links*(k-1)+1:input.n_links*k,:)  = temp_t;
            pa(:,:,:,3-k)                       = FK_SE3(M, Slist, theta(input.n_links*(k-1)+1:input.n_links*k,:)); 

        end
        
        ws = 1.0*[-20 20 -18 10 -10 25];
        
        q_L = theta(1:7,:);
        q_R = theta(8:end,:);
        clf(gcf)
        
%         q_R(1,1) = -pi/4;

        mini_chain_2.plot(q_R(:,1)', 'noshadow','workspace',ws, 'view',[-165 45],'tile1color',[10 1 1],'delay',0.01,'jointdiam',1.3,'scale',0.6,'wrist','jointcolor',[0.5 0.5 0.5],'linkcolor',[0.7 0.0 0.0])
        hold on
        q_R(3,1) = q_R(3,1);
        p = -q_L(:,1)';
        p(2,1) = pi/4;
        mini_chain_1.plot(q_L(:,1)', 'noshadow','workspace',ws, 'view',[-165 45],'tile1color',[10 1 1],'delay',0.01,'jointdiam',1.3,'scale',0.6,'wrist','jointcolor',[0.5 0.5 0.5],'linkcolor',[0.7 0.0 0.0])
        
        grid off
        
    end
end