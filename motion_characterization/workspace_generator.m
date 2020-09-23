function workspace_generator(data)
    n_ = 100;

    r_x = rand(1,n_)+1;
    r_y = rand(1,n_)+1;
    r_z = rand(1,n_)+1;
    t   = linspace(0,2*pi,100);

    close all


    tool_length = 0.5;
    c           = [1.5 1.5 1.5]';
    v           = [2,2,2]';
    vec_cv      = v - c;

    n           = null(vec_cv'./norm(vec_cv));
    cone_vec    = 2*n(:,1)*cos(t) + 2*n(:,2)*sin(t) + 2*c;

    figure(1)
    plot3(r_x,r_y,r_z,'k*')
    hold on
    plot3(cone_vec(1,:), cone_vec(2,:), cone_vec(3,:),'k*')
    hold on
    quiver3(ones(1,n_)*1.5, ones(1,n_)*1.5, ones(1,n_)*1.5, cone_vec(1,:)-1.5, cone_vec(2,:)-1.5, cone_vec(3,:)-1.5,1.5)
    hold on

    
    xyz_data = [];
    for j = 1:n_
        v_n = null([v(1)-r_x(j) v(2)-r_y(j) v(3)-r_z(j)]);
        v_n(:,1) = v_n(:,1)./norm(v_n(:,1));
        v_n(:,2) = v_n(:,2)./norm(v_n(:,2));
        xyz = generate_local_cloud(v_n, 200,4);

        v = [v(1)-r_x(j) v(2)-r_y(j) v(3)-r_z(j)];
        v = v./norm(v);
        %     [cross([0 0 1],v) acos(dot([0 0 1],v))]
        R     = axang2rotm([cross([0 0 1],v) real(acos(dot([0 0 1],v)))]);
        xyz_R = R * xyz;
        
%         d_o_rotm  = eul2rotm(d_o','ZYX');
%         quat_vec  = rotm2quat(d_o_rotm);
%         quat_mean = mean(quat_vec,1);

        xyz_data = [xyz_data xyz_R];
        plot3(r_x(j)+xyz_R(1,:),r_y(j)+xyz_R(2,:),r_z(j)+xyz_R(3,:),'r.')

        hold on
    end
    % k = boundary(xyz_data(1,:)', xyz_data(2,:)', xyz_data(3,:)');
    % plot3(xyz_data(1,k), xyz_data(2,k), xyz_data(3,k));

    axis([-5 5 -5 5 -5 5])
    % axis equal

    % end
    % axis([-5 5 -5 5 -5 5])
    % figure(2)


    for i=1:n
        R(:,:,i) = axang2rotm([cross(c,b) acos(dot(a,b))]);
    end
    
    quat_vec  = rotm2quat(R);
    quat_mean = mean(quat_vec,1);
    R_m       = quat2rotm(quat_mean);
    
    % plot3(xyz_data(1,k), xyz_data(2,k), xyz_data(3,k));
    
end

function xy = generate_local_cloud(vec, n,rd)
    for i = 1:n
        tr = 4*pi*(rand(1)-0.5);
        r  = rd*rand(1);
%         xy(:,i) = r*vec(:,1)*cos(tr) - r*vec(:,2)*sin(tr);
        xy(:,i) = r*[1;0;0]*cos(tr) + r*[0;1;0]*sin(tr);
        xy(3,i) = -sqrt(rd.^2-sum(xy(1:2,i).^2));
    end

end