function  [x]=tool_tip_approx(data)
    close all

    data_xyz = data(:,1:3);
    data_ang = data(:,4:6);
    rot_mat  = @(a,e,r)[cos(e)*cos(a) cos(e)*sin(a) -sin(e);-cos(r)*sin(a)+sin(r)*sin(e)*cos(a) cos(r)*cos(a)+sin(r)*sin(e)*sin(a) sin(r)*cos(e);sin(r)*sin(a)+cos(r)*sin(e)*cos(a) -sin(r)*cos(a)+cos(r)*sin(e)*sin(a) cos(r)*cos(e)]';
    
    num_points = size(data_xyz,1);   
    
    % Plot calibration data for visualization. 
    figure(1)
    plot3(data_xyz(:,1),data_xyz(:,2),data_xyz(:,3))
   
    options = optimset('MaxFunEvals',10000,'Display','iter');
    function f_out = objectivefcn(x)
        f = [0;0;0];
        for k = 1:num_points
            point = data_xyz(k,:);
            % f = f +
            % (point'+rot_mat(data_ang(k,1),data_ang(k,2),data_ang(k,3))*[x(1);x(2);x(3)]
            % - [x(4);x(5);x(6)]).^2; This is wrong since it assumes R = I
%             f = f + ([x(7);x(8);x(9)]-rot_mat(data_ang(k,1),data_ang(k,2),data_ang(k,3))'*[x(1),x(4),x(5);x(4),x(2),x(6);x(5),x(6),x(3)]*point' - [x(10);x(11);x(12)]).^2;
%             f = f + ([x(5);x(6);x(7)]-rot_mat(data_ang(k,1),data_ang(k,2),data_ang(k,3))'*axang2rotm([x(1) x(2) x(3) x(4)])*point' - [x(8);x(9);x(10)]).^2;
            f = f + (-rot_mat(data_ang(k,1),data_ang(k,2),data_ang(k,3))'*point'+rot_mat(data_ang(k,1),data_ang(k,2),data_ang(k,3))'*[x(4);x(5);x(6)] - [x(7);x(8);x(9)]).^2;
        end     
        f_out = sum(f)/num_points;
       
    end
    x0 = [0,0,0,0,0,0.5,0,0,0];
    % does not need neader mead. 
    A = [0 0 0 1 0 0 -1 0 0;0 0 0 0 1 0 0 -1 0;0 0 0 0 0 1 0 0 -1];
%     b = [pi/2;pi/2;pi/2;pi/2;pi/2;pi/2];
    b = [0;0;0];
    x = fmincon(@objectivefcn, x0,[],[],[],[],[],[],[], options);
    
    mean(data_xyz(1:end,:)-x(4:6),1);
end
