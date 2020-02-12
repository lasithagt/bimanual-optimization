function  [x,fval]=tool_tip_approx_(data,i)
    data_xyz = data(:,1:3);
    data_aer = data(:,4:6);
    
    num_points = size(data_xyz,1);  
    
    figure(i)
    plot3(data_xyz(:,1),data_xyz(:,2),data_xyz(:,3))
    
%     options = optimset('MaxFunEvals',100000,'Display','iter');
    options = optimset('Display','off');
    function f_out = objectivefcn1(x)
        f_ = [0 0 0]';
        for k = 1:num_points
            point = data_xyz(k,:);
            temp = (point' + eul2rotm(data_aer(k,:),'ZYX')*[x(1);x(2);x(3)] - [x(4);x(5);x(6)]).^2;
            f_ = f_ + temp;
        end     
        f_out = sum(f_)/num_points;
       
    end
    x0 = [10,0,0,-1.5,8.5,-1];
    % does not need neader mead. 
%     x = fmincon(@objectivefcn1, x0,[],[],[],[],[],[],[], options);
    [x,fval] = fminsearch(@objectivefcn1, x0, options);

end


