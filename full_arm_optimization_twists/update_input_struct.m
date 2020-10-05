% This function updates the shoulder orientation, current_link lengths and
% absically defines the robot.
% x_d = [length between two robots, orientation of the shoulder(mirror for other, link design parameters]; 

function input = update_input_struct(x_d, input)
    % passing curr_q is optional
    if (nargin > 2)
        input.curr_q = curr_q;
    end
    
    input.T_L(1,end) = -x_d(19); input.T_R(1,end) = x_d(19);
    input.T_L(2,end) = x_d(20); input.T_R(2,end) = x_d(20);
    input.T_L(3,end) = x_d(21); input.T_R(3,end) = x_d(21);
    
    w = {x_d(1:3),x_d(4:6),x_d(7:9)};
    for i=1:3
        w{i} = w{i} ./ norm(w{i});
    end
    
%     input.T_R(1:3,1:3) = roty(-x_d(3))*rotx(-x_d(4));
%     input.T_L(1:3,1:3) = roty(x_d(3))*rotx(-x_d(4));

    %     input.link_variables = x_d(5:end);
end