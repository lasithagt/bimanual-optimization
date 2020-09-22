function success = IK_trajectory()
    t = -pi:0.01:pi;
    r = 0.08;
    
    x = r * cos(t);
    y = r * sin(t);
    z = 0.8 * ones(1, length(t));    
    R = eye(3);

    terminate = @(i, c) c < 1 || i > 10;
    
    % initalize manipulator
    
    
    while ~terminate(iter, current_cost)
        
        [thetalist, success] = IKinSpace_modified(Slist, M, T, thetalist0, eomg, ev)
    end
    
    function cost(x_desired, x_trajectory)
        c = 0;
        
    end

end