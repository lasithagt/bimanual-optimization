%% Generates trajectories. Takes in the center for reference
function [yy, norm] = arc_traj(xx)
    % Predifined points to consider
    y         = [1.5 2 1.6 2.0 2.1] - 1.5 ; 
    x         = [1.5 1.4 0.8 0.9 0.6] + 0.6 ;

    p         = polyfit(x,y,4);
    norm_func = @(x,y)[-4*p(1)*x^3-3*p(2)*x^2-2*p(3)*x^1-p(4);1]; 
    yy        = polyval(p, xx);
    norm      = norm_func(xx, yy);
hold on
    plot(x,y,'k*')
end