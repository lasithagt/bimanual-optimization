function cost = cost_function(x, input)

a = x(end-3:end)';
q = x(1:end-4)';

pd = input.pd;
n_samples = input.m;

for i = 1:n_samples
    % end effector position constraint
    p = FK([q(4*(i-1)+1:4*(i-1)+4) a]);
    orientation_cost = sum(([1 0 0 0] - quatmultiply(quatconj(rotm2quat(pd(1:3,1:3,i))), rotm2quat(p(1:3,1:3,1)))).^2);
    cost_C1 = sum((p(1:3,end) - pd(1:3,end,i)).^2) + orientation_cost;
end

cost = [cost_C1];


end