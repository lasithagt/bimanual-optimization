function [f_x_ret] = gradient_(funcHandle, n_func, n_var, x, bool_hess)
%
% Numerical computation of gradient and the hessian
% this allows automatic gradient computation and hessian computation
% 
% first central finite difference
% hstep = 0.001; - programmed in

global hstep
if (nargin > 4) 
    if (bool_hess)
        hstep = 1.4901e-05;%eps; % Do not change this, for smaller values than this, vpa() should be used which makes slower.
    % f = feval(functname,x);
    end
else
    hstep = eps;
end


f_unpertb = funcHandle(x);
x_pertb_p   = repmat(x,1,n_var) + hstep .* eye(n_var);
x_pertb_n   = repmat(x,1,n_var) - hstep .* eye(n_var);
f_x       = zeros(n_var, n_func);
f_x       = gradient_s(funcHandle, f_unpertb, x_pertb_p, x_pertb_n, f_x);

% Return Values
f_x_ret   = (f_x);

%% 
    function f_x = gradient_s(funcHandle, f_unpertb, x_pertb_p, x_pertb_n, f_x)
        for k = 1:size(f_x,3)
            for j = 1:size(x_pertb_p,2)
                xs_p = x_pertb_p(:,j);
                xs_n = x_pertb_n(:,j);
                f_x(j,:,k)= ((funcHandle(xs_p) - f_unpertb) - (funcHandle(xs_n) - f_unpertb)) ./ 2 ./hstep;
            end
        end
    end

end

