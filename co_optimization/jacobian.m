function [f_x_ret] = jacobian(funcHandle, n_func, qx, bool_hess)
%
% Numerical computation of gradient and the hessian
% this allows automatic gradient computation and hessian computation
% 
% first central finite difference
% hstep = 0.001; - programmed in

global hstep
if (nargin > 3) 
    if (bool_hess)
        hstep = 1e-6;%eps; % Do not change this, for smaller values than this, vpa() should be used which makes slower.
    % f = feval(functname,x);
    end
else
    hstep = 1e-12;
end

n_var       = numel(qx);
f_unpertb   = funcHandle(qx);
x_pertb_p   = repmat(qx,1,n_var) + hstep .* eye(n_var);
x_pertb_n   = repmat(qx,1,n_var) - hstep .* eye(n_var);

if (size(f_unpertb,2) > 1)
    f_x         = zeros(n_var, n_var, n_func);
else
    f_x         = zeros(n_func, n_var);
end
f_x_ret         = gradient_s(funcHandle, f_unpertb, x_pertb_p, x_pertb_n, f_x);

% Return Values
% f_x_ret   = f_x;

%% 
    function f_x = gradient_s(funcHandle, f_unpertb, x_pertb_p, x_pertb_n, f_x)
        for k = 1:size(f_x,3)
            for j = 1:size(x_pertb_p,2)
                xs_p = x_pertb_p(:,j);
                xs_n = x_pertb_n(:,j);
                p = funcHandle(xs_p);
                n = funcHandle(xs_n);
                if (size(p,2) > 1)
                   f_x(:,j,k)= ((n(:,k) - f_unpertb(:,k))./hstep)  ;
%                    f_x(:,j,k)= -imag(p(:,k))./hstep ;

                else
                   f_x(:,j,k)= ((p - f_unpertb)-(n - f_unpertb)) ./2 ./hstep;
%                    f_x(:,j,k)= imag(p) ./hstep;
                end
            end
        end
    end

end

