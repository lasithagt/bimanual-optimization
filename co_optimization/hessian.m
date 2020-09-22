%% Computes the numerical hessian with central difference

function [f_xx,C] = hessian(funcHandle, n_func, qx)
    

    hess_func      = @(x)jacobian(funcHandle, n_func, x,true)';

    f_xx    = jacobian(hess_func, n_func, qx, true);

    C = permute(f_xx,[1 3 2]);
    C = reshape(C,[],size(f_xx,2),1);
   
end
