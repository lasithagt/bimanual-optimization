%% Computes the numerical hessian with central difference

function f_xx = hessian_(funcHandle, n_func, n_var, x)
    hstep          = 1.4901e-05; % Do not change this

    hess_x_pertb_p = repmat(x,1,n_var) + hstep .* eye(n_var);
    hess_x_pertb_n = repmat(x,1,n_var) - hstep .* eye(n_var);
    
    f_xx           = (zeros(n_var,n_var,n_func));

    for i = 1:n_var
        temp_fxx = ((gradient_(funcHandle, n_func, n_var, hess_x_pertb_p(:,i),true) ...
                            - gradient_(funcHandle, n_func, n_var, x,true)) + (-gradient_(funcHandle, n_func, n_var, hess_x_pertb_n(:,i),true) ...
                            + gradient_(funcHandle, n_func, n_var, x,true))) ./ 2.0 ./ hstep;
        for j = 1:n_func
            f_xx(i,:,j) = temp_fxx(:,j)';
        end
    end
    
end
