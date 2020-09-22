function [FK_u_full, FK_u_stack, FK_uu_full, FK_uu_stack] = FK_partials(x, q, n_waypoints, is_hess)
        
    n = size(x,1) + size(q,1);
    FK_u_full  = zeros(6 * n_waypoints, n_waypoints*size(q,1)+size(x,1));
    FK_u_stack = {};
    
    for i = 1:n_waypoints
        xqi = [q(:,i); x];
        jac_temp = jacobian(@FK_analytical, 6, xqi);
        FK_u_full(6*(i-1)+1:6*i, (i-1)*size(q,1)+1:(i-1)*size(q,1)+size(q,1)) = jac_temp(:,1:size(q,1));
        FK_u_full(6*(i-1)+1:6*i, end-size(x,1)+1:end) = jac_temp(:,size(q,1)+1:end);
        FK_u_stack{i} = jac_temp(:,1:size(q,1));
    end
    
    if (is_hess)
        [FK_uu_full, FK_uu_stack] = hessian(@FK_analytical, 6, xqi);
    else
        FK_uu_full = [];
        FK_uu_stack = [];
    end
end



