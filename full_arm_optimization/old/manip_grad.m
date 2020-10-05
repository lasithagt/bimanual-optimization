function [A, b, diff_cost] = manip_grad(lq)

    [J, J_grad] = J_(lq);
    J = J(1:2,1:4);
    J_grad = J_grad(1:2,1:4,:);
    Mj     = J * J';
    [v_,e] = eig(Mj);
    e
    
    % e_avg = sum(mean(e)); % get the mean value of eigen values
    e_ = eye(2);
    e_(1,1) = 2;
    e_(2,2) = 2;
    Me_d  = v_*e_/v_;
    % desired manipulability
    % Me_d  = diag([10,10]);
    
    % calculating the manipulability distance on the manifold
    M_diff = logmap(Me_d, Mj);
    
    
    Mj_q  = tmprod(J_grad,J,2) + tmprod(permute(J_grad,[2,1,3]),J,1);
    Jm_z  = compute_red_manipulability_Jacobian(Mj_q);

    dz  = pinv(Jm_z)*symmat2vec(M_diff);
    A   = Jm_z;
    diff_cost = symmat2vec(M_diff);
    
    % Line search to find the optimal step length (too many function calls, but test it)
    M_cfunc  = @(x)symmat2vec(Mj + M_diff.*x);
    
    lambda   = line_search(M_cfunc, 50, 0.00001, 0.0001,0.000005);
    b        = lambda * symmat2vec(M_diff);
    
end

function Jm_red = compute_red_manipulability_Jacobian(Jm)
    % Compute the force manipulability Jacobian (symbolic) in the form of a
    % matrix using Mandel notation.

    Jm_red = [];
    for i = 1:size(Jm,3)
        Jm_red = [Jm_red, symmat2vec(Jm(:,:,i))];
    end

end

function U = logmap(X,S)
% Logarithm map (SPD manifold)

    N = size(X,3);

    for n = 1:N
    	U(:,:,n) = S^.5 * logm(S^-.5 * X(:,:,n) * S^-.5) * S^.5;
    % 	tic
    % 	U(:,:,n) = S * logm(S\X(:,:,n));
    % 	toc
    % 	tic
%       [v,d] = eig(S\X(:,:,n));
%       U(:,:,n) = S * v*diag(log(diag(d)))*v^-1;
    % 	toc
    end
end

function v = symmat2vec(M)
% Vectorization of a symmetric matrix

    N = size(M,1);
    v = [];

    v = diag(M);
    for n = 1:N-1
      v = [v; sqrt(2).*diag(M,n)]; % Mandel notation
    end
end