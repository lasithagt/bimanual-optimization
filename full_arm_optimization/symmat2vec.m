function v = symmat2vec(M)
    % Vectorization of a symmetric matrix
    N = size(M,1);
    v = [];

    v = diag(M);
    for n = 1:N-1
      v = [v; sqrt(2).*diag(M,n)]; % Mandel notation
    end
end