function [S,iperm] = tmprod(T,U,mode)
    % Mode-n tensor-matrix product
    size_tens = ones(1,mode);
    size_tens(1:ndims(T)) = size(T);
    N = length(size_tens);

    % Compute the complement of the set of modes.
    bits = ones(1,N);
    bits(mode) = 0;
    modec = 1:N;
    modec = modec(logical(bits(modec)));

    % Permutation of the tensor
    perm = [mode modec];
    size_tens = size_tens(perm);
    S = T; 
    if mode ~= 1
        S = permute(S,perm); 
    end

    % n-mode product
    size_tens(1) = size(U,1);
    S = reshape(U*reshape(S,size(S,1),[]),size_tens);

    % Inverse permutation
    iperm(perm(1:N)) = 1:N;
    S = permute(S,iperm); 

end

