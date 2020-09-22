function dexterity_zones(M,S)
    n = 100000;
    l_limit = [-1.5, -1.5, -1.5, -1.5, -1.5, -1.5, -1.5]';
    u_limit = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 1.5]';
    rng default
    
    T_store = zeros(4,4,n);
    JJ_store = zeros(6,7,n);
    for i = 1:n
        q = random_config(l_limit, u_limit);
        T_store(:,:,i)  = FKinSpace(M,S,q);
        J = JacobianSpace(S,q);
        JJ_store(:,:,i) = J;
    end
    save('workspace.mat', 'T_store', 'JJ_store');
end


function q = random_config(l_limit, u_limit)
    q = (rand(7,1)-0.5)*2;
    q = q.*(u_limit - l_limit)/2;

end

