function res = dex(Slist, q)
    n = size(q,2);
    d = 0;  
    for i=1:n
        J  = JacobianSpace(Slist, q(:,i)) ;
        e  = eig(J*J');
        d  = d + abs(1-min(e)/max(e));
    end
    res = d;
end