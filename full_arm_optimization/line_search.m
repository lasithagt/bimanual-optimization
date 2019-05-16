%% Line search using dichtomous search
function lambda = line_search(func, n_dich, l_l, l_h,d)
    l_l_ = l_l;
    l_h_ = l_h;
    for i = 1:n_dich
        lamd_l = l_l_ + (l_h_-l_l_)/2 - d/2;
        lamd_h = l_l_ + (l_h_-l_l_)/2 + d/2;
        
        if (func(lamd_l) <= func(lamd_h))
            l_h_ = lamd_h;
        else
            l_l_ = lamd_l; 
        end
        lambda = (l_l_ + l_h_) / 2;
    end
end