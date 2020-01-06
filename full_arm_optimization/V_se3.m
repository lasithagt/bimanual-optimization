%% Vector function for V_se3
function [V] = V_se3(pd, pa) 
    global input
    %     if size(q,1)~=input.n_links || size(q,2)~=input.m
    %         error('q is not with correct dimensions. 1-dim should be n and 2-dim should be m')
    %     end
    
    V = zeros(6, size(pd,3));
        
    for i=1:size(pd,3)
        V(:,i) = Adjoint(pa(:,:,i)) * se3ToVec(MatrixLog6(TransInv(pa(:,:,i)) * pd(:,:,i)));
    end
   
    
end