%% Vector function for V_se3
function [V] = V_se3(pd, pa) 

    V = zeros(6, size(pd,3));
        
    for i=1:size(pd,3)
        V(:,i) = Adjoint(pa(:,:,i)) * se3ToVec(MatrixLog6(TransInv(pa(:,:,i)) * pd(:,:,i)));
    end
   
    
end