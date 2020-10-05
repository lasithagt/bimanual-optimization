%% Vector function for FK
function [pose_K] = FK_SE3(Slist, M, T, q) 
    global input
 
    if size(q,1)~=input.n_links 
        error('q is not with correct dimensions. 1-dim should be n and 2-dim should be m')
    end
    
    theta  = q;
    pose_K = zeros(4,4,size(theta,2));
    m = size(q,2);
    for i=1:m
        pose_K(:,:,i)  = T * FKinSpace(M, Slist, theta(:,i));
    end
   
end