function [theta, w] = rotm2symQuat(R)

theta = (acos((trace(R)-1)./2));
if theta==0
    w = [1;0;0];
else
    w = (1./sin(theta)).*[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)];
end

end