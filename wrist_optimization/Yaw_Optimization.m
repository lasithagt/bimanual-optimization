yawRange = @(h, w, lp, Ep, e, rp) -(atan((Ep + e) / rp) - (90 - acos((-lp^2 + rp^2 + Ep^2 + 2*Ep*e + e^2 + h^2 + w^2)/(2*sqrt(rp^2 + Ep^2 + 2*Ep*e + e^2)*sqrt(h^2 + w^2))) - atan(w/h))) + (atan(Ep / rp) - (90 - acos((-lp^2 + rp^2 + Ep^2 + h^2 + w^2)/(2*sqrt(rp^2 + Ep^2)*sqrt(h^2 + w^2))) - atan(w/h)));
startingLengths = [0.6915, 0.3936, 0.91, 0.125, 0.5, 0.95];
yawRange2 = @(input) yawRange(input(1), input(2), input(3), input(4), input(5), input(6));
yawRange(0.6915, 0.3936, 0.91, 0.125, 0.5, 0.95)
yawRange2(startingLengths)
lowerBounds = [0.6915, 0.25, 0.25, 0.1, 0.125, 0.675];
upperBounds = [0.6915, 0.5, 2, 2, 2, 1.2];
[x, fval] = fmincon(yawRange2, startingLengths, [], [], [], [], lowerBounds, upperBounds)