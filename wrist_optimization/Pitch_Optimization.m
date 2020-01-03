


pitchRange = @(input) acos((-input(3)^2+input(2)^2+1^2+input(1)^2)/(2*sqrt(1^2+input(1)^2)*input(2))) - acos((-(1.7*input(3))^2+input(2)^2+1^2+input(1)^2)/(2*sqrt(1^2+input(1)^2)*input(2)));

lowerBounds = [0.5, 0.25, .2];
upperBounds = [3, 1.2, 2];
CLrange = linspace(lowerBounds(1), upperBounds(1), 6);
PLrange = linspace(lowerBounds(2), upperBounds(2), 6);
ALrange = linspace(lowerBounds(3), upperBounds(3), 6);

A = [];
b = [];
Aeq = [];
beq = [];
options = optimoptions('fmincon', 'algorithm', 'interior-point');

xList = [];
fvalList = [];
for i = CLrange
    for j = PLrange
        for k = ALrange
            test = pitchRange([i,j,k])
            test2 = [i, j, k]
            if isreal(pitchRange([i, j, k]))
                [x, fval] = fmincon(pitchRange, [i,j,k], A, b, Aeq, beq, lowerBounds, upperBounds, [], options);
                xList = [xList; x];
                fvalList = [fvalList; fval];                
            end
        end
    end
end
min(xList)