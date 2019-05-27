function R = aer2rotm(angle)
a = deg2rad(angle(1));
e = deg2rad(angle(2));
r = deg2rad(angle(3));
R = [cos(a)*cos(e), cos(a)*sin(e)*sin(r)-sin(a)*cos(r), cos(a)*sin(e)*cos(r)+sin(a)*sin(r); ...
    sin(a)*cos(e),  sin(a)*sin(e)*sin(r)+cos(a)*cos(r), sin(a)*sin(e)*cos(r)-cos(a)*sin(r); ...
    -sin(e),        cos(e)*sin(r),                      cos(e)*cos(r)];
end