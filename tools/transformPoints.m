% Shift points by [dx, dy, dtheta] as defined in r
function pts = transformPoints(pts, r)
    alpha = r(3);
    R = [cos(alpha) -sin(alpha); sin(alpha) cos(alpha)];
    pts = R * pts;
    pts = [pts(1,:) + r(1); pts(2,:) + r(2)];
end