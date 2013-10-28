%   d = sqrt(px^2 + py^2) + rd : rd = measurement noise
%   a = atan(py/px) + ra

function [y, Y_x] = getMeasurement(x)

    px = x(1, :);
    py = x(2, :);
    
    d = sqrt(px.^2 + py.^2);
    a = atan2(py, px);
    
    y = [d; a];
    
    if nargout > 1 % We want Jacobians (only works for single point)
        Y_x = [...
            px/(px^2 + py^2)^(1/2),     py/(px^2 + py^2)^(1/2);
            -py/(px^2*(py^2/px^2 + 1)), 1/(px*(py^2/px^2 + 1))];
    end

end