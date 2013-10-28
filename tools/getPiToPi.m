% Convert input angle so that it is in the range -pi to pi.

function angle = getPiToPi(angle)
    if angle > pi
        angle = angle - 2*pi;
    elseif angle < -pi
        angle = angle + 2*pi;
    end
end