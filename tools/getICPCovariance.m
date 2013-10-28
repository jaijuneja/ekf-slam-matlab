function cov = getICPCovariance(icp_var, data)
    M = zeros(size(data, 2) * 2, 3);
    for k = 1:size(data,2)
        M_k = [ 1   0   -data(2, k)
                0   1   data(1, k)];
        M(2*k-1:2*k, :) = M_k;
    end

    cov = icp_var * inv(M' * M);
end