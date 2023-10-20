function [ds_q, ds_qd, ds_tau] = downsampling(raw_q, raw_qd, raw_tau, down_size)
% @brief: downsample raw data from robot sensors
% @param[in] raw_q, raw_qd, raw_qdd, raw_tau:
%            raw joint angle, velocity, acceleration, torque data from sensors
% @param[in] down_size: downsampling size
% @param[out] ds_q, ds_qd, ds_qdd, ds_tau:
%             downsampled joint angle, velocity, acceleration, torque

% filtering
% uni_data = unique([raw_q, raw_qd, raw_qdd, raw_tau], 'rows');
% uni_q = uni_data(:, 1:6);
% uni_qd = uni_data(:, 7:12);
% uni_qdd = uni_data(:, 13:18);
% uni_tau = uni_data(:, 19:24);

ds_q = zeros(down_size, 6);
ds_qd = zeros(down_size, 6);
% ds_qdd = zeros(down_size, 6);
ds_tau = zeros(down_size, 6);

% downsampling
step = fix(size(raw_q, 1) / down_size);%fix：截尾取整; size(raw_q, 1):返回raw_q的行数
for i = 1:down_size
    ds_q(i, :) = raw_q(step * i, :);
    ds_qd(i, :) = raw_qd(step * i, :);
%     ds_qdd(i, :) = raw_qdd(step * i, :);
    ds_tau(i, :) = raw_tau(step * i, :);
end

