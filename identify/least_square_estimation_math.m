%% least_square_estimation.m
% @brief: 用算术表达式的最小二乘法估计参数
%% PARAMETER
% load('.\data\filt.mat');

% filtered data
q_filt = evalin('base', 'q_filt');
qd_filt = evalin('base', 'qd_filt');
qdd_filt = evalin('base', 'qdd_filt');
t_filt = evalin('base', 't_filt');
% DH和负载参数 % 单位：m
fe1 = 0; fe2 = 0; fe3 = 0; ne1 = 0; ne2 = 0; ne3 = 0;
d2 = 0.18876; d3 = -0.08124; d5 = 0.11303; d6 = 0.11150;
a5 = 0.2675; alp4 = 4/9*pi; g = 9.802;
% 最小参数集
pnum_min = evalin('base', 'pnum_min');

n = length(q_filt);		% number of sampling points
ww = zeros(n * 6, pnum_min);
TT = zeros(n * 6, 1);
for k = 1:n
	q = q_filt(k, :);
	qd = qd_filt(k, :);
	qdd = qdd_filt(k, :);

	row1 = 1+(k-1)*6;
	row2 = 6+(k-1)*6;
	ww(row1:row2, :) = min_regressor(q, qd, qdd);		
	TT(row1:row2, 1) = 1e3 * t_filt(k, :)';  % in Nmm
end

%% 映射到最小参数集
% ww * P = TT  ->  P =  inv(ww.T*ww)*ww.T*TT
% ww:(n*6, pnum_min), TT:(n*6, 1), P:(pnum_min, 1)
P = ((ww' * ww)^(-1)) * ww' * TT;
assignin('base', 'P_min', P);	% assign variable in workspace

%% SAVE TO FILE
% fid = fopen('.\data\P_min.txt', 'w');
% fprintf(fid, 'P_min = [');
% for j = 1:pnum_min
%     fprintf(fid, '%s;', P(j));
% end
% fprintf(fid, '];');


