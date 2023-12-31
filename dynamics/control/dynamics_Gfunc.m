%% dynamics_Gfunc.m
%  从封闭形式的动力学方程中得到动力学状态空间方程的重力矢量（6*1）
function G = dynamics_Gfunc(TAU)

%%
G = sym(zeros(6,1));
for ii = 1:6
    G(ii) = subs(TAU(ii), ...
				{'qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6', ...
				 'qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6', ...
				 'fe1', 'fe2', 'fe3', 'ne1', 'ne2', 'ne3', ...
				 'fv1', 'fv2', 'fv3', 'fv4', 'fv5', 'fv6', ...
				 'fc1', 'fc2', 'fc3', 'fc4', 'fc5', 'fc6'}, ...
				 {0, 0, 0, 0, 0, 0, ...
				  0, 0, 0, 0, 0, 0, ...
				  0, 0, 0, 0, 0, 0, ...
				  0, 0, 0, 0, 0, 0, ...
				  0, 0, 0, 0, 0, 0});
end
