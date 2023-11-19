function [T] = compute_dyn_matrix(q,qd,qdd)

%% GENERAL PARAMETERS
s1 = sin(q(1)); c1 = cos(q(1)); 
s2 = sin(q(2)); c2 = cos(q(2));
s3 = sin(q(3)); c3 = cos(q(3));
s4 = sin(q(4)); c4 = cos(q(4));
s5 = sin(q(5)); c5 = cos(q(5));
s6 = sin(q(6)); c6 = cos(q(6));

dQ1 = qd(1); ddQ1 = qdd(1); 
dQ2 = qd(2); ddQ2 = qdd(2);
dQ3 = qd(3); ddQ3 = qdd(3); 
dQ4 = qd(4); ddQ4 = qdd(4);
dQ5 = qd(5); ddQ5 = qdd(5);
dQ6 = qd(6); ddQ6 = qdd(6);

%需要更新DH坐标
d2 = -188.76; d3 = -81.24; d5 = -12.15; d6 = 112;
a5 = 252.5; alp4 = 4/9*pi; g = 9802;%单位：mm
fe1 = 0; fe2 = 0; fe3 = 0; ne1 = 0; ne2 = 0; ne3 = 0;

%% LOAD PARAMETERS RESPECTIVELY FROM WORKSPACE
m1 = evalin('base','m1'); 
m2 = evalin('base','m2'); 
m3 = evalin('base','m3'); 
m4 = evalin('base','m4'); 
m5 = evalin('base','m5');
m6 = evalin('base','m6');

mc11 = evalin('base','mc11')/m1; mc12 = evalin('base','mc12')/m1; mc13 = evalin('base','mc13')/m1;
mc21 = evalin('base','mc21')/m2; mc22 = evalin('base','mc22')/m2; mc23 = evalin('base','mc23')/m2;
mc31 = evalin('base','mc31')/m3; mc32 = evalin('base','mc32')/m3; mc33 = evalin('base','mc33')/m3;
mc41 = evalin('base','mc41')/m4; mc42 = evalin('base','mc42')/m4; mc43 = evalin('base','mc43')/m4;
mc51 = evalin('base','mc51')/m5; mc52 = evalin('base','mc52')/m5; mc53 = evalin('base','mc53')/m5;
mc61 = evalin('base','mc61')/m6; mc62 = evalin('base','mc62')/m6; mc63 = evalin('base','mc63')/m6;

Ic111 = evalin('base','Ic111'); Ic122 = evalin('base','Ic122'); Ic133 = evalin('base','Ic133'); Ic112 = evalin('base','Ic112'); Ic113 = evalin('base','Ic112'); Ic123 = evalin('base','Ic123');
Ic211 = evalin('base','Ic211'); Ic222 = evalin('base','Ic222'); Ic233 = evalin('base','Ic233'); Ic212 = evalin('base','Ic212'); Ic213 = evalin('base','Ic213'); Ic223 = evalin('base','Ic223');
Ic311 = evalin('base','Ic311'); Ic322 = evalin('base','Ic322'); Ic333 = evalin('base','Ic333'); Ic312 = evalin('base','Ic312'); Ic313 = evalin('base','Ic313'); Ic323 = evalin('base','Ic323');
Ic411 = evalin('base','Ic411'); Ic422 = evalin('base','Ic422'); Ic433 = evalin('base','Ic433'); Ic412 = evalin('base','Ic412'); Ic413 = evalin('base','Ic413'); Ic423 = evalin('base','Ic423');
Ic511 = evalin('base','Ic511'); Ic522 = evalin('base','Ic522'); Ic533 = evalin('base','Ic533'); Ic512 = evalin('base','Ic512'); Ic513 = evalin('base','Ic513'); Ic523 = evalin('base','Ic523');
Ic611 = evalin('base','Ic611'); Ic622 = evalin('base','Ic622'); Ic633 = evalin('base','Ic633'); Ic612 = evalin('base','Ic612'); Ic613 = evalin('base','Ic613'); Ic623 = evalin('base','Ic623');

Ia1 = evalin('base','Ia1');
Ia2 = evalin('base','Ia2');
Ia3 = evalin('base','Ia3');
Ia4 = evalin('base','Ia4');
Ia5 = evalin('base','Ia5');
Ia6 = evalin('base','Ia6');

fv1 = evalin('base','fv1');
fc1 = evalin('base','fc1');
fv2 = evalin('base','fv2');
fc2 = evalin('base','fc2');
fv3 = evalin('base','fv3');
fc3 = evalin('base','fc3');
fv4 = evalin('base','fv4');
fc4 = evalin('base','fc4');
fv5 = evalin('base','fv5');
fc5 = evalin('base','fc5');
fv6 = evalin('base','fv6');
fc6 = evalin('base','fc6');

disp("<INFO> PARAMETERS Loaded!!");

%% DYNAMIC ITEMS
M_ = evalin('base','M');
C_ = evalin('base','C');
G_ = evalin('base','G');
F_ = evalin('base','F');

M = eval(subs(M_, ...
        {'m1','mc11','mc12','mc13','Ic111','Ic122','Ic133','Ic112','Ic113','Ic123','Ia1','fv1','fc1',...
         'm2','mc21','mc22','mc23','Ic211','Ic222','Ic233','Ic212','Ic213','Ic223','Ia2','fv2','fc2',...
         'm3','mc31','mc32','mc33','Ic311','Ic322','Ic333','Ic312','Ic313','Ic323','Ia3','fv3','fc3',...
         'm4','mc41','mc42','mc43','Ic411','Ic422','Ic433','Ic412','Ic413','Ic423','Ia4','fv4','fc4',...
         'm5','mc51','mc52','mc53','Ic511','Ic522','Ic533','Ic512','Ic513','Ic523','Ia5','fv5','fc5',...
         'm6','mc61','mc62','mc63','Ic611','Ic622','Ic633','Ic612','Ic613','Ic623','Ia6','fv6','fc6',...
         's1','c1','s2','c2','s3','c3','s4','c4','s5','c5','s6','c6',...
         'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6',...
         'ddQ1','ddQ2','ddQ3','ddQ4','ddQ5','ddQ6',...
         'fe1','fe2','fe3','ne1','ne2','ne3',...
         'g','d1','d3','d5'},...
        {m1,mc11,mc12,mc13,Ic111,Ic122,Ic133,Ic112,Ic113,Ic123,Ia1,fv1,fc1,...
         m2,mc21,mc22,mc23,Ic211,Ic222,Ic233,Ic212,Ic213,Ic223,Ia2,fv2,fc2,...
         m3,mc31,mc32,mc33,Ic311,Ic322,Ic333,Ic312,Ic313,Ic323,Ia3,fv3,fc3,...
         m4,mc41,mc42,mc43,Ic411,Ic422,Ic433,Ic412,Ic413,Ic423,Ia4,fv4,fc4,...
         m5,mc51,mc52,mc53,Ic511,Ic522,Ic533,Ic512,Ic513,Ic523,Ia5,fv5,fc5,...
         m6,mc61,mc62,mc63,Ic611,Ic622,Ic633,Ic612,Ic613,Ic623,Ia6,fv6,fc6,...
         s1,c1,s2,c2,s3,c3,s4,c4,s5,c5,s6,c6,...
         dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,...
         ddQ1,ddQ2,ddQ3,ddQ4,ddQ5,ddQ6,...
         fe1,fe2,fe3,ne1,ne2,ne3,...
         g,d1,d3,d5,d7}));

C = zeros(7, 7, 7);
for k = 1:7
C(:, :, k) = eval(subs(C_(:, :, k), ...
		{'m1','mc11','mc12','mc13','Ic111','Ic122','Ic133','Ic112','Ic113','Ic123','Ia1','fv1','fc1',...
         'm2','mc21','mc22','mc23','Ic211','Ic222','Ic233','Ic212','Ic213','Ic223','Ia2','fv2','fc2',...
         'm3','mc31','mc32','mc33','Ic311','Ic322','Ic333','Ic312','Ic313','Ic323','Ia3','fv3','fc3',...
         'm4','mc41','mc42','mc43','Ic411','Ic422','Ic433','Ic412','Ic413','Ic423','Ia4','fv4','fc4',...
         'm5','mc51','mc52','mc53','Ic511','Ic522','Ic533','Ic512','Ic513','Ic523','Ia5','fv5','fc5',...
         'm6','mc61','mc62','mc63','Ic611','Ic622','Ic633','Ic612','Ic613','Ic623','Ia6','fv6','fc6',...
		 'm7','mc71','mc72','mc73','Ic711','Ic722','Ic733','Ic712','Ic713','Ic723','Ia7','fv7','fc7',...
         's1','c1','s2','c2','s3','c3','s4','c4','s5','c5','s6','c6','s7','c7',...
         'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6','dQ7',...
         'ddQ1','ddQ2','ddQ3','ddQ4','ddQ5','ddQ6','ddQ7',...
         'fe1','fe2','fe3','ne1','ne2','ne3',...
         'g','d1','d3','d5','d7'},...
        {m1,mc11,mc12,mc13,Ic111,Ic122,Ic133,Ic112,Ic113,Ic123,Ia1,fv1,fc1,...
         m2,mc21,mc22,mc23,Ic211,Ic222,Ic233,Ic212,Ic213,Ic223,Ia2,fv2,fc2,...
         m3,mc31,mc32,mc33,Ic311,Ic322,Ic333,Ic312,Ic313,Ic323,Ia3,fv3,fc3,...
         m4,mc41,mc42,mc43,Ic411,Ic422,Ic433,Ic412,Ic413,Ic423,Ia4,fv4,fc4,...
         m5,mc51,mc52,mc53,Ic511,Ic522,Ic533,Ic512,Ic513,Ic523,Ia5,fv5,fc5,...
         m6,mc61,mc62,mc63,Ic611,Ic622,Ic633,Ic612,Ic613,Ic623,Ia6,fv6,fc6,...
		 m7,mc71,mc72,mc73,Ic711,Ic722,Ic733,Ic712,Ic713,Ic723,Ia7,fv7,fc7,...
         s1,c1,s2,c2,s3,c3,s4,c4,s5,c5,s6,c6,s7,c7,...
         dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,dQ7,...
         ddQ1,ddQ2,ddQ3,ddQ4,ddQ5,ddQ6,ddQ7,...
         fe1,fe2,fe3,ne1,ne2,ne3,...
         g,d1,d3,d5,d7}));
end

G = eval(subs(G_, ...
		{'m1','mc11','mc12','mc13','Ic111','Ic122','Ic133','Ic112','Ic113','Ic123','Ia1','fv1','fc1',...
         'm2','mc21','mc22','mc23','Ic211','Ic222','Ic233','Ic212','Ic213','Ic223','Ia2','fv2','fc2',...
         'm3','mc31','mc32','mc33','Ic311','Ic322','Ic333','Ic312','Ic313','Ic323','Ia3','fv3','fc3',...
         'm4','mc41','mc42','mc43','Ic411','Ic422','Ic433','Ic412','Ic413','Ic423','Ia4','fv4','fc4',...
         'm5','mc51','mc52','mc53','Ic511','Ic522','Ic533','Ic512','Ic513','Ic523','Ia5','fv5','fc5',...
         'm6','mc61','mc62','mc63','Ic611','Ic622','Ic633','Ic612','Ic613','Ic623','Ia6','fv6','fc6',...
		 'm7','mc71','mc72','mc73','Ic711','Ic722','Ic733','Ic712','Ic713','Ic723','Ia7','fv7','fc7',...
         's1','c1','s2','c2','s3','c3','s4','c4','s5','c5','s6','c6','s7','c7',...
         'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6','dQ7',...
         'ddQ1','ddQ2','ddQ3','ddQ4','ddQ5','ddQ6','ddQ7',...
         'fe1','fe2','fe3','ne1','ne2','ne3',...
         'g','d1','d3','d5','d7'},...
        {m1,mc11,mc12,mc13,Ic111,Ic122,Ic133,Ic112,Ic113,Ic123,Ia1,fv1,fc1,...
         m2,mc21,mc22,mc23,Ic211,Ic222,Ic233,Ic212,Ic213,Ic223,Ia2,fv2,fc2,...
         m3,mc31,mc32,mc33,Ic311,Ic322,Ic333,Ic312,Ic313,Ic323,Ia3,fv3,fc3,...
         m4,mc41,mc42,mc43,Ic411,Ic422,Ic433,Ic412,Ic413,Ic423,Ia4,fv4,fc4,...
         m5,mc51,mc52,mc53,Ic511,Ic522,Ic533,Ic512,Ic513,Ic523,Ia5,fv5,fc5,...
         m6,mc61,mc62,mc63,Ic611,Ic622,Ic633,Ic612,Ic613,Ic623,Ia6,fv6,fc6,...
		 m7,mc71,mc72,mc73,Ic711,Ic722,Ic733,Ic712,Ic713,Ic723,Ia7,fv7,fc7,...
         s1,c1,s2,c2,s3,c3,s4,c4,s5,c5,s6,c6,s7,c7,...
         dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,dQ7,...
         ddQ1,ddQ2,ddQ3,ddQ4,ddQ5,ddQ6,ddQ7,...
         fe1,fe2,fe3,ne1,ne2,ne3,...
         g,d1,d3,d5,d7}));

F = eval(subs(F_, ...
		{'m1','mc11','mc12','mc13','Ic111','Ic122','Ic133','Ic112','Ic113','Ic123','Ia1','fv1','fc1',...
         'm2','mc21','mc22','mc23','Ic211','Ic222','Ic233','Ic212','Ic213','Ic223','Ia2','fv2','fc2',...
         'm3','mc31','mc32','mc33','Ic311','Ic322','Ic333','Ic312','Ic313','Ic323','Ia3','fv3','fc3',...
         'm4','mc41','mc42','mc43','Ic411','Ic422','Ic433','Ic412','Ic413','Ic423','Ia4','fv4','fc4',...
         'm5','mc51','mc52','mc53','Ic511','Ic522','Ic533','Ic512','Ic513','Ic523','Ia5','fv5','fc5',...
         'm6','mc61','mc62','mc63','Ic611','Ic622','Ic633','Ic612','Ic613','Ic623','Ia6','fv6','fc6',...
		 'm7','mc71','mc72','mc73','Ic711','Ic722','Ic733','Ic712','Ic713','Ic723','Ia7','fv7','fc7',...
         's1','c1','s2','c2','s3','c3','s4','c4','s5','c5','s6','c6','s7','c7',...
         'dQ1','dQ2','dQ3','dQ4','dQ5','dQ6','dQ7',...
         'ddQ1','ddQ2','ddQ3','ddQ4','ddQ5','ddQ6','ddQ7',...
         'fe1','fe2','fe3','ne1','ne2','ne3',...
         'g','d1','d3','d5','d7'},...
        {m1,mc11,mc12,mc13,Ic111,Ic122,Ic133,Ic112,Ic113,Ic123,Ia1,fv1,fc1,...
         m2,mc21,mc22,mc23,Ic211,Ic222,Ic233,Ic212,Ic213,Ic223,Ia2,fv2,fc2,...
         m3,mc31,mc32,mc33,Ic311,Ic322,Ic333,Ic312,Ic313,Ic323,Ia3,fv3,fc3,...
         m4,mc41,mc42,mc43,Ic411,Ic422,Ic433,Ic412,Ic413,Ic423,Ia4,fv4,fc4,...
         m5,mc51,mc52,mc53,Ic511,Ic522,Ic533,Ic512,Ic513,Ic523,Ia5,fv5,fc5,...
         m6,mc61,mc62,mc63,Ic611,Ic622,Ic633,Ic612,Ic613,Ic623,Ia6,fv6,fc6,...
		 m7,mc71,mc72,mc73,Ic711,Ic722,Ic733,Ic712,Ic713,Ic723,Ia7,fv7,fc7,...
         s1,c1,s2,c2,s3,c3,s4,c4,s5,c5,s6,c6,s7,c7,...
         dQ1,dQ2,dQ3,dQ4,dQ5,dQ6,dQ7,...
         ddQ1,ddQ2,ddQ3,ddQ4,ddQ5,ddQ6,ddQ7,...
         fe1,fe2,fe3,ne1,ne2,ne3,...
         g,d1,d3,d5,d7}));

%% NEED NON-DIGNOAL COEFFS TO BE 0.5


T = M * [ddQ1;ddQ2;ddQ3;ddQ4;ddQ5;ddQ6;ddQ7] + CC * [dQ1;dQ2;dQ3;dQ4;dQ5;dQ6;dQ7] + G + F;

end