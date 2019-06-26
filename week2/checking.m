close all;
clear all;clc;
function prob(mean,covarience,D)


end

%% Parameters tunning
dt = 0.033;
var_px = 20;
var_py = 30;
var_vx = 0.1;
var_vy = 0.2;
var_zx = 0.03;
var_zy = 0.03;
%% Dynamic Model
s = [0 0 0 0];%[px py vx vy]
A = [1 0 dt 0;   %transition matrix
      0 1 0 dt;
      0 0 1 0 ;
      0 0 0 1 ;
      ];
sys_noise_covar = [var_px 0 0 0;
                   0 var_py 0 0;
                   0 0 var_vx 0;
                   0 0 0 var_vy;
                   ];
P = [10 0 0 0;
     0 10 0 0;
     0 0 10 0;
     0 0 0 10];
 
% s = tm*s' + sys_noise_covar;
%% Measurement Model

C = [1 0 0 0;
     0 1 0 0;
     ];

msr_noise_covar = [var_zx  0;
                   0  var_zy;
                   ];

% Z = C*y' + msr_noise_covar;
%% Importing observed data
load training5.mat
for i = 1:length(ball)
    Px = prob()
    Pz = prob(ball(1:2,i),C*P*C'+msr_noise_covar,2);%make prob func mean,covar,dimension
    
end