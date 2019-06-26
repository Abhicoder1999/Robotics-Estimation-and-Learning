function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
    % P = eye(4)
    % R = eye(2)

    % Check if the first time running this function
    dt = 0.033;
    var_px = 20;
    var_py = 30;
    var_vx = 0.1;
    var_vy = 0.3;
    var_zx = 2;
    var_zy = 1;

    if previous_t<0
        state = [x, y, 0, 0];%[px py vx vy]
        param.P = 1 * eye(4); %this is the covarience matrix of prob
        predictx = x;
        predicty = y;
        return;
    end

    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    vx = (x - state(1)) / (t - previous_t);
    vy = (y - state(2)) / (t - previous_t);
%     Predict 330ms into the future
    cal_x = state(1) + vx * 0.330;
    cal_y = state(2) + vy * 0.330;
%     State is a four dimensional element
state = [cal_x cal_y vx vy]';% Xk/k-1 updated from Xk-1/k-1 this is prior  
%% Dynamic Model
 %this is from the above calculations
A = [1 0 dt 0;   %transition matrix
      0 1 0 dt;  %Here the problem is it wont update its 
      0 0 1 0 ;
      0 0 0 1 ;
      ];
sys_noise_covar = [var_px 0 0 0;
                   0 var_py 0 0;
                   0 0 var_vx 0;
                   0 0 0 var_vy;
                   ];
C = [1 0 0 0;  %do not change this as this dimension is necessary for
     0 1 0 0;   % updation and if possible avoid probality model first
     ];
Z = [x;
     y];
 
msr_noise_covar = [var_zx  0;
                   0  var_zy;
                   ];

P = A*param.P*A' + sys_noise_covar;
R = C*param.P*C' + msr_noise_covar;
K = P*C'*inv(R + C*P*C');
size(K)
size(C)
size(P)
size(R)
state = state + K*(Z - C*state);
param.P = param.P*(eye(4) - K*C );

predictx = state(1);
predicty = state(2);

end
