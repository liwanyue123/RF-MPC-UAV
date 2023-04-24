function p = get_params_UAV(gait)

p.gait =gait ;
p.Rground = eye(3);


%% MPC
p.Umax = 50;
p.freq = 30;
p.predHorizon = 20;
p.simTimeStep = 1/100;
p.Tmpc = 4/100; % MPC prediction step time
p.SimTimeDuration = 3;
 
% 0-control; 1-traj;
if gait==1
    p.decayRate = 1;
    %control
    p.R = diag([1e-5 1e-5 1e-5 1e-5]);
    %stage state
    p.Q = diag([1e1 1e1 7e4 ...
        1e1 1e1 5e3 ...
        1e6 2e6 1e6 ...
        2e4 2e4 2e4]);
    %final state
    p.Qf = diag([1e5 2e5 3e5 ...
        5e2 1e3 150 ...
        5e5 5e5 5e5 ...
        40 40 10]);

elseif gait==0
    p.decayRate = 0.9;
    %control
    p.R = diag([1e-5 1e-5 1e-5 1e-5]);
    %stage state
    p.Q = diag([5e4 9e4 5e4 ...
        1e3 1e3 5e5 ...
        1e5 3e5 2e5 ...
        1e2 1e2 1e2]);
    %final state
    p.Qf = diag([1e4 2e4 3e5 ...
        1e2 1e3 5e5 ...
        1e5 3e5 2e5 ...
        40 40 10]);

end

%% Physical Parameters
p.mass = 5.5;
p.J = diag([0.026,0.112,0.075]);
p.g = 9.81;
p.z0 = 0.5;     % nominal COM height

p.r34=[0.15 0.15 -0.15 -0.15;
    0.15 -0.15 0.15 -0.15;
    0 0  0  0];
p.L = 0.30;    % body length
p.W = 0.22;    % body width
p.h = 0.05;     % body height
p.d = 0.05;     % ABAD offset


%% motor
p.c_Mf=1.6e-2;

%% color
p.body_color    = [42 80 183]/255;
p.ground_color  = [195 232 243]/255;












