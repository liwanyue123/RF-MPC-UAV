%% initialization
clear all;close all;clc
addpath(genpath("Utils"));
addpath fcns fcns_MPC

%% --- parameters ---
% ---- gait ----
% 0-control; 1-traj; 
gait = 1;
p = get_params_UAV(gait);

p.playSpeed = 1;
p.flag_movie = 1;       % 1 - make movie
use_qpSWIFT = 0;        % 0 - quadprog, 1 - qpSWIFT (external)


dt_sim = p.simTimeStep;
SimTimeDuration =p.SimTimeDuration;  % [sec]
MAX_ITER = floor(SimTimeDuration/p.simTimeStep);

% desired trajectory
p.acc_d = 0; %acceleration
p.vel_d = [0;0];% x,y
p.yaw_d = 2;%yaw

%% Model Predictive Control
% --- initial condition ---
% Xt = [pc dpc vR wb]': [18,1] represents the robot's position, velocity, attitude, and angular velocity information.
% Ut=[t1 t2 t3 t4]' : [4,1]  represents thrusts of four propeller

 
[Xt,Ut] = fcn_gen_XdUd_UAV(0,[],p);


% --- logging ---
tstart = 0;
tend = dt_sim;

[tout,Xout,Uout,Xdout,Udout,Uext,FSMout] = deal([]);

% --- simulation ----
h_waitbar = waitbar(0,'Calculating...');
tic
% 离线计算
for ii = 1:MAX_ITER
    % --- time vector ---
    t_ = dt_sim * (ii-1) + p.Tmpc * (0:p.predHorizon-1);%MPC horizon

    % --- Traj ---
 
    if gait == 1
        % Ud: the simple force allocation of gravity based on the number of propeller.
       [p,Xd,Ud] = fcn_UAV_ref_traj(t_, Xt, p);
    else
        [Xd,Ud] = fcn_gen_XdUd_UAV(t_,Xt,p);
    end

    % --- MPC ----
    % form QP 
    [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta_UAV(Xt,Ut,Xd,Ud,p);

    if ~use_qpSWIFT
        % solve QP using quadprog
        % options1 = optimset('Display', 'iter');
        % [zval] = quadprog(H,g,Aineq,bineq,Aeq,beq,[],[],[],options1);
        [zval] = quadprog(H,g,Aineq,bineq,Aeq,beq,[],[],[]);
    else
        % interface with the QP solver qpSWIFT
        [zval,basic_info] = qpSWIFT(sparse(H),g,sparse(Aeq),beq,sparse(Aineq),bineq);
    end
    Ut = Ut + zval(1:4);

    % --- external disturbance ---
    [u_ext,p_ext] = fcn_get_disturbance(tstart,p);
    p_ext=0*p_ext;
    u_ext = 0*u_ext;
    p.p_ext = p_ext;        % position of external force


    % --- simulate ---
    [t,X] = ode45(    @(t,X)dynamics_SRB_UAV(t,X,Ut,Xd,0*u_ext,p),  [tstart,tend] ,Xt);


    % --- update ---
    %  更新
    Xt = X(end,:)';
    tstart = tend;
    tend = tstart + dt_sim;

    % --- log ---
    %  拼接和记录
    lent = length(t(2:end));
    tout = [tout;t(2:end)];
    Xout = [Xout;X(2:end,:)];
    Uout = [Uout;repmat(Ut',[lent,1])];
    Xdout = [Xdout;repmat(Xd(:,1)',[lent,1])];
    Udout = [Udout;repmat(Ud(:,1)',[lent,1])];
    Uext = [Uext;repmat(u_ext',[lent,1])];
    %     FSMout = [FSMout;repmat(FSM',[lent,1])];

    waitbar(ii/MAX_ITER,h_waitbar,'Calculating...');
end
close(h_waitbar)%关闭等待条
fprintf('Calculation Complete!\n')
toc

%% Animation
disp("Size of tout:");
disp(size(tout));
disp("Size of Uout:");
disp(size(Uout));
[t,EA,EAd] = fig_animate_UAV(tout,Xout,Uout,Xdout,Udout,Uext,p);


