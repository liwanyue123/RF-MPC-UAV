function [A,B,D] = fcn_get_ABD_eta_UAV(Xt,Ut,p)
% linear dynamics for rotation
% evolution variable is eta

%% parameters
dt = p.Tmpc;
c_Mf=p.c_Mf;

%% unpack
xop = reshape(Xt(1:3),[3,1]);
vop = reshape(Xt(4:6),[3,1]);
Rop = reshape(Xt(7:15),[3,3]);
wop = reshape(Xt(16:18),[3,1]);
r34 = p.r34;%[3,4]

%% constants for linear matrices
% [x,v,eta,w,constant]
[Cx_x,Cx_v,Cv_v,Cv_u,Cv_c] = eta_co_xv(Ut,dt,p.mass,p.g,Rop);%% postion and velocity
[CE_eta, CE_w, CE_c] = eta_co_R(Rop,wop,dt);% orientaion
[Cw_p,Cw_eta,Cw_w, Cw_u, Cw_c] = eta_co_w(xop,Rop,wop,Ut,dt,p.J,r34,c_Mf);% angular velocity


%% Assemble matrices
A = [Cx_x, Cx_v, zeros(3,6);
    zeros(3), Cv_v, zeros(3,6);
    zeros(3,6),CE_eta,CE_w;
    Cw_p,zeros(3),Cw_eta,Cw_w];
B = [zeros(3,4);
    Cv_u;
    zeros(3,4);
    Cw_u];
D = [zeros(3,1);
    Cv_c;
    CE_c;
    Cw_c];

end

%% Core fcns for constant matrix

%% postion and velocity
function [Cx_x,Cx_v,Cv_v,Cv_u,Cv_c] = eta_co_xv(Uop,dt,mass,g,Rop)

Cx_x = eye(3);
Cx_v = eye(3 ) * dt;

Cv_v = eye(3);
% Cv_u = dt/mass *Rop*[0;0;1]*ones(1,4);
Cv_u = dt/mass * [eye(3),eye(3),eye(3),eye(3)] * fcn_get_T_fw_u(Rop);
Cv_c = dt/mass * [eye(3),eye(3),eye(3),eye(3)] * fcn_get_T_fw_u(Rop)* Uop + [0;0;-g] * dt;


end

%% orientaion
function [CE_eta, CE_w, CE_c] = eta_co_R(Rop,wop,dt)
% the input arguments are composed of variables at the operating point
% and parameters

N = fcn_get_N;

% debugged code
invN = pinv(N);

C_eta = kron(eye(3),Rop*hatMap(wop))*N + kron(eye(3),Rop)*fcn_get_D(wop);
C_w = kron(eye(3),Rop) * N;
C_c = vec(Rop*hatMap(wop)) - kron(eye(3),Rop)*N*wop;

CE_eta = eye(3) + invN * dt * kron(eye(3),Rop') * C_eta;
CE_w = invN * dt * kron(eye(3),Rop') * C_w;
CE_c = invN * dt * kron(eye(3),Rop') * C_c;

end

%% angular velocity
function [Cw_p,Cw_eta,Cw_w, Cw_u, Cw_c] = eta_co_w(xop,Rop,wop,Uop,dt,J,r34,c_Mf)
% the input arguments are composed of variables at the operating point
% parameters
e3=[0 0 1]';

N = fcn_get_N;
rw=Rop*r34;
%world frame
r1w = rw(:,1) ;
r2w = rw(:,2) ;
r3w = rw(:,3) ;
r4w = rw(:,4) ;
%body frame
r1b = r34(:,1) ;
r2b = r34(:,2) ;
r3b = r34(:,3) ;
r4b = r34(:,4) ;

Mop = [hatMap(r1w) hatMap(r2w) hatMap(r3w) hatMap(r4w)] *fcn_get_T_fw_u(Rop)* Uop;%total moment in world frame

temp_J_w = hatMap(J*wop) - hatMap(wop) * J;

%Cx
%Cv
Ceta = fcn_get_F(Mop) * N - temp_J_w * hatMap(wop);
Cw = temp_J_w;
Cu = [( hatMap(r1b)   + c_Mf * eye(3)) * e3, ...
    (hatMap(r2b) -c_Mf * eye(3)) * e3, ...
    (hatMap(r3b) - c_Mf * eye(3)) * e3,...
    (hatMap(r4b)  + c_Mf * eye(3)) * e3];
Cc = -hatMap(wop)*J*wop + Rop'*Mop - temp_J_w * wop ;%delete - Cx*xop

Cw_p = zeros(3);
%Cw_v= zeros(3);%delete Cw_v
Cw_eta = dt*(J\Ceta);
Cw_w = dt*(J\Cw) + eye(3);
Cw_u = dt*(J\Cu)   ;
Cw_c = dt*(J\Cc);

end

%% Aux fcns
function F = fcn_get_F(k)

F = [k', zeros(1,3),zeros(1,3);...
    zeros(1,3),k',zeros(1,3);...
    zeros(1,3),zeros(1,3),k'];
end

function N = fcn_get_N

N = [0 0 0;...
    0 0 1;...
    0 -1 0;...
    0 0 -1;...
    0 0 0;...
    1 0 0;...
    0 1 0;...
    -1 0 0;...
    0 0 0];
end

function D = fcn_get_D(in)

d = in(1);
e = in(2);
f = in(3);
D = [0 0 0;
    e -d 0;
    f 0 -d;
    -e d 0;
    0 0 0;
    0 f -e;
    -f 0 d;
    0 -f e;
    0 0 0];
end


