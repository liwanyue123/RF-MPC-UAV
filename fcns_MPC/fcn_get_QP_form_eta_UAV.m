function [H,g,Aineq,bineq,Aeq,beq] = fcn_get_QP_form_eta_UAV(Xt,Ut,Xd,Ud,p)
% min. 0.5 * x' * H *x + g' * x
% s.t. Aineq *x <= bineq
%      Aeq * x = beq
% X = [pc dpc vR wb pf]': [30,1]
% q = [pc dpc eta wb]: [12 1]
% lb/ub - [4,n_hor]

%% parameters
n_hor = p.predHorizon;
Umax = p.Umax;
decayRate = p.decayRate;

R = p.R;%control
Q = p.Q;%stage state
Qf = p.Qf;%final state 
[Qx,Qv,Qeta,Qw] = deal(Q(1:3,1:3),Q(4:6,4:6),Q(7:9,7:9),Q(10:12,10:12));
[Qxf,Qvf,Qetaf,Qwf] = deal(Qf(1:3,1:3),Qf(4:6,4:6),Qf(7:9,7:9),Qf(10:12,10:12));

nX = 12;%dim of state vector
nU = 4;%dim of control vector

%%%%%%% A,B,d matrices for linear dynamics %%%%%%%%%%%
[A,B,d] = fcn_get_ABD_eta_UAV(Xt,Ut,p);

%% Decompose
Rt = reshape(Xt(7:15,1),[3,3]);
qt = [Xt(1:6);[0;0;0];Xt(16:18)];
% lb <= Fz <= ub
Fzd=repmat(Umax*[1 1 1 1]', 1, n_hor);
lb = 0 * Fzd;
ub = Fzd;

%% Matrices for QP
H = zeros((nX + nU) * n_hor);
g = zeros(size(H,1),1);
Aeq = zeros(nX * n_hor,(nX+nU) * n_hor);
beq = zeros(size(Aeq,1),1);

%only one propeller
Aineq_unit = [-1 ;
    1];


nAineq_unit = size(Aineq_unit,1);
Aineq = zeros(4*nAineq_unit*n_hor,(nX+nU)*n_hor);
bineq = zeros(size(Aineq,1),1);
for i_hor = 1:n_hor
    xd = Xd(1:3,i_hor);
    vd = Xd(4:6,i_hor);
    Rd = reshape(Xd(7:15,i_hor),[3,3]);
    wd = Xd(16:18,i_hor);

    %% Objective function
    idx_u = (i_hor-1) * (nX + nU) + (1:nU);
    idx_x = (i_hor-1) * (nX + nU) + nU + (1:nX);
    if i_hor == n_hor
        H(idx_x,idx_x) = Qf * decayRate^(i_hor-1);
        g(idx_x) = [-Qxf * xd;
            -Qvf * vd;
            Qetaf * veeMap(logm(Rd' * Rt));
            -Qwf * wd] * decayRate^(i_hor-1);
    else
        H(idx_x,idx_x) = Q * decayRate^(i_hor-1);
        g(idx_x) = [-Qx * xd;
            -Qv * vd;
            Qeta * veeMap(logm(Rd' * Rt));
            -Qw * wd] * decayRate^(i_hor-1);
    end
    H(idx_u,idx_u) = R * decayRate^(i_hor-1);
    g(idx_u) = R' * (Ut - Ud(:,i_hor)) * decayRate^(i_hor-1);

    %% Equality constraints
    if i_hor == 1
        Aeq(1:nX,1:(nU+nX)) = [-B,eye(nX)];
        beq(1:nX) = A * qt + d;
    else
        Aeq((i_hor-1)*nX+(1:nX),(i_hor-2)*(nX+nU)+nU+(1:(2*nX+nU)))= [-A -B eye(nX)];
        beq((i_hor-1)*nX+(1:nX)) = d;
    end

    %% Inequality constraints
    Fi = zeros(4*nAineq_unit,4);
    hi = zeros(size(Fi,1),1);

    %The matrix b is related to the control input of the rotor at the operation point,
    % so the constraint matrices of the four propellers need to be filled in separately
    for i_leg = 1:4
        idx_F = (i_leg-1)*nAineq_unit + (1:nAineq_unit);
        idx_u = (i_leg-1)*1 + 1;

        Fi(idx_F,idx_u) = Aineq_unit;
        hi(idx_F) = [  Ut(i_leg)-lb(i_leg,i_hor);
            ub(i_leg,i_hor)- Ut(i_leg) ];

    end
    idx_A = (i_hor-1) * 4*nAineq_unit + (1:4*nAineq_unit);
    idx_z = (i_hor-1) * (nX+nU) + (1:nU);
    Aineq(idx_A,idx_z) = Fi;
    bineq(idx_A) = hi;

end



end



