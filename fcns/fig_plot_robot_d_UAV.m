function fig_plot_robot_d_UAV(Xt,Xd,Ud,p)

%% parameters
L = p.L;
W = p.W;
h = p.h;

alpha_value = 0.5;
body_color = p.body_color;
ground_color = p.ground_color;

%% unpack
% pcom =    reshape(Xt(1:3),[3,1]);
pcom =    reshape(Xd(1:3),[3,1]);
dpc =   reshape(Xd(4:6),[3,1]);
R =     reshape(Xd(7:15),[3,3]);
wb =    reshape(Xd(16:18),[3,1]);

% GRF
% f= fcn_get_T_fw_u(R)* Ud;

%% Robot
Twd2com = [R, pcom; 0 0 0 1];
Tcom2hips = [[L/2; W/2; 0], [L/2; -W/2; 0], [-L/2; W/2; 0], [-L/2; -W/2; 0]];
Twd2hips = Twd2com * [Tcom2hips; ones(1,4) ];
Twd2hips_up=Twd2com * [Tcom2hips+ [0; 0; h]; ones(1,4) ];
corner_down = Twd2hips(1:3, :);
corner_up = Twd2hips_up(1:3, :);

% Create chains
chains = {corner_down(:, [1 2 4 3]), corner_down(:, [1 3 4 2]), ...
          [corner_down(:, 1), corner_up(:, 1), corner_up(:, 2), corner_down(:, 2)], ...
          [corner_down(:, 1), corner_up(:, 1), corner_up(:, 3), corner_down(:, 3)], ...
          [corner_down(:, 3), corner_up(:, 3), corner_up(:, 4), corner_down(:, 4)], ...
          [corner_down(:, 2), corner_up(:, 2), corner_up(:, 4), corner_down(:, 4)], ...
          corner_up(:, [1 2 4 3])};

%% plot

% body
for i = 1:length(chains)
    h =fill3(chains{i}(1, :), chains{i}(2, :), chains{i}(3, :), body_color);
    alpha(h, alpha_value);
end
% alpha(f1,0.2)

% GRF
% scale = 1e-2;

%     chain_f = [pf34(:,i_leg),pf34(:,i_leg) + scale * f34(:,i_leg)];
%     plot3(chain_f(1,:),chain_f(2,:),chain_f(3,:),'g','linewidth',1.5)


end
