function fig_plot_robot_UAV(Xt,Ut,Ue,p)

% Parameters
L = p.L;
W = p.W;
h = p.h;

body_color = p.body_color;
ground_color = p.ground_color;

% Unpack
pcom =  reshape(Xt(1:3),[3,1]);
dpc =   reshape(Xt(4:6),[3,1]);
R =     reshape(Xt(7:15),[3,3]);
wb =    reshape(Xt(16:18),[3,1]);

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

% Plot rectangles
for i = 1:length(chains)
    fill3(chains{i}(1, :), chains{i}(2, :), chains{i}(3, :), body_color);
end
 


% % C_root=showWorldCoordinate(1);
% % C_body=genCoordinateCoord(C_root,Twd2com);
% % showCoodinate(C_body,"C_body",1)
 

%% ground
Rground = p.Rground;

goffset = 0.5;      % ground offset
chain0 = repmat([pcom(1);pcom(2);0],[1,4]) +...
    Rground * goffset * [[-1 1 0]',[1 1 0]',[1 -1 0]',[-1 -1 0]'];

fill3(chain0(1,:),chain0(2,:),chain0(3,:),ground_color)

%% GRF
r34b=p.r34;%the positions of propellers in the body frame {B}
r34w=Twd2com * [r34b; ones(1,4) ];
r34w=r34w(1:3,:);

f34=reshape(fcn_get_T_fw_u(R)*Ut,[3,4]);

scale = 7e-2;
% scale = 3e-3;
for i_leg = 1:4
    chain_f = [r34w(:,i_leg),r34w(:,i_leg) + scale * f34(:,i_leg)];
    plot3(chain_f(1,:),chain_f(2,:),chain_f(3,:),'r','linewidth',1.5)
end


% % external force
% p_ext_R = R * p.p_ext + pcom;
% chain_Ue = [p_ext_R,p_ext_R + 0.01 * Ue];
% plot3(chain_Ue(1,:),chain_Ue(2,:),chain_Ue(3,:),'c','linewidth',1.5)
    


end
