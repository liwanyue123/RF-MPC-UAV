function [t,EA,EAd] = fig_animate_UAV(tout,Xout,Uout,Xdout,Udout,Uext,p)

flag_movie = p.flag_movie;

if flag_movie

    try
        name = ['test.mp4'];
        vidfile = VideoWriter(name,'MPEG-4');
    catch ME
        name = ['test'];
        vidfile = VideoWriter(name,'Motion JPEG AVI');
    end
    open(vidfile);
end

%% smoothen for animation

t = (tout(1):p.simTimeStep:tout(end));
X = interp1(tout,Xout,t);
U = interp1(tout,Uout,t);
Xd = interp1(tout,Xdout,t);
Ud = interp1(tout,Udout,t);
Ue = interp1(tout,Uext,t);

%% loop through frames
figure('Position',[200 100 1000 600]);
set(0, 'DefaultFigureRenderer', 'opengl');
set(gcf, 'Color', 'white')
N = 3;M = 3;    % subplot size

nt = length(t);
EA = zeros(nt,3);
EAd = zeros(nt,3);
for ii = 1:nt
    EA(ii,:) = fcn_X2EA(X(ii,:));
    EAd(ii,:) = fcn_X2EA(Xd(ii,:));
end

% for ZOH force
% for ZOH force
% t2 = repelem(t,2);
% t2(1) = []; t2(end+1) = t2(end);
% U2 = repelem(U,2,1);






%%%%%% subfigure handles %%%%%%%
h_x = subplot(N,M,3);
h_dx = subplot(N,M,6);
h_w = subplot(N,M,9);
h_euler = subplot(N,M,8);%姿态角
h_u = subplot(N,M,7);



for ii = 1:p.playSpeed:nt
    %% The main animation
    % plot setting
    pcom = X(ii,1:3)';
    h_main = subplot(N,M,[1,2,4,5]);
    hold on; grid on;axis square;axis equal;

    range=0.6;
    h_main.XLim = [pcom(1)-range pcom(1)+range];
    h_main.YLim = [pcom(2)-range pcom(2)+range];
    %     h_main.ZLim = [-0.2 0.6];
    h_main.ZLim = [pcom(3)-0.7 pcom(3)+0.6];
    viewPt = [0.2,0.5,0.2];
    view(h_main, viewPt);





    % plot robot & GRF
    %画机器人的身体和腿部位置
    % real
    fig_plot_robot_UAV(X(ii,:)',U(ii,:)',Ue(ii,:)',p)
    % desired
    % fig_plot_robot_d_UAV(X(ii,:)',Xd(ii,:)',Ud(ii,:)',p)

    plot3(X(1:ii,1)',X(1:ii,2)', X(1:ii,3)');


    % text
    txt_v = ['v_z = ',num2str(X(ii,6),2),'m/s'];
    text(pcom(1),pcom(2),pcom(3)+0.2,txt_v)
    txt_thrust = ['f = ',num2str(sum(U(ii,:)),2),'N'];
    text(pcom(1),pcom(2),pcom(3)+0.3,txt_thrust)
    txt_time = ['t = ',num2str(t(ii),2),'s'];
    text(pcom(1),pcom(2),pcom(3)+0.4,txt_time)

    %% states
    % X = [pc dpc R wb]'
    %%%%%%%%% position %%%%%%%%%
    plot(h_x,t(1:ii),X(1:ii,1),'r',...
        t(1:ii),X(1:ii,2),'g',...
        t(1:ii),X(1:ii,3),'b',...
        t(1:ii),Xd(1:ii,1),'r--',...
        t(1:ii),Xd(1:ii,2),'g--',...
        t(1:ii),Xd(1:ii,3),'b--','linewidth',1)
    h_x.XLim = [t(1) t(end)];
    set( get(h_x,'Title'), 'String', 'Position [m]');
    legend(h_x, 'X', 'Y', 'Z', 'X_d', 'Y_d', 'Z_d');
    %%%%%%%%% velocity %%%%%%%%%
    plot(h_dx,t(1:ii),X(1:ii,4),'r',...
        t(1:ii),X(1:ii,5),'g',...
        t(1:ii),X(1:ii,6),'b',...
        t(1:ii),Xd(1:ii,4),'r--',...
        t(1:ii),Xd(1:ii,5),'g--',...
        t(1:ii),Xd(1:ii,6),'b--','linewidth',1)
    h_dx.XLim = [t(1) t(end)];
    set( get(h_dx,'Title'), 'String', 'Velocity [m/s]');
    legend(h_dx, 'Vx', 'Vy', 'Vz', 'Vx_d', 'Vy_d', 'Vz_d');

    %%%%%%%%% Angle %%%%%%%%%
    R_matrices = X(1:ii, 7:15);
    Rd_matrices = Xd(1:ii, 7:15);
    % Initialize the variables for storing Euler angles
    roll = zeros(ii, 1);
    pitch = zeros(ii, 1);
    yaw = zeros(ii, 1);

    rolld = zeros(ii, 1);
    pitchd = zeros(ii, 1);
    yawd = zeros(ii, 1);
    % Loop through the rotation matrices and convert them to Euler angles
    for k = 1:ii
        R = reshape(R_matrices(k, :), [3, 3])';
        Rd = reshape(Rd_matrices(k, :), [3, 3])';
        eulerAngles = rotm2eul(R, 'XYZ');  % The default convention is ZYX, which corresponds to yaw-pitch-roll
        roll(k) = eulerAngles(1);
        pitch(k) = eulerAngles(2);
        yaw(k) = eulerAngles(3);

        eulerAngles = rotm2eul(Rd, 'XYZ');  % The default convention is ZYX, which corresponds to yaw-pitch-roll
        rolld(k) = eulerAngles(1);
        pitchd(k) = eulerAngles(2);
        yawd(k) = eulerAngles(3);
    end

    plot(h_euler ...
        , t(1:ii), -rad2deg(roll), 'r', ...
        t(1:ii), -rad2deg(pitch), 'g', ...
        t(1:ii), -rad2deg(yaw), 'b',...
        t(1:ii), -rad2deg(rolld), 'r--', ...
        t(1:ii), -rad2deg(pitchd), 'g--', ...
        t(1:ii), -rad2deg(yawd), 'b--', 'linewidth', 1);
    h_euler.XLim = [t(1) t(end)];
    set(get(h_euler, 'Title'), 'String', 'Euler angles [deg]');
    legend(h_euler, 'Roll', 'Pitch', 'Yaw','Rolld', 'Pitchd', 'Yawd');

    %%%%%%%%% Angular velocity %%%%%%%%%
    plot(h_w,t(1:ii),X(1:ii,16),'r',...
        t(1:ii),X(1:ii,17),'g',...
        t(1:ii),X(1:ii,18),'b',...
        t(1:ii),Xd(1:ii,16),'r--',...
        t(1:ii),Xd(1:ii,17),'g--',...
        t(1:ii),Xd(1:ii,18),'b--','linewidth',1)
    h_w.XLim = [t(1) t(end)];
    set( get(h_w,'Title'), 'String', 'Angular velocity [rad/s]' );
    legend(h_w, 'Wx', 'Wy', 'Wz', 'Wx_d', 'Wy_d', 'Wz_d');

    %% control
    plot(h_u,t(1:ii),U(1:ii,1),'r',...
        t(1:ii),U(1:ii,2),'g',...
        t(1:ii),U(1:ii,3),'b',...
        t(1:ii),U(1:ii,4),'k',...
        'linewidth',1)
    h_u.XLim = [t(1) t(end)];
    set( get(h_u,'Title'), 'String', 'Fz [N]' );
    legend(h_u, 'u1', 'u2', 'u3', 'u4');

    %% make movie
    if flag_movie
        writeVideo(vidfile, getframe(gcf));
    end

    drawnow
    if ii < nt
        cla(h_main);
    end

end

if flag_movie
    close(vidfile);
end




