% HW6: Control Barrier Function (CBF) Navigation
% - Omni robot in 2D
% - Two moving pedestrians (dynamic obstacles)
% - Maintain distance >= d_safe for each pedestrian
% - Robot reaches goal using nominal go-to-goal controller + CBF filter

clear; clc; close all;

%Parameters
% Robot start / goal
p0     = [0; 0.2];
p_goal = [2.8; 0.6];

% Safe distance
d_safe = 0.5;

% Nominal controller + bounds
k_nom  = 1.0;
v_max  = 0.8;
alpha  = 2.0;       % class-K gain in CBF inequality

% Simulation
dt = 0.01;
T  = 12.0;
N  = round(T/dt);

% Pedestrian 1 (moving)
p1_0 = [1.2; 0.1];
v1   = [0.05; 0.03];            % m/s (constant drift)

% Pedestrian 2 (moving, crossing)
p2_0 = [2.2; 1.0];
v2   = [-0.06; -0.02];          % m/s

% Optional "wobble" (small sinusoid)
use_wobble = true;
wob_amp = 0.05;                 % meters
wob_w   = 2*pi*0.25;            % rad/s (0.25 Hz)

% Stop when close to goal
goal_tol = 0.08;

% Storage
p_hist    = zeros(2, N+1);
u_hist    = zeros(2, N);

p1_hist   = zeros(2, N+1);
p2_hist   = zeros(2, N+1);

h1_hist   = zeros(1, N+1);
h2_hist   = zeros(1, N+1);

d1_hist   = zeros(1, N+1);
d2_hist   = zeros(1, N+1);

p_hist(:,1) = p0;

% Helper (single CBF projection)
% Enforce: a' u >= b, adjust u minimally if violated
project_halfspace = @(u, a, b) (u + ((b - a'*u)/(norm(a)^2 + 1e-9))*a);

% Simulation loop
for k = 1:N
    t = (k-1)*dt;

    % Current robot position
    p = p_hist(:,k);

    % Update moving pedestrians
    p_ped1 = p1_0 + v1*t;
    p_ped2 = p2_0 + v2*t;

    if use_wobble
        p_ped1 = p_ped1 + [0; wob_amp*sin(wob_w*t)];
        p_ped2 = p_ped2 + [wob_amp*sin(wob_w*t + pi/3); 0];
    end

    p1_hist(:,k) = p_ped1;
    p2_hist(:,k) = p_ped2;

    % Nominal go-to-goal control (single integrator model)
    u_nom = -k_nom * (p - p_goal);

    % Saturate nominal speed
    vn = norm(u_nom);
    if vn > v_max
        u_nom = (v_max / vn) * u_nom;
    end

    % Multi-pedestrian CBF filter 
    u_safe = u_nom;

    % Apply CBF for pedestrian 1
    dist1 = norm(p - p_ped1);
    h1    = dist1 - d_safe;

    if dist1 > 1e-6
        grad_h1 = (p - p_ped1) / dist1;    % a = ∂h/∂p
    else
        grad_h1 = [1;0];
    end
    a1 = grad_h1;
    b1 = -alpha*h1;

    if a1' * u_safe < b1
        u_safe = project_halfspace(u_safe, a1, b1);
    end

    % Apply CBF for pedestrian 2
    dist2 = norm(p - p_ped2);
    h2    = dist2 - d_safe;

    if dist2 > 1e-6
        grad_h2 = (p - p_ped2) / dist2;
    else
        grad_h2 = [0;1];
    end
    a2 = grad_h2;
    b2 = -alpha*h2;

    if a2' * u_safe < b2
        u_safe = project_halfspace(u_safe, a2, b2);
    end

    % Final speed bound
    vs = norm(u_safe);
    if vs > v_max
        u_safe = (v_max / vs) * u_safe;
    end

    % Integrating robot dynamics
    p_next = p + dt*u_safe;

    % Store
    p_hist(:,k+1) = p_next;
    u_hist(:,k)   = u_safe;

    h1_hist(k) = h1;  d1_hist(k) = dist1;
    h2_hist(k) = h2;  d2_hist(k) = dist2;

    % Stop early if goal reached
    if norm(p_next - p_goal) < goal_tol
        % fill last ped states for consistency
        p1_hist(:,k+1) = p_ped1;
        p2_hist(:,k+1) = p_ped2;
        p_hist = p_hist(:,1:k+1);
        p1_hist = p1_hist(:,1:k+1);
        p2_hist = p2_hist(:,1:k+1);
        h1_hist = h1_hist(1:k);
        h2_hist = h2_hist(1:k);
        d1_hist = d1_hist(1:k);
        d2_hist = d2_hist(1:k);
        N = k;
        break;
    end
end

t = (0:N-1)*dt;

function [hBody, hWheels] = drawOmniRobot(ax, p, theta, Rb, wheelLen)

    % Body circle points
    ang = linspace(0,2*pi,80);
    xb = p(1) + Rb*cos(ang);
    yb = p(2) + Rb*sin(ang);

    hBody = plot(ax, xb, yb, 'b-', 'LineWidth', 2);

    wheelAngles = theta + [0, 2*pi/3, 4*pi/3];
    hWheels = gobjects(1,3);
    for i = 1:3
        a = wheelAngles(i);
        % wheel center on body perimeter
        wc = p + Rb*[cos(a); sin(a)];
        % wheel line segment
        dir = [cos(a+pi/2); sin(a+pi/2)];
        p1 = wc - 0.5*wheelLen*dir;
        p2 = wc + 0.5*wheelLen*dir;

        hWheels(i) = plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 3);
    end
end

function updateOmniRobot(hBody, hWheels, p, theta, Rb, wheelLen)
    ang = linspace(0,2*pi,80);
    xb = p(1) + Rb*cos(ang);
    yb = p(2) + Rb*sin(ang);
    set(hBody, 'XData', xb, 'YData', yb);

    wheelAngles = theta + [0, 2*pi/3, 4*pi/3];
    for i = 1:3
        a = wheelAngles(i);
        wc = p + Rb*[cos(a); sin(a)];
        dir = [cos(a+pi/2); sin(a+pi/2)];
        p1 = wc - 0.5*wheelLen*dir;
        p2 = wc + 0.5*wheelLen*dir;

        set(hWheels(i), 'XData', [p1(1) p2(1)], 'YData', [p1(2) p2(2)]);
    end
end


% Plots
figure('Name','XY Trajectory (2 Moving Pedestrians)','Color','w');
hold on; axis equal; grid on;
plot(p_hist(1,:), p_hist(2,:), 'b-', 'LineWidth', 2);
plot(p0(1), p0(2), 'ko', 'MarkerFaceColor','k');
plot(p_goal(1), p_goal(2), 'gx', 'MarkerSize',10, 'LineWidth',2);

plot(p1_hist(1,1:N), p1_hist(2,1:N), 'r--', 'LineWidth', 1.5);
plot(p2_hist(1,1:N), p2_hist(2,1:N), 'm--', 'LineWidth', 1.5);

xlabel('x [m]'); ylabel('y [m]');
legend('Robot path','Start','Goal','Ped 1 path','Ped 2 path','Location','Best');
title('CBF Navigation with Two Moving Pedestrians');

figure('Name','Barrier Functions h_i(t)','Color','w');
plot(t, h1_hist, 'r','LineWidth',1.5); hold on; grid on;
plot(t, h2_hist, 'm','LineWidth',1.5);
yline(0,'k--','LineWidth',1.2);
xlabel('Time [s]'); ylabel('h_i = ||p - p_{ped,i}|| - d_{safe}');
legend('h_1(t)','h_2(t)','h=0 bound','Location','Best');
title('Barrier Functions (must remain \ge 0)');

figure('Name','Distances to Pedestrians','Color','w');
plot(t, d1_hist, 'r','LineWidth',1.5); hold on; grid on;
plot(t, d2_hist, 'm','LineWidth',1.5);
yline(d_safe,'k--','d_{safe}','LineWidth',1.2);
xlabel('Time [s]'); ylabel('Distance [m]');
legend('||p - p_{ped,1}||','||p - p_{ped,2}||','d_{safe}','Location','Best');
title('Robot Distance vs Safe Bound');

% Animation + Video
figure('Name','CBF Animation (2 moving pedestrians)','Color','w','Position',[100 100 900 520]);
hold on; grid on; axis equal;

% axis limits
xmin = min([p_hist(1,:), p1_hist(1,1:N), p2_hist(1,1:N)]) - 0.5;
xmax = max([p_hist(1,:), p1_hist(1,1:N), p2_hist(1,1:N)]) + 0.5;
ymin = min([p_hist(2,:), p1_hist(2,1:N), p2_hist(2,1:N)]) - 0.5;
ymax = max([p_hist(2,:), p1_hist(2,1:N), p2_hist(2,1:N)]) + 0.5;
xlim([xmin xmax]); ylim([ymin ymax]);

xlabel('x [m]'); ylabel('y [m]');
title('Animated CBF Navigation');

plot(p_goal(1), p_goal(2), 'gx', 'MarkerSize',10,'LineWidth',2);
plot(p0(1), p0(2), 'ko', 'MarkerFaceColor','k');

% Robot drawing parameters
Rb = 0.10;          % robot body radius (m) - visual size
wheelLen = 0.08;    % wheel segment length (m)

% Initial heading guess
theta_vis = 0;

[robotBody, robotWheels] = drawOmniRobot(gca, p_hist(:,1), theta_vis, Rb, wheelLen);
path_plot  = plot(p_hist(1,1), p_hist(2,1), 'b-', 'LineWidth',2);

ped1_plot  = plot(p1_hist(1,1), p1_hist(2,1), 'ro', 'MarkerFaceColor','r');
ped2_plot  = plot(p2_hist(1,1), p2_hist(2,1), 'mo', 'MarkerFaceColor','m');

th = linspace(0,2*pi,200);
c1 = plot(p1_hist(1,1)+d_safe*cos(th), p1_hist(2,1)+d_safe*sin(th), 'r--','LineWidth',1.2);
c2 = plot(p2_hist(1,1)+d_safe*cos(th), p2_hist(2,1)+d_safe*sin(th), 'm--','LineWidth',1.2);

video = VideoWriter('CBF_2ped_animation.mp4', 'MPEG-4');
video.FrameRate = 30;
open(video);

skip = 3;
for k = 1:skip:(N+1)
    kk = min(k, size(p1_hist,2)); % safety
    
if kk > 1
    dp = p_hist(:,kk) - p_hist(:,kk-1);
    if norm(dp) > 1e-6
        theta_vis = atan2(dp(2), dp(1));
    end
end

updateOmniRobot(robotBody, robotWheels, p_hist(:,kk), theta_vis, Rb, wheelLen);

    set(path_plot, 'XData', p_hist(1,1:kk), 'YData', p_hist(2,1:kk));

    set(ped1_plot,'XData', p1_hist(1,kk), 'YData', p1_hist(2,kk));
    set(ped2_plot,'XData', p2_hist(1,kk), 'YData', p2_hist(2,kk));

    set(c1,'XData', p1_hist(1,kk)+d_safe*cos(th), 'YData', p1_hist(2,kk)+d_safe*sin(th));
    set(c2,'XData', p2_hist(1,kk)+d_safe*cos(th), 'YData', p2_hist(2,kk)+d_safe*sin(th));

    drawnow;
    writeVideo(video, getframe(gcf));
end

close(video);
disp('Animation saved to CBF_2ped_animation.avi');
