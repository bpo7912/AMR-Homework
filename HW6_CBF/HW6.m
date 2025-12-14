%  Control Barrier Function HW: safe navigation around pedestrian
%  Robot:omni robot in 2D
%  Goal:  reach p_goal
%  Obstacle: pedestrian at p_ped, keep distance >= d_safe
%  h(p) = ||p - p_ped|| - d_safe >= 0   (safety condition)
clear; clc; close all;

%Parameters
% environment
p0     = [0; 0.2];        % robot start
p_goal = [2.5; 0.5];      % goal
p_ped  = [1.5; 0.4];    % pedestrian

d_safe = 0.5;           % safe distance [m]

% Controller gains
k_nom  = 1.0;           % nominal attractive control gain
v_max  = 0.8;           % max speed (control bound)
alpha  = 2.0;           % CBF class-K gain

% Simulation
dt  = 0.01;             % time step [s]
T   = 10.0;             % total time [s]
N   = round(T/dt);      % number of steps

%Storage
p_hist    = zeros(2,N+1);   % robot position history
u_hist    = zeros(2,N);     % control inputs
h_hist    = zeros(1,N+1);   % barrier function h(p)
dist_hist = zeros(1,N+1);   % distance to pedestrian

p_hist(:,1)  = p0;
dist0        = norm(p0 - p_ped);
h_hist(1)    = dist0 - d_safe;
dist_hist(1) = dist0;

%Simulation Loop
p = p0;

for k = 1:N
    % current position
    p = p_hist(:,k);
    
    % nominal controller (go-to-goal)
    u_nom = -k_nom * (p - p_goal);    % gradient descent to goal
    
    % saturate to max speed
    v_nom = norm(u_nom);
    if v_nom > v_max
        u_nom = (v_max / v_nom) * u_nom;
    end
    
    % CBF safety filter
    dist = norm(p - p_ped);
    h    = dist - d_safe;
    
    % gradient of h wrt p
    if dist > 1e-6
        grad_h = (p - p_ped) / dist;   % 2x1
    else
        grad_h = [1;0];
    end
    
    % CBF inequality: grad_h' * u >= -alpha * h
    a = grad_h;          % column
    b = -alpha * h;
    
    if a' * u_nom >= b
        u_safe = u_nom;   % nominal is already safe
    else
    
        lambda = (b - a' * u_nom) / (norm(a)^2 + 1e-9);
        u_safe = u_nom + lambda * a;
        
        % bound speed
        v_safe = norm(u_safe);
        if v_safe > v_max
            u_safe = (v_max / v_safe) * u_safe;
        end
    end
    
    %Integrating dynamics
    p_next = p + dt * u_safe;   % Euler integration
    
    %Store
    p_hist(:,k+1)  = p_next;
    u_hist(:,k)    = u_safe;
    h_hist(k+1)    = h;
    dist_hist(k+1) = dist;
end

t = 0:dt:T;

%Static Plots

%XY trajectory with pedestrian & safe circle
figure('Name','Robot Trajectory with CBF Safety','Color','w');
hold on; axis equal; grid on;

%safety circle
th = linspace(0,2*pi,200);
xc = p_ped(1) + d_safe*cos(th);
yc = p_ped(2) + d_safe*sin(th);

plot(p_hist(1,:), p_hist(2,:), 'b-', 'LineWidth', 2);           % robot path
plot(p0(1),     p0(2),   'ko', 'MarkerFaceColor','k');          % start
plot(p_goal(1), p_goal(2), 'gx', 'MarkerSize',10,'LineWidth',2);% goal
plot(p_ped(1),  p_ped(2), 'ro', 'MarkerFaceColor','r');         % pedestrian
plot(xc, yc,'r--','LineWidth',1.5);                             % safe circle

xlabel('x [m]'); ylabel('y [m]');
legend('Robot path','Start','Goal','Pedestrian','Safe region','Location','Best');
title('CBF Navigation: Robot avoids pedestrian while reaching goal');

%Barrier function h(t) = dist - d_safe
figure('Name','Barrier Function','Color','w');
plot(t, h_hist, 'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('h(p) = ||p - p_{ped}|| - d_{safe}');
yline(0,'r--','LineWidth',1.2);
title('Control Barrier Function h(t) (must stay >= 0 for safety)');

%Distance to pedestrian vs safe bound
figure('Name','Distance to Pedestrian','Color','w');
plot(t, dist_hist, 'b','LineWidth',1.5); hold on; grid on;
yline(d_safe,'r--','Safe distance','LineWidth',1.5);
xlabel('Time [s]'); ylabel('Distance to pedestrian [m]');
title('Robot distance to pedestrian over time');
legend('||p - p_{ped}||','d_{safe}','Location','Best');


%Animation 

figure('Name','Robot Animation','Color','w','Position',[100 100 900 500]);
hold on; grid on;
axis equal;

%Fix axis so circle & path are fully visible
xmin = min([p_hist(1,:), xc]) - 0.3;
xmax = max([p_hist(1,:), xc]) + 0.3;
ymin = min([p_hist(2,:), yc]) - 0.3;
ymax = max([p_hist(2,:), yc]) + 0.3;
xlim([xmin xmax]);
ylim([ymin ymax]);

xlabel('x [m]'); ylabel('y [m]');
title('Animated CBF Navigation');

%static elements
plot(p_goal(1), p_goal(2), 'gx', 'MarkerSize',10,'LineWidth',2);
plot(p_ped(1),  p_ped(2), 'ro', 'MarkerFaceColor','r');
plot(xc, yc,'r--','LineWidth',1.5);
plot(p0(1), p0(2), 'ko', 'MarkerFaceColor','k');

%initial robot drawing
robot_plot = plot(p_hist(1,1), p_hist(2,1), 'bo', 'MarkerSize',8,'MarkerFaceColor','b');
path_plot  = plot(p_hist(1,1), p_hist(2,1), 'b-', 'LineWidth',2);

%Video writer setup
video = VideoWriter('CBF_animation.avi', 'Motion JPEG AVI');
video.FrameRate = 30;   % 20–30 is good
open(video);

%animation loop
skip = 3;
for k = 1:skip:(N+1)
    set(robot_plot,'XData', p_hist(1,k),   'YData', p_hist(2,k));
    set(path_plot, 'XData', p_hist(1,1:k), 'YData', p_hist(2,1:k));
    
    drawnow;

    frame = getframe(gcf);
    writeVideo(video, frame);
end

close(video);
disp('Animation saved to CBF_animation.avi');

