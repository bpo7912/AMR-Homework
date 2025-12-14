clear; clc; close all;
run('PARAMETERS.m');              % loads dt etc.

model = 'HW3'; load_system(model);
out = sim(model);                 

%  fetch To-Workspace arrays
if ~evalin('base','exist(''e1'',''var'')')
    error('e1/e2/e3 not found. Check To Workspace block names & wiring.');
end
e1    = evalin('base','e1');     e1dot = evalin('base','e1dot');
e2    = evalin('base','e2');     e2dot = evalin('base','e2dot');
e3    = evalin('base','e3');     e3dot = evalin('base','e3dot');

% time vector
if evalin('base','exist(''tout'',''var'')')
    t = evalin('base','tout');
else
    N = numel(e1); t = (0:N-1)'*dt;
end

% sliding surfaces
c = 5;
s1 = c*e1 + e1dot;  s2 = c*e2 + e2dot;  s3 = c*e3 + e3dot;

% Phase portraits
figure('Name','Phase Portraits','Color','w');
tiledlayout(2,2);
nexttile; plot(e1, e1dot); grid on; title('Wheel 1');
nexttile; plot(e2, e2dot); grid on; title('Wheel 2');
nexttile; plot(e3, e3dot); grid on; title('Wheel 3');
nexttile; plot(e1, e1dot, e2, e2dot, e3, e3dot); grid on; title('Overlay');

% sliding surfaces
figure('Name','Sliding Surfaces','Color','w');
plot(t,s1,'r',t,s2,'g',t,s3,'b','LineWidth',1.5); grid on;
xlabel('Time (s)'); ylabel('s_i = c e_i + \ite_i'); legend('s_1','s_2','s_3');

% Optional time plot
figure('Name','Wheel Speed Error vs Time','Color','w');
plot(t, e1, t, e2, t, e3, 'LineWidth',1.2); grid on;
xlabel('Time (s)');
ylabel('Error (rad/s)');
legend('e_1','e_2','e_3');
title('Wheel Speed Errors');