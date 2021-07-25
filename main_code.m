%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Advance Neuroscience Motor Control Project 
% Porf. Dr. Ghazizadeh
% Ali Ghavampour - Student Number: 97102293
% Maryam Maghsoudi - Student Number: 97102503
% Spring 2023

% %% Caution: Change your directory to the project "Codes" folder
% cd('C:\Desktop\Advance Neuroscience\Project\Codes');

%% Part I - Question 1:
clear; clc; close all;
% from theoretical calculations: a0 = a1 = a2 = 0
M = [0.5^3 0.5^4 0.5^5; 3*0.5^2 4*0.5^3 5*0.5^4; 6*0.5 12*0.5^2 20*0.5^3];
ai = inv(M)*[10;0;0]; % 10 cm
ai = [0, 0, 0, transpose(ai)];
x_f = @(t) ai(1) + ai(2)*t + ai(3)*t^2 + ai(4)*t^3 + ai(5)*t^4 + ai(6)*t^5;    % cm
v_f = @(t) ai(2) + 2*ai(3)*t + 3*ai(4)*t^2 + 4*ai(5)*t^3 + 5*ai(6)*t^4;       % cm/sec
a_f = @(t) 2*ai(3) + 6*ai(4)*t + 12*ai(5)*t^2 + 20*ai(6)*t^3;                % cm^2/sec 

t = [0: 0.01: 0.5];

for it = 1: length(t)
    time = t(it);
    x(it) = x_f(time);
    v(it) = v_f(time);
    a(it) = a_f(time);
end

clear x_f v_f a_f M time it

figure;
subplot(3,6,[1:3])
plot(t, x,'linewidth', 2);
xlabel('time (sec)', 'interpreter', 'latex');
ylabel('Position (cm)', 'interpreter', 'latex');
title("position")

subplot(3,6,[7:9])
plot(t, v,'color','#D95319','linewidth', 2);
xlabel('time (sec)', 'interpreter', 'latex');
title("velocity")
ylabel('Velocity ($\frac{cm}{sec}$)', 'interpreter', 'latex');

subplot(3,6,[13:15])
plot(t, a,'color','#EDB120','linewidth', 2);
xlabel('time (sec)', 'interpreter', 'latex');
title("acceleration")
ylabel('Acceleration ($\frac{cm^2}{sec}$)', 'interpreter', 'latex');

subplot(3, 6, [4:6,10:12,16:18]); plot(t, x, t, v, t, a, 'linewidth', 2);
xlabel('time (sec)', 'interpreter', 'latex');
legend('Position (cm)', 'Velocity ($\frac{cm}{sec}$)','Acceleration ($\frac{cm^2}{sec}$)', 'interpreter', 'latex');
yline(10, '--', 'color', 'k', 'linewidth', 2, 'handlevisibility', 'off');
yline(0 , '--', 'color', 'k', 'linewidth', 2, 'handlevisibility', 'off');
% xline(0.5, '--', 'color', 'k', 'linewidth', 2, 'handlevisibility', 'off');


%% Part I - Question 2:

xdot = -0.5: 0.1: 0.5;
ydot = -0.5: 0.1: 0.5;

B_SCF = [0 13; -13 0];      %clockwise curl field
B_OSCF = -B_SCF;            %counter clockwise curl field
B_CAF = [11 11; -11 11];    %curl assist field
B_SF = [11 11; 11 -11];     %saddle field

B = {B_SCF, B_OSCF, B_CAF, B_SF};   %Field Matrices
clear B_SCF B_OSCF B_CAF B_SF

fieldnames = {'clockwise curl field', 'counter clockwise curl field', ...
    'curl assist field', 'saddle field'};

for i = 1: length(B)
    for ix = 1: length(xdot)
        for iy = 1: length(ydot)
            F{i}(ix, iy, :) = B{i} * [xdot(ix); ydot(iy)]; %Force Fields
        end
    end
end
clear ix iy i

% According to figure 1 of reference 1, forces of curl fields (the first
% two fields are perpendicular to velocity. curl assist field and saddle 
% field forces are both perpendicular and parallel to the velocity.

forces{1}(:, :, 1) = -F{1}(:, :, 2);
forces{1}(:, :, 2) = -F{1}(:, :, 1);

forces{2}(:, :, 1) = -F{2}(:, :, 2);
forces{2}(:, :, 2) = -F{2}(:, :, 1);

forces{3}(:, :, 1) = F{3}(:, :, 1);
forces{3}(:, :, 2) = -F{3}(:, :, 2);

forces{4}(:, :, 1) = F{4}(:, :, 1);
forces{4}(:, :, 2) = -F{4}(:, :, 2);

figure;
for i = 1: length(F)
    subplot(2, 2, i)
    quiver(xdot, ydot, forces{i}(:, :, 1), forces{i}(:, :, 2), ...
        'Autoscalefactor', 0.9); 
    title(fieldnames{i});
    xlabel('X velocity');
    ylabel('Y velocity');
end
clear i ans

%% Part I - Question 3:
clear forces F
theta = 0: pi/4: 2*pi-pi/4;

for iB = 1: length(B)
    for iD = 1: length(theta)
       % calculate velocity
       v_vec{iB, iD}(1, :) = v * cos(theta(iD));    %vx
       v_vec{iB, iD}(2, :) = v * sin(theta(iD));    %vy
       coor{iD}(1, :) = x * cos(theta(iD));         %x
       coor{iD}(2, :) = x * sin(theta(iD));         %y
       
       % calculate field
       F{iB, iD} = B{iB} * v_vec{iB, iD};
    end
end

figure;
for iB = 1: length(B)
    subplot(2, 2, iB)
    for iD = 1: length(theta)
        quiver(coor{iD}(1, :), coor{iD}(2, :), F{iB, iD}(1, :), F{iB, iD}(2, :));
        hold on;
        plot([coor{iD}(1,1),coor{iD}(1,end)],[coor{iD}(2,1),coor{iD}(2,end)],'k','linewidth',1.2)
    end
    title(fieldnames{iB});
    xlabel('x (cm)'); ylabel('y (cm)');
end
clear iB iD itheta ix iy
%% part II - Question 1:
% clear; clc; close all;

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %two state, gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %two-state, gain-independent, multi-rate model 

% parameters (As in the materials & methods Smith 2006 paper)
A = 0.99;
B = 0.013;
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

nd_s = 30;
nd_g = 17;
nd_m = 30;
N = 400;

%inputs (The deadaptation trials are set so that the final value of
%unlearning reaches zero)
r_s = [ones(1, 400), -ones(1, nd_s), ones(1, N-nd_s)];   %single state model
r_g = [ones(1, 400), -ones(1, nd_g), ones(1, N-nd_g)];   %two state, gain specific model
r_m = [ones(1, 400), -ones(1, nd_m), ones(1, N-nd_m)];   %two-state, gain-independent,...
                                                         %multi-rate model 

% creating output
for n = 1: length(r_s)-1  %trials
    
    %single state model output
    x_s(n+1) = A * x_s(n) + B * (r_s(n) - x_s(n));
    
    %two state, gain specific model output
    x1_g(n+1) = min(0, A * x1_g(n) + B * (r_g(n) - x1_g(n)));
    x2_g(n+1) = max(0, A * x2_g(n) + B * (r_g(n) - x2_g(n)));
    x_g(n+1) = x1_g(n+1) + x2_g(n+1);
    
    %two-state, gain-independent, multi-rate model output
    x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x1_m(n));
    x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x2_m(n));
    x_m(n+1) = x1_m(n+1) + x2_m(n+1);
end
clear n

% provide Figure 1 panel B
figure;
subplot(2, 3, 1);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_s N+nd_s N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_s 2*N 2*N N+nd_s], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(r_s, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_s, 'color', 'r', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Singel-State Model');
legend('Net Adaptation', 'location', 'southeast');

subplot(2, 3, 2);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_g N+nd_g N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_g 2*N 2*N N+nd_g], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(r_g, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_g, 'color', 'r', 'linewidth', 2);
plot(x2_g, '--', 'color', 'b', 'linewidth', 2);
plot(x1_g, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Gain-Specific Model');
legend('Net Adaptation', 'Up State', 'Down State', 'location', 'southeast');

subplot(2, 3, 3);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_m 2*N 2*N N+nd_m], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(r_m, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_m, 'color', 'r', 'linewidth', 2);
plot(x2_m, '--', 'color', 'b', 'linewidth', 2);
plot(x1_m, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Multi-Rate Model');
legend('Net Adaptation', 'Slow State', 'Fast State', 'location', 'southeast');

subplot(2, 3, 4);
xlim([0 N])
plot(x_s(1: N), 'color', 'r', 'linewidth', 1.5);
hold on;
plot(x_s(N+nd_s: end), '--', 'color', 'r', 'linewidth', 3);
xlabel('Trial Number');
ylabel('Adaptation');
legend('Initial learning', 'Relearning', 'location', 'southeast');

subplot(2, 3, 5);
xlim([0 N])
plot(x_g(1: N), 'color', 'r', 'linewidth', 1.5);
hold on;
plot(x_g(N+nd_g: end), '--', 'color', 'r', 'linewidth', 3);
xlabel('Trial Number');
ylabel('Adaptation');
legend('Initial learning', 'Relearning', 'location', 'southeast');

subplot(2, 3, 6);
xlim([0 N])
plot(x_m(1: N), 'color', 'r', 'linewidth', 1.5);
hold on;
plot(x_m(N+nd_m: end), '--', 'color', 'r', 'linewidth', 3);
xlabel('Trial Number');
ylabel('Adaptation');
legend('Initial learning', 'Relearning', 'location', 'southeast');
sgtitle('Relearning Experiment');

%% Part II - Question 2:

clear x_s x1_g x2_g x_g x1_m x2_m x_m

iw = 1;
%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

svgTrl = 30;    % calculate saving in the 30th trial

for iw = 2: 300
    clear r_s r_g r_m
    %create input with washout trials:
    r_s = [ones(1, 400), -ones(1, nd_s), zeros(1, iw), ones(1, N-nd_s)];   %single state
    r_g = [ones(1, 400), -ones(1, nd_g), zeros(1, iw), ones(1, N-nd_g)];   %gain specific
    r_m = [ones(1, 400), -ones(1, nd_m), zeros(1, iw), ones(1, N-nd_m)];   %multi-rate
    
    for n = 1: length(r_s)-1  %trials

        %single state model output
        x_s(n+1) = A * x_s(n) + B * (r_s(n) - x_s(n));

        %two state, gain specific model output
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r_g(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r_g(n) - x2_g(n)));
        x_g(n+1) = x1_g(n+1) + x2_g(n+1);

        %two-state, gain-independent, multi-rate model output
        x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x2_m(n));
        x_m(n+1) = x1_m(n+1) + x2_m(n+1);
    end
       
    saving_s(iw) = abs(x_s(N + nd_s + iw + svgTrl) - x_s(svgTrl))/x_s(svgTrl)*100;
    saving_g(iw) = abs(x_g(N + nd_g + iw + svgTrl) - x_g(svgTrl))/x_g(svgTrl)*100;
    saving_m(iw) = abs(x_m(N + nd_m + iw + svgTrl) - x_m(svgTrl))/x_m(svgTrl)*100;
end
clear iw

figure;
subplot(2, 3, 2);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([N N+nd_g N+nd_g N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none')
patch([N+nd_g N+nd_g+300 N+nd_g N+nd_g+300], [-1 -1 1 1], [1 1 1], ...
    'FaceAlpha', 0.1, 'edgecolor', 'none')
patch([N+nd_g+300 2*N+300 2*N+300 N+nd_g+300], [-1 -1 1 1], [0 0 0], ...
    'FaceAlpha', 0.1, 'edgecolor', 'none')
xlim([0 2*N+300]);
plot(r_g, 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
legend('Adaptation Trials', 'Deadaptation Trials', 'Null Trials');
xlabel('Trials'); ylabel('State');
subplot(2, 3, 4); plot(saving_s(2:end), 'k', 'linewidth', 2); ylim([0 100]);
title('Single-State Model'); xlabel('Number of Washout Trials'); ylabel('Saving (%)');
subplot(2, 3, 5); plot(saving_g(2:end), 'k', 'linewidth', 2); ylim([0 100]);
title('Gain-Specific Model'); xlabel('Number of Washout Trials'); ylabel('Saving (%)');
subplot(2, 3, 6); plot(saving_m(2:end), 'k', 'linewidth', 2); ylim([0 100]);
title('Multi-Rate Model'); xlabel('Number of Washout Trials'); ylabel('Saving (%)');
sgtitle('Relearning Experiment with Washout');


%% part II - Question 3:

clear x_s x1_g x2_g x_g x1_m x2_m x_m

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;
r_s = [ones(1, N), -ones(1, nd_s), zeros(1, N-nd_s)];   %single state
r_g = [ones(1, N), -ones(1, nd_g), zeros(1, N-nd_g)];   %gain specific
r_m = [ones(1, N), -ones(1, nd_m), zeros(1, N-nd_m)];   %multi-rate

for n = 1: length(r_s)-1  %trials
    
    %single state model output
    if n < N + nd_s
        x_s(n+1) = A * x_s(n) + B * (r_s(n) - x_s(n)); 
    else
        x_s(n+1) = A * x_s(n);
    end
        
    %two state, gain specific model output
    if n < N + nd_g
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r_g(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r_g(n) - x2_g(n)));
    else
        x1_g(n+1) = min(0, A * x1_g(n));
        x2_g(n+1) = max(0, A * x2_g(n));
    end
    x_g(n+1) = x1_g(n+1) + x2_g(n+1);

    %two-state, gain-independent, multi-rate model output
    if n < N + nd_m
        x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x2_m(n));
    else
        x1_m(n+1) = Af * x1_m(n);
        x2_m(n+1) = As * x2_m(n);
    end
    x_m(n+1) = x1_m(n+1) + x2_m(n+1);
end


figure;
subplot(2, 3, 2)
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([N N+nd_s N+nd_s N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none')
patch([N+nd_s 2*N 2*N N+nd_s], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
xlim([0 2*N])
plot(r_s, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(N + nd_s: 2*N, zeros(1, length(N + nd_s: 2*N)), '--', 'color', 'k', 'linewidth', 3)
xlabel('Trials');
ylabel('State');
legend('Adaptation Trials', 'Deadaptation Trials', 'Error-Clamp Trials');

subplot(2, 3, 4);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_s N+nd_s N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_s 2*N 2*N N+nd_s], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
% plot(r_s, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_s, 'color', 'r', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Singel-State Model');
legend('Net Adaptation', 'location', 'southeast');
subplot(2, 3, 5);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_g N+nd_g N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_g 2*N 2*N N+nd_g], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_g, 'color', 'r', 'linewidth', 2);
plot(x2_g, '--', 'color', 'b', 'linewidth', 2);
plot(x1_g, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Gain-Specific Model');
legend('Net Adaptation', 'Up State', 'Down State', 'location', 'southeast');
subplot(2, 3, 6);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_m 2*N 2*N N+nd_m], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
% plot(r_m, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_m, 'color', 'r', 'linewidth', 2);
plot(x2_m, '--', 'color', 'b', 'linewidth', 2);
plot(x1_m, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Multi-Rate Model');
legend('Net Adaptation', 'Slow State', 'Fast State', 'location', 'southeast');
sgtitle('Error-Clamp Experiment');

%% part II - Question 3 - Optional:
% Simulation of Symmetric and Assymetric Gain-Specific Model and Comparision 
% with Multi-Rate models in Error-Clamp Experiment

clear x_s x1_g x2_g x_g x1_m x2_m x_m

%initial values
x1_gS(1) = 0; x2_gS(1) = 0; x_gS(1) = 0;   %symmetric gain specific model
x1_gA(1) = 0; x2_gA(1) = 0; x_gA(1) = 0;   %Asymmetric gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;      %multi-rate model 

A = 0.99;
B = 0.013;
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

nd_gS = 17;
nd_gA = 17;
nd_m = 30;

N = 400;
r_gS = [ones(1, N), -ones(1, nd_gS), zeros(1, N-nd_s)];   %symm gain specific
r_gA = [ones(1, N), -ones(1, nd_gA), zeros(1, N-nd_g)];   %asymm gain specific
r_m = [ones(1, N), -ones(1, nd_m), zeros(1, N-nd_m)];     %multi-rate

for n = 1: length(r_gS)-1  %trials
         
    % symm gain specific model output
    if n < N + nd_g
        x1_gS(n+1) = min(0, A * x1_gS(n) + B * (r_gS(n) - x_gS(n)));
        x2_gS(n+1) = max(0, A * x2_gS(n) + B * (r_gS(n) - x_gS(n)));
    else
        x1_gS(n+1) = min(0, A * x1_gS(n));
        x2_gS(n+1) = max(0, A * x2_gS(n));
    end
    x_gS(n+1) = x1_gS(n+1) + x2_gS(n+1);
    
    % Asymm gain specific model output
    if n < N + nd_g
        x1_gA(n+1) = min(0, Af * x1_gA(n) + Bf * (r_gA(n) - x_gA(n)));
        x2_gA(n+1) = max(0, As * x2_gA(n) + Bs * (r_gA(n) - x_gA(n)));
    else
        x1_gA(n+1) = min(0, Af * x1_gA(n));
        x2_gA(n+1) = max(0, As * x2_gA(n));
    end
    x_gA(n+1) = x1_gA(n+1) + x2_gA(n+1);

    %two-state, gain-independent, multi-rate model output
    if n < N + nd_m
        x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x_m(n));
    else
        x1_m(n+1) = Af * x1_m(n);
        x2_m(n+1) = As * x2_m(n);
    end
    x_m(n+1) = x1_m(n+1) + x2_m(n+1);
end


figure;
subplot(2, 4, 1)
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none')
patch([N+nd_m 2*N 2*N N+nd_m], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
xlim([0 2*N])
plot(r_m, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(N + nd_m: 2*N, zeros(1, length(N + nd_m: 2*N)), '--', 'color', 'k', 'linewidth', 3)
ylabel('State'); 
% ylim([-1.1 1.1]);
title('Paradigm for Error-Clamp Experiment');
legend('Adaptation Trials', 'Deadaptation Trials', 'Error-Clamp Trials', ...
    'location', 'southeast');

subplot(2, 4, 2);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_gS N+nd_gS N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_gS 2*N 2*N N+nd_gS], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_gS, 'color', 'r', 'linewidth', 2);
plot(x2_gS, '--', 'color', 'b', 'linewidth', 2);
plot(x1_gS, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Symmetric Gain-Specific Model');
legend('Net Adaptation', 'location', 'southeast');

subplot(2, 4, 3);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_gA N+nd_gA N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_gA 2*N 2*N N+nd_gA], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_gA, 'color', 'r', 'linewidth', 2);
plot(x2_gA, '--', 'color', 'b', 'linewidth', 2);
plot(x1_gA, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Assymetric Gain-Specific Model');
legend('Net Adaptation', 'Up State', 'Down State', 'location', 'southeast');

subplot(2, 4, 4);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_m 2*N 2*N N+nd_m], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_m, 'color', 'r', 'linewidth', 2);
plot(x2_m, '--', 'color', 'b', 'linewidth', 2);
plot(x1_m, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Multi-Rate Model');
legend('Net Adaptation', 'Slow State', 'Fast State', 'location', 'southeast');


clear x_s x1_g x2_g x_g x1_m x2_m x_m

%initial values
x1_gS(1) = 0; x2_gS(1) = 0; x_gS(1) = 0;   %symmetric gain specific model
x1_gA(1) = 0; x2_gA(1) = 0; x_gA(1) = 0;   %Asymmetric gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;      %multi-rate model 

A = 0.99;
B = 0.013;
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

nd_gS = 17;
nd_gA = 17;
nd_m = 30;

N = 400;
r_gS = [-ones(1, N), ones(1, nd_gS), zeros(1, N-nd_s)];   %symm gain specific
r_gA = [-ones(1, N), ones(1, nd_gA), zeros(1, N-nd_g)];   %asymm gain specific
r_m = [-ones(1, N), ones(1, nd_m), zeros(1, N-nd_m)];   %multi-rate

for n = 1: length(r_gS)-1  %trials
         
    % symm gain specific model output
    if n < N + nd_g
        x1_gS(n+1) = min(0, A * x1_gS(n) + B * (r_gS(n) - x_gS(n)));
        x2_gS(n+1) = max(0, A * x2_gS(n) + B * (r_gS(n) - x_gS(n)));
    else
        x1_gS(n+1) = min(0, A * x1_gS(n));
        x2_gS(n+1) = max(0, A * x2_gS(n));
    end
    x_gS(n+1) = x1_gS(n+1) + x2_gS(n+1);
    
    % Asymm gain specific model output
    if n < N + nd_g
        x1_gA(n+1) = min(0, Af * x1_gA(n) + Bf * (r_gA(n) - x_gA(n)));
        x2_gA(n+1) = max(0, As * x2_gA(n) + Bs * (r_gA(n) - x_gA(n)));
    else
        x1_gA(n+1) = min(0, Af * x1_gA(n));
        x2_gA(n+1) = max(0, As * x2_gA(n));
    end
    x_gA(n+1) = x1_gA(n+1) + x2_gA(n+1);

    %two-state, gain-independent, multi-rate model output
    if n < N + nd_m
        x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x_m(n));
    else
        x1_m(n+1) = Af * x1_m(n);
        x2_m(n+1) = As * x2_m(n);
    end
    x_m(n+1) = x1_m(n+1) + x2_m(n+1);
end

subplot(2, 4, 5)
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none')
patch([N+nd_m 2*N 2*N N+nd_m], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
xlim([0 2*N])
plot(r_m, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(N + nd_m: 2*N, zeros(1, length(N + nd_m: 2*N)), '--', 'color', 'k', 'linewidth', 3)
xlabel('Trials'); ylabel('State');
% ylim([-1.1 1.1]);
legend('Adaptation Trials', 'Deadaptation Trials', 'Error-Clamp Trials', ...
    'location', 'southeast');

subplot(2, 4, 6);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_gS N+nd_gS N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_gS 2*N 2*N N+nd_gS], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_gS, 'color', 'r', 'linewidth', 2);
plot(x2_gS, '--', 'color', 'b', 'linewidth', 2);
plot(x1_gS, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Symmetric Gain-Specific Model');
legend('Net Adaptation', 'location', 'northeast');

subplot(2, 4, 7);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_gA N+nd_gA N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_gA 2*N 2*N N+nd_gA], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_gA, 'color', 'r', 'linewidth', 2);
plot(x2_gA, '--', 'color', 'b', 'linewidth', 2);
plot(x1_gA, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Assymetric Gain-Specific Model');
legend('Net Adaptation', 'Up State', 'Down State', 'location', 'northeast');

subplot(2, 4, 8);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_m 2*N 2*N N+nd_m], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
xlim([0 2*N])
plot(x_m, 'color', 'r', 'linewidth', 2);
plot(x2_m, '--', 'color', 'b', 'linewidth', 2);
plot(x1_m, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Multi-Rate Model');
legend('Net Adaptation', 'Slow State', 'Fast State', 'location', 'northeast');
sgtitle('Simulation of Spontaneous Recovery for Symmetric Gain-Specific, Assymetric Gain-Specific, and Multi-Rate Models in the Error-Clamp Experiment'); 




%% Part II - Question 4:

clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;
n_err = 100;
r_s = [ones(1, N), -ones(1, nd_s), zeros(1, n_err), ones(1, N-nd_s-n_err)];   %single state
r_g = [ones(1, N), -ones(1, nd_g), zeros(1, n_err), ones(1, N-nd_g-n_err)];   %gain specific
r_m = [ones(1, N), -ones(1, nd_m), zeros(1, n_err), ones(1, N-nd_m-n_err)];   %multi-rate

for n = 1: length(r_s)-1  %trials
    
    %single state model output
    if n < N + nd_s || n > N + nd_s + n_err
        x_s(n+1) = A * x_s(n) + B * (r_s(n) - x_s(n)); 
    else
        x_s(n+1) = A * x_s(n);
    end
        
    %two state, gain specific model output
    if n < N + nd_g || n > N + nd_g + n_err
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r_g(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r_g(n) - x2_g(n)));
    else
        x1_g(n+1) = min(0, A * x1_g(n));
        x2_g(n+1) = max(0, A * x2_g(n));
    end
    x_g(n+1) = x1_g(n+1) + x2_g(n+1);

    %two-state, gain-independent, multi-rate model output
    if n < N + nd_m || n > N + nd_m + n_err
        x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x2_m(n));
    else
        x1_m(n+1) = Af * x1_m(n);
        x2_m(n+1) = As * x2_m(n);
    end
    x_m(n+1) = x1_m(n+1) + x2_m(n+1);
end


figure;
subplot(2, 3, 2)
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([N N+nd_s N+nd_s N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none')
patch([N+nd_s N+nd_s+n_err N+nd_s+n_err N+nd_s], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none')
patch([N+nd_s+n_err 2*N 2*N N+nd_s+n_err], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none')
xlim([0 2*N])
plot(r_s, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(N + nd_s: N+nd_s+n_err, zeros(1, length(N + nd_s: N+nd_s+n_err)), '--', 'color', ...
    'k', 'linewidth', 3)
xlabel('Trials');
ylabel('State');
legend('Adaptation Trials', 'Deadaptation Trials', 'Error-Clamp Trials', ...
    'Readaptation Trials');
subplot(2, 3, 4);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_s N+nd_s N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_s N+nd_s+n_err N+nd_s+n_err N+nd_s], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none', 'handlevisibility', 'off')
patch([N+nd_s+n_err 2*N 2*N N+nd_s+n_err], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none', 'handlevisibility', 'off')
xlim([0 2*N])
% plot(r_s, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_s, 'color', 'r', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Singel-State Model');
legend('Net Adaptation', 'location', 'southeast');
subplot(2, 3, 5);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_g N+nd_g N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_g N+nd_g+n_err N+nd_g+n_err N+nd_g], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none', 'handlevisibility', 'off')
patch([N+nd_g+n_err 2*N 2*N N+nd_g+n_err], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none', 'handlevisibility', 'off')
xlim([0 2*N])
% plot(r_g, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_g, 'color', 'r', 'linewidth', 2);
plot(x2_g, '--', 'color', 'b', 'linewidth', 2);
plot(x1_g, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Gain-Specific Model');
legend('Net Adaptation', 'Up State', 'Down State', 'location', 'southeast');
subplot(2, 3, 6);
patch([0 N N 0], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
hold on;
patch([N N+nd_m N+nd_m N], [-1 -1 1 1], [1 0 0], 'FaceAlpha', 0.2, 'edgecolor', 'none'...
    , 'handlevisibility', 'off')
patch([N+nd_m N+nd_m+n_err N+nd_m+n_err N+nd_m], [-1 -1 1 1], [0 1 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none', 'handlevisibility', 'off')
patch([N+nd_m+n_err 2*N 2*N N+nd_m+n_err], [-1 -1 1 1], [0 0 0], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none', 'handlevisibility', 'off')
xlim([0 2*N])
% plot(r_m, 'color', 'k', 'linewidth', 1.5, 'handlevisibility', 'off');
plot(x_m, 'color', 'r', 'linewidth', 2);
plot(x2_m, '--', 'color', 'b', 'linewidth', 2);
plot(x1_m, '--', 'color', 'g', 'linewidth', 2);
xlabel('Trial Number');
ylabel('Adaptation');
title('Multi-Rate Model');
legend('Net Adaptation', 'Slow State', 'Fast State', 'location', 'southeast');
sgtitle('Error-Clamp/Relearning Experiment');


clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;
n_err = 100;
r_s = [ones(1, N), -ones(1, nd_s), zeros(1, n_err), ones(1, N-nd_s-n_err)];   %single state
r_g = [ones(1, N), -ones(1, nd_g), zeros(1, n_err), ones(1, N-nd_g-n_err)];   %gain specific
r_m = [ones(1, N), -ones(1, nd_m), zeros(1, n_err), ones(1, N-nd_m-n_err)];   %multi-rate

for n = 1: length(r_s)-1  %trials
    
    %single state model output
    if n < N + nd_s || n > N + nd_s + n_err
        x_s(n+1) = A * x_s(n) + B * (r_s(n) - x_s(n)); 
    else
        x_s(n+1) = A * x_s(n);
    end
        
    %two state, gain specific model output
    if n < N + nd_g || n > N + nd_g + n_err
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r_g(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r_g(n) - x2_g(n)));
    else
        x1_g(n+1) = min(0, A * x1_g(n));
        x2_g(n+1) = max(0, A * x2_g(n));
    end
    x_g(n+1) = x1_g(n+1) + x2_g(n+1);

    %two-state, gain-independent, multi-rate model output
    if n < N + nd_m || n > N + nd_m + n_err
        x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x2_m(n));
    else
        x1_m(n+1) = Af * x1_m(n);
        x2_m(n+1) = As * x2_m(n);
    end
    x_m(n+1) = x1_m(n+1) + x2_m(n+1);
end



%% Part II - Question 5:
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m

%input
N = 50;                         % # of null trials, deadaptation
N_ia_vec = [150, 300, 600];     % # of initial adaptation trials

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

% creating output
for iN_ia = 1: length(N_ia_vec)
    r = [zeros(1, N), ones(1, N_ia_vec(iN_ia)), -ones(1, 15*N)];
    for n = 1: length(r)-1  %trials
        
        %single state model output
        x_s(n+1) = A * x_s(n) + B * (r(n) - x_s(n));

        %two state, gain specific model output
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r(n) - x2_g(n)));
        x_g(n+1) = x1_g(n+1) + x2_g(n+1);

        %two-state, gain-independent, multi-rate model output
        x1_m(n+1) = Af * x1_m(n) + Bf * (r(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r(n) - x2_m(n));
        x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        
    end
    
    X_s{iN_ia} = x_s;
    X_g{iN_ia} = x_g;
    X_m{iN_ia} = x_m;
    
end

figure;
subplot(4, 4, [5 9])
patch([50 650 650 50], [-1.2 -1.2 1.2 1.2], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([650 650+15*N 650+15*N 650], [-1.2 -1.2 1.2 1.2], [1 0 0], 'FaceAlpha', 0.2, ...
    'edgecolor', 'none')
plot(r, 'k', 'linewidth', 1.5);
xlim([0 length(r)]);
ylim([-1.2 1.2]);
ylabel('Disturbance');
xlabel('Trial Number');
title('Paradigm');

subplot(4, 4, [2 6])
plot(X_s{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_s{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_s{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_s{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Single-State Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, [3 7])
plot(X_g{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_g{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_g{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_g{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Gain-Specific Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, [4 8])
plot(X_m{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_m{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_m{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_m{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Multi-Rate Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, [10 14]);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_s{i}(N + N_ia_vec(i):end);
    idx = find(temp < 0, 1);
    temp = -temp(idx:end);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_s{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1);
xlabel('Trial Number');

subplot(4, 4, [11 15]);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_g{i}(N + N_ia_vec(i):end);
    idx = find(temp < 0, 1);
    temp = -temp(idx:end);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_g{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1);
xlabel('Trial Number');

subplot(4, 4, [12 16]);
c = ['r', 'g', 'c'];
clear temp
for i = 1: 3
    temp{i} = X_m{i}(N + N_ia_vec(i):end);
    idx = find(temp{i} < 0, 1);
    temp{i} = -temp{i}(idx:end);
    plot(temp{i}, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_m{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
xlabel('Trial Number');
yline(0, '--', 'color', 'k', 'linewidth', 1, 'handlevisibility', 'off');
legend('Opposite FF (150 Trials)', 'Opposite FF (300 Trials)', ...
    'Opposite FF (600 Trials)', 'Original Adaptation', 'Location', 'SouthEast');
sgtitle('Anterograde Interference Simulation');



%% Part II - Question 6:
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%input
N = 50;                         % # of null trials, deadaptation
N_ia_vec = [300, 500, 700];       % # of initial adaptation trials

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

for iN_ia = 1: length(N_ia_vec)
    r = [zeros(1, N), ones(1, N_ia_vec(iN_ia)), zeros(1, 15*N)];
    for n = 1: length(r)-1  %trials
        
        %single state model output
        x_s(n+1) = A * x_s(n) + B * (r(n) - x_s(n));

        %two state, gain specific model output
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r(n) - x2_g(n)));
        x_g(n+1) = x1_g(n+1) + x2_g(n+1);

        %two-state, gain-independent, multi-rate model output
        x1_m(n+1) = Af * x1_m(n) + Bf * (r(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r(n) - x2_m(n));
        x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        
    end
    
    X_s{iN_ia} = x_s;
    X_g{iN_ia} = x_g;
    X_m{iN_ia} = x_m;
    
end


figure;
subplot(4, 4, [1 5])
patch([N N_ia_vec(end)+N N_ia_vec(end)+N N], [-1.2 -1.2 1.2 1.2], [0 0 0], ...
    'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
plot(r, 'k', 'linewidth', 1.5);
xlim([0 length(r)]);
ylim([-1.2 1.2]);
ylabel('Deadaptation Simulation');
title('Paradigm');


subplot(4, 4, 2)
plot(X_s{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_s{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_s{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_s{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Single-State Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 3)
plot(X_g{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_g{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_g{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_g{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Gain-Specific Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 4)
plot(X_m{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_m{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_m{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_m{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Multi-Rate Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 6);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_s{i}(N + N_ia_vec(i):end);
    temp = -temp + temp(1);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_s{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 7);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_g{i}(N + N_ia_vec(i):end);
    temp = -temp + temp(1);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_g{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 8);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_m{i}(N + N_ia_vec(i):end);
    temp = -temp + temp(1);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_m{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1, 'handlevisibility', 'off');
legend('Deadaptation (300 Trials)', 'Deadaptation (500 Trials)', ...
    'Deadaptation (700 Trials)', 'Original Adaptation', 'Location', 'SouthEast');


clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%input
N = 50;                           % # of null trials, deadaptation
N_ia_vec = [300, 500, 700];       % # of initial adaptation trials

%initial values
x_s(1) = 0;                             %single state model
x1_g(1) = 0; x2_g(1) = 0; x_g(1) = 0;   %gain specific model
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

for iN_ia = 1: length(N_ia_vec)
    r = [zeros(1, N), ones(1, N_ia_vec(iN_ia)), 0.3*ones(1, 15*N)];
    for n = 1: length(r)-1  %trials
        
        %single state model output
        x_s(n+1) = A * x_s(n) + B * (r(n) - x_s(n));

        %two state, gain specific model output
        x1_g(n+1) = min(0, A * x1_g(n) + B * (r(n) - x1_g(n)));
        x2_g(n+1) = max(0, A * x2_g(n) + B * (r(n) - x2_g(n)));
        x_g(n+1) = x1_g(n+1) + x2_g(n+1);

        %two-state, gain-independent, multi-rate model output
        x1_m(n+1) = Af * x1_m(n) + Bf * (r(n) - x1_m(n));
        x2_m(n+1) = As * x2_m(n) + Bs * (r(n) - x2_m(n));
        x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        
    end
    
    X_s{iN_ia} = x_s;
    X_g{iN_ia} = x_g;
    X_m{iN_ia} = x_m;
    
end

% figure;
subplot(4, 4, [9 13])
patch([N N_ia_vec(end)+N N_ia_vec(end)+N N], [-1.2 -1.2 1.2 1.2], [0 0 0], ...
    'FaceAlpha', 0.1, 'edgecolor', 'none')
hold on;
patch([N_ia_vec(end)+N N_ia_vec(end)+N+15*N N_ia_vec(end)+N+15*N N_ia_vec(end)+N], ...
    [-1.2 -1.2 1.2 1.2], [0 0 1], 'FaceAlpha', 0.1, ...
    'edgecolor', 'none')
plot(r, 'k', 'linewidth', 1.5);
xlim([0 length(r)]);
ylim([-1.2 1.2]);
ylabel('Down-Scaling Simulation');
xlabel('Trial Number');

subplot(4, 4, 10)
plot(X_s{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_s{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_s{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_s{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Single-State Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 11)
plot(X_g{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_g{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_g{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_g{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Gain-Specific Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 12)
plot(X_m{1}, '--', 'color', 'r', 'linewidth', 2); hold on; 
plot(X_m{2}, '--', 'color', 'g', 'linewidth', 2);
plot(X_m{3}, '--', 'color', 'c', 'linewidth', 2);
plot(X_m{3}(1: N + N_ia_vec(3)), 'b', 'linewidth', 2);
title('Multi-Rate Model');
ylabel('Adaptation');
yline(0, '--', 'color', 'k', 'linewidth', 1);

subplot(4, 4, 14);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_s{i}(N + N_ia_vec(i):end);
    temp = -temp + temp(1);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_s{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1);
xlabel('Trial Number');

subplot(4, 4, 15);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_g{i}(N + N_ia_vec(i):end);
    temp = -temp + temp(1);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_g{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
yline(0, '--', 'color', 'k', 'linewidth', 1);
xlabel('Trial Number');

subplot(4, 4, 16);
c = ['r', 'g', 'c'];
for i = 1: 3
    temp = X_m{i}(N + N_ia_vec(i):end);
    temp = -temp + temp(1);
    plot(temp, '--', 'color', c(i), 'linewidth', 2.2);
    hold on;
end
plot(X_m{3}(N: N+N_ia_vec(3)), 'b', 'linewidth', 0.5);
title('Direct Performance Comparison');
ylim([-0.1 0.8]);
xlabel('Trial Number');
yline(0, '--', 'color', 'k', 'linewidth', 1, 'handlevisibility', 'off');
legend('Down Scaling (300 Trials)', 'Down Scaling (500 Trials)', ...
    'Down Scaling (700 Trials)', 'Original Adaptation', 'Location', 'SouthEast');




%% Part II - Question 7:
clear; clc;
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

% 1) 1-A2 vs. 1-A1
%initial values
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;

A1 = ones(1,10)-10.^(linspace(log10(0.047),log10(0.94),10));    %   Fast decay factor
A2 = ones(1,10)-10.^(linspace(log10(0.0008),log10(0.08),10));    %   Slow decay factor
B1 = 10.^linspace(log10(0.021),log10(0.84),10);  %   Fast learning rate
B2 = 10.^linspace(log10(0.0018),log10(0.18),10);  %   Slow learning rate
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

figure;
subplot(3,2,1)
for i1 = 1: 1: length(A1)
    for i2 = 1: 1: length(A2)
        r_m = zeros(1, 2*N);
        x1_m = zeros(1, 2*N);
        x2_m = zeros(1, 2*N);
        x_m = zeros(1, 2*N);
        
        flag = 0; n3m = 2*N;
        
        for n = 1: 2*N  %trials
            if n <= N
                r_m(n) = 1;
            end
            if n > N && x_m(n) >= 0.02 && ~flag
                r_m(n) = -1;       
            end
            if n > N && x_m(n) < 0.02 && ~flag
                r_m(n) = 0; flag = 1; n3m = n;
            end           
            %multi-rate model output
            if n < n3m
                x1_m(n+1) = A1(i1) * x1_m(n) + Bf * (r_m(n) - x_m(n));
                x2_m(n+1) = A2(i2) * x2_m(n) + Bs * (r_m(n) - x_m(n));
            else
                x1_m(n+1) = A1(i1) * x1_m(n);
                x2_m(n+1) = A2(i2) * x2_m(n);
            end
            x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        end    
%         plot(r_m, 'k'); hold on; plot(x_m, 'r'); plot(x1_m, 'g'); plot(x2_m, 'b');
        nd_m = find(r_m == 0, 1) - N;
        matAsAf(i1, i2) = max(x_m(N+nd_m: end))/max(x_m(1: N));
    end
end
% figure;
h = heatmap(1.-A1,1.-A2, matAsAf,'colormap',jet,'CellLabelColor','none');
h.YDisplayData = flipud(h.YDisplayData);
h.XLabel = '1 - A1';    h.YLabel = '1 - A2';
h.GridVisible = 'off';

% 2) B2 vs. B1
clear; clc;
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%initial values
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;

% 1) 1-Af vs. 1-As:

A1 = ones(1,10)-10.^(linspace(log10(0.047),log10(0.94),10));    %   Fast decay factor
A2 = ones(1,10)-10.^(linspace(log10(0.0008),log10(0.08),10));    %   Slow decay factor
B1 = 10.^linspace(log10(0.021),log10(0.84),10);  %   Fast learning rate
B2 = 10.^linspace(log10(0.0018),log10(0.18),10);  %   Slow learning rate
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

subplot(3,2,2)
for i1 = 1: 1: length(B1)
    for i2 = 1: 1: length(B2)   
        r_m = zeros(1, 2*N);
        x1_m = zeros(1, 2*N);
        x2_m = zeros(1, 2*N);
        x_m = zeros(1, 2*N);
        
        flag = 0; n3m = 2*N;
        
        for n = 1: 2*N  %trials
            if n <= N
                r_m(n) = 1;
            end
            if n > N && x_m(n) >= 0.02 && ~flag
                r_m(n) = -1;       
            end
            if n > N && x_m(n) < 0.02 && ~flag
                r_m(n) = 0; flag = 1; n3m = n;
            end           
            %multi-rate model output
            if n < n3m
                x1_m(n+1) = Af * x1_m(n) + B1(i1) * (r_m(n) - x_m(n));
                x2_m(n+1) = As * x2_m(n) + B2(i2) * (r_m(n) - x_m(n));
            else
                x1_m(n+1) = Af * x1_m(n);
                x2_m(n+1) = As * x2_m(n);
            end
            x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        end    
%         plot(r_m, 'k'); hold on; plot(x_m, 'r'); plot(x1_m, 'g'); plot(x2_m, 'b');
        nd_m = find(r_m == 0, 1) - N;
        matAsAf(i1, i2) = max(x_m(N+nd_m: end))/max(x_m(1: N));
    end
end
% figure;
h = heatmap(B1,B2, matAsAf,'colormap',jet,'CellLabelColor','none');
h.YDisplayData = flipud(h.YDisplayData);
h.XLabel = 'B1';    h.YLabel = 'B2';
h.GridVisible = 'off';

% 3) B1 vs. 1-A1
clear; clc;
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%initial values
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;

A1 = ones(1,10)-10.^(linspace(log10(0.047),log10(0.94),10));    %   Fast decay factor
A2 = ones(1,10)-10.^(linspace(log10(0.0008),log10(0.08),10));    %   Slow decay factor
B1 = 10.^linspace(log10(0.021),log10(0.84),10);  %   Fast learning rate
B2 = 10.^linspace(log10(0.0018),log10(0.18),10);  %   Slow learning rate
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

subplot(3,2,3)
for iAs = 1: 1: length(A1)
    for iAf = 1: 1: length(B1)   
        r_m = zeros(1, 2*N);
        x1_m = zeros(1, 2*N);
        x2_m = zeros(1, 2*N);
        x_m = zeros(1, 2*N);
        
        flag = 0; n3m = 2*N;
        
        for n = 1: 2*N  %trials
            if n <= N
                r_m(n) = 1;
            end
            if n > N && x_m(n) >= 0.02 && ~flag
                r_m(n) = -1;       
            end
            if n > N && x_m(n) < 0.02 && ~flag
                r_m(n) = 0; flag = 1; n3m = n;
            end           
            %multi-rate model output
            if n < n3m
                x1_m(n+1) = A1(iAs) * x1_m(n) + B1(iAf) * (r_m(n) - x_m(n));
                x2_m(n+1) = As * x2_m(n) + Bs * (r_m(n) - x_m(n));
            else
                x1_m(n+1) = A1(iAs) * x1_m(n);
                x2_m(n+1) = As * x2_m(n);
            end
            x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        end    
%         plot(r_m, 'k'); hold on; plot(x_m, 'r'); plot(x1_m, 'g'); plot(x2_m, 'b');
        nd_m = find(r_m == 0, 1) - N;
        matAsAf(iAs, iAf) = max(x_m(N+nd_m: end))/max(x_m(1: N));
    end
end
% figure;
h = heatmap(1.-A1,B1, matAsAf,'colormap',jet,'CellLabelColor','none');
h.YDisplayData = flipud(h.YDisplayData);
h.XLabel = '1-A1';    h.YLabel = 'B1';
h.GridVisible = 'off';

% 4) B2 vs. 1-A2
clear; clc;
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%initial values
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;

A1 = ones(1,10)-10.^(linspace(log10(0.047),log10(0.94),10));    %   Fast decay factor
A2 = ones(1,10)-10.^(linspace(log10(0.0008),log10(0.08),10));    %   Slow decay factor
B1 = 10.^linspace(log10(0.021),log10(0.84),10);  %   Fast learning rate
B2 = 10.^linspace(log10(0.0018),log10(0.18),10);  %   Slow learning rate
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

subplot(3,2,4)
for i1 = 1: 1: length(A2)
    for i2 = 1: 1: length(B2)   
        r_m = zeros(1, 2*N);
        x1_m = zeros(1, 2*N);
        x2_m = zeros(1, 2*N);
        x_m = zeros(1, 2*N);
        
        flag = 0; n3m = 2*N;
        
        for n = 1: 2*N  %trials
            if n <= N
                r_m(n) = 1;
            end
            if n > N && x_m(n) >= 0.02 && ~flag
                r_m(n) = -1;       
            end
            if n > N && x_m(n) < 0.02 && ~flag
                r_m(n) = 0; flag = 1; n3m = n;
            end           
            %multi-rate model output
            if n < n3m
                x1_m(n+1) = Af * x1_m(n) + Bf * (r_m(n) - x_m(n));
                x2_m(n+1) = A2(i1) * x2_m(n) + B2(i2) * (r_m(n) - x_m(n));
            else
                x1_m(n+1) = Af * x1_m(n);
                x2_m(n+1) = A2(i1) * x2_m(n);
            end
            x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        end    
%         plot(r_m, 'k'); hold on; plot(x_m, 'r'); plot(x1_m, 'g'); plot(x2_m, 'b');
        nd_m = find(r_m == 0, 1) - N;
        matAsAf(i1, i2) = max(x_m(N+nd_m: end))/max(x_m(1: N));
    end
end
% figure;
h = heatmap(1.-A2,B2, matAsAf,'colormap',jet,'CellLabelColor','none');
h.YDisplayData = flipud(h.YDisplayData);
h.XLabel = '1-A2';    h.YLabel = 'B2';
h.GridVisible = 'off';


% 5) B2 vs. 1-A1
clear; clc;
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%initial values
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;

A1 = ones(1,10)-10.^(linspace(log10(0.047),log10(0.94),10));    %   Fast decay factor
A2 = ones(1,10)-10.^(linspace(log10(0.0008),log10(0.08),10));    %   Slow decay factor
B1 = 10.^linspace(log10(0.021),log10(0.84),10);  %   Fast learning rate
B2 = 10.^linspace(log10(0.0018),log10(0.18),10);  %   Slow learning rate
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

subplot(3,2,5)
for i1 = 1: 1: length(A1)
    for i2 = 1: 1: length(B2)   
        r_m = zeros(1, 2*N);
        x1_m = zeros(1, 2*N);
        x2_m = zeros(1, 2*N);
        x_m = zeros(1, 2*N);
        
        flag = 0; n3m = 2*N;
        
        for n = 1: 2*N  %trials
            if n <= N
                r_m(n) = 1;
            end
            if n > N && x_m(n) >= 0.02 && ~flag
                r_m(n) = -1;       
            end
            if n > N && x_m(n) < 0.02 && ~flag
                r_m(n) = 0; flag = 1; n3m = n;
            end           
            %multi-rate model output
            if n < n3m
                x1_m(n+1) = A1(i1) * x1_m(n) + Bf * (r_m(n) - x_m(n));
                x2_m(n+1) = As * x2_m(n) + B2(i2) * (r_m(n) - x_m(n));
            else
                x1_m(n+1) = Af * x1_m(n);
                x2_m(n+1) = As * x2_m(n);
            end
            x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        end    
%         plot(r_m, 'k'); hold on; plot(x_m, 'r'); plot(x1_m, 'g'); plot(x2_m, 'b');
        nd_m = find(r_m == 0, 1) - N;
        matAsAf(i1, i2) = max(x_m(N+nd_m: end))/max(x_m(1: N));
    end
end
% figure;
h = heatmap(1.-A1,B2, matAsAf,'colormap',jet,'CellLabelColor','none');
h.YDisplayData = flipud(h.YDisplayData);
h.XLabel = '1-A1';    h.YLabel = 'B2';
h.GridVisible = 'off';

% 6) B1 vs. 1-A2
clear; clc;
clear x_s x1_g x2_g x_g x1_m x2_m x_m r_s r_g r_m r X_s X_g X_m

%initial values
x1_m(1) = 0; x2_m(1) = 0; x_m(1) = 0;   %multi-rate model 

N = 400;
A1 = ones(1,10)-10.^(linspace(log10(0.047),log10(0.94),10));    %   Fast decay factor
A2 = ones(1,10)-10.^(linspace(log10(0.0008),log10(0.08),10));    %   Slow decay factor
B1 = 10.^linspace(log10(0.021),log10(0.84),10);  %   Fast learning rate
B2 = 10.^linspace(log10(0.0018),log10(0.18),10);  %   Slow learning rate
Af = 0.92;
As = 0.996;
Bf = 0.03;
Bs = 0.004;

subplot(3,2,6)
for i1 = 1: 1: length(A2)
    for i2 = 1: 1: length(B1)   
        r_m = zeros(1, 2*N);
        x1_m = zeros(1, 2*N);
        x2_m = zeros(1, 2*N);
        x_m = zeros(1, 2*N);
        
        flag = 0; n3m = 2*N;
        
        for n = 1: 2*N  %trials
            if n <= N
                r_m(n) = 1;
            end
            if n > N && x_m(n) >= 0.02 && ~flag
                r_m(n) = -1;       
            end
            if n > N && x_m(n) < 0.02 && ~flag
                r_m(n) = 0; flag = 1; n3m = n;
            end           
            
            
            % man
            
%             if it<=n3m, e_tmp = (1,it-1) - mx(it-1);   end   %   Error
%                 mx1(it) = Af*mx1(it-1) + B1(ib1)*e(1,it-1);  %   Fast state
%                 mx2(it) = A2(ia2)*mx2(it-1) + Bs*e(1,it-1);  %   Slow state
%                 mx(it) = mx1(it) + mx2(it);
%             
            % man
            
            %multi-rate model output
            if n < n3m
                x1_m(n+1) = Af * x1_m(n) + B1(i2) * (r_m(n) - x_m(n));
                x2_m(n+1) = A2(i1) * x2_m(n) + Bs * (r_m(n) - x_m(n));
            else
                x1_m(n+1) = Af * x1_m(n);
                x2_m(n+1) = A2(i1) * x2_m(n);
            end
            x_m(n+1) = x1_m(n+1) + x2_m(n+1);
        end    
%         plot(r_m, 'k'); hold on; plot(x_m, 'r');
        nd_m = find(r_m == 0, 1) - N;
        matAsAf(i1, i2) = max(x_m(N+nd_m: end))/max(x_m(1: N));
    end
end
% figure;
h = heatmap(1-A2,B1, matAsAf,'colormap',jet,'CellLabelColor','none');
h.YDisplayData = flipud(h.YDisplayData);
h.XLabel = '1-A2';    h.YLabel = 'B1';
h.GridVisible = 'off';


%% Part III - Question 1:
clear all; clc;

a = 1;
beta = 0.001;
e_pref = linspace(-5,5,10);
sigma = 1;
w0 = 0.05*ones(1,10);
trlNum = 101;
paradigm = zeros(1,trlNum);
paradigm(21:60) = 1;
[x,e,wVec,etaVec,gVec] = HarzfeldModel(a,beta,e_pref,sigma,w0,paradigm);

figure;
subplot(1,2,1)
plot(0:trlNum-1,paradigm,'k','linewidth',1.5)
hold on
plot(0:trlNum-1,x,'linewidth',1.5,'color',[0.7,0.7,0.7])
xlim([0,100])
ylim([-0.1,1.1])
title("Black: Paradigm , Gray: Estimation")
xlabel("Trial")


paradigm = zeros(1,trlNum);
paradigm(11:30) = 1;
paradigm(31:60) = 0.5;
[x,e,wVec,etaVec,gVec] = HarzfeldModel(a,beta,e_pref,sigma,w0,paradigm);

subplot(1,2,2)
plot(0:trlNum-1,paradigm,'k','linewidth',1.5)
hold on
plot(0:trlNum-1,x,'linewidth',1.5,'color',[0.7,0.7,0.7])
xlim([0,100])
ylim([-0.1,1.1])
title("Black: Paradigm , Gray: Estimation")
xlabel("Trial")


%% Part III - Question 2:
clear all; clc;
rng shuffle

% Z is the probability of staying in the current state
% Slow pardigm Z = 0.9
a = 1;
beta = 0.001;
e_pref = linspace(-5,5,10);
sigma = 1;
w0 = 0.05*ones(1,10);
trlNum = 101;

% Building paradigms
Z = 0.9;
slowParadigm = parad(trlNum,Z);
Z = 0.5;
mediumParadigm = parad(trlNum,Z);
Z = 0.1;
fastParadigm = parad(trlNum,Z);

% Simulation
[x0,e0,wVec0,etaVec0,gVec0] = HarzfeldModel(a,beta,e_pref,sigma,w0,slowParadigm);
[x1,e1,wVec1,etaVec1,gVec1] = HarzfeldModel(a,beta,e_pref,sigma,w0,mediumParadigm);
[x2,e2,wVec2,etaVec2,gVec2] = HarzfeldModel(a,beta,e_pref,sigma,w0,fastParadigm);

figure;
subplot(3,1,1)
plot(0:trlNum-1,slowParadigm,'k','linewidth',1.5)
hold on
plot(0:trlNum-1,x0,'linewidth',1.5,'color',[0.7,0.7,0.7])
xlim([0,100])
ylim([-0.1,1.1])
title("Z = 0.9 , Black: Paradigm , Gray: Estimation")
xlabel("Trial")

subplot(3,1,2)
plot(0:trlNum-1,mediumParadigm,'k','linewidth',1.5)
hold on
plot(0:trlNum-1,x1,'linewidth',1.5,'color',[0.7,0.7,0.7])
xlim([0,100])
ylim([-0.1,1.1])
title("Z = 0.5 , Black: Paradigm , Gray: Estimation")
xlabel("Trial")

subplot(3,1,3)
plot(0:trlNum-1,fastParadigm,'k','linewidth',1.5)
hold on
plot(0:trlNum-1,x2,'linewidth',1.5,'color',[0.7,0.7,0.7])
xlim([0,100])
ylim([-0.1,1.1])
title("Z = 0.1 , Black: Paradigm , Gray: Estimation")
xlabel("Trial")

%% Part III - Question 3 - Slow Paradigm
clear all; clc;
rng shuffle

% Parameters
a = 1;
beta = 0.001;
e_pref = linspace(-1,1,10);
sigma = 0.1;
w0 = 0.05*ones(1,10);
trlNum = 101;
Z = 0.9;
slowParadigm = parad(trlNum,Z);

% Simulation
[x,e,wVec,etaVec,gVec] = HarzfeldModel(a,beta,e_pref,sigma,w0,slowParadigm);

% basis functions
basisNum = length(e_pref);
n = 200;
errorVec = linspace(-1.3,1.3,n);
g = zeros(basisNum,trlNum,n);
for i = 1:trlNum
    for j = 1:basisNum
        g(j,i,:) = wVec(j,i)*exp(-(errorVec - e_pref(j)).^2/(2*sigma^2));
    end
end

% plot
eta = reshape(sum(g,1),trlNum,n);
h = figure;
h.Position = [50 50 1600 700];
Demo = VideoWriter('Demo_Z0.9'); 
Demo.FrameRate = 5;  
open(Demo)
for i = 1:trlNum
    subplot(4,1,1)
    plot(0:i-1,slowParadigm(1:i),'k','linewidth',1.5)
    hold on
    plot(0:i-1,x(1:i),'linewidth',1.5,'color',[0.7,0.7,0.7])
    hold on
    xlim([0,100])
    ylim([0,1.1])
    title("Z = 0.9 , Black: Paradigm , Gray: Estimation")
    xlabel("Trial")
    
    subplot(4,1,2)
    plot(0:i-1,e(1:i),'k','linewidth',1.5)
    xlim([0,100])
    ylim([-1.05,1.05])
    xlabel("Trial")
    title("Error Value")
    ylabel("error")
    
    subplot(4,1,[3,4])
    hold off
    for j = 1:basisNum
        plot(errorVec,reshape(g(j,i,:),1,n),'--')
        hold on
    end
    plot(errorVec,eta(i,:),'k','linewidth',1.5)
    ylim([0,0.1])
    xlim([errorVec(1),errorVec(end)])
    title("eta and basis functions")
    xlabel("error")
    ylabel("error-sensetivity")
    frame = getframe(gcf); 
    writeVideo(Demo, frame);
end
close(Demo)

%% Part III - Question 3 - Fast Paradigm
clear all; clc;
rng shuffle

% Parameters
a = 1;
beta = 0.001;
e_pref = linspace(-1,1,10);
sigma = 0.1;
w0 = 0.05*ones(1,10);
trlNum = 101;
Z = 0.1;
slowParadigm = parad(trlNum,Z);

% Simulation
[x,e,wVec,etaVec,gVec] = HarzfeldModel(a,beta,e_pref,sigma,w0,slowParadigm);

% basis functions
basisNum = length(e_pref);
n = 200;
errorVec = linspace(-1.3,1.3,n);
g = zeros(basisNum,trlNum,n);
for i = 1:trlNum
    for j = 1:basisNum
        g(j,i,:) = wVec(j,i)*exp(-(errorVec - e_pref(j)).^2/(2*sigma^2));
    end
end

% plot
eta = reshape(sum(g,1),trlNum,n);
h = figure;
h.Position = [50 50 1600 700];
Demo = VideoWriter('Demo_Z0.1'); 
Demo.FrameRate = 5;  
open(Demo)
for i = 1:trlNum
    subplot(4,1,1)
    plot(0:i-1,slowParadigm(1:i),'k','linewidth',1.5)
    hold on
    plot(0:i-1,x(1:i),'linewidth',1.5,'color',[0.7,0.7,0.7])
    hold on
    xlim([0,100])
    ylim([0,1.1])
    title("Z = 0.1 , Black: Paradigm , Gray: Estimation")
    xlabel("Trial")
    
    subplot(4,1,2)
    plot(0:i-1,e(1:i),'k','linewidth',1.5)
    xlim([0,100])
    ylim([-1.05,1.05])
    xlabel("Trial")
    title("Error Value")
    ylabel("error")
    
    subplot(4,1,[3,4])
    hold off
    for j = 1:basisNum
        plot(errorVec,reshape(g(j,i,:),1,n),'--')
        hold on
    end
    plot(errorVec,eta(i,:),'k','linewidth',1.5)
    ylim([0,0.1])
    xlim([errorVec(1),errorVec(end)])
    title("eta and basis functions")
    xlabel("error")
    ylabel("error-sensetivity")
    frame = getframe(gcf); 
    writeVideo(Demo, frame);
end
close(Demo)

%% Part III - Optional 1 - Effect of Z on eta
clear all; clc;
rng shuffle

% Parameters
a = 1;
beta = 0.001;
e_pref = linspace(-1,1,10);
sigma = 0.1;
w0 = 0.05*ones(1,10);
trlNum = 101;

iteration = 200;
eta1 = zeros(iteration,trlNum);
eta2 = zeros(iteration,trlNum);
eta3 = zeros(iteration,trlNum);
eta4 = zeros(iteration,trlNum);
eta5 = zeros(iteration,trlNum);
for it = 1:iteration
    % Building paradigms
    Z = 0.9;
    slowParadigm = parad(trlNum,Z);
    Z = 0.7;
    medium01Paradigm = parad(trlNum,Z);
    Z = 0.5;
    medium02Paradigm = parad(trlNum,Z);
    Z = 0.3;
    medium03Paradigm = parad(trlNum,Z);
    Z = 0.1;
    fastParadigm = parad(trlNum,Z);

    % Simulation
    [x0,e0,wVec0,etaVec0,gVec0] = HarzfeldModel(a,beta,e_pref,sigma,w0,slowParadigm);
    [x1,e1,wVec1,etaVec1,gVec1] = HarzfeldModel(a,beta,e_pref,sigma,w0,medium01Paradigm);
    [x2,e2,wVec2,etaVec2,gVec2] = HarzfeldModel(a,beta,e_pref,sigma,w0,medium02Paradigm);
    [x3,e3,wVec3,etaVec3,gVec3] = HarzfeldModel(a,beta,e_pref,sigma,w0,medium03Paradigm);
    [x4,e4,wVec4,etaVec4,gVec4] = HarzfeldModel(a,beta,e_pref,sigma,w0,fastParadigm);
    
    eta1(it,:) = etaVec0;
    eta2(it,:) = etaVec1;
    eta3(it,:) = etaVec2;
    eta4(it,:) = etaVec3;
    eta5(it,:) = etaVec4;
end

% plot
figure;
hold all;
errorbar(0:trlNum-1,mean(eta1(:,1:end),1),std(eta1(:,1:end),1)/sqrt(iteration),...
    'k','LineStyle','none','HandleVisibility','off');
scatter(0:trlNum-1,mean(eta1,1),20,'k','filled','linewidth',1,'HandleVisibility','off')
plot(0:trlNum-1,mean(eta1,1),'k','linewidth',0.3)

errorbar(0:trlNum-1,mean(eta2(:,1:end),1),std(eta2(:,1:end),1)/sqrt(iteration),...
    'color','#0072BD','LineStyle','none','HandleVisibility','off');
scatter(0:trlNum-1,mean(eta2,1),20,'filled','MarkerFaceColor',[0 0.4470 0.7410],'HandleVisibility','off')
plot(0:trlNum-1,mean(eta2,1),'color','#0072BD','linewidth',0.3)

errorbar(0:trlNum-1,mean(eta3(:,1:end),1),std(eta3(:,1:end),1)/sqrt(iteration),...
    'color','#D95319','LineStyle','none','HandleVisibility','off');
scatter(0:trlNum-1,mean(eta3,1),20,'filled','MarkerFaceColor',[0.8500 0.3250 0.0980],'HandleVisibility','off')
plot(0:trlNum-1,mean(eta3,1),'color','#D95319','linewidth',0.3)

errorbar(0:trlNum-1,mean(eta4(:,1:end),1),std(eta4(:,1:end),1)/sqrt(iteration),...
    'color','#EDB120','LineStyle','none','HandleVisibility','off');
scatter(0:trlNum-1,mean(eta4,1),20,'filled','MarkerFaceColor',[0.9290 0.6940 0.1250],'HandleVisibility','off')
plot(0:trlNum-1,mean(eta4,1),'color','#EDB120','linewidth',0.3)

plot(0:trlNum-1,mean(eta5,1),'color','#7E2F8E','linewidth',0.3)
errorbar(0:trlNum-1,mean(eta5(:,1:end),1),std(eta5(:,1:end),1)/sqrt(iteration),...
    'color','#7E2F8E','LineStyle','none');
scatter(0:trlNum-1,mean(eta5,1),20,'filled','MarkerFaceColor',[0.4940 0.1840 0.5560],'HandleVisibility','off')

title("Effect of Z on error-sensetivity ($\eta$)",'interpreter','latex')
xlabel("Trial")
ylabel("$\eta$",'interpreter','latex')
xlim([1,100])
legend("$Z$=0.9","$Z$=0.7","$Z$=0.5","$Z$=0.3","$Z$=0.1","SEM",'interpreter','latex',...
    'location','northwest')

%% Part III - Optional 2 - Error Sensetivity Convergence and Neagtivity
clear all; clc;
rng shuffle

% Parameters
a = 1;
beta = 0.001;
e_pref = linspace(-1,1,10);
sigma = 0.1;
w0 = 0.05*ones(1,10);
trlNum = 2001;

iteration = 400;
eta1 = zeros(1,trlNum);
eta2 = zeros(1,trlNum);
eta3 = zeros(1,trlNum);
eta4 = zeros(1,trlNum);
eta5 = zeros(1,trlNum);
for it = 1:iteration
    disp(it)
    % Building paradigms
    Z = 0.6;
    slowParadigm = parad(trlNum,Z);
    Z = 0.55;
    medium01Paradigm = parad(trlNum,Z);
    Z = 0.5;
    medium02Paradigm = parad(trlNum,Z);
    Z = 0.2;
    medium03Paradigm = parad(trlNum,Z);
    Z = 0.1;
    fastParadigm = parad(trlNum,Z);

    % Simulation
    [x0,e0,wVec0,etaVec0,gVec0] = HarzfeldModel(a,beta,e_pref,sigma,w0,slowParadigm);
    [x1,e1,wVec1,etaVec1,gVec1] = HarzfeldModel(a,beta,e_pref,sigma,w0,medium01Paradigm);
    [x2,e2,wVec2,etaVec2,gVec2] = HarzfeldModel(a,beta,e_pref,sigma,w0,medium02Paradigm);
    [x3,e3,wVec3,etaVec3,gVec3] = HarzfeldModel(a,beta,e_pref,sigma,w0,medium03Paradigm);
    [x4,e4,wVec4,etaVec4,gVec4] = HarzfeldModel(a,beta,e_pref,sigma,w0,fastParadigm);
    
    eta1(1,:) = eta1(1,:) + etaVec0;
    eta2(1,:) = eta2(1,:) + etaVec1;
    eta3(1,:) = eta3(1,:) + etaVec2;
    eta4(1,:) = eta4(1,:) + etaVec3;
    eta5(1,:) = eta5(1,:) + etaVec4;
end

% plot
figure;
hold all;
% errorbar(0:trlNum-1,mean(eta1(:,1:end),1),std(eta1(:,1:end),1)/sqrt(iteration),...
%     'k','LineStyle','none','HandleVisibility','off');
% scatter(0:trlNum-1,mean(eta1,1),20,'k','filled','linewidth',1,'HandleVisibility','off')
plot(0:trlNum-1,mean(eta1,1)/iteration,'k','linewidth',0.3)

% errorbar(0:trlNum-1,mean(eta2(:,1:end),1),std(eta2(:,1:end),1)/sqrt(iteration),...
%     'color','#0072BD','LineStyle','none','HandleVisibility','off');
% scatter(0:trlNum-1,mean(eta2,1),20,'filled','MarkerFaceColor',[0 0.4470 0.7410],'HandleVisibility','off')
plot(0:trlNum-1,mean(eta2,1)/iteration,'color','#0072BD','linewidth',0.3)

% errorbar(0:trlNum-1,mean(eta3(:,1:end),1),std(eta3(:,1:end),1)/sqrt(iteration),...
%     'color','#D95319','LineStyle','none','HandleVisibility','off');
% scatter(0:trlNum-1,mean(eta3,1),20,'filled','MarkerFaceColor',[0.8500 0.3250 0.0980],'HandleVisibility','off')
plot(0:trlNum-1,mean(eta3,1)/iteration,'color','#D95319','linewidth',0.3)

% errorbar(0:trlNum-1,mean(eta4(:,1:end),1),std(eta4(:,1:end),1)/sqrt(iteration),...
%     'color','#EDB120','LineStyle','none','HandleVisibility','off');
% scatter(0:trlNum-1,mean(eta4,1),20,'filled','MarkerFaceColor',[0.9290 0.6940 0.1250],'HandleVisibility','off')
plot(0:trlNum-1,mean(eta4,1)/iteration,'color','#EDB120','linewidth',0.3)

plot(0:trlNum-1,mean(eta5,1)/iteration,'color','#7E2F8E','linewidth',0.3)
% errorbar(0:trlNum-1,mean(eta5(:,1:end),1),std(eta5(:,1:end),1)/sqrt(iteration),...
%     'color','#7E2F8E','LineStyle','none');
% scatter(0:trlNum-1,mean(eta5,1),20,'filled','MarkerFaceColor',[0.4940 0.1840 0.5560],'HandleVisibility','off')

title("Effect of Z on error-sensetivity ($\eta$)",'interpreter','latex')
xlabel("Trial")
ylabel("$\eta$",'interpreter','latex')
% xlim([1,100])
legend("$Z$=0.6","$Z$=0.55","$Z$=0.5","$Z$=0.2","$Z$=0.1",'interpreter','latex',...
    'location','northwest')


%% Part III - Optional 3 - Changing Paradigm
clear all; clc;
rng shuffle

% Parameters
a = 1;
beta = 0.001;
e_pref = linspace(-1,1,10);
sigma = 0.1;
w0 = 0.05*ones(1,10);
trlNum = 100;

Z = 0.9;
paradigm1 = parad(trlNum,Z);
Z = 0.4;
paradigm2 = parad(trlNum,Z);
Z = 0.1;
paradigm3 = parad(trlNum,Z);
paradigm = [paradigm1,paradigm2,paradigm3];
paradigm_plot = [paradigm1(1:50),zeros(1,20),paradigm2(1:50),zeros(1,20),paradigm3(1:50)];

figure;
hold all
plot(paradigm_plot,'k','linewidth',1,'color',[0.3,0.3,0.3])
patch([55 66 66 55], [1.1 1.1 0 0], [0 0 1], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
patch([126 136 136 126], [1.1 1.1 0 0], [0 0 1], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
patch([0 51 51 0], [1.1 1.1 0 0], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
patch([70 121 121 70], [1.1 1.1 0 0], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
patch([140 191 191 140], [1.1 1.1 0 0], [0 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
ylim([0,1.1])
xlim([0,190])
title("Z = 0.9 ... Z=0.4 ... Z=0.1")

iter = 400;
eta = zeros(1,3*trlNum);
for it = 1:iter
    Z = 0.9;
    paradigm1 = parad(trlNum,Z);
    Z = 0.4;
    paradigm2 = parad(trlNum,Z);
    Z = 0.1;
    paradigm3 = parad(trlNum,Z);
    paradigm = [paradigm1,paradigm2,paradigm3];
    [x,e,wVec,etaVec,gVec] = HarzfeldModel(a,beta,e_pref,sigma,w0,paradigm);
    eta = eta + etaVec;
end
eta = eta/100;

figure;
hold all
plot(eta,'k','linewidth',1.2)
patch([0 100 100 0], [1.1 1.1 0 0], [0 1 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
patch([100 200 200 100], [1.1 1.1 0 0], [0 0 1], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
patch([200 300 300 200], [1.1 1.1 0 0], [1 0 0], 'FaceAlpha', 0.1, 'edgecolor',...
    'none', 'handlevisibility', 'off')
xlim([2,3*trlNum])
ylim([0.1,0.35])
title("error-sensetivity in a changin paradigm")
xlabel("trial")
ylabel("$\eta$",'Interpreter','latex')
text(40,0.3,"Z=0.9")
text(140,0.3,"Z=0.4")
text(240,0.3,"Z=0.1")

%% END!