%% Controller for IMECE Paper
% 1/31/22
% Connor Herron
clc; clear;

kff = 226.6/1000;
eta = 0.27;
w_n = 7.08*2*pi;

Plant = kff*tf(w_n^2,[1 2*eta*w_n w_n^2]);

% targets:
PO = 0.1;
ts = 0.2;


%% build pid controller
Kp = 5.76;    
Ki = 100.4;
Kd = 0.279;

% Kp = 30;    
% Ki = 2;
% Kd = 0.25;

Tf = 0.00304;
C = pid(Kp, Ki, Kd, Tf);

cl_Plant = feedback(C*Plant,1);




%% simulations

step(cl_Plant)

%% Make a nice plot
opts.Colors     = get(groot,'defaultAxesColorOrder');
opts.saveFolder = 'img/';
opts.width      = 8;
opts.height     = 6;
opts.fontType   = 'Times';
opts.fontSize   = 9;

load force_step.mat



fig = figure; clf


plot(t,force_output,'LineWidth',1.8)
hold on
plot(t,des_force,'--','LineWidth',1.8)



axis tight
ylim([-10 340])
%ylim([-320 320])
xlim([0 0.5])
xlabel('Time (s)')
ylabel('Force (N)')
legend('Output Force','Desired Force','Location','southeast')

% scaling
fig.Units               = 'centimeters';
fig.Position(3)         = opts.width;
fig.Position(4)         = opts.height;

% set text properties
set(fig.Children, ...
    'FontName',     'Times', ...
    'FontSize',     9);
set(legend,'fontsize',7);
% remove unnecessary white space
set(gca,'LooseInset',max(get(gca,'TightInset'), 0.02))

% export to png
fig.PaperPositionMode   = 'auto';
print([opts.saveFolder 'my_figure_step'], '-dpng', '-r600')
