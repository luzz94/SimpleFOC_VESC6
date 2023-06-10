% clear->clean workspace, clc->clean command window, close all-> close all currently open figures
clear; clc; close all;

fname = 'Torque_1.json'; 
fid = fopen(fname); 
raw = fread(fid,inf); 
str = char(raw'); 
fclose(fid); 
val = jsondecode(str);

x_start = 150;
x_end = 10000;

target = val.telemetries.target.data;
target(1,:) = target(1,:) - target(1,x_start);
target = target(:,x_start:end);


id = val.telemetries.id.data;
id(1,:) = id(1,:) - id(1,1);

iq = val.telemetries.iq.data;
iq(1,:) = iq(1,:) - iq(1,x_start);
iq = iq(:,x_start:end);


% h1=figure(1);
% set(h1,'Position',[10 10 1000 350])
% 
% plot(target(1,:),target(2,:),'r')
% hold on
% grid on
% plot(iq(1,:),iq(2,:),'b')
% 
% xlabel('Current[A]')
% ylabel('Time[s]')

% The standard values for colors saved in PLOT_STANDARDS() will be accessed from the variable PS
PS = PLOT_STANDARDS();
%========================================================
% GENERATE DATA TO PLOT
x1 = target(1,:);
y1 = target(2,:);
x2 = iq(1,:);
y2 = iq(2,:);
x3 = id(1,:);
y3 = id(2,:);

%========================================================
% STEPS FOR PLOTTING
figure(1);
set(gcf,'Position',[50 50 1000 350])
fig1_comps.fig = gcf;
hold on

fig1_comps.p1 = plot(x1, y1);
fig1_comps.p2 = plot(x2, y2);
fig1_comps.p3 = plot(x3, y3);
grid on
hold off


%========================================================
% ADD LABELS, TITLE, LEGEND
title('');
xlabel('Time[s]');
ylabel('Current[A]');

legend([fig1_comps.p1, fig1_comps.p2, fig1_comps.p3], 'Target', 'Iq', 'Id');
legendX = .20; legendY = .77; legendWidth = 0.001; legendHeight = 0.001;
fig1_comps.legendPosition = [legendX, legendY, legendWidth, legendHeight];
% If you want the tightest box set width and height values very low matlab automatically sets the tightest box

%========================================================
% SET PLOT PROPERTIES
% Choices for COLORS can be found in ColorPalette.png
set(fig1_comps.p1, 'LineStyle', '-', 'LineWidth', 1, 'Color', PS.MyBlack);
set(fig1_comps.p2, 'LineStyle', '-', 'LineWidth', 1, 'Color', PS.MyRed);
set(fig1_comps.p3, 'LineStyle', '-', 'LineWidth', 1, 'Color', PS.MyOrange);


%========================================================
% INSTANTLY IMPROVE AESTHETICS-most important step
STANDARDIZE_FIGURE(fig1_comps);

%========================================================
% HOW TO CITE THIS TOOLBOX? (Please support :) )
% Refer - https://in.mathworks.com/matlabcentral/answers/1575908-how-to-cite-matlab-file-exchange-file-using-bibtex?s_tid=es_ans_avr_ans_view#answers_list
% OR
% Include the following in your LaTeX .bib file:
% @misc
% {  atharva2021,
%    author = {Atharva Aalok}, 
%    title = {Professional Plots}, 
%    year = 2021
%    howpublished = "\url{https://www.mathworks.com/matlabcentral/fileexchange/100766-professional_plots}",
%    note = "[Online; accessed October 31, 2022]"
% }


% PLEASE SUPPORT ME BY CITING THIS TOOLBOX. THANK YOU!!!



