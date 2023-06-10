clear;
close;
clc;

load("kneeangles_cut.mat")
load("Position_t1.mat")

ka_ds = downsample(kneeangles_cut,1)
pos_ds = downsample(pos,52)
pos_ds = pos_ds*0.1
plot(kneeangles_cut,'r')
hold on
plot(pos_ds,'--b')

legend('Desired','Actuated')