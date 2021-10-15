% TomDebug

close all 
clear all 
clc 

quat = [0.9922778767136677;0;0;-0.12403473458920847];
output = DCM(quat);

unit_x = [1;0;0];

rot = [  0.9692308,  0.2461538,  0.0000000; -0.2461538,  0.9692308,  0.0000000; 0.0000000,  0.0000000,  1.0000000 ];

test1 = output*unit_x;
test2 = rot*unit_x;



