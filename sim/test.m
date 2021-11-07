function [x,fval,exitflag,output] = test(x1,x2,TolX_Data)
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimset;
%% Modify options setting
options = optimset(options,'Display', 'iter');
options = optimset(options,'TolX', TolX_Data);
options = optimset(options,'PlotFcns', { @optimplotfval });
[x,fval,exitflag,output] = ...
fminbnd([],x1,x2,options);
