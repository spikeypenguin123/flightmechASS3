function [x,fval,exitflag,output,grad,hessian] = test(x0,MaxIterations_Data)
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('fminunc');
%% Modify options setting
options = optimoptions(options,'Display', 'iter-detailed');
options = optimoptions(options,'MaxIterations', MaxIterations_Data);
options = optimoptions(options,'PlotFcn', { @optimplotfval });
options = optimoptions(options,'Algorithm', 'quasi-newton');
[x,fval,exitflag,output,grad,hessian] = ...
fminunc(@optimise_controls,x0,options);
