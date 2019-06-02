% MEC529 Matlab Homework 1 Problem 1 Codes Created by Yongxin Guo
addpath('/Users/guoyongxin/Desktop/Assignment_Academics/Senior_Second semester/MEC529/Myfunctions');
close all
clear
clc
% Test for function one
% l = [2;4];
% conf = [pi/4;pi/3];
% pos_end = RR_direct_2D(l,conf);
% disp(pos_end);
l = [0.6;0.4];
endPoint = [0.7;-0.7];
result = RR_inverse_2D(l,endPoint);
if (isstring(result) == 1) %check if the returned result is string. if it is string, the error has occured.
    disp(result);  %display the error message
else
    startPoint = [0;0];
    
    midPointXSoln1 = l(1)*cos(result(1,1));
    midPointYSoln1 = l(1)*sin(result(1,1));
    midPointSoln1 = [midPointXSoln1;midPointYSoln1];
    
    midPointXSoln2 = l(1)*cos(result(1,2));
    midPointYSoln2 = l(1)*sin(result(1,2));
    midPointSoln2 = [midPointXSoln2;midPointYSoln2];
    plotLinks(startPoint,midPointSoln1,endPoint);
    hold on;
    plotLinks(startPoint,midPointSoln2,endPoint);
end
    
function graph = plotLinks(startP,midP,endP)
sumP = [startP,midP,endP];  %merge the three point into one 2 by 3 matrix
rowX = sumP(1,:);
rowY = sumP(2,:);
plot(rowX,rowY);   %plot the graph
xlabel("x-axis");
ylabel("y-axis");
end
