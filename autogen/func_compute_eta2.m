function [eta2]= func_compute_eta2(q,dq,param)
%%%%%%  func_compute_eta2.m
%%%%  04/26/24
%%%%
%%%%
%%%%
%Inputs
q1=q(1);
q2=q(2);
q3=q(3);
%%%%
%%%%
dq1=dq(1);
dq2=dq(2);
dq3=dq(3);
%%%%
%%%%
r=param(1);
m=param(2);
Mh=param(3);
Mt=param(4);
l=param(5);
g=param(6);
%%%%
%%%%
%%%%
%%%%
eta2=zeros(1,1);
eta2(1,1) = (Mt*(2*(dq1*(r*cos(q1 + q3) - r*cos(q1)) + dq3*r*cos(q1 + q3))*(r*cos(q1 + q3) - r*cos(q1)) + 2*(r*sin(q1 + q3) - r*sin(q1))*(dq1*(r*sin(q1 + q3) - r*sin(q1)) + dq3*r*sin(q1 + q3))))/2 + (m*(2*(dq1*((r*cos(q1 + q2))/2 - r*cos(q1)) + (dq2*r*cos(q1 + q2))/2)*((r*cos(q1 + q2))/2 - r*cos(q1)) + 2*((r*sin(q1 + q2))/2 - r*sin(q1))*(dq1*((r*sin(q1 + q2))/2 - r*sin(q1)) + (dq2*r*sin(q1 + q2))/2)))/2 + (Mh*(2*dq1*r^2*cos(q1)^2 + 2*dq1*r^2*sin(q1)^2))/2 + (m*((dq1*r^2*cos(q1)^2)/2 + (dq1*r^2*sin(q1)^2)/2))/2;
%%%%
%%%%
%%End of code