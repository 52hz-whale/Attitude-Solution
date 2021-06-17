%% 一维牛顿法

% 迭代公式
   %  x(k+1) = x(k) - f'/f''
   
% f(x) =x^4 -3 * x^3 - 5* x^2
% f1 = 4 * x^3 - 9*x^2 - 10*x
% f2 = 12*x^2 - 18*x - 10
clc;
clear;
close all;

len = 100;
x = ones(len, 1);
f = ones(len,1);
% 迭代初始值影响最终结果，比如局部最小值有两个
x(1) =100;
f(1) = x(1)^4 - 3*x(1)^3 - 5*x(1)^2;

for i=2:len
    f1 = 4*x(i-1)^3 - 9*x(i-1)^2 - 10*x(i-1);
    f2 = 12*x(i-1)^2 - 18*x(i-1) -10;
    x(i) = x(i-1) - f1/f2;
    f(i) = x(i)^4 - 3*x(i)^3 - 5*x(i)^2;
end

figure
plot(f)

%% 多维牛顿法
% 迭代公式
    % x(k+1) = x(k)-H^-1 * gradient;
    % 本质上和一维一样
    
    
 %%  牛顿高斯法
 

a=1;b=2;c=1;              %待求解的系数

x=(0:0.01:1)';
w=rand(length(x),1)*2-1;   %生成噪声
y=exp(a*x.^2+b*x+c)+w;     %带噪声的模型 
figure
plot(x,y,'.')

pre=rand(3,1);      %步骤1

for i=1:1000
    
    f = exp(pre(1)*x.^2+pre(2)*x+pre(3));
    g = y-f;                    %步骤2中的误差 
    
    p1 = exp(pre(1)*x.^2+pre(2)*x+pre(3)).*x.^2;    %对a求偏导
    p2 = exp(pre(1)*x.^2+pre(2)*x+pre(3)).*x;       %对b求偏导
    p3 = exp(pre(1)*x.^2+pre(2)*x+pre(3));          %对c求偏导
    J = [p1'
            p2'
            p3'];             %步骤2中的雅克比矩阵
    
    delta = inv(J*J')*J* g;    %步骤3，inv(J'*J)*J'为H的逆
    
    pcur = pre+delta;           %步骤4
    if norm(delta) <1e-16
        break;
    end
    pre = pcur;
end

hold on;
plot(x,exp(a*x.^2+b*x+c),'r');
plot(x,exp(pre(1)*x.^2+pre(2)*x+pre(3)),'g');

%比较一下
[a b c];
pre';