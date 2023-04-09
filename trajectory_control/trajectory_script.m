%controller for the point to point navigation of a diffrendial drive mobile
%robot (DDMR)

%currently able to g

L = 235; %width of base in mm
v_d = 100; %desired velocity in mm/s

%%Generating parametric trajectory equations
%s=linspace(0,1,1000); % s from 0 -> 1
syms s(t) ;
assume(s(t) >= 0);
cond = s(0) == 0;
%Parametric equations for a circular path, these are the equations I will
%be getting from Umang!
x = 1000*cos(s(t)*2*pi)+1000 ;
y = 1000*sin(s(t)*2*pi)+1000; 

%x = 10000*s(t);
%y = x^2;
disp(x)


%% finding s(t)
x_dot = diff(x,t);
y_dot = diff(y,t);
eqn = sqrt(x_dot^2 + y_dot^2)== 100;
sol = dsolve(eqn, cond);
for i = 1:length(sol)
    disp(i)
    disp(length(sol))
    solution = sol(i);
    T = double(solve(solution == 1,t));
    disp(T)
    if T > 0
        s_t = sol(i);
        break
        
    end

end

%% Calculating Theta 

%figure
%plot(x,y); 

ds = 0.001; %small step of s for calculating angle
x1 = x;
y1 = y;
x2 = subs(x, s(t), s(t) + ds);
y2 = subs(y, s(t), s(t) + ds);
dx = x2 - x1;
dy = y2 - y1;
        
theta = atan2(dy,dx);
%disp(theta)
%% x(t), y(t), theta(t), x_dot(t), y_dot(t), theta_dot(t)
x_t = subs(x, s(t), s_t);
y_t = subs(y, s(t), s_t);
theta_t = subs(theta, s(t), s_t);

q_t = [x_t;y_t;theta_t];


x_dot_t = diff(x_t,t);
y_dot_t = diff(y_t,t);
theta_dot_t = diff(theta_t,t);


q_dot_t = [x_dot_t;y_dot_t;theta_dot_t];

%% creating time series by evaluating time functions
t = double(linspace(0,T,1000));
q_t_vals = double(subs(q_t));
for i = 1:length(q_t_vals(3,:))
    if q_t_vals(3,i) < 0
        q_t_vals(3,i) = q_t_vals(3,i) + 2*pi;
    end
end


q_dot_t_vals = double(subs(q_dot_t));
q_dot_t_vals = q_dot_t_vals.*ones(3,length(q_t_vals(3,:)));
figure
plot(q_t_vals, t)

q_t_series = timeseries(q_t_vals,transpose(t));
q_dot_t_series = timeseries(q_dot_t_vals,transpose(t));



%% visualization

s=linspace(0,1,1000); % s from 0 -> 1
x_vals = subs(x);
y_vals = subs(y);

theta_vals = subs(theta);
disp(theta_vals)
figure
plot(x_vals,y_vals);
figure
plot(s,theta_vals);
ylim([-pi, pi]);





%%
% function s_t_vals = find_s(x_vals,y_vals)
%     for i = range(length(x_vals))
%         eqn = 
% 
% 
%         
% 
%     end    


%end
% function theta = ang_prof(x,y)
%     %finding theta using the four quadrant convention
% 
%     ds = 0.001; %small step of s for calculating angle
%     x1 = 1000*cos(s*2*pi);
%     y1 = 1000*sin(s*2*pi); 
%     x2 = 1000*cos((s+ds)*2*pi);
%     y2 = 1000*sin((s+ds)*2*pi);
%     dx = x2 - x1;
%     dy = y2 - y1;
%         
%     theta = atan2(dy,dx);
% 
%     theta = []; %angle from the origin x axis (abs angle)
%     for i = 1:(length(x)-1)
%         %disp(i)
%         s=linspace(0,1,1000); % s from 0 -> 1
% 
%         x1 = x(i);
%         y1 = y(i);
%         x2 = x(i+1);
%         y2 = y(i+1);
%         dx = x2 - x1;
%         dy = y2 - y1;
%         
%        
%         %disp(atan(y/x));
%         theta(end+1) = atan2(dy,dx);
%         
%     end
%     disp(theta)
% end
% 
% 
% function d = dist(xy)
    %calculates the total distance traveled along a function
%     d = 0;
%     for i = 1:(length(xy)-1)
%         x1 = xy(i, 1);
%         y1 = xy(i, 2);
%         x2 = xy(i+1, 1);
%         y2 = xy(i+1, 2);
%         dx = x2 - x1;
%         dy = y2 - y1;
%         d = d + sqrt(dx^2 + dy^2);
% 
%     end
% end

