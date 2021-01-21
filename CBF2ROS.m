% parpool;
clear

global detection_flg
global subscribe_flg1
global subscribe_flg2
global x_d
global y_d
global l
global phi
global DATASIZE

DEBUG_STEP = 1200*0.5;
DEBUG_LOG_U = zeros(2, DEBUG_STEP);
DEBUG_LOG_U_CTR = zeros(2, DEBUG_STEP);
DEBUG_LOG_DESIRE = zeros(2, DEBUG_STEP);
DEBUG_LOG_MIN_L  = zeros(1, DEBUG_STEP);

detection_flg = false;
subscribe_flg1 = false;
subscribe_flg2 = false;
x_d = 0;
y_d = 0;
l = zeros(DATASIZE, 1);
phi = zeros(DATASIZE, 1);
DATASIZE = 726;

A = zeros(DATASIZE, 2);
b = zeros(DATASIZE, 1);

u_current = [0.0; 0.0];
u = [0; 0];

delta = 0.4;
minDis = 0.05;

k_1 = 0.8;
k_2 = 0.3;

x_d_bfr = 0;
y_d_bfr = 0;

x_d_lcl = 0;
y_d_lcl = 0;

options = optimoptions("fmincon","Algorithm", "sqp", "UseParallel", true, "Display", "off", "StepTolerance", 0.01, "OptimalityTolerance", 0.01, "ConstraintTolerance", 0.001);

rosinit("169.254.15.238", 11311);

sleep = rosrate(0.5);
waitfor(sleep);

lrf_sub = rossubscriber("/scan", "sensor_msgs/LaserScan", @lrf_callback);
hp_sub  = rossubscriber("/status", "std_msgs/Float32MultiArray", @hp_callback);
u_pub   = rospublisher("/u_cbf", "std_msgs/Float32MultiArray");

msg = rosmessage("std_msgs/Float32MultiArray");
rate = rosrate(20);

for i=1:DEBUG_STEP
    if(subscribe_flg1 && subscribe_flg2)
        l(isnan(l)) = 0;
        bool_mattix = (l <= minDis);
        
        A_obj = cos(phi)';
        b_obj = l-delta;

        A_obj(bool_mattix) = 0;
        b_obj(bool_mattix) = 0;

        A(:, 1) = A_obj;
        b(:, 1) = b_obj;
        
        if(detection_flg) % 非検出
            x_d_lcl = x_d_bfr;
            y_d_lcl = y_d_bfr;
        else
            x_d_bfr = x_d;
            y_d_bfr = y_d;
            x_d_lcl = x_d;
            y_d_lcl = y_d;
        end

        th_d = atan2(y_d_lcl, x_d_lcl);

        u_ctr = [k_1*x_d_lcl; k_2*y_d_lcl+k_1*sin(th_d)*cos(th_d)];

        fun = @(u)(double(0.5*vecnorm(u-u_ctr)^2));
        Aeq = [];
        beq = [];
        lb  = [];
        ub  = [];
        nonlcon = @u_norm;

        u = fmincon(fun, u_current, A, b, Aeq, beq, lb, ub, nonlcon, options);

        msg.Data = u;
        send(u_pub, msg);
        u_current = u;

        DEBUG_LOG_U(:, i) = u;
        DEBUG_LOG_U_CTR(:, i) = u_ctr;
        DEBUG_LOG_DESIRE(:, i) = [x_d_lcl; y_d_lcl];
        DEBUG_LOG_MIN_L(1, i)  = min(l);
    end
    waitfor(rate);
end

u = [0; 0];
msg.Data = u;
send(u_pub, msg);
u_current = u;

rosshutdown

function [c, ceq] = u_norm(x)
    alpha = 1;
    c = vecnorm(x)-alpha;
    ceq = [];
end

function lrf_callback(~, message, ~)
    global l
    global phi
    global DATASIZE
    global subscribe_flg1
    
    subscribe_flg1 = true;
    l = message.Ranges;
    phi = message.AngleMin:message.AngleIncrement:message.AngleMin+message.AngleIncrement*(DATASIZE-1);
end

function hp_callback(~, message, ~)
    global detection_flg
    global subscribe_flg2
    global x_d
    global y_d
    
    subscribe_flg2 = true;
    detection_flg = message.Data(1);
    x_d = message.Data(2);
    y_d = message.Data(3);
end
