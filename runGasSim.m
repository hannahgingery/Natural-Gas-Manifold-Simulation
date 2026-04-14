%% runGasSim.m
% Description: Combined Open/Closed Loop simulation and PI tuning
%   for EELE 488 gas manifold model using GasSimModel.slx
%
% By Hannah Gingery, 2026

%% Housekeeping
close all;
clear;
clc;

%% Set Up
mode = 1;   % 1 = CL, 0 = OL
step_case = 7;        % 0–6 
step_size = 1;        % 1 = small, 2 = large

% For CL tuning:
a      = 0.8;         % a = Kp/Ki 0.8 selected for undercompensation

Kp_vec_Pm = [0.1 0.15 0.2 0.5 ];   % Pressure controller
Kp_vec_Q2 = [1 2 3 4];  % Flow controller
%% Constants
SG = 0.6;      % Specific Gravity of Natural Gas
T  = 0.5;      % Time Constant (0.5 seconds)
Ttotal = 12;   % length of simulation
dt = 0.1;
t  = (0:dt:Ttotal)';
u_max = 1;
u_min = 0;

% Actuator and sensor gains
Ka  = 1/20;           % Actuator gain
Ks1 = 20/(60-0);     % volts to psi
Ks2 = 20/(60-0);    % volts to SCFH

%% User Inputs (keep OL structure)
P1 = 40;   % Upstream Pressure of Valve 1 (psia)
P2 = 40;   % Upstream Pressure of Valve 2 (psia)

Q2_ref = 1*1000/60;     % Desired flow rate (SCFM) SCFM = MCFH*1000/60 --> MCFH is what NWE uses
Qout   = 1.5*1000/60;    % Load/ Disturbance of Gas being pulled of the system by customers (SCFH)
Pm_ref = 20;    % Desired Manifold Pressure (psia)


Pm_max = P1;
Pm_min = 0;

Q2_max = 200*1000/60;
Q2_min = 0;

%% Initialization
Q1   = Qout - Q2_ref;
cv2  = Q2_ref * sqrt(SG) / sqrt(P2 - Pm_ref);
s2int = funCV(cv2,false);

cv1  = Q1 * sqrt(SG) / sqrt(P1 - Pm_ref);
s1int = funCV(cv1,false);

%% Step magnitude selection
switch step_size
    case 1  % Small steps
        ds = 0.05;
        dP = 5;
        dQ = 5;
        magText = "Small";
    case 2  % Large steps
        ds = 0.5 * s1int;   % 50% valve change
        dP = 0.15 * P1;     % 15% pressure change
        dQ = 0.3 * Qout;    % 30% load change
        magText = "Large";
    otherwise
        error("Invalid step_size selected")
end

%% Step case selection
switch step_case
    case 0  % No step
        s1final   = s1int;
        s2final   = s2int;
        Qoutfinal = Qout;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref;
        plotTitle = "Open Loop - " + magText + " Step at s1" ;
        
    case 1  % Step at s1 only
        s1final   = s1int + ds;
        s2final   = s2int;
        Qoutfinal = Qout;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref;
        plotTitle = "Open Loop - " + magText + " Step at s1";
        
    case 2  % Step at s2 only
        s1final   = s1int;
        s2final   = s2int + ds;
        Qoutfinal = Qout;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref;
        plotTitle = "Open Loop - " + magText + " Step at s2";

    case 3  % Step at s1, then 5 sec later step at s2
        plotTitle = "Open Loop - " + magText + " Step at s1 then s2";
        % First step at s1
        s1final = s1int + ds;
        % s2 below
        s2final = s2int; 
        Qoutfinal = Qout;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref;
        
    case 4  % Step at Qout only
        s1final   = s1int;
        s2final   = s2int;
        Qoutfinal = Qout + dQ;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref;
        plotTitle = "Open Loop Step at Qout";
    case 5  % Step at Pm_ref
        s1final   = s1int;
        s2final   = s2int;
        Qoutfinal = Qout;
        Pmfinal = Pm_ref + dP;
        Q2final = Q2_ref;
        plotTitle = "Step at Pmref";
    case 6  % Step at Q2_ref
        s1final   = s1int;
        s2final   = s2int;
        Qoutfinal = Qout;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref + dQ;
        plotTitle = "Closed Loop Step at Q2ref";
    case 7  % Step at Qout only
        s1final   = s1int;
        s2final   = s2int;
        Qoutfinal = Qout + dQ;
        Pmfinal = Pm_ref;
        Q2final = Q2_ref;
        plotTitle = "Closed Loop Step at Qout";
    
    otherwise
        error("Invalid step_case selected")
end

%% Build input signals for Simulink (shared by OL and CL)
tStep = 2;

if step_case ~= 3
    s1_sig   = funmakeStep(s1int, s1final, tStep, Ttotal, dt);
    s2_sig   = funmakeStep(s2int, s2final, tStep, Ttotal, dt);
else
    % TWO STEP CASE
    s1_vals = s1int * ones(length(t),1);
    s1_vals(t >= 10) = s1int + ds;
    s1_sig = [t s1_vals];
    
    s2_vals = s2int * ones(length(t),1);
    s2_vals(t >= 15) = s2int + ds;
    s2_sig = [t s2_vals];
end

Qout_sig = funmakeStep(Qout,  Qoutfinal,  tStep, Ttotal, dt);
Pm_ref_sig = funmakeStep(Pm_ref,  Pmfinal,  tStep, Ttotal, dt);
Q2_ref_sig = funmakeStep(Q2_ref,  Q2final,  tStep, Ttotal, dt);



% Reference signals
n = length(t);
x = ones(n,1);
P1 = [t P1*x];
P2 = [t P2*x];

%% Open the unified model
myModel = 'GasSimModel.slx';
open_system(myModel);

%% Run simulation(s)

if mode == 0
    Kp_Pm = 0; Ki_Pm = 0; Kp_Q2 = 0; Ki_Q2 = 0;
    simOut = sim(myModel,'ReturnWorkspaceOutputs','on');

    % Plot like original OL script
    figure
    subplot(2,1,1)
    plot(simOut.tout, simOut.Pm,'LineWidth',2)
    grid
    legend('PmOut')
    ylabel('PSI')
    title(plotTitle)

    subplot(2,1,2)
    plot(simOut.tout,simOut.Q1,'k', ...
         simOut.tout,simOut.Q2,'b', ...
         Qout_sig(:,1),Qout_sig(:,2),'r','LineWidth',2)
    legend('Q_1','Q_2','Qout')
    ylabel('SCFM')
    xlabel('Time (s)')
    grid

    figure
    plot(simOut.tout,simOut.s1,'k',simOut.tout,simOut.s2,'b','LineWidth',2)
    legend('s1','s2')
    ylabel('pu')
    xlabel('Time (s)')
    grid
else
    % CLOSED LOOP: PI tuning with Kp_vec
    % Keep a = Kp/Ki constant => Ki = Kp / a
    nK = length(Kp_vec_Pm);
    for k = 1:nK
        % Pressure controller gains
        Kp_Pm = Kp_vec_Pm(k);
        Ki_Pm = Kp_Pm / a;
    
        % Flow controller gains
        Kp_Q2 = Kp_vec_Q2(k);
        Ki_Q2 = Kp_Q2 / a;
    
        % Send to Simulink
        assignin('base','Kp_Pm',Kp_Pm);
        assignin('base','Ki_Pm',Ki_Pm);
        assignin('base','Kp_Q2',Kp_Q2);
        assignin('base','Ki_Q2',Ki_Q2);
    
        assignin('base','mode',1);
    
        simOut = sim(myModel,'ReturnWorkspaceOutputs','on');
    
        tout_store{k} = simOut.tout;
        Pm_store{k}   = simOut.Pm;
        Q2_store{k}   = simOut.Q2;
    end
    % Overlay plots
    figure;
    hold on;
    colors = lines(nK);
    for k = 1:nK
        plot(tout_store{k},Pm_store{k},'Color',colors(k,:),'LineWidth',1.5, ...
             'DisplayName',sprintf('Kp_{Pm}=%.2g, Kp_{Q2}=%.2g', Kp_vec_Pm(k), Kp_vec_Q2(k)));
    end
    grid on;
    xlabel('Time (s)');
    ylabel('PmOut (psi)');
    title('Closed Loop Pm - ' + plotTitle);
    legend('Location','best');
    hold off;

    figure;
    hold on;
    for k = 1:nK
        plot(tout_store{k},Q2_store{k},'Color',colors(k,:),'LineWidth',1.5, ...
             'DisplayName',sprintf('Kp_{Pm}=%.2g, Kp_{Q2}=%.2g', Kp_vec_Pm(k), Kp_vec_Q2(k)));
    end
    grid on;
    xlabel('Time (s)');
    ylabel('Q_2 (SCFM)');
    title('Closed Loop Q2 - ' + plotTitle);
    legend('Location','best');
    hold off;
end
