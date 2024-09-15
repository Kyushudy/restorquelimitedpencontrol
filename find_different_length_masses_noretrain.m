clear;
clc;
close all;
g = 9.80665;

robot_penrho = 1000;
robot_penlength = 0.2;
robot_penwidth = 0.02;
robot_pendepth = 0.02;
robot_penposz = 0.3;
robot_sphere_mass = 0.1;
robot_sphere_radius = 0.015;
robot_damping_coeff = 0; %1e-3
torquelim = 0.05; % 1 for fully acutated <0.08 for underactuated
amp_factor=0.01;

Win = eye(2);
model_name = "PR_controlv8planning2.slx";
sizeinput = 6;

Nr = 4;
SR = 1.1;  %at one point it was 1.7
timeConstant = 0.01;

dt = 0.1;
stoptime = 20;

normmethod = 1;
ifxnorm = 1;
samplingrate = 0.01;
washout = 0.03*stoptime/samplingrate;
sizeoutput = 3;

g = 9.80665;

current = [];
Wr_mats = [];
Wr_mats_not = [];


mass = robot_penrho*robot_penlength*robot_penwidth*robot_pendepth;

fraction = 0.05/mass/robot_penlength;

connectivity = 0.7;
timeConstant = 0.085;
 
% load('successful_matrix.mat');

% disp(M)

load("v11_succesful_reservoirs.mat");

mass_length_pairs = {};

%%%%%%%%%%%%%%% Change the robot_penlength limit to this %%%%%%%%%%%%%%%%
% for robot_penlength = 0.05:0.01:0.4
%     for robot_penrho = 100:100:5000
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for robot_penlength = 0.05:0.01:0.4
    for robot_penrho = 100:100:5000

       try
       
       mass = robot_penrho*robot_penlength*robot_penwidth*robot_pendepth; 
       torquelim = fraction*mass*robot_penlength;
        rc = ESN_trainbysim(Nr, 'sizeinput', sizeinput, 'nodenuminput', 3, 'sizeoutput', sizeoutput, ...
            'timeConst', timeConstant, 'spectralRadius', SR, 'inputScaling', 1, 'connectivity', 0.5, 'biasScaling', 0, ...
            'regularization', 1e-1, 'delaylen', 0.1, 'timestep', 0.01, 'normmethod', normmethod, 'ifxnorm', ifxnorm, 'sizereadout', Nr + sizeinput + 1);
        
        % rc.Wr = M(1:4,:);
        rc.Wr = successful_runs{1}{2};
        rc.Woutmat = successful_runs{1}{3};
        % rc.clearrecord();
        % iscontrol = false;
        % robot_ini_theta_p = pi - 0.1;
        % robot_ini_thetadot_p = 0;
        % control_ref = [1; 0];
        % stoptime = 20;
        % [SinusoidBus, busin] = create_bus_input(control_ref, dt);
        % 
        % rc.traintest(model_name);
        % rc.traindatacollect(model_name, washout);
        % 
        % [train_output, train_target] = rc.train('planning', true);
        
        iscontrol = true;
        robot_ini_theta_p = 0;
        robot_ini_thetadot_p = 0;
        control_ref = [pi; 0];
        stoptime = 100;
        [SinusoidBus, busin] = create_bus_input(control_ref, dt);
        
        controloutput = rc.robotcontrol(model_name);
        tau = controloutput(13,:);
        theta = controloutput(2,1:5:2001);
        thetatarget = pi*ones(size(theta));
        mse = immse(theta, thetatarget);
        
        if abs(controloutput(2, end) - pi) < pi/4
            new_pairs = {robot_penlength, robot_penrho, mse, (controloutput(2, end) - pi)};
            mass_length_pairs{end+1} = new_pairs;
            % mass_length_pairs = [new_pairs; mass_length_pairs];
            save('v2_successful_pairsnoretrain.mat', 'mass_length_pairs');
        end 

       catch
           disp('errors');
       end
        
    end 
end
        

%% function
function [SinusoidBus, busin] = create_bus_input(input, dt)
    inputDim = 2;

    [~, numy] = size(input);

    time = 0:dt:(numy-1)*dt;

    clear elems;
    for j = 1:inputDim
        elems(j) = Simulink.BusElement;
        elems(j).Name = ['x' num2str(j)];
    end
    SinusoidBus = Simulink.Bus;
    SinusoidBus.Elements = elems;
    clear busin;
    busin.x1 = timeseries(input(1,:)',time');
    busin.x2 = timeseries(input(2,:)',time');

end