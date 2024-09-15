clear;
clc;
close all;


penrho = 1000*ones(2,2);
penlength = [0.08 0.08;
    0.10 0.10];
penwidth = 0.02*ones(2,2);
pendepth = 0.02*ones(2,2);

sphere_mass = 0.1*ones(2,2);
sphere_radius = 0.015*ones(2,2);
damping_coeff = [1e-4*ones(2,1) 1e-4*ones(2,1)];
ini_theta_p = 0*ones(2,2);
ini_thetadot_p = 0*ones(2,2);
amp_factor = 0.01;



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

robot_mass = robot_penrho*robot_pendepth*robot_penwidth*robot_penlength;

Win = eye(2);
model_name = "PR_controlv8planning.slx";
sizeinput = 6;

Nr = 4;
SR = 1.1;  %at one point it was 1.7
timeConstant = 0.01;

dt = 0.1;
stoptime = 20;

normmethod = 1;
ifxnorm = 1;
rod_mass = penrho .* penlength .* penwidth .* pendepth;
samplingrate = 0.01;
washout = 0.03*stoptime/samplingrate;
sizeoutput = 3;

m1 = rod_mass(1,1) + sphere_mass(1,1);
m2 = rod_mass(2,2) + sphere_mass(2,2);

% Lengths of the pendulums
L1 = penlength(1,1);
L2 = penlength(2,2);

g = 9.80665;

current = [];
Wr_mats = [];
Wr_mats_not = [];

mass = robot_penrho*robot_penlength*robot_penwidth*robot_pendepth;
fraction = 0.05/mass;

connectivity = 0.7;
timeConstant = 0.085;
 
% load('successful_matrix.mat');

mass_length_pairs = [];
new_pairs = [];

% disp(M)
i = 0;

successful_runs = {}; 

for i = 1:3000
    try
    mass = robot_penrho * robot_penlength * robot_penwidth * robot_pendepth; 
    torquelim = fraction * mass;
    rc = ESN_trainbysim(Nr, 'sizeinput', sizeinput, 'nodenuminput', 3, 'sizeoutput', sizeoutput, ...
        'timeConst', timeConstant, 'spectralRadius', SR, 'inputScaling', 1, 'connectivity', 0.5, 'biasScaling', 0, ...
        'regularization', 1e-1, 'delaylen', 0.1, 'timestep', 0.01, 'normmethod', normmethod, 'ifxnorm', ifxnorm, 'sizereadout', Nr + sizeinput + 1);
    
    % rc.Wr = M(4*i-3:4*i,:);
    rc.clearrecord();
    iscontrol = false;
    robot_ini_theta_p = pi - 0.1;
    robot_ini_thetadot_p = 0;
    control_ref = [1; 0];
    stoptime = 20;
    [SinusoidBus, busin] = create_bus_input(control_ref, dt);

    rc.traintest(model_name);
    rc.traindatacollect(model_name, washout);

    [train_output, train_target] = rc.train('planning', true);

    iscontrol = true;
    robot_ini_theta_p = 0;
    robot_ini_thetadot_p = 0;
    control_ref = [pi; 0];
    stoptime = 100;
    [SinusoidBus, busin] = create_bus_input(control_ref, dt);

    controloutput = rc.robotcontrol(model_name);
    tau = controloutput(13,:);
    theta = controloutput(2,:);
    thetatarget = pi*ones(size(theta));
    mse = immse(theta, thetatarget);

    disp('This is i')
    disp(i)
    disp('This is size')
    disp(size(new_pairs)/4)
    disp(size(new_pairs))

    if abs(controloutput(2, end) - pi) < 0.1
        % Store the matrices in a cell
        run_data = {controloutput, rc.Wr, rc.Woutmat, mse};
        
        % Append the cell to the successful_runs cell array
        successful_runs{end+1} = run_data;

        % Save the updated successful_runs cell array to a .mat file
        save('v1_succesful_reservoirs.mat', 'successful_runs')
    end
    catch
        disp('Error');
    end
end


% function
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