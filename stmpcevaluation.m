clear; clc;
close all;

load("v1_succesful_reservoirs.mat");
load("reservoircapabilities.mat");
data = [data_stm data_pc];

n = size(successful_runs, 2);

connectivity = 0.7;
timeConstant = 0.085;

Nr = 4;
SR = 1.1;  %at one point it was 1.7out = sim(simname, TimeOut=60);

Win = eye(2);
model_name = "esn.slx";
sizeinput = 1;

dt = 0.10;
stoptime = dt*2000;

normmethod = 1;
ifxnorm = 1;
samplingrate = 0.01;
washout = 0.1*stoptime/samplingrate;
sizeoutput = 20;

for j = 1:n

    try
    
    esn = ESN_trainbysim(Nr, 'sizeinput', sizeinput, 'nodenuminput', 3, 'sizeoutput', sizeoutput, ...
        'timeConst', timeConstant, 'spectralRadius', SR, 'inputScaling', 1, 'connectivity', 0.5, 'biasScaling', 0, ...
        'regularization', 1e-1, 'delaylen', 0.1, 'timestep', 0.01, 'normmethod', normmethod, 'ifxnorm', ifxnorm, 'sizereadout', Nr + sizeinput + 1);
    esn.Wr = successful_runs{j}{2};
    
    [SinusoidBus, busin, SinusoidBustar, busintar] = create_bus_input(input(1:stoptime/dt,1), data(1:stoptime/dt,:), dt);
    
    esn.traindatacollect(model_name, washout);
    
    [trainout, traintarget] = esn.train();
    for i = 1:sizeoutput
        trainout{i} = double(trainout{i}>0.5);
    end
    
    datanum = stoptime/dt;
    [SinusoidBus, busin, SinusoidBustar, busintar] = create_bus_input(input(1+datanum:stoptime/dt+datanum,1), data(1+datanum:stoptime/dt+datanum,:), dt);
    
    [predictout, predicttarget] = esn.predict(model_name);
    for i = 1:sizeoutput
        predictout{i} = double(predictout{i}>0.5);
    end
    
    traincoeff = zeros(sizeoutput,1);
    predictcoeff = zeros(sizeoutput,1);
    
    for i = 1:sizeoutput
        coeff = cov(trainout{i}(washout+1:end), traintarget{i}(washout+1:end));
        traincoeff(i) = coeff(1,2)*coeff(1,2)/coeff(1,1)/coeff(2,2);
        coeff = cov(predictout{i}(washout+1:end), predicttarget{i}(washout+1:end));
        predictcoeff(i) = coeff(1,2)*coeff(1,2)/coeff(1,1)/coeff(2,2);
    end

    successful_runs{j}{5} = traincoeff;
    successful_runs{j}{6} = predictcoeff;

    if mod(j,100) == 0
        save("v2_successful_reservoirs.mat", "successful_runs");
    end

    catch
        disp("error");
    end

end

save("v2_successful_reservoirs.mat", "successful_runs");


function [SinusoidBus, busin, SinusoidBustar, busintar] = create_bus_input(input, tar, dt)
    inputDim = 1;    
    tarDim = 20; 
    
    % input = input*2-1;
    [numx, ~] = size(input);

    time = 0:dt:(numx-1)*dt;

    clear elems;
    for j = 1:inputDim
        elems(j) = Simulink.BusElement;
        elems(j).Name = ['x' num2str(j)];
    end
    SinusoidBus = Simulink.Bus;
    SinusoidBus.Elements = elems;
    clear busin;
    busin.x1 = timeseries(input,time);

    clear elems;
    for j = 1:tarDim
        elems(j) = Simulink.BusElement;
        elems(j).Name = ['x' num2str(j)];
    end
    SinusoidBustar = Simulink.Bus;
    SinusoidBustar.Elements = elems;
    clear busintar;
    busintar.x1 = timeseries(tar(:,1),time);
    busintar.x2 = timeseries(tar(:,2),time);
    busintar.x3 = timeseries(tar(:,3),time);
    busintar.x4 = timeseries(tar(:,4),time);
    busintar.x5 = timeseries(tar(:,5),time);
    busintar.x6 = timeseries(tar(:,6),time);
    busintar.x7 = timeseries(tar(:,7),time);
    busintar.x8 = timeseries(tar(:,8),time);
    busintar.x9 = timeseries(tar(:,9),time);
    busintar.x10 = timeseries(tar(:,10),time);
    busintar.x11 = timeseries(tar(:,11),time);
    busintar.x12 = timeseries(tar(:,12),time);
    busintar.x13 = timeseries(tar(:,13),time);
    busintar.x14 = timeseries(tar(:,14),time);
    busintar.x15 = timeseries(tar(:,15),time);
    busintar.x16 = timeseries(tar(:,16),time);
    busintar.x17 = timeseries(tar(:,17),time);
    busintar.x18 = timeseries(tar(:,18),time);
    busintar.x19 = timeseries(tar(:,19),time);
    busintar.x20 = timeseries(tar(:,20),time);

end