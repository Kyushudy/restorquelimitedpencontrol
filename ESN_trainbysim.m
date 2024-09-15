 classdef ESN_trainbysim < handle
   
    properties

        Nr
        rho
        tau
        inputScaling
        biasScaling
        lambda
        connectivity
        Win
        Wb
        Wr
        randweight
        initialstate
        
        Wout
        Woutmat
        train_internalState
        train_internalStatedot
        train_reservoirReadout
        train_reservoirReadoutWashed
        train_reservoirTarget
        train_reservoirTargetWashed
        predict_internalState
        predict_internalStatedot
        predict_reservoirReadout
        sizeinput
        nodenuminput
        sizeoutput
        delaylen
        timestep
        ifxnorm
        xnorm
        delaynorm
        normmethod
        theta_matrix 
        potential_matrix
        thetadot_matrix
        nodes_per_input 
        sizeinputvariables
        Win_adjust
        sizereadout
    end

    methods
        function esn = ESN_trainbysim(Nr, varargin)
            disp("Class is created")
            esn.theta_matrix = [];
            esn.thetadot_matrix = [];
            esn.potential_matrix = [];

            esn.Nr = Nr;
            esn.tau = 10;
            esn.inputScaling = 1;
            esn.biasScaling = 1;
            esn.lambda = 1;
            esn.connectivity = 1;
            esn.sizeinput = 1;
            esn.randweight = true;
            esn.nodenuminput = 1;
            esn.sizeoutput = 1;
            esn.initialstate = zeros(esn.Nr,1);
            esn.delaynorm = nan;
            esn.xnorm = 1;
            esn.train_reservoirReadoutWashed = [];
            esn.Woutmat = [];
            esn.Win_adjust = 0;

            
            numvarargs = length(varargin);
            for i = 1:2:numvarargs
                switch varargin{i}
                    case 'timeConst', esn.tau = varargin{i+1};
                    case 'spectralRadius', esn.rho = varargin{i+1};
                    case 'inputScaling', esn.inputScaling = varargin{i+1};
                    case 'biasScaling', esn.biasScaling = varargin{i+1};
                    case 'regularization', esn.lambda = varargin{i+1};
                    case 'connectivity', esn.connectivity = varargin{i+1};
                    case 'sizeinput', esn.sizeinput = varargin{i+1};
                    case 'nodenuminput', esn.nodenuminput = varargin{i+1};
                    case 'sizeoutput', esn.sizeoutput = varargin{i+1};
                    case 'randweight', esn.randweight = varargin{i+1};
                    case 'delaylen', esn.delaylen = varargin{i+1};
                    case 'timestep', esn.timestep = varargin{i+1};
                    case 'normmethod', esn.normmethod = varargin{i+1};
                    case 'ifxnorm', esn.ifxnorm = varargin{i+1};
                    case 'nodes_per_input', esn.nodes_per_input = varargin{i+1};
                    case 'variablesize', esn.sizeinputvariables = varargin{i+1};
                    case 'sizereadout' , esn.sizereadout = varargin{i+1};
                    % case 'a11', esn.a11 = varargin{i+1};
                    % case 'a22', esn.a22 = varargin{i+1};
                    % case 'a21', esn.a21 = varargin{i+1};
                    % case 'a12', esn.a12 = varargin{i+1};



                    otherwise, error('the option does not exist');
                end
            end
            esn.train_reservoirTarget = cell(esn.sizeoutput,1);
            esn.train_reservoirTargetWashed = cell(esn.sizeoutput,1);
            for i = 1:esn.sizeoutput
                esn.train_reservoirTargetWashed{i} = [];
            end
            esn.Wout = cell(1, esn.Nr+esn.sizeinput+1);
            esn.Woutmat = zeros(3, esn.Nr+esn.sizeinput+1);
            disp(size(esn.Woutmat))
            disp("This it the size")
            %esn.Woutmat = esn.Woutmat';

            if esn.randweight
                % esn.Win = esn.inputScaling * ((rand(esn.Nr, 1)) * 2 - 1);
                % esn.Win(esn.nodenuminput*esn.sizeinput+1:end) = 0;
                esn.Win = esn.inputScaling * ones(esn.Nr, 1);
                esn.Win(2:2:end) = -esn.Win(2:2:end);%alternatate at a everyother rate 
                %esn.Win = zeros(Nr,1);
                %esn.Win(1) = 1;
                %esn.Win(13) = 1;
                %esn.Win(37) = 1;
                %esn.Win(Nr/2+1) = 1;
                % esn.Win = randi([1, 3], esn.Nr, 2);
                % esn.Win(esn.Win == 1) = -1;
                % esn.Win(esn.Win== 2) = 0;
                % esn.Win(esn.Win== 3) = 1;

                esn.Wb = esn.biasScaling * (rand(esn.Nr, 1) * 2 - 1);
                esn.Wr = full(sprand(esn.Nr, esn.Nr, esn.connectivity));
                % esn.Wr(2,4) = rand();
                % esn.Wr(1,4) = rand();
                % esn.Wr(3,4) = rand();
                %esn.Wr(3,1) = rand();
                esn.Wr(esn.Wr ~= 0) = esn.Wr(esn.Wr ~= 0) * 2 - 1;
                esn.Wr = esn.Wr * (esn.rho / max(abs(eig(esn.Wr))));
                esn.xnorm = ones(esn.sizeinput,1);


            else
               
                esn.Nr = esn.nodes_per_input * esn.sizeinput;
                esn.initialstate = zeros(esn.Nr,1);
                esn.Wr = zeros(esn.Nr, esn.Nr);
                non_zero_value = 0.9;
            
                for i = 1:esn.Nr %so each node leads to each other
                    if ~mod(i, esn.nodes_per_input) == 0
                        disp(i)
                        esn.Wr(i, i+1) = non_zero_value;
                        disp(esn.Wr(i,i+1))
                    end 
                end 
                esn.Win = zeros(esn.Nr, esn.sizeinput);
                for j = 1:esn.Nr
                    if mod(j, esn.nodes_per_input) == 1 
                        esn.Win(j, floor((j-1)/esn.nodes_per_input) + 1) = 1; 
                    end 
                end
                esn.Wb = esn.biasScaling * (rand(esn.Nr, 1) * 2 - 1);
            end
            %disp(size(esn.Wr))
            %disp(size(esn.initialstate))
 
            if esn.Nr < esn.sizeinput
                    disp(esn.Nr)
                    disp(esn.sizeinput)
                    choices = [1,-1];
                    random_indices = randi(length(choices),esn.Nr, esn.sizeinput);
                    esn.Win_adjust = choices(random_indices);

            end
        end


        function [] = clearrecord(esn, varargin)
            numvarargs = length(varargin);
            for i = 1:2:numvarargs
                switch varargin{i}
                    %case 'timeConst', esn.tau = varargin{i+1};
                    case 'inputScaling', esn.inputScaling = varargin{i+1};
                    case 'regularization', esn.lambda = varargin{i+1};
                    case 'sizeinput', esn.sizeinput = varargin{i+1};
                    case 'nodenuminput', esn.nodenuminput = varargin{i+1};
                    case 'sizeoutput', esn.sizeoutput = varargin{i+1};
                    case 'randweight', esn.randweight = varargin{i+1};
                    case 'delaylen', esn.delaylen = varargin{i+1};
                    case 'timestep', esn.timestep = varargin{i+1};
                    case 'normmethod', esn.normmethod = varargin{i+1};
                    case 'ifxnorm', esn.ifxnorm = varargin{i+1};
                      
                 
                    
                    otherwise, error('the option does not exist');
                end
            end

            esn.train_internalState = [];
            esn.train_internalStatedot = [];
            esn.train_reservoirReadout = [];
            esn.train_reservoirReadoutWashed = [];
            esn.train_reservoirTarget = cell(esn.sizeoutput,1);
            esn.train_reservoirTargetWashed = cell(esn.sizeoutput,1);
            for i = 1:esn.sizeoutput
                esn.train_reservoirTargetWashed{i} = [];
            end
            esn.predict_internalState = [];
            esn.predict_internalStatedot = [];
            esn.predict_reservoirReadout = [];
            esn.xnorm = 0;
            esn.delaynorm = nan;
            esn.predict_internalState = [];
            esn.predict_reservoirReadout = [];
            esn.xnorm = ones(esn.sizeinput,1);
        end


        function [x, target] = traintest(esn, simname)
            [const, x, target, internalState, internalStatedot] = esn.runsim(simname);

            v1 = x(1:end/2,:);
            if esn.ifxnorm
                % esn.xnorm = median(abs(v1),2);
                esn.xnorm = max(abs(v1),[],2)/2;
                esn.xnorm = [esn.xnorm; esn.xnorm];
            end
        end


         function [] = traindatacollect(esn, simname, washout)

            [const, x, target, internalState, internalStatedot] = esn.runsim(simname);
            esn.train_internalState = internalState;
            esn.train_internalStatedot = internalStatedot;

            %%Added a transpose for valees Nr < Size of Input
            esn.train_reservoirReadout = [const'; x; esn.train_internalState'];
            esn.train_reservoirReadoutWashed = [esn.train_reservoirReadoutWashed esn.train_reservoirReadout(:,washout+1:end)];
            
            for i = 1:esn.sizeoutput
                esn.train_reservoirTarget{i} = target{i};
                esn.train_reservoirTargetWashed{i} = [esn.train_reservoirTargetWashed{i}; esn.train_reservoirTarget{i}(washout+1:end)];
            end
         end


function [controloutput] = robotcontrol(esn, simname)

            [const, x, target, internalState, internalStateDot, tau] = esn.runsim(simname);
            disp(size(tau'))
            disp(size(const'))
            disp(size(x))
            disp(size(internalState'))
            disp(size(target{1}'))

            controloutput = [const'; x; internalState'; target{1}'; tau'];
            
        end



        function [train_output, train_target] = train(esn, varargin)

            planning  = false;

            numvarargs = length(varargin);
            for i = 1:2:numvarargs
                switch varargin{i}
                    case 'planning', planning = varargin{i+1};
                    otherwise, error('the option does not exist');
                end
            end

            if planning
                train_output = cell(esn.sizeoutput,1);
                train_target = cell(esn.sizeoutput,1);
                esn.Woutmat = [];


                for i = 1:esn.sizeoutput
                    if i == 1  %this is training to control   %skipping the last from x 
                        % X = esn.train_reservoirReadoutWashed(1:5,:);
                        X = esn.train_reservoirReadoutWashed([1:5 8:(esn.Nr+esn.sizeinput+1)],:);
                        Y = esn.train_reservoirTargetWashed{i};

                        esn.Wout{i} = Y'*X'*inv(X*X'+esn.lambda*eye(size(X,1))); 
                        % esn.Wout{i} = [esn.Wout{i}(1:5) zeros(1,6)];
                        esn.Wout{i} = [esn.Wout{i}(1:5) 0 0 esn.Wout{i}(6:(esn.Nr+esn.sizeinput-1))];
                        esn.Woutmat = [esn.Woutmat; esn.Wout{i}];

                    else    %this is training to plan   skipping the middle from x 
                        X = esn.train_reservoirReadoutWashed([1:3 6:(esn.Nr+esn.sizeinput+1)],:);
                        Y = esn.train_reservoirTargetWashed{i};

                        esn.Wout{i} = Y'*X'*inv(X*X'+esn.lambda*eye(size(X,1))); 
                        esn.Wout{i} = [esn.Wout{i}(1:3) 0 0 esn.Wout{i}(4:(esn.Nr+esn.sizeinput-1))];
                        esn.Woutmat = [esn.Woutmat; esn.Wout{i}];
                    end

                    disp(size(esn.Woutmat))

                    train_output{i} = esn.Wout{i}*esn.train_reservoirReadout;
                    train_output{i} = train_output{i}';
                    train_target{i} = esn.train_reservoirTarget{i};
                end
            else
                train_output = cell(esn.sizeoutput,1);
                train_target = cell(esn.sizeoutput,1);
                esn.Woutmat = [];

                for i = 1:esn.sizeoutput
                    X = esn.train_reservoirReadoutWashed;
                    Y = esn.train_reservoirTargetWashed{i};

                    esn.Wout{i} = Y'*X'*inv(X*X'+esn.lambda*eye(size(X,1))); 
                    esn.Woutmat = [esn.Woutmat; esn.Wout{i}];

                    train_output{i} = esn.Wout{i}*esn.train_reservoirReadout;
                    train_output{i} = train_output{i}';
                    train_target{i} = esn.train_reservoirTarget{i};
                end
            end

        end

        function [y, prY] = predict(esn, simname)
            % esn.plotThetaThetaDotPairs(esn.theta_matrix, esn.thetadot_matrix, esn.potential_matrix)
            y = cell(esn.sizeoutput,1);
            prY = cell(esn.sizeoutput,1);
        
            [const, x, target, internalState, internalStatedot] = esn.runsim(simname);

            esn.predict_internalState = internalState;
            esn.predict_internalStatedot = internalStatedot;

            esn.predict_reservoirReadout = [const'; x; esn.predict_internalState'];
%             esn.predict_reservoirReadout = [const'; esn.predict_internalState'];
            disp(size(esn.Wout{1}))
            disp(size(esn.predict_reservoirReadout))
            for i = 1:esn.sizeoutput
                prY{i} = target{i};
                y{i} = esn.Wout{i}*esn.predict_reservoirReadout;
                y{i} = y{i}';
            end

        end

        function [const, x, target, internalState, internalStatedot, tau] = runsim(esn, simname)
            
            tau = 0;
            out = sim(simname, TimeOut=60);
            switch simname
                  
                case {"PR_controlv8planning.slx", "PR_controlv8planning2.slx"}
                    disp('Running for this')
                    for i = 1:esn.Nr
                        internalState(:,i) = out.yout{2}.Values.S_esn.data(i,1,:);
                        internalStatedot(:,i) = out.yout{2}.Values.Sdot_esn.data(i,1,:);
                    end
                    
                    const = squeeze(out.yout{1}.Values.const.Data);
                    x = squeeze(out.yout{1}.Values.x.Data);

                    target = cell(esn.sizeoutput,1);
                    tg = squeeze(out.yout{1}.Values.target.Data);
                    target{1} = tg(1,:)';
                    target{2} = tg(2,:)';
                    target{3} = tg(3,:)';
                    tau = squeeze(out.yout{4}.Values.actuationtorque.Data);
                    disp(size(tau))

                case "esn.slx"
                    for i = 1:esn.Nr
                        internalState(:,i) = out.yout{1}.Values.S_esn.data(i,1,:);
                        internalStatedot(:,i) = out.yout{1}.Values.Sdot_esn.data(i,1,:);
                    end
                    const = out.yout{1}.Values.const.data(:);
                    x = squeeze(out.yout{1}.Values.xplan.data(:,1,:))';
                    target = cell(20,1);
                    targets = squeeze(out.yout{1}.Values.target.data);
                    for i = 1:20
                        target{i} = targets(i,:)';
                    end


            end

        end

    end
end