classdef SimDCMotor < handle
    %SIMDCMOTOR Simulation of a DC motor
    %   This class implement a simulation of a coreless DC Motor. The motor
    %   is driven through an anlogue input (voltage). The motor is equiped
    %   with three sensors: current (A), angular velocity tachometer
    %   (units/s) and angle position encoder (units).
    %   SimDCMotor can be initialized using a custom resistance and
    %   inertia.
    %   * Syntax:
    %       Parameters.Inertia = 5e-4;      % Rotor inertia (kg m2)
    %       Parameters.Resistance = 1.5;    % Terminal resistance (Ohm)
    %       Motor = SimDCMotor(Parameters); % create a handle to simDCMotor
    %   A SimDCMotor object is interfaced through the function <drive>
    %   * Syntax:
    %       Motor.drive(0,0,1);   % send 0 volt at time 0 for 1 second.
    %       Motor.drive(2,[],3);  % send 2 volts for 3 second.
    %       Motor.drive(-2,10,1); % At time = 10 send -2 volts for 1 second.
    %                             % keep applying the previous voltage untill
    %                             % time = 10 seconds.
    %       Motor.drive(0);       % send 0 volts for one sample time.
    %   A SimDCMotor object provides a history of time, applied voltage,
    %   armature current, angular velocity and motor angle.
    %   * Syntax:
    %       time = Motor.time;       % get the time from the last reset
    %       voltage = Motor.voltage; % get the voltage (V).
    %       current = Motor.current; % get the current electric current (A).
    %       velocity = Motor.velocity;     % get the current angluar velocity (rad/s).
    %       angle = Motor.angle;     % get the current angle (deg).
    %
    %   August 2017; Last revision: 24-Sept-2017.
    
    properties (GetAccess = public, SetAccess = private)
        Power = false;       % DC motor Power. use ON and OFF to set and reset Motor.
        SamplingTime = 0.01; % Sampling time (seconds), use SETSAMPLINGTIME to modify sampling time.
        Parameters = [];     % Structure to hold DC motor parameter
    end
    
    properties (GetAccess = public, SetAccess = public)
        MaxVoltage = 5.0;    % Maximum voltage to send to DC motor
        MinVoltage = -5.0;   % Minimum voltage to send to DC motor
        Units = 'rad';       % Determines the units of the angle, use 'deg' for degrees or 'rad' for radians.
        BufferSize = 10*60;  % Buffer size in seconds, changes in buffer size are only effective after a motor RESET.
        RealTimeFactor = 0;  % Factor which adjust the real time speed of the simulation. Choose 1 for realtime simulation 0: for instantenous run.
    end
    
    properties (GetAccess = private, SetAccess = private)
        % Data buffers
        TimeData = [];
        VoltageData = [];
        CurrentData = [];
        SpeedData = [];
        AngleData = [];
        
        % Scope parameter
        ScopeFig = [];
        ScopeTimer = [];
        ScopeAxes = [];
        
        StartTime = []; % Holds the time when first drive command is issued, used for real-time simulation.
        
        SS;  % Structure to hold state space representation of DC Motor
    end
    
    methods (Access = public)
        function this = SimDCMotor(varargin)
            %SIMDCMOTOR Constructor
            % set up all DC motor parameters, set up default sampling time, set up
            % scope parameter.
            
            % set-up default parameter
            this.Parameters.R = 1.5;    % Terminal resistance (Ohm)
            this.Parameters.J = 5e-4;   % Rotor inertia (kg.m2)
            this.Parameters.B = 1e-6;   % Friction (Kg.m2/s) (N.m.s)
            this.Parameters.Kt = 0.05;  % Torque constant (N.m/A)
            this.Parameters.Ke = 0.05;  % Motor back-emf constant (V/rad/s)
            this.Parameters.L = 0.5;    % Rotor inductance (H)
            % get options
            if(nargin == 1)
                if(isstruct(varargin{1}))
                    fields = fieldnames(varargin{1});
                    for iField = 1:numel(fields)
                        switch lower(fields{iField})
                            case 'resistance'
                                this.Parameters.R = varargin{1}.(fields{iField});
                            case 'inertia'
                                this.Parameters.J = varargin{1}.(fields{iField});
                            otherwise
                                error('SimDCMotor::Constructor::Error: unkown DC motor parameter %s, fields may be: Resistance, Inductance or Inertia.', name)
                        end
                    end
                else
                    error('SimDCMotor::Constructor::Error: Argument should be a structur containing DC motor parameter, fields may be: Resistance, Inductance or Inertia.')
                end
            elseif(nargin > 1)
                error('SimDCMotor::Constructor::Error: Too much arguments!')
            end
            
            % set default Sampling Time
            this.setSamplingTime(this.SamplingTime);
            
            % set on
            this.on();
            
            % initialize monitoring utilities
            this.createFigure();
            this.createScope();
        end
        
        function setSamplingTime(this, dt_)
            %SETSAMPLINGTIME Set the sampling time (seconds).
            this.SamplingTime = dt_;
            
            % state X = [angle, velocity, current]
            A = [...
                0, 1                       , 0                        ;...
                0, -this.Parameters.B/this.Parameters.J, this.Parameters.Kt/this.Parameters.J  ;...
                0, -this.Parameters.Ke/this.Parameters.L, -this.Parameters.R/this.Parameters.L];
            
            B = [...
                0;...
                0;...
                1/this.Parameters.L];
            
            C = eye(3);
            
            this.SS.n = length(A);
            this.SS.m = size(B, 2);
            this.SS.p = size(C, 1);
            
            this.SS.X = zeros(this.SS.n,1);
            
            % get exact discrete form
            S = expm([[A B]*this.SamplingTime; zeros(this.SS.m,this.SS.n+this.SS.m)]);
            this.SS.A = S(1:this.SS.n, 1:this.SS.n);
            this.SS.B = S(1:this.SS.n, this.SS.n+1:this.SS.n+this.SS.m);
            this.SS.C = C;
        end
        
        function on(this)
            %ON set ON DC motor.
            if(~this.Power)
                % reset internal buffers
                this.resetBuffers();
                
                % reset real time counter
                this.StartTime = [];
                
                % set status to on
                this.Power = true;
            end
        end
        
        function off(this)
            %OFF set OFF DC motor.
            if(this.Power)
                this.Power = false;
            end
        end
        
        function reset(this)
            %RESET Reset DC motor.
            % Clear all buffers, power off and
            % on the DC motor, reset DC motor internal timer.
            this.off();
            this.on();
        end
        
        function drive(this, voltage, time, dTime)
            %DRIVE Drives DC motor
            % Drives DC motor with "voltage" at "time" during specified
            % "dTime". if RealTimeFactor is non-null Matlab will be
            % blocking during this time.
            % * Inputs:
            %  - voltage: Voltage (V) to be sent to DC motor.
            %  - time   : Time (sec) when the voltage is applied. if time
            %  is empty [], last time is used.
            %  - dTime  : Duration (sec) of the drive command. if dTime is
            %  not specified dTime is equal to sampling time SamplingTime
            % * Syntax:
            %   motor.drive(2, 1, 5), will send 2 volt at time 1 for 5 sec
            
            % handle warnings and errors
            if(~this.Power)
                warning('SimDCMotor::drive::Warning: Nothing to do, Motor is off!');
                return
            end
            if(nargin < 2)
                error('SimDCMotor::drive::Error: Not enough arguments, use help SimDCMotor.drive!');
            elseif(nargin == 2)
                time = [];
                dTime = this.SamplingTime;
            elseif(nargin == 3)
                dTime = this.SamplingTime;
            elseif(nargin > 5)
                error('SimDCMotor::drive::Error: Too much arguments, use help SimDCMotor.drive!');
            end
            if(~isscalar(voltage))
                error('SimDCMotor::drive::Error: Voltage should be a scaler value.');
            end
            if(~isempty(time))
                if(~isscalar(time))
                    error('SimDCMotor::drive::Error: Time should be a scaler value.');
                elseif(time < 0)
                    error('SimDCMotor::drive::Error: Time should be bigger than zero.');
                end
            end
            if(~isscalar(dTime))
                error('SimDCMotor::drive::Error: Time should be a scaler value.');
            elseif(dTime <= 0)
                error('SimDCMotor::drive::Error: Time should be bigger than zero.');
            end
            if(voltage > this.MaxVoltage)
                warning('SimDCMotor::drive::Warning: Value saturated to %fV', this.MaxVoltage);
            end
            if(voltage < this.MinVoltage)
                warning('SimDCMotor::drive::Warning: Value saturated to %fV', this.MinVoltage);
            end
            
            % set default time
            if(isempty(time))
                time = this.TimeData(end);
            end
            
            % simulate real time if requested
            if(this.RealTimeFactor > 0 && isempty(this.StartTime))
                this.StartTime = tic;
            end
            
            if(time < this.TimeData(end) - 10*this.SamplingTime)
                warning('SimDCMotor::drive::Warning: Nothing to do, requested time %f behind current time %f, use RESET to reset time or give an empty time <[]> to drive from last time.', time, this.TimeData(end));
                return
            end
            
            % saturate voltage
            voltage = min(this.MaxVoltage, voltage);
            voltage = max(this.MinVoltage, voltage);
            
            % check number of iteration to run
            nbrIter = (time + dTime - this.TimeData(end))/this.SamplingTime;
            if(nbrIter < 1) % simulate down-sampling
                if(nbrIter < this.SamplingTime)
                    nbrIter = 0;
                else
                    nbrIter = 1;
                end
            else
                nbrIter = round(nbrIter);
            end
            
            % drive
            stopRealTime = 0;
            Y = nan(3, nbrIter);
            for k = 1:nbrIter
                this.TimeData(1:end-1)        = this.TimeData(2:end);
                this.TimeData(end)            = this.TimeData(end-1) + this.SamplingTime;
                this.VoltageData(1:end-1)     = this.VoltageData(2:end);
                
                if(this.TimeData(end) < time)
                    this.SS.X = this.SS.A*this.SS.X + this.SS.B*this.VoltageData(end);
                    this.VoltageData(end)         = this.VoltageData(end);
                else
                    this.SS.X = this.SS.A*this.SS.X + this.SS.B*voltage;
                    this.VoltageData(end)         = voltage;
                end
                Y(:,k) = this.SS.C*this.SS.X;
                                
                % simulate real-time
                if(this.RealTimeFactor > 0)
                    this.CurrentData(1:end-1)     = this.CurrentData(2:end);
                    this.CurrentData(end)         = Y(3, k);
                    this.SpeedData(1:end-1)       = this.SpeedData(2:end);
                    this.SpeedData(end)           = Y(2, k);
                    this.AngleData(1:end-1)       = this.AngleData(2:end);
                    this.AngleData(end)           = Y(1, k);
                    if(isempty(this.StartTime))
                        warning('SimDCMotor::drive::Warning: can''t simulate realtime, please use RESET before driving motor!')
                    else
                        d = this.RealTimeFactor*this.TimeData(end) - toc(this.StartTime);
                        if(d > 0.5*this.SamplingTime)
                            pause(d)
                        elseif(d < -0.5*this.SamplingTime && ~stopRealTime)
                            warning('SimDCMotor::drive::Warning: t = %f realtime simulation failed, delay = %f.\n Please be aware that realtime plotting might slow down Matlab a sampling rate of 20Hz is recommended in this case.', time, d);
                            if(d < -5*this.SamplingTime)
                                warning('SimDCMotor::drive::Warning: Delay is %f too big compared to sampling time %f. Real-time flag will be reset!', d, this.SamplingTime);
                                stopRealTime = 1;
                            end
                        end
                    end
                end
            end
            if(this.RealTimeFactor == 0)
                this.CurrentData(1:end-nbrIter)     = this.CurrentData(nbrIter+1:end);
                this.CurrentData(end-nbrIter+1:end) = Y(3, :);
                this.SpeedData(1:end-nbrIter)       = this.SpeedData(nbrIter+1:end);
                this.SpeedData(end-nbrIter+1:end)   = Y(2, :);
                this.AngleData(1:end-nbrIter)       = this.AngleData(nbrIter+1:end);
                this.AngleData(end-nbrIter+1:end)   = Y(1, :);
            end
            if(stopRealTime)
                this.RealTimeFactor = 0;
            end
        end
        
        function scope(this, bufferTime)
            %SCOPE Opens a figure to monitor the DC motor, type 'p' if you
            %want to take a snapshot of the scope or 'ESC' to close the
            %scope.
            if(nargin < 2)
                bufferTime = min(30, this.BufferSize); % default 30 seconds
            end
            if(~this.ScopeFig.isvalid)
                this.createFigure();
            end
            figure(this.ScopeFig);
            this.ScopeAxes.Voltage = subplot(2,2,1);
            this.ScopeAxes.Velocity = subplot(2,2,2);
            this.ScopeAxes.Current = subplot(2,2,3);
            this.ScopeAxes.Angle = subplot(2,2,4);
            if(~isvalid(this.ScopeTimer))
                this.createScope();
            end
            stop(this.ScopeTimer);
            this.ScopeTimer.TimerFcn  = @(src, event)this.updateScope(src, event, bufferTime);
            start(this.ScopeTimer);
        end
        
        function val = time(this)
            %TIME Get history of time (sec).
            val = this.TimeData(~isnan(this.TimeData));
        end
        
        function val = angle(this)
            %ANGLE Get history of angle (rad or deg).
            val = this.AngleData(~isnan(this.TimeData)) * (180/pi)^strcmpi(this.Units, 'deg');
        end
        
        function val = velocity(this)
            %VELOCITY Get history of angular velocity (rad/s or deg/s).
            val = this.SpeedData(~isnan(this.TimeData)) * (180/pi)^strcmpi(this.Units, 'deg');
        end
        
        function val = current(this)
            %CURRENT Get history of current (A).
            val = this.CurrentData(~isnan(this.TimeData));
        end
        
        function val = voltage(this)
            %VOLTAGE Get history of applied voltage (V).
            val = this.VoltageData(~isnan(this.TimeData));
        end
    end
    
    methods (Access = private)
        function resetBuffers(this)
            %RESETBUFFERS reset internal buffer and set time to 0.
            this.TimeData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.VoltageData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.CurrentData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.SpeedData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.AngleData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            
            this.SS.X = zeros(this.SS.n,1);
            Y = this.SS.C*this.SS.X;
            this.TimeData(end)            = 0;
            this.VoltageData(end)         = 0;
            this.CurrentData(end)         = Y(3);
            this.SpeedData(end)           = Y(2);
            this.AngleData(end)           = Y(1);
        end
        function createFigure(this)
            %CREATEFIGURE create figure for scope
            this.ScopeFig = findobj(groot, 'Number', 7070707);
            if(isempty(this.ScopeFig))
                this.ScopeFig = figure(7070707);
            end
            this.ScopeFig.Visible = 'off';
            clf(this.ScopeFig);
            this.ScopeFig.Name = 'DC Motor Monitor';
            this.ScopeFig.KeyPressFcn = 'set(gcbf,''UserData'',double(get(gcbf,''Currentcharacter''))) ; uiresume ';
            this.ScopeFig.NumberTitle = 'off';
        end
        function createScope(this)
            %CREATESCOPE create scope
            
            % clean up all previous scopes, if any.
            for t = timerfind('Name', 'DC-Motor-Monitor')
                stop(t);
                delete(t);
            end
            % set up scope parameter
            this.ScopeTimer = timer;
            this.ScopeTimer.Period = 0.2;
            this.ScopeTimer.ExecutionMode = 'fixedRate';
            this.ScopeTimer.Name = 'DC-Motor-Monitor';
        end
        function updateScope(this, src, ~, bufferTime)
            %UPDATESCOPE Callback to update the scope plot
            if(~this.ScopeFig.isvalid)
                stop(src)
            elseif(~isempty(this.ScopeFig.UserData))
                switch(this.ScopeFig.UserData)
                    case 27 % ESC key
                        this.ScopeFig.Visible = 'off';
                        stop(src)
                    case 'p'
                        savefig(this.ScopeFig, ['Snapshot_DC_Motor_' datestr(now, 'yymmmdd_HHMMSS')]);
                    otherwise
                        fprintf('Pressed <%s> ... Use ''p'' to save a snapshot of the figure, ''ESC'' to close monitor\n', char(this.ScopeFig.UserData))
                end
                this.ScopeFig.UserData = [];
            else
                idx = find(this.TimeData >= bufferTime*floor(this.TimeData(end)/bufferTime), 1);
                if(~isempty(idx))
                    plot(this.ScopeAxes.Voltage, this.TimeData(idx:end), this.VoltageData(idx:end))
                    xlim(this.ScopeAxes.Voltage, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0  1]))
                    
                    plot(this.ScopeAxes.Current, this.TimeData(idx:end), this.CurrentData(idx:end))
                    xlim(this.ScopeAxes.Current, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0  1]))
                    
                    plot(this.ScopeAxes.Velocity, this.TimeData(idx:end), this.SpeedData(idx:end) * (180/pi)^strcmpi(this.Units, 'deg'))
                    xlim(this.ScopeAxes.Velocity, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0  1]))
                    grid(this.ScopeAxes.Velocity, 'on');
                    title(this.ScopeAxes.Velocity, sprintf('Angular Velocity (%s/s)', this.Units))
                    
                    plot(this.ScopeAxes.Angle, this.TimeData(idx:end), this.AngleData(idx:end) * (180/pi)^strcmpi(this.Units, 'deg'))
                    xlim(this.ScopeAxes.Angle, bufferTime*(floor(this.TimeData(end)/bufferTime) + [0  1]))
                end
                grid(this.ScopeAxes.Voltage, 'on');
                title(this.ScopeAxes.Voltage, 'Voltage (V)')
                grid(this.ScopeAxes.Current, 'on');
                title(this.ScopeAxes.Current, 'Current (A)')
                grid(this.ScopeAxes.Angle, 'on');
                title(this.ScopeAxes.Angle, sprintf('Angle (%s)', this.Units))
            end
        end
    end
end

