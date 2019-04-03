classdef QnetDCMotor < handle
    %QNETDCMOTOR Stimulation of a DC motor.
    %   This class implement an interface to QNET Allied Motion CL40 Series
    %   Coreless DC Motor (model 16705). The motor is driven through an
    %   anlogue input (voltage). The motor is equiped with three sensors:
    %   current (A), angular velocity tachometer (units/s) and angle position
    %   encoder (units).
    %   * Syntax:
    %       Motor = QnetDCMotor();
    %   A QnetDCMotor object is interfaced through the function <drive>
    %   * Syntax:
    %       Motor.drive(0,0,1);   % send 0 volt at time 0 for 1 second.
    %       Motor.drive(2,[],3);  % send 2 volts for 3 second.
    %       Motor.drive(-2,10,1); % At time = 10 send -2 volts for 1 second.
    %                             % keep applying the previous voltage untill
    %                             % time = 10 seconds.
    %       Motor.drive(0);       % send 0 volts for one sample time.
    %   A QnetDCMotor object provides a history of time, applied voltage,
    %   armature current, angular velocity and motor angle.
    %   * Syntax:
    %       time = Motor.time;         % get the time from the last reset
    %       voltage = Motor.voltage;   % get the voltage (V).
    %       current = Motor.current;   % get the current electric current (A).
    %       velocity = Motor.velocity; % get the current angluar velocity (rad/s).
    %       angle = Motor.angle;       % get the current angle (deg).
    %
    %   August 2017; Last revision: 04-Nov-2017.
    
    properties (GetAccess = public, SetAccess = private)
        Power = false;       % DC motor Power. use ON and OFF to set and reset Motor.
        SamplingTime = 0.01; % Sampling time (seconds), use SETSAMPLINGTIME to modify sampling time.
    end
    
    properties (GetAccess = public, SetAccess = public)
        MaxVoltage = 5.0;    % Maximum voltage to send to DC motor
        MinVoltage = -5.0;   % Minimum voltage to send to DC motor
        Units = 'rad';       % Determines the units of the angle, use 'deg' for degrees or 'rad' for radians.
        BufferSize = 10*60;  % Buffer size in seconds, changes in buffer size are only effective after a motor RESET.
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
        
        StartTime = []; % Holds the time when first drive command is issued, internal variable used to sync the DC Motor time
        
        Sess = [];  % structure to hold Data Acquisission sessions
        Voltage = 0; % last changed voltage
        
        Dist;
    end
    
    methods (Access = public)
        function this = QnetDCMotor()
            %QNETDCMOTOR Constructor.
            % set up all Data Acquisission sessions, set up default sampling time, set up
            % scope parameter
            
            try
                this.setUpChannels();
                
                % set default Sampling Time
                this.setSamplingTime(this.SamplingTime);
                
                % set on
                this.on();
            catch Except
                this.Sess = [];
                error('QnetDCMotor::Constructor::%s:: Qnet DC Motor is not connected or already in use.\n Make sure Qnet DC Motor is connected to this computer.\n If you are using Qnet DC Motor in another script please call clear on the used motor object or call <daq.reset>.\n If you are using it in an external program exit that program.\n If nothing works restart MATLAB!', 'error')
            end
            
            % initialize monitoring utilities
            this.createFigure();
            this.createScope();
        end
        
        function delete(this)
            %DELETE Safely disconnect motor.
            if(~isempty(this.Sess))
                % reset drive
                if(isvalid(this.Sess.drive))
                    this.Sess.drive.outputSingleScan(0);
                    pause(0.5);
                    this.Sess.drive.stop;
                    this.Sess.drive.release;
                end
                
                % reste power
                if(isvalid(this.Sess.power))
                    this.Sess.power.outputSingleScan([0, 1]);
                    pause(1.0);
                    this.Sess.power.stop;
                    this.Sess.power.release;
                end
                
                %reset monitor
                if(isvalid(this.Sess.monitor))
                    this.Sess.monitor.stop;
                    this.Sess.monitor.release;
                end
                
                this.Sess = [];
            end
        end
        
        function setSamplingTime(this, dt_)
            %SETSAMPLINGTIME Set the sampling time (seconds).
            this.SamplingTime = dt_;
            
            prevStatus = this.Power;
            this.off;
            
            % rest monitoring Rate
            this.Sess.monitor.Rate = 1/dt_; % Hz
            this.Sess.monitor.NotifyWhenDataAvailableExceeds = 1;
            if(this.Sess.monitor.NotifyWhenDataAvailableExceeds > this.BufferSize)
                warning('QnetDCMotor::setSamplingTime::Warning::Insufficient BufferSize %d for the request Sampling time %d. Buffer size increased to %d.',...
                    this.BufferSize,...
                    dt_,...
                    this.Sess.monitor.NotifyWhenDataAvailableExceeds);
                this.BufferSize = this.Sess.monitor.NotifyWhenDataAvailableExceeds;
            end
            
            % reset driving Rate
            this.Sess.drive.Rate = 1/dt_; % Hz
            this.Sess.drive.NotifyWhenScansQueuedBelow = 1;
            
            if(prevStatus)
                this.on;
            end
        end
        
        function on(this)
            %ON set ON DC motor.
            if(~this.Power)
                % reset internal buffers
                this.resetBuffers();
                
                % restart motor
                this.Sess.power.outputSingleScan([1, 0]);
                pause(0.5);
                
                % restart acquiring data
                this.Sess.monitor.startBackground();
                pause(1.0);
                
                % reset real time counter
                this.StartTime = this.TimeData(end);
                
                % set power flag to on
                this.Power = true;
            end
        end
        
        function off(this)
            %OFF set OFF DC motor.
            if(this.Power)
                % set disturabance to empty
                this.Dist = [];
                
                % reset motor
                this.drive(0);
                pause(0.5);
                
                this.Sess.power.outputSingleScan([0, 1]);
                pause(0.5);
                
                % stop acquiring data
                this.Sess.monitor.stop();
                pause(0.5);
                
                % set power flag to off
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
        
        function disturbance(this, time, value)
            %DISTURBANCE Program a speed disturbance for DC motor.
            % * Inputs:
            %  - time  : Time interval when the disturbance is applied.
            %  - value : Time (sec) when the voltage is applied. if time
            time = time(:);
            value = value(:);
            if(any(size(time) ~= size(value)))
                error('SimDCMotor::disturbance::Error: time and value should have the same size!');
            end
            this.Dist.Time = (time(1):this.SamplingTime:time(end))';
            this.Dist.Value = pchip(time, value, this.Dist.Time)/28.5;
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
            
            % save current time
            currentTime = this.TimeData(end);
            
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
                error('SimDCMotor::drive::Error: Voltage should be a scaler');
            end
            if(voltage > this.MaxVoltage)
                warning('SimDCMotor::drive::Warning: Value saturated to %fV', this.MaxVoltage);
            end
            if(voltage < this.MinVoltage)
                warning('SimDCMotor::drive::Warning: Value saturated to %fV', this.MinVoltage);
            end
            
            % set default time
            if(isempty(time))
                if(isempty(this.StartTime))
                    time = 0;
                else
                    time = currentTime - this.StartTime;
                end
            end
            
            if(time < currentTime - this.StartTime - 10*this.SamplingTime)
                warning('SimDCMotor::drive::Warning: Nothing to do, requested time %f behind current time %f, use RESET to reset time or give an empty time <[]> to drive from last time.', time, currentTime - this.StartTime);
                return
            end
            
            % saturate voltage
            voltage = min(this.MaxVoltage, voltage);
            voltage = max(this.MinVoltage, voltage);
            
            % sleep untill requested time
            while(this.TimeData(end) - this.StartTime < time - 0.5*this.SamplingTime)
                pause(1e-4);
            end
            
            % update the voltage command
            this.Voltage = voltage;
            if(~isempty(this.Dist))
                idx = floor((this.TimeData(end) - this.StartTime - this.Dist.Time(1))/this.SamplingTime) + 1;
                if(idx >= 1 && idx < length(this.Dist.Time))
                    voltage = voltage + this.Dist.Value(idx);
                end
            end
            this.Sess.drive.outputSingleScan(voltage);
            
            % sleep for extra dTime
            while(this.TimeData(end) - this.StartTime < time + dTime - 0.5*this.SamplingTime)
                pause(1e-4);
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
        
        function val = time(this, start_, end_)
            %TIME Get history of time (sec).
            if(nargin < 2)
                start_ = -this.StartTime - 0.5*this.SamplingTime;
                end_ = inf;
            elseif(nargin < 3)
                end_ = start_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            elseif(nargin < 4)
                end_ = end_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            end
            val = this.TimeData((this.TimeData >= start_ + this.StartTime) & (this.TimeData <= end_ + this.StartTime)) - this.StartTime;
        end
        
        function val = angle(this, start_, end_)
            %ANGLE Get history of angle (rad or deg).
            if(nargin < 2)
                start_ = -this.StartTime - 0.5*this.SamplingTime;
                end_ = inf;
            elseif(nargin < 3)
                end_ = start_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            elseif(nargin < 4)
                end_ = end_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            end
            val = this.AngleData(this.TimeData >= start_ + this.StartTime & this.TimeData <= end_ + this.StartTime) * (180/pi)^strcmpi(this.Units, 'deg');
        end
        
        function val = velocity(this, start_, end_)
            %VELOCITY Get history of angular velocity (rad/s or deg/s).
            if(nargin < 2)
                start_ = -this.StartTime - 0.5*this.SamplingTime;
                end_ = inf;
            elseif(nargin < 3)
                end_ = start_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            elseif(nargin < 4)
                end_ = end_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            end
            val = this.SpeedData(this.TimeData >= start_ + this.StartTime & this.TimeData <= end_ + this.StartTime) * (180/pi)^strcmpi(this.Units, 'deg');
        end
        
        function val = current(this, start_, end_)
            %CURRENT Get history of current (A).
            if(nargin < 2)
                start_ = -this.StartTime - 0.5*this.SamplingTime;
                end_ = inf;
            elseif(nargin < 3)
                end_ = start_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            elseif(nargin < 4)
                end_ = end_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            end
            val = this.CurrentData(this.TimeData >= start_ + this.StartTime & this.TimeData <= end_ + this.StartTime);
        end
        
        function val = voltage(this, start_, end_)
            %VOLTAGE Get history of applied voltage (V).
            if(nargin < 2)
                start_ = -this.StartTime - 0.5*this.SamplingTime;
                end_ = inf;
            elseif(nargin < 3)
                end_ = start_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            elseif(nargin < 4)
                end_ = end_ + 0.5*this.SamplingTime;
                start_ = start_ - 0.5*this.SamplingTime;
            end
            val = this.VoltageData(this.TimeData >= start_ + this.StartTime & this.TimeData <= end_ + this.StartTime);
        end
    end
    
    methods (Access = private)
        function setUpChannels(this)
            %SETUPCHANNELS create all sessions and connect appropriate
            % reset any previous connection
            daq.reset;
            
            %channels
            this.Sess.monitor = daq.createSession('ni');
            this.Sess.monitor.IsContinuous = true;
            % input channel to read MotorEncoder
            this.Sess.monitor.addCounterInputChannel('Dev1', 'ctr0', 'Position');
            this.Sess.monitor.Channels(end).EncoderType = 'X4';
            this.Sess.monitor.Channels(end).ZResetEnable = true;
            this.Sess.monitor.Channels(end).ZResetValue = 2048;
            % input channel to read Current
            this.Sess.monitor.addAnalogInputChannel('Dev1', 'ai0', 'Voltage');
            % input channel to read Velocity
            this.Sess.monitor.addAnalogInputChannel('Dev1', 'ai4', 'Voltage');
            % add listner to acquire data
            this.Sess.monitor.addlistener('DataAvailable',@this.updateMeasurement);
            
            this.Sess.drive = daq.createSession('ni');
            % output channel to send voltage command
            this.Sess.drive.addAnalogOutputChannel('Dev1','ao0','Voltage');
            
            this.Sess.power = daq.createSession('ni');
            this.Sess.power.Rate = 10;
            % output channel to enable/disable amplifier
            this.Sess.power.addDigitalChannel('Dev1', 'Port0/Line21:22', 'OutputOnly');
        end
        
        function updateMeasurement(this, src, event)
            %%UPDATEMEASUREMENT Callback to update local buffers from DC motor
            % sensors
            
            % cycle
            if(event.TimeStamps(end) > this.TimeData(end))
                this.TimeData(1:end-src.NotifyWhenDataAvailableExceeds)    = this.TimeData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.VoltageData(1:end-src.NotifyWhenDataAvailableExceeds) = this.VoltageData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.CurrentData(1:end-src.NotifyWhenDataAvailableExceeds) = this.CurrentData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.SpeedData(1:end-src.NotifyWhenDataAvailableExceeds)   = this.SpeedData(src.NotifyWhenDataAvailableExceeds+1:end);
                this.AngleData(1:end-src.NotifyWhenDataAvailableExceeds)   = this.AngleData(src.NotifyWhenDataAvailableExceeds+1:end);
            end
            
            % update last data
            this.TimeData(end-src.NotifyWhenDataAvailableExceeds+1:end)    = event.TimeStamps;
            this.VoltageData(end-src.NotifyWhenDataAvailableExceeds+1:end) = this.Voltage*ones(size(event.TimeStamps));
            this.CurrentData(end-src.NotifyWhenDataAvailableExceeds+1:end) = event.Data(:,2) * 0.59;
            this.SpeedData(end-src.NotifyWhenDataAvailableExceeds+1:end)   = event.Data(:,3) * 78;
            unsignedAngle = event.Data(:,1);
            unsignedAngle(unsignedAngle > 2^31) = unsignedAngle(unsignedAngle > 2^31) - 2^32;
            this.AngleData(end-src.NotifyWhenDataAvailableExceeds+1:end)   = unsignedAngle * (pi / 1024);
        end
        
        function resetBuffers(this)
            %RESETBUFFERS reset internal buffer and set time to 0.
            this.TimeData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.VoltageData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.CurrentData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.SpeedData = nan(ceil(this.BufferSize/this.SamplingTime),1);
            this.AngleData = nan(ceil(this.BufferSize/this.SamplingTime),1);
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

