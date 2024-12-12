function myurviz(group_number, use_mqtt, broker_address, visuals, frames, step_size)
%% Assign Default Input Values Based on Provided Function Arguments
    if nargin < 6
        % default inverse kinematics step size if not specified
        step_size = 0.1;
    end

    if nargin < 5
        frames = 'off';
    end
    
    if nargin < 4
      visuals = 'on';
    end

    if nargin < 3
        % defualt to my broker if not specified
        %broker_address = "10.243.82.33:1883";
        broker_address = "10.0.0.25";
    end

    if nargin < 2
        % default to no mqtt if parameter is not specified
        use_mqtt = false;
    end

    %% MQTT Setup

   if use_mqtt
        brokerAddress = strcat("tcp://",broker_address,":1883");
        ClientID = strcat("group",num2str(group_number),"matlab");
        topic_angles = strcat("group",num2str(group_number),"_joint_angles");
        topic_gripper = strcat("group",num2str(group_number),"_gripper");
        topic_status = strcat("group",num2str(group_number),"_status");
        topic_get_pose = strcat("group",num2str(group_number),"_getpose");
        topic_feedback = strcat("group",num2str(group_number),"_pose");
        
        % Establish MQTT connection
        Client = connect_mqtt(brokerAddress, ClientID);
        if isempty(Client)
            disp('[ERROR] Failed to connect to MQTT broker at startup');
            return;
        end

        % Publish "done" to indicate readiness
        write(Client, topic_status, "done");
        disp('[INFO] Published "done" to status MQTT topic on startup');

        subscribe(Client, topic_angles);
        disp('[INFO] Subscribed to joint angles MQTT topic on startup');

        subscribe(Client, topic_gripper);
        disp('[INFO] Subscribed to gripper topic on startup');
    
        subscribe(Client, topic_get_pose);
        disp('[INFO] Subscribed to pose request MQTT topic on startup');

    else
        disp('[INFO] MQTT functionality disabled at startup');
        Client = [];
    end
    
    %% Robot Config Variables
    global pose joint_angles last_angles last_pose gripper_pos stopProgram...
           sliderLock default_joint_timespan default_gripper_timespan ...
           recordTrajectory trajectoryPoints trajectoryPlot;
        
    pose = [0, -pi/2, pi/2, 0, 0, 0, 0, 0]; % Initial position including end-effector joints
    joint_angles = pose(1:6);
    last_angles = joint_angles;
    gripper_pos = 0; % Initial end-effector position (0 = fully closed)
    default_joint_timespan = 3.0; % Default time for joint motion
    default_gripper_timespan = 1.0; % Default time for gripper motion
    recordTrajectory = false;
    trajectoryPoints = [];
    trajectoryPlot = [];

    %% Program Config Variables and Flags
    stopProgram = false;
    busy = false;
    moveQueue = {};
    sliderLock = false(1, 7); % Array to lock joint sliders to avoid double movement
    
    %% Create Base UI Figure
    fig_name = 'Robot Joint and End-Effector Control';
    ui_fig = uifigure('Name', fig_name, 'Position', [0 0 1500 875], ...
                      'CloseRequestFcn', @(src, event) closeProgram()); % callback to close figure when window close is requested
    
    gl = uigridlayout(ui_fig, [1, 2]); % configure ui to have 1 row and 2 columns
    gl.ColumnWidth = {'3x', '1.5x'}; % Adjust to make space for controls on the right
    
    % Create Robot Viz Axes
        ax = axes(gl);
        ax.Title.String = 'Robot Visualizer';
        hold(ax, 'on');
        view(ax, 3);
        axis(ax, 'vis3d');
        rotate3d(ax, 'on');
        ax.XLim = [-0.7 0.7];
        ax.YLim = [-0.7 0.7];
        ax.ZLim = [-0.5 0.8];
    
    %% Load UR3 Robot in intial configuration
    robot = loadrobot('universalUR3e', ...
        'DataFormat', 'row', ...
        'Gravity', [0, 0, -9.81] ...
        );
    % Add end effector to robot
    end_effector = load("robotiq_hande.mat").robotiq_hande; % custom end effector config file saved as .mat to improve loading speed
    addSubtree(robot,"tool0",end_effector);

    show(robot, pose, 'Parent', ax, 'Visuals', visuals, 'Frames', frames, 'PreservePlot', false);
    % Visual adjustments
    camlight(ax, "headlight"); % adjust lighting
    create_ground_plane([-0.5, 0.5], [-0.5, 0.5], [0 1 0], 0.5, ax);
    disp('[DEBUG] Robot visualization initialized');
    
    %% Add Robot Controls to UI
    % initalize container panel for sub control panels
    control_panel = uipanel(gl, 'Title', 'Robot Controls');
    control_layout = uigridlayout(control_panel, [3, 1]);
    control_layout.RowHeight = {'1x', '0.25x', '1x'};

        % Initialize sliders for joint angles
        slider_panel = uipanel(control_layout, 'Title', 'Joint Angle Control');
        slider_layout = uigridlayout(slider_panel, [6, 3]);
        
        % Set dynamic row and column sizes
        slider_layout.RowHeight = repmat({'fit'}, 1, 6); % Adjust row height for each slider
        slider_layout.ColumnWidth = {'fit', '2,5x', 'fit'}; % Dynamic width allocation
        
        % Create sliders and displays for each joint
        sliders = gobjects(1, 6);
        slider_displays = gobjects(1, 6);
        slider_labels = ["Joint 1", "Joint 2", "Joint 3", "Joint 4", "Joint 5", "Joint 6"];
        
        for i = 1:6
            % Add label to the left column
            label = uilabel(slider_layout, 'Text', slider_labels(i), 'HorizontalAlignment', 'left');
            label.Layout.Row = i;
            label.Layout.Column = 1;
        
            % Add slider to the middle column
            sliders(i) = uislider(slider_layout, ...
                                  'Limits', [-pi, pi], ...
                                  'Value', joint_angles(i), ...
                                  'MajorTicks', [-pi, -pi/2, 0, pi/2, pi], ...
                                  'ValueChangedFcn', @(src, event) handle_joint_angles_slider(src, i));
            sliders(i).Layout.Row = i;
            sliders(i).Layout.Column = 2;
        
            % Add display field to the right column
            slider_displays(i) = uieditfield(slider_layout, 'numeric', ...
                                             'Value', joint_angles(i), ...
                                             'Editable', 'off', ...
                                             'HorizontalAlignment', 'center',...
                                             'ValueDisplayFormat', '%.3f');
            slider_displays(i).Layout.Row = i;
            slider_displays(i).Layout.Column = 3;
        end

        
        % Gripper control slider
        gripper_panel = uipanel(control_layout, 'Title', 'End-Effector Control (Gripper % Open)');
        gripper_layout = uigridlayout(gripper_panel, [3,3]);
        gripper_layout.RowHeight = {'0.25x', 'fit', '0.25x'};
        gripper_layout.ColumnWidth = {'0.1x', '1x', '0.1x'};
        gripper_slider = uislider(gripper_layout, ...
                                   'Limits', [0, 100], ...
                                   'Value', gripper_pos, ...
                                   'MajorTicks', [0, 25, 50, 75, 100], ...
                                   'ValueChangedFcn', @(src, event) handle_gripper_slider(src));
        gripper_slider.Layout.Row = 2;
        gripper_slider.Layout.Column = 2;
        
        % Gripper control panel with 3x2 button layout for spatial control
        arrow_pad = uipanel(control_layout, 'Title', 'End-Effector Movement');
        pad_layout = uigridlayout(arrow_pad, [3, 2]);
    
        % Buttons for end-effector movement
        uibutton(pad_layout, 'Text', '+Y', 'ButtonPushedFcn', @(src, event) queueMovement([0, step_size, 0]));
        uibutton(pad_layout, 'Text', '-Y', 'ButtonPushedFcn', @(src, event) queueMovement([0, -step_size, 0]));
        uibutton(pad_layout, 'Text', '-X', 'ButtonPushedFcn', @(src, event) queueMovement([-step_size, 0, 0]));
        uibutton(pad_layout, 'Text', '+X', 'ButtonPushedFcn', @(src, event) queueMovement([step_size, 0, 0]));
        uibutton(pad_layout, 'Text', '+Z', 'ButtonPushedFcn', @(src, event) queueMovement([0, 0, step_size]));
        uibutton(pad_layout, 'Text', '-Z', 'ButtonPushedFcn', @(src, event) queueMovement([0, 0, -step_size]));
        
        % Indicator light for robot status
        off_color = [235 235 235]/255;
        lamp_position = [830, 800, 20, 20];
        status_lamp = uilamp(ui_fig, 'Position', lamp_position, 'Color', off_color);
        status_label = uilabel(ui_fig, ...
            'Text', 'Action in Progress', ... 
            'Position', lamp_position + [30, 0, 100, 0], ... % Adjust the position and size of the label
            'HorizontalAlignment', 'left'); % Align text to the right for proximity to the lamp
        
        % record trajectory button
        uibutton(ui_fig, ...
            'Text', 'Show Trajectory', ...
            'Position', lamp_position - [0 40 -100 -10], ...
            'ButtonPushedFcn', @(src, event) toggle_trajectory());

        % clear trajectory button
        uibutton(ui_fig, ...
            'Text', 'Clear Trajectory', ...
            'Position', lamp_position - [0 80 -100 -10], ...
            'ButtonPushedFcn', @(src, event) clear_trajectory());

    %% MQTT Message Handling Loop
    if use_mqtt
        try
            while ishghandle(ui_fig) && ~stopProgram
                % Check for MQTT messages
                if isempty(Client)
                    % Reconnect if disconnected
                    disp('[ERROR] MQTT connection lost. Reconnecting...');
                    Client = connect_mqtt(brokerAddress, ClientID);
                    if isempty(Client)
                        disp('[ERROR] MQTT reconnection failed');
                        pause(2);
                        continue;
                    end
                    subscribe(Client, topic_angles);
                    subscribe(Client, topic_get_pose);
                    subscribe(Client, topic_gripper); % Add subscription for gripper topic
                    disp('[INFO] Resubscribed to MQTT topics after reconnection');
                end
                
                % Read any available message
                msg = read(Client);
                if ~isempty(msg)
                    % Handle messages based on topic
                    if strcmp(msg.Topic, topic_angles)
                        handle_joint_angles_message(msg.Data);
                    elseif strcmp(msg.Topic, topic_get_pose)
                        handle_pose_request(msg.Data);
                    elseif strcmp(msg.Topic, topic_gripper) % Handle gripper topic
                        handle_gripper_message(msg.Data);
                    end
                end
                
                pause(0.1); % Avoid busy-waiting
            end
        catch ME
            disp('[ERROR] Program interrupted or encountered an error');
            disp(['[DEBUG] Error Message: ', ME.message]);
            stop_mqtt(Client);
            rethrow(ME);
        end
    end

    %% Callback and Helper Function Definitions
% ----------------------------------------------------------------------- %
%                         ROBOT PLOTTING FUNCTIONS
% ----------------------------------------------------------------------- %
    function update_pose(new_pose, timespan, is_slider_update)
        last_pose = pose;
        disp(['[DEBUG] New goal pose received: ' num2str(new_pose, 4)])
        
        % Define start time
        start_time = tic;
        status_lamp.Color = 'green';
    
        % Precompute spline trajectory for each joint
        t_spline = [0 1]; % Normalized time for spline
        num_joints = length(new_pose);
        spline_coeffs = cell(1, num_joints);
        for j = 1:num_joints
            spline_coeffs{j} = spline(t_spline, [last_pose(j), new_pose(j)]);
        end
    
        % Main loop to update pose
        trajectory_complete = false;
        while ~trajectory_complete
            % Calculate elapsed time and normalized fraction
            elapsed_time = toc(start_time);
            t_frac = min(elapsed_time / timespan, 1); % Normalized time fraction [0, 1]
            
            % Calculate current pose using spline interpolation
            current_pose = zeros(1, num_joints);
            for j = 1:num_joints
                current_pose(j) = ppval(spline_coeffs{j}, t_frac);
            end
    
            % Update robot visualization
            show(robot, current_pose, 'Parent', ax, 'Visuals', 'on', 'Frames', 'off', ...
                 'FastUpdate', true, 'PreservePlot', false);
            pose = current_pose; % Update global pose
            update_sliders();
    
            if recordTrajectory
                record_trajectory();
            end
    
            % Check if trajectory is complete
            if t_frac >= 1
                trajectory_complete = true;
            else
                pause(0.01); % Small pause to save CPU resources
            end
        end
    
        % Final updates after trajectory completion
        pose = new_pose; % Ensure global pose is set to the final goal
        elapsed_time = toc(start_time);
        beep; % MATLAB feedback beep
        disp(['[DEBUG] Pose updated to: ' num2str(pose, 4)])
        fprintf('[INFO] Pose update completed in %.3f seconds.\n', elapsed_time);
        status_lamp.Color = off_color;
        if is_slider_update
            sliderLock(:) = false;
        end
    end
% ----------------------------------------------------------------------- %    
    function create_ground_plane(x_range, y_range, color, alpha, ax)
    % Creates ground plane patch to show z=0 for robot visualization
        patch(ax, 'XData', [x_range(1), x_range(2), x_range(2), x_range(1)], ...
              'YData', [y_range(1), y_range(1), y_range(2), y_range(2)], ...
              'ZData', [0, 0, 0, 0], ...
              'FaceColor', color, ...
              'FaceAlpha', alpha, ...
              'EdgeColor', 'none');
    end
% ----------------------------------------------------------------------- %    
    function move_end_effector(delta)
        ik = inverseKinematics('RigidBodyTree', robot);
        end_effector = 'tool0';
    
        % Get current end-effector position and orientation
        tform = getTransform(robot, pose, end_effector);
        new_tform = tform;
        new_tform(1:3, 4) = new_tform(1:3, 4) + delta';
    
        % Solve IK for the new position
        [new_pose, ~] = ik(end_effector, new_tform, [1 1 1 1 1 1], pose);
        if isempty(new_pose)
            disp('[WARN] IK solution not found for requested delta movement');
            return;
        end
        
        update_pose(new_pose, default_joint_timespan*0.6, false);
    end
% ----------------------------------------------------------------------- %
    function queueMovement(delta)
        moveQueue{end+1} = delta;
        if ~busy
            processQueue();
        end
    end
% ----------------------------------------------------------------------- %  
    function processQueue()
        if isempty(moveQueue)
            busy = false;
            return;
        end
    
        busy = true;
        delta = moveQueue{1};
        moveQueue(1) = []; % Remove the movement from the queue
        move_end_effector(delta);
    
        % Process the queue again if it’s not empty
        busy = false;
        if ~isempty(moveQueue)
            processQueue();
        end
    end
% ----------------------------------------------------------------------- %
    function record_trajectory()
        % Get the current end-effector position
        tool0Transform = getTransform(robot, pose, 'tool0');
        tool0Position = tool0Transform(1:3, 4)'; % Extract 3D position
    
        % Append to trajectory points
        trajectoryPoints = [trajectoryPoints; tool0Position];
    
        % Update trajectory plot
        if isempty(trajectoryPlot)
            trajectoryPlot = plot3(ax, trajectoryPoints(:, 1), ...
                                       trajectoryPoints(:, 2), ...
                                       trajectoryPoints(:, 3), ...
                                       'r-', 'LineWidth', 2);
        else
            % Update existing plot
            set(trajectoryPlot, 'XData', trajectoryPoints(:, 1), ...
                                'YData', trajectoryPoints(:, 2), ...
                                'ZData', trajectoryPoints(:, 3));
        end
    end
% ----------------------------------------------------------------------- %
%                          MQTT CALLBACKS
% ----------------------------------------------------------------------- %
    function handle_gripper_message(data)
        
        default_gripper_timespan = 1.0;
        
        % Decode MQTT Message: extract gripper percent displacement and
        % time for trajectory or use default time.
        try
            msg = jsondecode(data);
       
            if isfield(msg, 'disp')
                gripper_percentage = msg.disp;
                grip_dist_target = 0.025 * (gripper_percentage / 100); % Target distance (0 to 0.025)
            else
                error('Missing "disp" field in message.');
            end
           
            if gripper_percentage < 0 || gripper_percentage > 100
                error('Invalid gripper percentage received. Expected a value between 0 and 100.');
            end
            
            if isfield(msg, 'time') && isnumeric(msg.time) && msg.time > 0
                timespan = msg.time;
            else
                disp('[WARN] Invalid or missing "time" field. Using default.');
                timespan = default_gripper_timespan;
            end

            new_pose = pose;
            new_pose(7:8) = [grip_dist_target, grip_dist_target];
            sliderLock(:) = false;
            update_pose(new_pose, timespan, false)
            
    
            % Publish "gripper_updated" acknowledgment
            write(Client, topic_status, "gripper_updated");
            disp('[INFO] Published "gripper_updated" acknowledgment to status topic');

        catch
            disp('[WARN] Invalid message format for gripper. Expected JSON format with "disp" and optional "time".');
            return;
        end
   
    end
% ----------------------------------------------------------------------- %
    function handle_joint_angles_message(data)
    
        default_timespan = default_joint_timespan;
    
        try
            % Parse the JSON message
            msg = jsondecode(data);
    
            % Check if it's a trajectory
            if isfield(msg, 'trajectory')
                trajectory = msg.trajectory;
    
                % Validate trajectory
                if ~isstruct(trajectory)
                    error('Trajectory must be a struct array of poses.');
                end
    
                % Iterate through each point in the trajectory
                previous_time = 0; % Initialize the previous time
    
                for ii = 1:length(trajectory)
                    point = trajectory(ii);
    
                    % Validate the "angles" field
                    if ~isfield(point, 'angles') || ~isnumeric(point.angles) || length(point.angles) ~= 6
                        error(['Invalid trajectory point ' num2str(ii) ': Missing or invalid "angles" field.']);
                    end
    
                    % Validate the "time" field
                    if ~isfield(point, 'time') || ~isnumeric(point.time) || point.time <= 0
                        disp(['[WARN] Invalid or missing "time" field for point ' num2str(ii) '. Using default.']);
                        timespan = default_timespan;
                    else
                        % Compute the elapsed time (difference between current and previous time)
                        timespan = point.time - previous_time;
                        if timespan <= 0
                            error(['Invalid time_from_start at point ' num2str(ii) ': Time must be increasing.']);
                        end
                        previous_time = point.time; % Update the previous time
                    end
    
                    % Move to the current trajectory point
                    new_pose = pose;
                    new_pose(1:6) = point.angles;
                    sliderLock(:) = false;
                    update_pose(new_pose, timespan, false);
    
                    % Publish acknowledgment for this point
                    acknowledgment = sprintf('point %d of trajectory completed', ii);
                    write(Client, topic_status, acknowledgment);
                    disp(['[INFO] Published "' acknowledgment '" to status topic']);
                end
    
                % Publish "done" after completing the entire trajectory
                write(Client, topic_status, "done");
                disp('[INFO] Published "done" to status topic after completing trajectory.');
    
            % Check if it's a single pose
            elseif isfield(msg, 'angles')
                if ~isnumeric(msg.angles) || length(msg.angles) ~= 6
                    error('Invalid single pose: Missing or invalid "angles" field.');
                end
    
                if isfield(msg, 'time') && isnumeric(msg.time) && msg.time > 0
                    timespan = msg.time;
                else
                    disp('[WARN] Invalid or missing "time" field. Using default.');
                    timespan = default_timespan;
                end
    
                % Move to the single pose
                new_pose = pose;
                new_pose(1:6) = msg.angles;
                sliderLock(:) = false;
                update_pose(new_pose, timespan, false);
    
                % Publish "done" after completing the single pose
                write(Client, topic_status, "done");
                disp('[INFO] Published "done" to status topic after single pose movement.');
    
            else
                error('Message must contain either "trajectory" or "angles" field.');
            end
    
        catch ME
            % Handle any parsing or validation errors
            disp('[WARN] Invalid message format for joint angles or trajectory.');
            disp(['[ERROR] ' ME.message]);
            return;
        end
    end

% ----------------------------------------------------------------------- %
    % % old function for just moving to a single point
    % function handle_joint_angles_message(data)
    % 
    %         default_timespan = default_joint_timespan;
    % 
    %         % parse mqtt message and extract angles vector and goal timespan
    %         try
    %             msg = jsondecode(data);
    % 
    %             if isfield(msg, 'angles')
    %                 new_angles = msg.angles;
    %             else
    %                 error('Missing "angles" field in message.');
    %             end
    % 
    %             if length(new_angles) ~= 6
    %                 error('Invalid number of joint angles received. Expected a vector of 6 numbers.');
    %             end
    % 
    %             if isfield(msg, 'time') && isnumeric(msg.time) && msg.time > 0
    %                 timespan = msg.time;
    %             else
    %                 disp('[WARN] Invalid or missing "time" field. Using default.');
    %                 timespan = default_timespan;
    %             end
    % 
    %             % Publish "received" acknowledgment
    %             write(Client, topic_status, "received");
    %             disp('[INFO] Published "received" acknowledgment to status topic');
    % 
    %             % Combine new angles with current gripper configuration
    %             new_pose = pose;
    %             new_pose(1:6) = new_angles;
    %             sliderLock(:) = false;
    %             update_pose(new_pose, timespan, false);
    % 
    %             % Publish "done" once the movement is complete
    %             write(Client, topic_status, "done");
    %             disp('[INFO] Published "done" to status topic after movement');
    % 
    %         catch
    %             disp('[WARN] Invalid message format for joint angles. Expected JSON format with "angles" and optional "time".');
    %             return;
    %         end
    %     end
% ----------------------------------------------------------------------- %
    function handle_pose_request(data)
        if strcmp(data, "pose_requested")
            % Gather current joint angles and gripper state
            pose_data = struct( ...
                'joint_angles', pose(1:6), ...
                'gripper', pose(7)*(100/0.025) ...
            );
            % Convert to JSON string
            pose_json = jsonencode(pose_data);
    
            % Publish the pose to feedback topic
            write(Client, topic_feedback, pose_json);
            disp(['[INFO] Published current pose to feedback topic: ', pose_json]);
        else
            disp('[WARN] Unrecognized pose request message');
        end
    end
% ----------------------------------------------------------------------- %  
    function Client = connect_mqtt(brokerAddress, ClientID)
        try
            Client = mqttclient(brokerAddress, 'ClientID', ClientID, 'Port', 1883);
            disp('[INFO] Connected to MQTT broker successfully');
        catch
            disp('[ERROR] Failed to connect to MQTT broker');
            Client = [];
        end
    end
% ----------------------------------------------------------------------- %
%                           GUI UPDATE CALLBACKS
% ----------------------------------------------------------------------- %
    function handle_gripper_slider(src)
        sliderLock(7) = true;
        new_pose = pose;
        gripper_percentage = src.Value;
        update_sliders();
        new_pose(7:8) = [0.025 * (gripper_percentage / 100), 0.025 * (gripper_percentage / 100)];
        update_pose(new_pose, default_gripper_timespan, true)
    end
% ----------------------------------------------------------------------- %
    function handle_joint_angles_slider(src, joint_index)
        sliderLock(joint_index) = true;
        new_pose = pose;
        new_pose(joint_index) = src.Value;
        update_sliders();
        update_pose(new_pose, default_joint_timespan, true);
    end
% ----------------------------------------------------------------------- %
    % Update sliders and displays with the latest joint angles
    function update_sliders()
        for k = 1:6
            if ~sliderLock(k) % if unlocked
                slider_value = mod(pose(k) + pi, 2 * pi) - pi;
                sliders(k).Value = slider_value;
                slider_displays(k).Value = slider_value;
            else
                slider_displays(k).Value = pose(k); % display value in dsiplay box still
            end
        end

        if ~sliderLock(7)
            gripper_slider.Value = pose(7) * (100/0.025);
        end

    end
% ----------------------------------------------------------------------- %
    function toggle_trajectory()
        if recordTrajectory == false
            recordTrajectory = true;
            disp('[INFO] Trajectory recording enabled.');
        else
            recordTrajectory = false;
            disp('[INFO] Trajectory recording disabled.');
        end
    end
% ----------------------------------------------------------------------- %
    function clear_trajectory()
        % Clear stored trajectory points
        trajectoryPoints = [];
        
        % Delete the trajectory plot
        if ~isempty(trajectoryPlot)
            delete(trajectoryPlot);
            trajectoryPlot = [];
        end
    
        disp('[INFO] Cleared trajectory.');
    end

% ----------------------------------------------------------------------- %
%                            CLEANUP FUNCTIONS
% ----------------------------------------------------------------------- %
    function stop_mqtt(Client)
            if use_mqtt
                disp('[INFO] Cleaning up resources...');
                if ~isempty(Client)
                    try
                        Client.disconnect();
                        disp('[INFO] MQTT Client disconnected');
                    catch
                        disp('[WARN] MQTT Client was already disconnected');
                    end
                end
            end
            close all force;
    end
% ----------------------------------------------------------------------- %
    function closeProgram()
        stopProgram = true;
        disp('[INFO] Program is shutting down');
        delete(ui_fig);
        stop_mqtt(Client);
    end
end