% Load the data
clear
close all
data = rosbag('Data/2019-03-27-16-45-24.bag');

b_joint_pos = select(data,'Topic','/kuka/state/KUKAJointPosition');
msg_Joint   = cell2mat(readMessages(b_joint_pos,'DataFormat','struct'));
joint_      = vertcat(msg_Joint(1:end).Position);
joint_array = horzcat(joint_.Quantity);
joint_time  = vertcat(msg_Joint(1:end).Header);
joint_time  = vertcat(joint_time(1:end).Stamp);
joint_time  = double(vertcat(joint_time.Sec)) + double(vertcat(joint_time.Nsec)) * 1e-9;

b_torque = select(data,'Topic','/kuka/state/KUKAActualTorque');
msg_Torque = cell2mat(readMessages(b_torque,'DataFormat','struct'));
torque_ = vertcat(msg_Torque(1:end).Torque);
torque_array = horzcat(torque_.Quantity);
torque_time  = vertcat(msg_Torque(1:end).Header);
torque_time  = vertcat(torque_time(1:end).Stamp);
torque_time  = double(vertcat(torque_time.Sec)) + double(vertcat(torque_time.Nsec)) * 1e-9;

b_ext_torque = select(data,'Topic','/kuka/state/KUKAExtTorque');
msg_Ext_Torque = cell2mat(readMessages(b_ext_torque,'DataFormat','struct'));
ext_torque_ = vertcat(msg_Ext_Torque(1:end).Torque);
ext_torque_array = horzcat(ext_torque_.Quantity);
ext_torque_time  = vertcat(msg_Ext_Torque(1:end).Header);
ext_torque_time  = vertcat(ext_torque_time(1:end).Stamp);
ext_torque_time  = double(vertcat(ext_torque_time.Sec)) + double(vertcat(ext_torque_time.Nsec)) * 1e-9;

%% Normalization (time scale)
% find the smallest interval of all trajaectories
[max_, max_in] = max([torque_time(1), joint_time(1)]);
[min_, min_in] = min([torque_time(end), joint_time(end)]);

% new time scale (1000Hz)
sample_frq = 100;
t = max_:1/sample_frq:min_;

% Time normalized trajectories
torque_ext_n = interp1(ext_torque_time,ext_torque_array',t,'spline');
torque_n = interp1(torque_time,torque_array',t,'spline');
joint_n  = interp1(joint_time,joint_array',t,'spline');

torque_n = torque_n - torque_ext_n;
ser = SerialChainCharacterization(t, joint_n, torque_n, sample_frq);