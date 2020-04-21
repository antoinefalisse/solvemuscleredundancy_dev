IC = 1.85;
TO = 2.25;

%% Example
% Add main folder and subfolder to matlab path (installation)
filepath=which('RunningExample2.m');
[MainDir,~,~]=fileparts(filepath);
addpath(genpath(MainDir));

% Needed Input Arguments
IK_path=fullfile(MainDir,'IKoutput_RN21.mot');
ID_path=fullfile(MainDir,'IDoutput_RN21.sto');
model_path=fullfile(MainDir,'GaitModelforGRunning_scaled.osim');
time=[IC TO];
Out_path=fullfile(MainDir,'Results');

Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r','hip_adduction_r','hip_rotation_r'};
Misc.MuscleNames_Input={}; % Selects all muscles for input DOFs when empty

% Optional Input Arguments
Misc.f_cutoff_ID = 15;    % cutoff frequency filtering ID
Misc.f_order_ID = 5;      % order frequency filtering ID
Misc.f_cutoff_lMT = 15;   % cutoff frequency filtering lMT
Misc.f_order_lMT = 5;     % order frequency filtering lMT
Misc.f_cutoff_dM= 15;     % cutoff frequency filtering MA
Misc.f_order_dM = 5;      % order frequency filtering MA
Misc.f_cutoff_IK= 15;     % cutoff frequency filtering IK
Misc.f_order_IK = 5;      % order frequency filtering IK
Misc.Loads_path = [];

%% Solve the problem

[Time_actdyn,MExcitation_actdyn,MActivation_actdyn,...
    RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,...
    lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,...
    OptInfo_actdyn,DatStore_actdyn]=...
    SolveMuscleRedundancy_FtildeState_actdyn_CasADi(...
    model_path,IK_path,ID_path,time,Out_path,Misc);
