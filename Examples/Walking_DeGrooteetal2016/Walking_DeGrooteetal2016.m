clear all; close all; clc
%% Choose formulation
% formulation = 'lMtildeState';
formulation = 'FtildeState';
%% Choose activation dynamics
% actdyn = 'DeGroote2016';
actdyn = 'DeGroote2009';
%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Walking_DeGrooteetal2016.m');
[DirExample_Walking,~,~]=fileparts(filepath); [DirExample,~]=fileparts(DirExample_Walking);[MainDir,~]=fileparts(DirExample);
addpath(genpath(MainDir));

% Needed Input Arguments
IK_path=fullfile(MainDir,'Examples','Walking_DeGrooteetal2016','WalkingData','inverse_kinematics.mot');
ID_path=fullfile(MainDir,'Examples','Walking_DeGrooteetal2016','WalkingData','inverse_dynamics.sto');
model_path=fullfile(MainDir,'Examples','Walking_DeGrooteetal2016','WalkingData','subject01.osim');
time=[0.516 1.95]; % Right stance phase (+50ms beginning and end of time interval, more details see manual and publication)
OutPath=fullfile(MainDir,'Examples','Walking_DeGrooteetal2016','Results');

Misc.MuscleNames_Input={}; % Selects all muscles for the Input DOFS when this is left empty.
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r','hip_rotation_r','hip_adduction_r'};

% Optional Input Arguments
Misc.f_cutoff_ID = 6;     % cutoff frequency filtering ID
Misc.f_order_ID = 4;      % order frequency filtering ID
Misc.f_cutoff_lMT = 6;    % cutoff frequency filtering lMT
Misc.f_order_lMT = 4;     % order frequency filtering lMT
Misc.f_cutoff_dM= 6;      % cutoff frequency filtering MA
Misc.f_order_dM = 4;      % order frequency filtering MA
Misc.f_cutoff_IK= 6;      % cutoff frequency filtering IK
Misc.f_order_IK = 4;      % order frequency filtering IK
Misc.Atendon = [];        % Tendon Stiffness for the selected muscles
% %% Solve the problem
% switch actdyn
%     case 'DeGroote2016' % Activation dynamics from De Groote et al. (2016)      
%         switch formulation
%             case 'lMtildeState'
%                 [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
%             case 'FtildeState'   
%                 [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
%         end
%         
%     case 'DeGroote2009' % Activation dynamics from De Groote et al. (2009) 
%         switch formulation
%             case 'lMtildeState'
%                 [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_lMtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
%             case 'FtildeState'   
%                 [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_FtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
%         end
% end

%% Comparison GPOPS and CasADi
[Time_actdyn.cas,MExcitation_actdyn.cas,MActivation_actdyn.cas,RActivation_actdyn.cas,TForcetilde_actdyn.cas,TForce_actdyn.cas,lMtilde_actdyn.cas,lM_actdyn.cas,MuscleNames_actdyn.cas,OptInfo_actdyn.cas,DatStore_actdyn.cas]=SolveMuscleRedundancy_FtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);
[Time_actdyn.gpo,MExcitation_actdyn.gpo,MActivation_actdyn.gpo,RActivation_actdyn.gpo,TForcetilde_actdyn.gpo,TForce_actdyn.gpo,lMtilde_actdyn.gpo,lM_actdyn.gpo,MuscleNames_actdyn.gpo,OptInfo_actdyn.gpo,DatStore_actdyn.gpo]=SolveMuscleRedundancy_FtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);

% Muscle activations
figure()
plot(Time_actdyn.gpo,MActivation_actdyn.gpo,'LineWidth',2);
hold on
plot(Time_actdyn.cas.collocationPoints,MActivation_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle activations')

% Muscle excitations
figure()
plot(Time_actdyn.gpo,MExcitation_actdyn.gpo,'LineWidth',2);
hold on
plot(Time_actdyn.cas.meshPoints(1:end-1),MExcitation_actdyn.cas.meshPoints,'LineStyle',':','LineWidth',2);
title('Muscle excitations')

% Muscle-tendon forces
figure()
plot(Time_actdyn.gpo,TForcetilde_actdyn.gpo,'LineWidth',2);
hold on
plot(Time_actdyn.cas.collocationPoints,TForcetilde_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle forces')

% Muscle fiber lengths
figure()
plot(Time_actdyn.gpo,lMtilde_actdyn.gpo,'LineWidth',2);
hold on
plot(Time_actdyn.cas.collocationPoints,lMtilde_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle fiber lengths')

% Reserve actuators
figure()
plot(Time_actdyn.gpo,RActivation_actdyn.gpo,'LineWidth',2);
hold on
plot(Time_actdyn.cas.meshPoints(1:end-1),RActivation_actdyn.cas.meshPoints,'LineStyle',':','LineWidth',2);
title('Reserve actuators')