clear all;close all;clc
%% Choose optimisation framework
% framework = 'GPOPS';
framework = 'CasADi';
%% Choose formulation
% formulation = 'lMtildeState';
formulation = 'FtildeState';
%% Choose activation dynamics formulation
% actdyn = 'DeGroote2016';
actdyn = 'DeGroote2009';

%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Example_Gait10dof18m.m'); [DirExample,~,~]=fileparts(filepath); [DirExample2,~,~]=fileparts(DirExample); [MainDir,~]=fileparts(DirExample2);
addpath(genpath(MainDir));

% Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait10dof18musc\OutputReference';
IK_path=fullfile(Datapath,'IK','subject01_walk_IK.mot');
ID_path=[]; % compute ID from the external loads
model_path=fullfile(Datapath,'subject01.osim');
time=[0.7 1.4];     % Part of the right stance phase
OutPath=fullfile(MainDir,'Examples','OpenSimInstallation_Gait10dof18m','Results');

Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r'};
Misc.Loads_path=fullfile(Datapath,'ExperimentalData','subject01_walk_grf.xml');
Misc.ID_ResultsPath=fullfile(Datapath,'ID','inversedynamics.sto');

% Optional Input Arguments
% Here is an example of how to adjust the Achilles tendon stiffness.
% We first add the input argument MuscleNames_Input with ALL muscles 
% that actuate the degrees of freedom listed in DofNames_Input.
Misc.MuscleNames_Input={'hamstrings_r','bifemsh_r','glut_max_r',...
    'iliopsoas_r','rect_fem_r','vasti_r','gastroc_r','soleus_r',...
    'tib_ant_r'}; 
% We then change the compliance of the Achilles tendon by changing the 
% parameter Atendon of the gastrocnemius and the soleus. The default 
% value is 35 and a lower value will result in a more compliant tendon.
Misc.Atendon=[35,35,35,35,35,35,15,15,35];

%% Solve the problem
% switch framework
%     case 'GPOPS'
%         switch actdyn
%             case 'DeGroote2016' % Activation dynamics from De Groote et al. (2016)        
%                 switch formulation
%                     case 'lMtildeState'
%                         [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
%                     case 'FtildeState'   
%                         [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
%                 end
% 
%             case 'DeGroote2009' % Activation dynamics from De Groote et al. (2009)   
%                 switch formulation
%                     case 'lMtildeState'
%                         [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_lMtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
%                     case 'FtildeState'   
%                         [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_FtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
%                 end
%         end
%     case 'CasADi'
%         switch actdyn
%             case 'DeGroote2016' % Activation dynamics from De Groote et al. (2016)        
%                 switch formulation
%                     case 'lMtildeState'
%                         [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
%                     case 'FtildeState'   
%                         [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
%                 end
% 
%             case 'DeGroote2009' % Activation dynamics from De Groote et al. (2009)   
%                 switch formulation
%                     case 'lMtildeState'
%                         [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_lMtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
%                     case 'FtildeState'   
%                         [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_FtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);
%                 end
%         end
% end

%% Comparison GPOPS and CasADi
[Time_actdyn.cas,MExcitation_actdyn.cas,MActivation_actdyn.cas,RActivation_actdyn.cas,TForcetilde_actdyn.cas,TForce_actdyn.cas,lMtilde_actdyn.cas,lM_actdyn.cas,MuscleNames_actdyn.cas,OptInfo_actdyn.cas,DatStore_actdyn.cas]=SolveMuscleRedundancy_FtildeState_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);

% [Time_actdyn.gpo,MExcitation_actdyn.gpo,MActivation_actdyn.gpo,RActivation_actdyn.gpo,TForcetilde_actdyn.gpo,TForce_actdyn.gpo,lMtilde_actdyn.gpo,lM_actdyn.gpo,MuscleNames_actdyn.gpo,OptInfo_actdyn.gpo,DatStore_actdyn.gpo]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
% [Time_actdyn.cas,MExcitation_actdyn.cas,MActivation_actdyn.cas,RActivation_actdyn.cas,TForcetilde_actdyn.cas,TForce_actdyn.cas,lMtilde_actdyn.cas,lM_actdyn.cas,MuscleNames_actdyn.cas,OptInfo_actdyn.cas,DatStore_actdyn.cas]=SolveMuscleRedundancy_lMtildeState_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);

% [Time_actdyn.gpo,MExcitation_actdyn.gpo,MActivation_actdyn.gpo,RActivation_actdyn.gpo,TForcetilde_actdyn.gpo,TForce_actdyn.gpo,lMtilde_actdyn.gpo,lM_actdyn.gpo,MuscleNames_actdyn.gpo,OptInfo_actdyn.gpo,DatStore_actdyn.gpo]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);

% Muscle activations
figure()
plot(Time_actdyn.gpo,MActivation_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.collocationPoints,MActivation_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle activations')

% Muscle excitations
figure()
plot(Time_actdyn.gpo,MExcitation_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.meshPoints(1:end-1),MExcitation_actdyn.cas.meshPoints,'LineStyle',':','LineWidth',2);
title('Muscle excitations')

% Muscle-tendon forces (normalized)
figure()
plot(Time_actdyn.gpo,TForcetilde_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.collocationPoints,TForcetilde_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle forces (normalized)')

% Muscle-tendon forces
figure()
plot(Time_actdyn.gpo,TForce_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.collocationPoints,TForce_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle forces')

% Muscle fiber lengths (normalized)
figure()
plot(Time_actdyn.gpo,lMtilde_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.collocationPoints,lMtilde_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle fiber lengths (normalized)')

% Muscle fiber lengths
figure()
plot(Time_actdyn.gpo,lM_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.collocationPoints,lM_actdyn.cas.collocationPoints,'LineStyle',':','LineWidth',2);
title('Muscle fiber lengths')

% Reserve actuators
figure()
plot(Time_actdyn.gpo,RActivation_actdyn.gpo,'LineWidth',2);
hold on
% plot(Time_actdyn.cas.meshPoints(1:end-1),RActivation_actdyn.cas.meshPoints./150,'LineStyle',':','LineWidth',2);
title('Reserve actuators')
