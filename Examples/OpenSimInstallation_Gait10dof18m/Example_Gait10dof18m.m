clear all;close all;clc

%% Choose optimization framework
% framework = 'GPOPS';
framework = 'CasADi';

%% Choose contraction dynamics formulation
% formulation_contdyn = 'lMtildeState';
formulation_contdyn = 'FtildeState';

%% Choose activation dynamics formulation
% formulation_actdyn = 'DeGroote2016';
formulation_actdyn = 'DeGroote2009';

%% Example
% Add main folder and subfolder to matlab path (installation)
filepath=which('Example_Gait10dof18m.m'); 
[DirExample,~,~]=fileparts(filepath); 
[DirExample2,~,~]=fileparts(DirExample); 
[MainDir,~]=fileparts(DirExample2);
addpath(genpath(MainDir));

% Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait10dof18musc\OutputReference';
IK_path=fullfile(Datapath,'IK','subject01_walk_IK.mot');
ID_path=[]; % Compute ID from the external loads
model_path=fullfile(Datapath,'subject01.osim');
time=[0.7 1.4]; % Part of the right stance phase
Out_path=fullfile(MainDir,'Examples','OpenSimInstallation_Gait10dof18m','Results');

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
[Time_l_G,MExcitation_l_G,MActivation_l_G,RActivation_l_G,TForcetilde_l_G,TForce_l_G,lMtilde_l_G,lM_l_G,MuscleNames_l_G,OptInfo_l_G,DatStore_l_G]=SolveMuscleRedundancy_lMtildeState_GPOPS(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_l_G','Time_l_G'); save('MExcitation_l_G','MExcitation_l_G'); save('MActivation_l_G','MActivation_l_G'); save('RActivation_l_G','RActivation_l_G');
save('TForcetilde_l_G','TForcetilde_l_G'); save('TForce_l_G','TForce_l_G'); save('lMtilde_l_G','lMtilde_l_G'); save('lM_l_G','lM_l_G');
[Time_l_C,MExcitation_l_C,MActivation_l_C,RActivation_l_C,TForcetilde_l_C,TForce_l_C,lMtilde_l_C,lM_l_C,MuscleNames_l_C,OptInfo_l_C,DatStore_l_C]=SolveMuscleRedundancy_lMtildeState_CasADi(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_l_C','Time_l_C'); save('MExcitation_l_C','MExcitation_l_C'); save('MActivation_l_C','MActivation_l_C'); save('RActivation_l_C','RActivation_l_C');
save('TForcetilde_l_C','TForcetilde_l_C'); save('TForce_l_C','TForce_l_C'); save('lMtilde_l_C','lMtilde_l_C'); save('lM_l_C','lM_l_C');
[Time_f_G,MExcitation_f_G,MActivation_f_G,RActivation_f_G,TForcetilde_f_G,TForce_f_G,lMtilde_f_G,lM_f_G,MuscleNames_f_G,OptInfo_f_G,DatStore_f_G]=SolveMuscleRedundancy_FtildeState_GPOPS(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_f_G','Time_f_G'); save('MExcitation_f_G','MExcitation_f_G'); save('MActivation_f_G','MActivation_f_G'); save('RActivation_f_G','RActivation_f_G');
save('TForcetilde_f_G','TForcetilde_f_G'); save('TForce_f_G','TForce_f_G'); save('lMtilde_f_G','lMtilde_f_G'); save('lM_f_G','lM_f_G');
[Time_f_C,MExcitation_f_C,MActivation_f_C,RActivation_f_C,TForcetilde_f_C,TForce_f_C,lMtilde_f_C,lM_f_C,MuscleNames_f_C,OptInfo_f_C,DatStore_f_C]=SolveMuscleRedundancy_FtildeState_CasADi(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_f_C','Time_f_C'); save('MExcitation_f_C','MExcitation_f_C'); save('MActivation_f_C','MActivation_f_C'); save('RActivation_f_C','RActivation_f_C');
save('TForcetilde_f_C','TForcetilde_f_C'); save('TForce_f_C','TForce_f_C'); save('lMtilde_f_C','lMtilde_f_C'); save('lM_f_C','lM_f_C');
[Time_l_actdyn_G,MExcitation_l_actdyn_G,MActivation_l_actdyn_G,RActivation_l_actdyn_G,TForcetilde_l_actdyn_G,TForce_l_actdyn_G,lMtilde_l_actdyn_G,lM_l_actdyn_G,MuscleNames_l_actdyn_G,OptInfo_l_actdyn_G,DatStore_l_actdyn_G]=SolveMuscleRedundancy_lMtildeState_actdyn_GPOPS(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_l_actdyn_G','Time_l_actdyn_G'); save('MExcitation_l_actdyn_G','MExcitation_l_actdyn_G'); save('MActivation_l_actdyn_G','MActivation_l_actdyn_G'); save('RActivation_l_actdyn_G','RActivation_l_actdyn_G');
save('TForcetilde_l_actdyn_G','TForcetilde_l_actdyn_G'); save('TForce_l_actdyn_G','TForce_l_actdyn_G'); save('lMtilde_l_actdyn_G','lMtilde_l_actdyn_G'); save('lM_l_actdyn_G','lM_l_actdyn_G');
[Time_l_actdyn_C,MExcitation_l_actdyn_C,MActivation_l_actdyn_C,RActivation_l_actdyn_C,TForcetilde_l_actdyn_C,TForce_l_actdyn_C,lMtilde_l_actdyn_C,lM_l_actdyn_C,MuscleNames_l_actdyn_C,OptInfo_l_actdyn_C,DatStore_l_actdyn_C]=SolveMuscleRedundancy_lMtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_l_actdyn_C','Time_l_actdyn_C'); save('MExcitation_l_actdyn_C','MExcitation_l_actdyn_C'); save('MActivation_l_actdyn_C','MActivation_l_actdyn_C'); save('RActivation_l_actdyn_C','RActivation_l_actdyn_C');
save('TForcetilde_l_actdyn_C','TForcetilde_l_actdyn_C'); save('TForce_l_actdyn_C','TForce_l_actdyn_C'); save('lMtilde_l_actdyn_C','lMtilde_l_actdyn_C'); save('lM_l_actdyn_C','lM_l_actdyn_C');
[Time_f_actdyn_G,MExcitation_f_actdyn_G,MActivation_f_actdyn_G,RActivation_f_actdyn_G,TForcetilde_f_actdyn_G,TForce_f_actdyn_G,lMtilde_f_actdyn_G,lM_f_actdyn_G,MuscleNames_f_actdyn_G,OptInfo_f_actdyn_G,DatStore_f_actdyn_G]=SolveMuscleRedundancy_FtildeState_actdyn_GPOPS(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_f_actdyn_G','Time_f_actdyn_G'); save('MExcitation_f_actdyn_G','MExcitation_f_actdyn_G'); save('MActivation_f_actdyn_G','MActivation_f_actdyn_G'); save('RActivation_f_actdyn_G','RActivation_f_actdyn_G');
save('TForcetilde_f_actdyn_G','TForcetilde_f_actdyn_G'); save('TForce_f_actdyn_G','TForce_f_actdyn_G'); save('lMtilde_f_actdyn_G','lMtilde_f_actdyn_G'); save('lM_f_actdyn_G','lM_f_actdyn_G');
[Time_f_actdyn_C,MExcitation_f_actdyn_C,MActivation_f_actdyn_C,RActivation_f_actdyn_C,TForcetilde_f_actdyn_C,TForce_f_actdyn_C,lMtilde_f_actdyn_C,lM_f_actdyn_C,MuscleNames_f_actdyn_C,OptInfo_f_actdyn_C,DatStore_f_actdyn_C]=SolveMuscleRedundancy_FtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,Out_path,Misc);
save('Time_f_actdyn_C','Time_f_actdyn_C'); save('MExcitation_f_actdyn_C','MExcitation_f_actdyn_C'); save('MActivation_f_actdyn_C','MActivation_f_actdyn_C'); save('RActivation_f_actdyn_C','RActivation_f_actdyn_C');
save('TForcetilde_f_actdyn_C','TForcetilde_f_actdyn_C'); save('TForce_f_actdyn_C','TForce_f_actdyn_C'); save('lMtilde_f_actdyn_C','lMtilde_f_actdyn_C'); save('lM_f_actdyn_C','lM_f_actdyn_C');

% load Time_l_G
% load MExcitation_l_G
% load MActivation_l_G
% load RActivation_l_G
% load TForcetilde_l_G
% load TForce_l_G
% load lMtilde_l_G
% load lM_l_G
% 
% load Time_l_C
% load MExcitation_l_C
% load MActivation_l_C
% load RActivation_l_C
% load TForcetilde_l_C
% load TForce_l_C
% load lMtilde_l_C
% load lM_l_C
% 
% load Time_f_G
% load MExcitation_f_G
% load MActivation_f_G
% load RActivation_f_G
% load TForcetilde_f_G
% load TForce_f_G
% load lMtilde_f_G
% load lM_f_G
% 
% load Time_f_C
% load MExcitation_f_C
% load MActivation_f_C
% load RActivation_f_C
% load TForcetilde_f_C
% load TForce_f_C
% load lMtilde_f_C
% load lM_f_C
% 
% load Time_l_actdyn_G
% load MExcitation_l_actdyn_G
% load MActivation_l_actdyn_G
% load RActivation_l_actdyn_G
% load TForcetilde_l_actdyn_G
% load TForce_l_actdyn_G
% load lMtilde_l_actdyn_G
% load lM_l_actdyn_G
% 
% load Time_l_actdyn_C
% load MExcitation_l_actdyn_C
% load MActivation_l_actdyn_C
% load RActivation_l_actdyn_C
% load TForcetilde_l_actdyn_C
% load TForce_l_actdyn_C
% load lMtilde_l_actdyn_C
% load lM_l_actdyn_C
% 
% load Time_f_actdyn_G
% load MExcitation_f_actdyn_G
% load MActivation_f_actdyn_G
% load RActivation_f_actdyn_G
% load TForcetilde_f_actdyn_G
% load TForce_f_actdyn_G
% load lMtilde_f_actdyn_G
% load lM_f_actdyn_G
% 
% load Time_f_actdyn_C
% load MExcitation_f_actdyn_C
% load MActivation_f_actdyn_C
% load RActivation_f_actdyn_C
% load TForcetilde_f_actdyn_C
% load TForce_f_actdyn_C
% load lMtilde_f_actdyn_C
% load lM_f_actdyn_C

% % Muscle activations
% figure()
% plot(Time_l_G,MActivation_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,MActivation_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,MActivation_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,MActivation_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.collocationPoints,MActivation_l_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.collocationPoints,MActivation_f_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.collocationPoints,MActivation_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.collocationPoints,MActivation_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% title('Muscle activations')
% 
% % Muscle excitations
% figure()
% plot(Time_l_G,MExcitation_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,MExcitation_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,MExcitation_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,MExcitation_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.meshPoints(1:end-1),MExcitation_l_C.meshPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.meshPoints(1:end-1),MExcitation_f_C.meshPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.meshPoints(1:end-1),MExcitation_f_actdyn_C.meshPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.meshPoints(1:end-1),MExcitation_f_actdyn_C.meshPoints,'LineStyle',':','LineWidth',2);
% title('Muscle excitations')
% 
% % Muscle tendon forces
% figure()
% plot(Time_l_G,TForce_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,TForce_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,TForce_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,TForce_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.collocationPoints,TForce_l_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.collocationPoints,TForce_f_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.collocationPoints,TForce_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.collocationPoints,TForce_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% title('Muscle tendon forces')
% 
% % Normalized Muscle tendon forces
% figure()
% plot(Time_l_G,TForcetilde_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,TForcetilde_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,TForcetilde_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,TForcetilde_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.collocationPoints,TForcetilde_l_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.collocationPoints,TForcetilde_f_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.collocationPoints,TForcetilde_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.collocationPoints,TForcetilde_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% title('Normalized muscle tendon forces')
% 
% % Muscle fiber lengths
% figure()
% plot(Time_l_G,lM_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,lM_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,lM_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,lM_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.collocationPoints,lM_l_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.collocationPoints,lM_f_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.collocationPoints,lM_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.collocationPoints,lM_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% title('Muscle fiber lengths')
% 
% % Normalized muscle fiber lengths
% figure()
% plot(Time_l_G,lMtilde_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,lMtilde_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,lMtilde_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,lMtilde_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.collocationPoints,lMtilde_l_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.collocationPoints,lMtilde_f_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.collocationPoints,lMtilde_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.collocationPoints,lMtilde_f_actdyn_C.collocationPoints,'LineStyle',':','LineWidth',2);
% title('Normalized muscle fiber lengths')
% 
% % Reserve actuators
% figure()
% plot(Time_l_G,RActivation_l_G,'LineWidth',2);
% hold on
% plot(Time_f_G,RActivation_f_G,'LineWidth',2);
% plot(Time_l_actdyn_G,RActivation_l_actdyn_G,'LineWidth',2);
% plot(Time_f_actdyn_G,RActivation_f_actdyn_G,'LineWidth',2);
% plot(Time_l_C.meshPoints(1:end-1),RActivation_l_C.meshPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_C.meshPoints(1:end-1),RActivation_f_C.meshPoints,'LineStyle',':','LineWidth',2);
% plot(Time_l_actdyn_C.meshPoints(1:end-1),RActivation_f_actdyn_C.meshPoints,'LineStyle',':','LineWidth',2);
% plot(Time_f_actdyn_C.meshPoints(1:end-1),RActivation_f_actdyn_C.meshPoints,'LineStyle',':','LineWidth',2);
% title('Reserve actuators')