%% Example solve muscle redundancy with an MRI model

% Install instructions:
%   Add the main path ...\solvemuscleredundancy_dev to your matlab folder,
%   using the following command (in my case)

MainPath = 'C:\Users\u0088756\Documents\PostDoc\Software\Original\solvemuscleredundancy_dev';
addpath(genpath(MainPath));

% Input arguments
Datapath =fullfile(MainPath,'Examples','EMG_constraintMRI');
IK_path = fullfile(Datapath,'gait1_kinematics.mot');      % point to the IK file
ID_path = fullfile(Datapath,'inverse_dynamics.sto');      % point to the ID file
model_path = fullfile(Datapath,'SEMLS_2_MRI_final.osim');         % point to the model
% time = [3.39 1.4];                                         % Time window for analysis
Out_path=fullfile(MainPath,'Results_MRI');                    % folder to store results

% Example for Hans: Note if you want to analyse all the data in the IK file
IK = importdata(IK_path);
time = [IK.data(1,1) IK.data(end,1)];

% settings
Misc.DofNames_Input={'ankle_flex_r','knee_flex_r','hip_flex_r'};    % select the DOFs you want to include in the optimization
Misc.RunAnalysis = 0;   % boolean to select if you want to run the muscle analysis
Misc.Par_Elastic = 0;   % boolean to select if you want a parallel elastic element in your model
Misc.FL_Relation = 0;   % boolean to select if you want a account for force-length and force-velocity properties
% Note: Currently no option to account for parallel elastic element, but
% not for force length properties

% Function to solve the muscle redundancy problem
[Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=...
    SolveMuscleRedundancy_FtildeState_actdyn_GPOPS(model_path,IK_path,ID_path,time,Out_path,Misc);


%% Example to work with output
mNames = DatStore.MuscleNames;
iSoleus = find(strcmp(mNames,'soleus_r'));
igas1 = find(strcmp(mNames,'gas_lat_r'));
igas2 = find(strcmp(mNames,'gas_med_r'));

figure();
plot(Time,MActivation(:,iSoleus)); hold on;         % Dynamic optimization (i.e. Full hill type model)
plot(DatStore.time,DatStore.SoAct(:,iSoleus));      % Static opt (i.e. rigid tendon)
legend('DynamicOpt','StaticOpt');

figure();
plot(DatStore.time,DatStore.T_exp);
legend(Misc.DofNames_Input);

figure();
plot(Time,RActivation); hold on;         % Dynamic optimization (i.e. Full hill type model)
legend(Misc.DofNames_Input);

figure();plot(Time,lMtilde(:,[iSoleus igas1 igas2]));
legend('soleus','gas_lat','gas_med');

figure();
plot(Time,MActivation(:,[iSoleus igas1 igas2])); hold on;
plot(DatStore.time,DatStore.SoAct(:,[iSoleus igas1 igas2]),'--k'); hold on;

legend('soleus','gas_lat','gas_med');