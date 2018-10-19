clear all; close all; clc
%% Choose formulation
% formulation = 'lMtildeState';
formulation = 'FtildeState';
%% Choose activation dynamics formulation
% actdyn = 'DeGroote2016';
actdyn = 'DeGroote2009';
%% Example
% add main folder and subfolder to matlab path (installation)
filepath=which('Example_Gait23dof54m.m'); [DirExample,~,~]=fileparts(filepath); [DirExample2,~,~]=fileparts(DirExample); [MainDir,~]=fileparts(DirExample2);
addpath(genpath(MainDir));

% Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait2354_Simbody\OutputReference';
IK_path=fullfile(Datapath,'subject01_walk1_ik.mot');
ID_path=fullfile(Datapath,'ResultsInverseDynamics','inverse_dynamics.sto');
model_path=fullfile(Datapath,'subject01_scaledOnly.osim');
time=[0.7 1.4];     % Part of the right stance phase
OutPath=fullfile(MainDir,'Examples','OpenSimInstallation_Gait23dof54m','Results');

Misc.MuscleNames_Input={};      % Selects all muscles for the Input DOFS when this is left empty.
Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r'};

%% Solve the problem
switch actdyn
    case 'DeGroote2016' % Activation dynamics from De Groote et al. (2016)            
        switch formulation
            case 'lMtildeState'
                [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
            case 'FtildeState'   
                [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState(model_path,IK_path,ID_path,time,OutPath,Misc);
        end
        
    case 'DeGroote2009' % Activation dynamics from De Groote et al. (2009)   
        switch formulation
            case 'lMtildeState'
                [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_lMtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
            case 'FtildeState'   
                [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_FtildeState_actdyn(model_path,IK_path,ID_path,time,OutPath,Misc);
        end
end