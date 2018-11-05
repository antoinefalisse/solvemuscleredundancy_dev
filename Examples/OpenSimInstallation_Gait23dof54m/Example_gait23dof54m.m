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
% add main folder and subfolder to matlab path (installation)
filepath=which('Example_Gait23dof54m.m'); 
[DirExample,~,~]=fileparts(filepath); [DirExample2,~,~]=fileparts(DirExample); [MainDir,~]=fileparts(DirExample2);
addpath(genpath(MainDir));

% Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait2354_Simbody\OutputReference';
IK_path=fullfile(Datapath,'subject01_walk1_ik.mot');
ID_path=fullfile(Datapath,'ResultsInverseDynamics','inverse_dynamics.sto');
model_path=fullfile(Datapath,'subject01_scaledOnly.osim');
time=[0.7 1.4]; % Part of the right stance phase
OutPath=fullfile(MainDir,'Examples','OpenSimInstallation_Gait23dof54m','Results');

Misc.DofNames_Input={'ankle_angle_r','knee_angle_r','hip_flexion_r'};
Misc.MuscleNames_Input={}; % Selects all muscles for input DOFs when empty

%% Solve the problem
switch framework
    case 'GPOPS'
        switch formulation_actdyn
            case 'DeGroote2016'     
                switch formulation_contdyn
                    case 'lMtildeState'
                        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState_GPOPS(model_path,IK_path,ID_path,time,OutPath,Misc);
                    case 'FtildeState'   
                        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState_GPOPS(model_path,IK_path,ID_path,time,OutPath,Misc);
                end

            case 'DeGroote2009' 
                switch formulation_contdyn
                    case 'lMtildeState'
                        [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_lMtildeState_actdyn_GPOPS(model_path,IK_path,ID_path,time,OutPath,Misc);
                    case 'FtildeState'   
                        [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_FtildeState_actdyn_GPOPS(model_path,IK_path,ID_path,time,OutPath,Misc);
                end
        end
    case 'CasADi'
        switch formulation_actdyn
            case 'DeGroote2016'      
                switch formulation_contdyn
                    case 'lMtildeState'
                        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);
                    case 'FtildeState'   
                        [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);
                end

            case 'DeGroote2009'
                switch formulation_contdyn
                    case 'lMtildeState'
                        [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_lMtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);
                    case 'FtildeState'   
                        [Time_actdyn,MExcitation_actdyn,MActivation_actdyn,RActivation_actdyn,TForcetilde_actdyn,TForce_actdyn,lMtilde_actdyn,lM_actdyn,MuscleNames_actdyn,OptInfo_actdyn,DatStore_actdyn]=SolveMuscleRedundancy_FtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc);
                end
        end
end
