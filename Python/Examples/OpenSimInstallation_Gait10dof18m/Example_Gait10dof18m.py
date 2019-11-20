# Example

global np
import numpy as np
import os
import sys

osim_path = 'C:/Users/u0101727/Documents/Visual Studio 2015/Projects/opensim-fork/opensim-core-install/sdk/Python'
sys.path.insert(0,osim_path)
import opensim

# Needed Input Arguments
Datapath='C:\OpenSim 3.3\Models\Gait10dof18musc\OutputReference'
IK_path=os.path.join(Datapath,'IK','subject01_walk_IK.mot')
ID_path=None # Compute ID from the external loads
model_path=os.path.join(Datapath,'subject01.osim')
time = np.append(0.7,1.4) # Part of the right stance phase
Out_path=os.path.join(os.getcwd(),'Results')

misc = {}
misc['DofNames_Input'] = ['ankle_angle_r','knee_angle_r','hip_flexion_r']
misc['Loads_path'] = os.path.join(Datapath,'ExperimentalData','subject01_walk_grf.xml')
misc['ID_ResultsPath'] = os.path.join(Datapath,'ID','inversedynamics.sto')


#DofNames_Input=['ankle_angle_r','knee_angle_r','hip_flexion_r']
#Loads_path=os.path.join(Datapath,'ExperimentalData','subject01_walk_grf.xml')
#ID_ResultsPath=os.path.join(Datapath,'ID','inversedynamics.sto')

# Solve the problem

main_path = os.path.dirname(os.path.dirname(os.getcwd()))
sys.path.append(main_path)

from SolveMuscleRedundancy_FtildeState_actdyn_CasADi import SolveMuscleRedundancy_FtildeState_actdyn_CasADi

time_out = SolveMuscleRedundancy_FtildeState_actdyn_CasADi(model_path,IK_path,ID_path,time,Out_path,misc)
print(time_out)