## Overview

The provided MATLAB code solves the muscle redundancy problem using the direct collocation optimal control software GPOPS-II as described in *De Groote F, Kinney AL, Rao AV, Fregly BJ. Evaluation of direct collocation optimal control problem formulations for solving the muscle redundancy problem. Annals of Biomedical Engineering (2016) ([DeGroote2016](http://link.springer.com/article/10.1007%2Fs10439-016-1591-9)).

From v1.1, an implicit formulation of activation dynamics can be used to solve the muscle redundancy problem. Additionally, by using the activation dynamics model proposed by Raasch et al. (1997), we could introduce a nonlinear change of variables to exactly impose activation dynamics in a continuously differentiable form, omitting the need for a smooth approximation such as described in De Groote et al. (2016). A result of this change of variables is that muscle excitations are not directly accessible during the optimization. Therefore, we replaced muscle excitations by muscle activations in the objective function. This implicit formulation is described in *De Groote F, Pipeleers G, Jonkers I, Demeulenaere B, Patten C, Swevers J, De Schutter J. A physiology based inverse dynamic analysis of human gait: potential and perspectives F. Computer Methods in Biomechanics and Biomedical Engineering (2009)* ([DeGroote2009](http://www.tandfonline.com/doi/full/10.1080/10255840902788587)).

Results from both formulations are very similar (differences can be attributed to the slightly different activation dynamics models and cost functions). However, the formulation with implicit activation dynamics (De Groote et al., 2009) is computationally faster. This can mainly be explained by the omission of a tanh function in the constraint definition, whose evaluation is computationally expensive when solving the NLP.

From v2.1, CasADi can be used as an alternative to GPOPS-II and ADiGator. CasADi is an open-source tool for nonlinear optimization and algorithmic differentiation (\url{https://web.casadi.org/}). Results using CasADi and GPOPS-II are very similar (differences can be attributed to the different direct collocation formulations and scaling). We used CasADi's Opti stack, which is a collection of CasADi helper classes that provides a close correspondence between mathematical NLP notation and computer code (\url{https://web.casadi.org/docs/#document-opti}). CasADi is actively maintained and developed, and has an active forum (\url{https://groups.google.com/forum/#!forum/casadi-users}). \\

From v1.1, an implicit formulation of activation dynamics can be used to solve the muscle redundancy problem. Additionally, by using the activation dynamics model proposed by Raasch et al. (1997), we could introduce a nonlinear change of variables to exactly impose activation dynamics in a continuously differentiable form, omitting the need for a smooth approximation such as described in De Groote et al. (2016). A result of this change of variables is that muscle excitations are not directly accessible during the optimization. Therefore, we replaced muscle excitations by muscle activations in the objective function. This implicit formulation is described in \textit{De Groote F, Pipeleers G, Jonkers I, Demeulenaere B, Patten C, Swevers J, De Schutter J. A physiology based inverse dynamic analysis of human gait: potential and perspectives F. Computer Methods in Biomechanics and Biomedical Engineering (2009).} \url{http://www.tandfonline.com/doi/full/10.1080/10255840902788587}. Results from both formulations are very similar (differences can be attributed to the slightly different activation dynamics models and cost functions). However, the formulation with implicit activation dynamics (De Groote et al., (2009)) is computationally faster. This can mainly be explained by the omission of a tanh function in the constraint definition, whose evaluation is computationally expensive when solving the NLP.

## Release Notes

### Release 2.1

- CasADi was added as an alternative to GPOPS-II and ADiGator

- The reserve actuators (RActivation) were unscaled in the output of the main functions.

- The time derivatives of the muscle contraction dynamics states, i.e. normalized muscle fiber velocities or derivatives of normalized tendon forces, were added to the cost function with a small weighting factor to prevent spiky outputs.
- The tendon stiffness was added as an optional user parameter

## Installation Instruction

Add the main folder and subfolder to your MATLAB path 

```
addpath(genpath('C/......./SimTK_optcntrlmuscle'))).
```

Several software packages are needed to run the program

- The OpenSim MATLAB interface is used to generate the inputs to the optimal control problem based on a scaled OpenSim model and the solution of inverse kinematics (providing the solution of inverse dynamics is optional). To this aim, install OpenSim and set up the OpenSim MATLAB interface (OpenSim: [https://simtk.org/frs/?group_id=91](https://simtk.org/frs/?group_id=91), OpenSim API: http://simtk-confluence.stanford.edu:8080/display/OpenSim/Scripting+with+Matlab.
- GPOPS implementation (v1)
  - GPOPS-II is used to solve the optimal control problem using direct collocation (http://www.gpops2.com/). A one-time 30-day trial license is avaiable for all users who register.
  - ADiGator is used for automatic differentiation https://sourceforge.net/projects/adigator/.
- Using CasADi
  - CasADi is used for nonlinear optimization and algorithmic differentiation (https://web.casadi.org/).

## Main Function

SolveMuscleRedundancy is the main function of this program and is used to solve the muscle redundancy problem. There are four variants of this function:

### Using GPOPS

With explicit activation dynamics formulation (De Groote et al. (2016)):
- SolveMuscleRedundancy_FtildeState uses the normalized tendon force as a state
- SolveMuscleRedundancy_lMtildeState uses the normalized muscle fiber length as a state

With implicit activation dynamics formulation (De Groote et al. (2009)):
- SolveMuscleRedundancy_FtildeState_actdyn uses the normalized tendon force as a state
- SolveMuscleRedundancy_lMtildeState_actdyn uses the normalized muscle fiber length as a state

### Using CasADi

With explicit activation dynamics formulation (De Groote et al. (2016)):
- SolveMuscleRedundancy_FtildeState_CasADi uses the normalized tendon force as a state
- SolveMuscleRedundancy_lMtildeState_CasADi uses the normalized muscle fiber length as a state


With implicit activation dynamics formulation (De Groote et al. (2009)):
- SolveMuscleRedundancy_FtildeState_actdyn_CasADi uses the normalized tendon force as a state
- SolveMuscleRedundancy_lMtildeState_actdyn_CasADi uses the normalized muscle fiber length as a state


### Input Arguments

Required Input arguments for SolveMuscleRedundancy

- **model_path:** directory and filename of the scaled OpenSim model (.osim). The code should work with any OpenSim model with valid muscle-tendon parameters for which OpenSim's Inverse Dynamics and Muscle Analysis Tools generate reliable results. Note that only the muscle-tendon parameters and not the muscle model specified in the osim-file are used (for details see Muscle model).
-  **IK_path**: directory and filename of the inverse kinematics solution (.mot file).
- **ID_path**: directory and filename of the inverse dynamics solution  (.sto file). If left empty, the inverse dynamics solution will be computed from the external loads (see Optional input arguments).
- **time**: 1 x 2 MATLAB array with the initial and final time of the analysis in seconds. Initial and final states influence the optimal controls over a period of about 50 ms at the beginning and end of the time interval over which the optimal control problem is solved. Since in practice the initial and final states are generally unknown, problems should be solved for a time interval containing five additional data points (considering a 100Hz sampling frequency) at the beginning and end of the motion cycle. Those additional data points should not be considered in further analyses. The user should thus not be surprised to observe unrealistically high muscle activation at the beginning of the motion (more details in companion paper).
- **OutPath**: directory where you want to store the results from the muscle analysis.
- **Misc**: miscellaneous input arguments
  - *DofNames_Input*  is a cell array specifying for which degrees of freedom you want to solve the muscle redundancy problem. Typically the muscle redundancy problem is solved for one leg at a time (there are no muscles spanning both legs).
  - *MuscleNames_Input* is a cell array that specifies the muscles to be included in when solving the muscle redundancy problem. All muscles that actuate (i.e. have a moment arm with respect to) the degrees of freedom specified in \textit{DofNames_Input} will be selected by default if this array is empty

Optional input arguments

- **Misc**.Loads_path: path to the external loads file (.xml). The program will use the OpenSim libraries to solve the inverse dynamics problem when the required input argument ID_path is empty and Misc.\textit{Loads_path} points to an external loads file.

- **Misc**.*ID_ResultsPath*: Path where the inverse dynamics results will be saved when the required input argument *ID_path* is empty.

- **Misc**.*f_cutoff_ID*: Cutoff frequency for the butterworth recursive low pass filter applied to the inverse dynamics data (default is 6 Hz).

- **Misc**.*f_order_ID*: order of the butterworth recursive low pass filter applied to the inverse dynamics data (default is 6).	

- **Misc**.*f_cutoff_LMT*: cutoff frequency for butterworth recursive low pass filter applied to the muscle tendon lengths from the muscle analysis (default 6 Hz).

- **Misc**.*f_order_LMT*: order of the butterworth recursive low pass filter applied to the muscle tendon lengths from the muscle analysis (default 6).		

- **Misc**.f_cutoff_dM: cutoff frequency for butterworth recursive low pass filter applied to the muscle moment arms from the muscle analysis (default 6 Hz).

- **Misc**.f_order_dM: order of the butterworth recursive low pass filter applied to the muscle moment arms from the muscle analysis (default 6).

- **Misc**.f_cutoff_IK: cutoff frequency for the butterworth recursive low pass filter applied to the inverse kinematics data (default is 6 Hz) when performing the muscle analysis to compute muscle-tendon lengths and moment arms.

- **Misc**.f_order_IK: order of the butterworth recursive low pass filter applied to the inverse kinematics data (default is 6).

- **Misc**.Mesh_Frequency: Number of mesh interval per second (default is 100, but a denser mesh might be required to obtain the desired accuracy especially for faster motions).

  ### Output arguments GPOPS

- Time: time vector.
- MExcitation: optimal muscle excitation (matrix dimension: number of collocation points x number of muscles).
- MActivation: optimal muscle activation (matrix dimension: number of collocation points x number of muscles).
- RActivation: activation of the reserve actuators (matrix dimension: number of collocation points x number of degrees of freedom).
- TForcetilde: normalized tendon force (matrix dimension: number of collocation points x number of muscles).
- TForce: tendon force (matrix dimension: number of collocation points x number of muscles).
- lMtilde: normalized muscle fiber length (matrix dimension: number of collocation 

 - MActivation: muscle activation
   - MActivation.meshPoints: muscle activation at mesh points (matrix dimension: number of mesh points x number of muscles).
  - MActivation.collocationPoints: muscle activation at collocation points (matrix dimension: number of collocation points x number of muscles). 
    \end{enumerate}

	- Activation.meshPoints: activation of the reserve actuators (matrix dimension: number of mesh points x number of degrees of freedom).

 - TForcetilde: normalized tendon force 
  - TForcetilde.meshPoints: normalized tendon force at mesh points (matrix dimension: number of mesh points x number of muscles).
  - TForcetilde.collocationPoints: normalized tendon force at collocation points (matrix dimension: number of collocation points x number of muscles). 
    \end{enumerate}

 - TForce: tendon force 
    - TForce.meshPoints: tendon force at mesh points (matrix dimension: number of mesh points x number of muscles).
    - TForce.collocationPoints: tendon force at collocation points (matrix dimension: number of collocation points x number of muscles). 

 - lMtilde: normalized muscle fiber length 
  - lMtilde.meshPoints: normalized muscle fiber length at mesh points (matrix dimension: number of mesh points x number of muscles).
  - lMtilde.collocationPoints: normalized muscle fiber length at collocation points (matrix dimension: number of collocation points x number of muscles). 
    \end{enumerate}	

- lM: muscle fiber length
   - M.meshPoints: muscle fiber length at mesh points (matrix dimension: number of mesh points x number of muscles).
   - lM.collocationPoints: muscle fiber length at collocation points (matrix dimension: number of collocation points x number of muscles). 

- MuscleNames: cell array that contains the names of the selected muscles (matrix dimension: number of muscles).

- OptInfo: output structure with settings used in CasADi.

- DatStore: data structure with input information for the optimal control problem.

## GPOPS-II

### Setup

The GPOPS-II setup is accessible through the function SolveMuscleRedundancy_(state).m under GPOPS setup. The user is referred to the GPOPS-II user guide for setup options. A higher accuracy can be reached by adjusting, for instance, the number of mesh intervals. This however comes at the expense of the computational time. 100 mesh intervals per second are used by default.

### Output

The GPOPS-II output, OptInfo, contains all information related to the optimal control problem solution. Convergence to an optimal solution is reached when output.result.nlpinfo is flagged 0 ("EXIT: Optimal solution found" in the command window of MATLAB). The mesh accuracy can be assessed with output.result.maxerrors. Cost functional, control, state (and costate) can be accessed in output.result.solution.phase.

To recall, the user should consider extending the time interval by 50-100 ms at the beginning and end of the motion to limit the influence of the unknown initial and final state on the solution. Results from those additional periods should not be considered realistic and will typically result in high muscle activation.

## Muscle model

The musculotendon properties are fully described in the supplementary materials of the aforementioned publication. Importantly, only the tendon slack length, optimal muscle fiber length, maximal isometric muscle force, optimal pennation angle and maximal muscle fiber contraction velocity are extracted from the referred OpenSim model. Other properties are defined in the code and can be changed if desired. By default, the activation and deactivation time constants are 15 and 60 ms respectively (see tau_act and tau_deact in SolveMuscleRedundancy_(state).m).
