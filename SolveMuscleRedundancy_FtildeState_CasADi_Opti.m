% SolveMuscleRedundancy_FtildeState, version 2.1 (October 2018)
%
% This function solves the muscle redundancy problem in the leg as 
% described in De Groote F, Kinney AL, Rao AV, Fregly BJ. Evaluation of 
% direct collocation optimal control problem formulations for solving the 
% muscle redundancy problem. Annals of Biomedical Engineering (2016).
%
% Change with regards to version 1.1: Use of CasADi instead of GPOPS-II.
% CasADi is an open-source tool for nonlinear optimization and algorithmic
% differentiation (see https://web.casadi.org/)
%
% Authors:  F. De Groote, M. Afschrift, A. Falisse, T. Van Wouwe
% Emails:   friedl.degroote@kuleuven.be
%           maarten.afschrift@kuleuven.be
%           antoine.falisse@kuleuven.be
%           tom.vanwouwe@kuleuven.be
%
% ----------------------------------------------------------------------- %
% This function uses the tendon force Ft as a state (see aforementionned
% publication for more details)
%
% INPUTS:
%           model_path: path to the .osim model
%           IK_path: path to the inverse kinematics results
%           ID_path: path to the inverse dynamics results
%           time: time window
%           OutPath: path to folder where results will be saved
%           Misc: structure of input data (see manual for more details)
%
% OUTPUTS:
%           Time: time window (as used when solving the optimal control
%           problem)
%           MExcitation: muscle excitation
%           MActivation: muscle activation
%           RActivation: activation of the reserve actuators
%           TForce_tilde: normalized tendon force
%           TForce: tendon force
%           lMtilde: normalized muscle fiber length
%           lM: muscle fiber length
%           MuscleNames: names of muscles
%           OptInfo: output of GPOPS-II
%           DatStore: structure with data used for solving the optimal
%           control problem



%
% ----------------------------------------------------------------------- %
%%

function [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtilde,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_FtildeState_CasADi_Opti(model_path,IK_path,ID_path,time,OutPath,Misc)

%% ---------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% PART I: INPUTS FOR OPTIMAL CONTROL PROBLEM ---------------------------- %
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% ----------------------------------------------------------------------- %
% Check for optional input arguments ------------------------------------ %

% Default low-pass filter:
%   Butterworth order: 6
%   Cutoff frequency: 6Hz
% Inverse Dynamics
if ~isfield(Misc,'f_cutoff_ID') || isempty(Misc.f_cutoff_ID)
    Misc.f_cutoff_ID=6;
end
if ~isfield(Misc,'f_order_ID') || isempty(Misc.f_order_ID)
    Misc.f_order_ID=6;
end
% Muscle-tendon lengths
if ~isfield(Misc,'f_cutoff_lMT') || isempty(Misc.f_cutoff_lMT)
    Misc.f_cutoff_lMT=6;
end
if ~isfield(Misc,'f_order_lMT') || isempty(Misc.f_order_lMT)
    Misc.f_order_lMT=6;
end
% Moment arms
if ~isfield(Misc,'f_cutoff_dM') || isempty(Misc.f_cutoff_dM)
    Misc.f_cutoff_dM=6;
end
if ~isfield(Misc,'f_order_dM') || isempty(Misc.f_order_dM)
    Misc.f_order_dM=6;
end
% Inverse Kinematics
if ~isfield(Misc,'f_cutoff_IK') || isempty(Misc.f_cutoff_IK)
    Misc.f_cutoff_IK=6;
end
if ~isfield(Misc,'f_order_IK') || isempty(Misc.f_order_IK)
    Misc.f_order_IK=6;
end
% Mesh Frequency
if ~isfield(Misc,'Mesh_Frequency') || isempty(Misc.Mesh_Frequency)
    Misc.Mesh_Frequency=100;
end


% ------------------------------------------------------------------------%
% Compute ID -------------------------------------------------------------%
if isempty(ID_path) || ~exist(ID_path,'file')
    disp('ID path was not specified or the file does not exist, computation ID started');
    if ~isfield(Misc,'Loads_path') || isempty(Misc.Loads_path) || ~exist(Misc.Loads_path,'file')
        error('External loads file was not specified or does not exist, please add the path to the external loads file: Misc.Loads_path');
    else
        %check the output path for the ID results
        if isfield(Misc,'ID_ResultsPath')
            [idpath,~]=fileparts(Misc.ID_ResultsPath);
            if ~isdir(idpath); mkdir(idpath); end
        else 
            % save results in the directory of the external loads
            [Lpath,name,~]=fileparts(Misc.Loads_path);
            Misc.ID_ResultsPath=fullfile(Lpath,name);
        end
        [ID_outPath,ID_outName,ext]=fileparts(Misc.ID_ResultsPath);
        output_settings=fullfile(ID_outPath,[ID_outName '_settings.xml']);
        Opensim_ID(model_path,[time(1)-0.1 time(2)+0.1],Misc.Loads_path,IK_path,ID_outPath,[ID_outName ext],output_settings);
        ID_path=Misc.ID_ResultsPath;
    end    
end

% ----------------------------------------------------------------------- %
% Muscle analysis ------------------------------------------------------- %

Misc.time=time;
MuscleAnalysisPath=fullfile(OutPath,'MuscleAnalysis'); if ~exist(MuscleAnalysisPath,'dir'); mkdir(MuscleAnalysisPath); end
disp('MuscleAnalysis Running .....');
OpenSim_Muscle_Analysis(IK_path,model_path,MuscleAnalysisPath,[time(1) time(end)])
disp('MuscleAnalysis Finished');
Misc.MuscleAnalysisPath=MuscleAnalysisPath;

% ----------------------------------------------------------------------- %
% Extract muscle information -------------------------------------------- %

% Get number of degrees of freedom (dofs), muscle-tendon lengths and moment
% arms for the selected muscles.
[~,Misc.trialName,~]=fileparts(IK_path);
if ~isfield(Misc,'MuscleNames_Input') || isempty(Misc.MuscleNames_Input)    
    Misc=getMuscles4DOFS(Misc);
end
[DatStore] = getMuscleInfo(IK_path,ID_path,Misc);

% ----------------------------------------------------------------------- %
% Solve the muscle redundancy problem using static optimization --------- %

% The solution of the static optimization is used as initial guess for the
% dynamic optimization
% Extract the muscle-tendon properties
[DatStore.params,DatStore.lOpt,DatStore.L_TendonSlack,DatStore.Fiso,DatStore.PennationAngle]=ReadMuscleParameters(model_path,DatStore.MuscleNames);
% Static optimization using IPOPT solver
DatStore = SolveStaticOptimization_IPOPT(DatStore);

%% ---------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% PART II: OPTIMAL CONTROL PROBLEM FORMULATION -------------------------- %
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% Input arguments
auxdata.NMuscles = DatStore.nMuscles;   % number of muscles
auxdata.Ndof = DatStore.nDOF;           % number of dofs
auxdata.ID = DatStore.T_exp;            % inverse dynamics
auxdata.params = DatStore.params;       % Muscle-tendon parameters
auxdata.scaling.dFTtilde = 10;          % Scaling factor: derivative muscle-tendon force
auxdata.w1 = 1000;                      % Weight objective function
auxdata.w2 = 0.01;                      % Weight objective function
auxdata.Topt = 150;                     % Scaling factor: reserve actuators

% ADiGator works with 2D: convert 3D arrays to 2D structure (moment arms)
for i = 1:auxdata.Ndof
    auxdata.MA(i).Joint(:,:) = DatStore.dM(:,i,:);  % moment arms
end
auxdata.DOFNames = DatStore.DOFNames;   % names of dofs

tau_act = 0.015; auxdata.tauAct = tau_act * ones(1,auxdata.NMuscles);       % activation time constant (activation dynamics)
tau_deact = 0.06; auxdata.tauDeact = tau_deact * ones(1,auxdata.NMuscles);  % deactivation time constant (activation dynamics)
auxdata.b = 0.1;                                                            % parameter determining transition smoothness (activation dynamics)

% Parameters of active muscle force-velocity characteristic
load('ActiveFVParameters.mat','ActiveFVParameters');
Fvparam(1) = 1.475*ActiveFVParameters(1); Fvparam(2) = 0.25*ActiveFVParameters(2);
Fvparam(3) = ActiveFVParameters(3) + 0.75; Fvparam(4) = ActiveFVParameters(4) - 0.027;
auxdata.Fvparam = Fvparam;

% Parameters of active muscle force-length characteristic
load('Faparam.mat','Faparam');                   
auxdata.Faparam = Faparam;

% Parameters of passive muscle force-length characteristic
e0 = 0.6; kpe = 4; t50 = exp(kpe * (0.2 - 0.10e1) / e0);
pp1 = (t50 - 0.10e1); t7 = exp(kpe); pp2 = (t7 - 0.10e1);
auxdata.Fpparam = [pp1;pp2];
auxdata.Atendon=Misc.Atendon;
auxdata.shift=Misc.shift;

% Problem bounds 
e_min = 0; e_max = 1;           % bounds on muscle excitation
a_min = 0; a_max = 1;           % bounds on muscle activation
F_min = 0; F_max = 5;           % bounds on normalized tendon force
dF_min = -50; dF_max = 50;      % bounds on derivative of normalized tendon force

% Time bounds
t0 = DatStore.time(1); tf = DatStore.time(end);
bounds.phase.initialtime.lower = t0; bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tf; bounds.phase.finaltime.upper = tf;
% Controls bounds
umin = e_min*ones(1,auxdata.NMuscles); umax = e_max*ones(1,auxdata.NMuscles);
dFMin = dF_min*ones(1,auxdata.NMuscles); dFMax = dF_max*ones(1,auxdata.NMuscles);
aTmin = -1*ones(1,auxdata.Ndof); aTmax = 1*ones(1,auxdata.Ndof);
bounds.phase.control.lower = [umin aTmin dFMin]; bounds.phase.control.upper = [umax aTmax dFMax];
% States bounds
actMin = a_min*ones(1,auxdata.NMuscles); actMax = a_max*ones(1,auxdata.NMuscles);
F0min = F_min*ones(1,auxdata.NMuscles); F0max = F_max*ones(1,auxdata.NMuscles);
Ffmin = F_min*ones(1,auxdata.NMuscles); Ffmax = F_max*ones(1,auxdata.NMuscles);
FMin = F_min*ones(1,auxdata.NMuscles); FMax = F_max*ones(1,auxdata.NMuscles);
bounds.phase.initialstate.lower = [actMin, F0min]; bounds.phase.initialstate.upper = [actMax, F0max];
bounds.phase.state.lower = [actMin, FMin]; bounds.phase.state.upper = [actMax, FMax];
bounds.phase.finalstate.lower = [actMin, Ffmin]; bounds.phase.finalstate.upper = [actMax, Ffmax];
% Integral bounds
bounds.phase.integral.lower = 0; 
bounds.phase.integral.upper = 10000*(tf-t0);

% Path constraints
HillEquil = zeros(1, auxdata.NMuscles);
ID_bounds = zeros(1, auxdata.Ndof);
bounds.phase.path.lower = [ID_bounds,HillEquil]; bounds.phase.path.upper = [ID_bounds,HillEquil];

% Eventgroup
% Impose mild periodicity
pera_lower = -1 * ones(1, auxdata.NMuscles); pera_upper = 1 * ones(1, auxdata.NMuscles);
perFtilde_lower = -1 * ones(1, auxdata.NMuscles); perFtilde_upper = 1 * ones(1, auxdata.NMuscles);
bounds.eventgroup.lower = [pera_lower perFtilde_lower]; bounds.eventgroup.upper = [pera_upper perFtilde_upper];

% Initial guess
N = length(DatStore.time);
guess.phase.time = DatStore.time;
guess.phase.control = [DatStore.SoAct DatStore.SoRAct./150 zeros(N,auxdata.NMuscles)];
guess.phase.state =  [DatStore.SoAct DatStore.SoAct];

guess.phase.integral = 0;

% Spline structures
for dof = 1:auxdata.Ndof
    for m = 1:auxdata.NMuscles       
        auxdata.JointMASpline(dof).Muscle(m) = spline(DatStore.time,auxdata.MA(dof).Joint(:,m));       
    end
    auxdata.JointIDSpline(dof) = spline(DatStore.time,DatStore.T_exp(:,dof));
end

for m = 1:auxdata.NMuscles
    auxdata.LMTSpline(m) = spline(DatStore.time,DatStore.LMT(:,m));
end

%% CasADi setup
import casadi.*
opti = casadi.Opti(); % Create opti instance

% Collocation scheme
d = 3; % degree of interpolating polynomial
method = 'radau'; % other option is 'legendre' (collocation scheme)
[tau_root,C,D,B] = CollocationScheme(d,method);
% Load CasADi functions
CasADiFunctions

% Discretization
N = round((tf-t0)*Misc.Mesh_Frequency);
h = (tf-t0)/N;

% Interpolation
step = (tf-t0)/(N);
time_opt = t0:step:tf;
LMTinterp = zeros(length(time_opt),auxdata.NMuscles);
VMTinterp = zeros(length(time_opt),auxdata.NMuscles);
for m = 1:auxdata.NMuscles
    [LMTinterp(:,m),VMTinterp(:,m),~] = SplineEval_ppuval(auxdata.LMTSpline(m),time_opt,1);
end
MAinterp = zeros(length(time_opt),auxdata.Ndof*auxdata.NMuscles);
IDinterp = zeros(length(time_opt),auxdata.Ndof);
for dof = 1:auxdata.Ndof
    for m = 1:auxdata.NMuscles
        index_sel=(dof-1)*(auxdata.NMuscles)+m;
        MAinterp(:,index_sel) = ppval(auxdata.JointMASpline(dof).Muscle(m),time_opt);   
    end
    IDinterp(:,dof) = ppval(auxdata.JointIDSpline(dof),time_opt);
end

% Initial guess
% Based on SO
% guess.phase.control = [zeros(N,auxdata.NMuscles) DatStore.SoRAct./150 zeros(N,auxdata.NMuscles)];
% guess.phase.state =  [DatStore.SoAct DatStore.SoAct];
% Random
guess.phase.control = [zeros(N,auxdata.NMuscles) zeros(N,auxdata.Ndof) 0.01*ones(N,auxdata.NMuscles)];
guess.phase.state =  [0.2*ones(N,auxdata.NMuscles) 0.2*ones(N,auxdata.NMuscles)];

% Empty NLP
w   = {};
w0  = [];
lbw = [];
ubw = [];
J   = 0;
g   = {};
lbg = [];
ubg = [];

% States
% Muscle activations
a = opti.variable(auxdata.NMuscles,N+1);
amesh = opti.variable(auxdata.NMuscles,d*N);
opti.subject_to(a_min < a < a_max);
opti.subject_to(a_min < amesh < a_max);
opti.set_initial(a,0.2);
opti.set_initial(amesh,0.2);
% Muscle-tendon forces
FTtilde = opti.variable(auxdata.NMuscles,N+1);
FTtildemesh = opti.variable(auxdata.NMuscles,d*N);
opti.subject_to(F_min < FTtilde < F_max);
opti.subject_to(F_min < FTtildemesh < F_max);
opti.set_initial(FTtilde,0.2);
opti.set_initial(FTtildemesh,0.2);

% Controls
% Muscle excitations
e = opti.variable(auxdata.NMuscles,N);
opti.subject_to(e_min < e < e_max);
% Reserve actuators
aT = opti.variable(auxdata.Ndof,N);
opti.subject_to(-1 < aT <1);
% Time derivative of muscle-tendon forces (states)
dFTtilde = opti.variable(auxdata.NMuscles,N);
opti.subject_to(dF_min < dFTtilde < dF_max);
opti.set_initial(dFTtilde,0.01);


% Loop over mesh points
for k=1:N
    ak = a(:,k); FTtildek = FTtilde(:,k);
    ak_colloc = [ak amesh(:,(k-1)*d+1:k*d)]; FTtildek_colloc = [FTtildek FTtildemesh(:,(k-1)*d+1:k*d)];
    dFTtildek = dFTtilde(:,k); aTk = aT(:,k); ek = e(:,k);

    % Loop over collocation points
    for j=1:d
        % Expression of the state derivatives at the collocation points
        ap = ak_colloc*C(:,j+1);
        FTtildep = FTtildek_colloc*C(:,j+1);
        
        % Append collocation equations
        % Activation dynamics (explicit formulation)  
        opti.subject_to(h*f_ActivationDynamics(ek,ak_colloc(:,j+1)) - ap == 0);

        % Contraction dynamics (implicit formulation)    
        opti.subject_to(h*dFTtildek.*auxdata.scaling.dFTtilde - FTtildep == 0)

        % Add contribution to the quadrature function
        J = J + ...
            B(j+1)*f_ssNMuscles(ek)*h + ...   
            auxdata.w1*B(j+1)*f_ssNdof(aTk')*h + ...
            auxdata.w2*B(j+1)*f_ssNMuscles(dFTtildek)*h;
    end
    
    % State continuity at mesh transition
    opti.subject_to(a(:,k+1)== ak_colloc*D);  
    opti.subject_to(FTtilde(:,k+1) == FTtildek_colloc*D); 
    
 % Get muscle-tendon forces and derive Hill-equilibrium
[Hilldiffk,FTk] = f_forceEquilibrium_FtildeState(ak',FTtildek',dFTtildek'.*auxdata.scaling.dFTtilde,LMTinterp(k,:)',VMTinterp(k,:)'); 

% Add path constraints
% Moment constraints
for dof = 1:auxdata.Ndof
    T_exp = IDinterp(k,dof);    
    index_sel = (dof-1)*(auxdata.NMuscles)+1:(dof-1)*(auxdata.NMuscles)+auxdata.NMuscles;
    T_sim = f_spNMuscles(MAinterp(k,index_sel),FTk) + auxdata.Topt*aTk(dof);   
    opti.subject_to(T_exp-T_sim == 0)
end     

% Hill-equilibrium constraint
opti.subject_to(Hilldiffk == 0); 
end

opti.minimize(J);

% Create an NLP solver
optionssol.ipopt.nlp_scaling_method = 'gradient-based'; 
optionssol.ipopt.linear_solver = 'mumps';
optionssol.ipopt.tol = 1e-6;
optionssol.ipopt.max_iter = 10000;
opti.solver('ipopt',optionssol)
% Solve
diary('DynamicOptimization_FtildeState_CasADi_Opti.txt'); 
sol = opti.solve();
diary off

output.setup.bounds = bounds;
output.setup.auxdata = auxdata;
output.setup.guess = guess;
output.setup.lbw = lbw;
output.setup.ubw = ubw;
output.setup.nlp.solver = 'ipopt';
output.setup.nlp.ipoptoptions.linear_solver = 'mumps';
output.setup.derivatives.derivativelevel = 'second';
output.setup.nlp.ipoptoptions.tolerance = optionssol.ipopt.tol;
output.setup.nlp.ipoptoptions.maxiterations = optionssol.ipopt.max_iter;

%% Extract results
% Variables at mesh points
% Muscle activations and muscle-tendon forces
a_opt = sol.value(a)';
amesh_opt = sol.value(amesh)';
FTtilde_opt = sol.value(FTtilde)';
FTtildemesh_opt = sol.value(FTtildemesh)';
% Muscle excitations
e_opt = sol.value(e)';
% Reserve actuators
aT_opt = sol.value(aT)';
% Time derivatives of muscle-tendon forces
dFTtilde_opt = sol.value(dFTtilde)';

% Variables at collocation points
% Muscle activations
a_opt_ext = zeros(N*(d+1)+1,auxdata.NMuscles);
a_opt_ext(1:(d+1):end,:) = a_opt;
a_opt_ext(2:(d+1):end,:) = amesh_opt(1:d:end,:);
a_opt_ext(3:(d+1):end,:) = amesh_opt(2:d:end,:);
a_opt_ext(4:(d+1):end,:) = amesh_opt(3:d:end,:);
% Muscle-tendon forces
FTtilde_opt_ext = zeros(N*(d+1)+1,auxdata.NMuscles);
FTtilde_opt_ext(1:(d+1):end,:) = FTtilde_opt;
FTtilde_opt_ext(2:(d+1):end,:) = FTtildemesh_opt(1:d:end,:);
FTtilde_opt_ext(3:(d+1):end,:) = FTtildemesh_opt(2:d:end,:);
FTtilde_opt_ext(4:(d+1):end,:) = FTtildemesh_opt(3:d:end,:);

% Grid    
% Mesh points
tgrid = linspace(t0,tf,N+1);
dtime = zeros(1,d+1);
for i = 1:d+1
    dtime(i)=tau_root(i)*((tf-t0)/N);
end
% Mesh points and collocation points
tgrid_ext = zeros(1,(d+1)*N+1);
for i = 1:N
    tgrid_ext(((i-1)*(d+1)+1):1:i*(d+1)) = tgrid(i) + dtime;
end
tgrid_ext(end) = tf; 

% Save results
Time.meshPoints = tgrid;
Time.collocationPoints = tgrid_ext;
MActivation.meshPoints = a_opt;
MActivation.collocationPoints = a_opt_ext;  
TForcetilde.meshPoints = FTtilde_opt;
TForcetilde.collocationPoints = FTtilde_opt_ext;  
TForce.meshPoints = FTtilde_opt.*repmat(DatStore.Fiso,length(Time.meshPoints),1);
TForce.collocationPoints = FTtilde_opt_ext.*repmat(DatStore.Fiso,length(Time.collocationPoints),1);
MExcitation.meshPoints = e_opt;
RActivation.meshPoints = aT_opt*auxdata.Topt;
MuscleNames = DatStore.MuscleNames;
OptInfo = output;
% Muscle fiber lengths from Ftilde
lMTinterp.meshPoints = interp1(DatStore.time,DatStore.LMT,Time.meshPoints);
[lM.meshPoints,lMtilde.meshPoints] = FiberLength_Ftilde(TForcetilde.meshPoints,auxdata.params,lMTinterp.meshPoints,auxdata.Atendon,auxdata.shift);
lMTinterp.collocationPoints = interp1(DatStore.time,DatStore.LMT,Time.collocationPoints);
[lM.collocationPoints,lMtilde.collocationPoints] = FiberLength_Ftilde(TForcetilde.collocationPoints,auxdata.params,lMTinterp.collocationPoints,auxdata.Atendon,auxdata.shift);
end
