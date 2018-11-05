% SolveMuscleRedundancy_lMtildeState, version 2.1 (October 2018)
%
% This function solves the muscle redundancy problem in the leg using the
% direct collocation optimal control software GPOPS-II as described in De
% Groote F, Kinney AL, Rao AV, Fregly BJ. Evaluation of direct
% collocation optimal control problem formulations for solving the muscle
% redundancy problem. Annals of Biomedical Engineering (2016).
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
% This function uses the normalized muscle fiber length as a state (see
% aforementioned publication for more details)
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

function [Time,MExcitation,MActivation,RActivation,TForcetilde,TForce,lMtildeopt,lM,MuscleNames,OptInfo,DatStore]=SolveMuscleRedundancy_lMtildeState_CasADi(model_path,IK_path,ID_path,time,OutPath,Misc)

%% ---------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %
% PART I: INPUTS FOR OPTIMAL CONTROL PROBLEM ---------------------------- %
% ----------------------------------------------------------------------- %
% ----------------------------------------------------------------------- %

% ----------------------------------------------------------------------- %
% Check for optional input arguments, see manual for details------------- %

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
% Mes Frequency
if ~isfield(Misc,'Mesh_Frequency') || isempty(Misc.Mesh_Frequency)
   Misc.Mesh_Frequency=100;
end

% Round time window to 2 decimals
time=round(time,2);
if time(1)==time(2)
   warning('Time window should be at least 0.01s'); 
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
auxdata.scaling.vMtilde = 10;           % Scaling factor: derivative muscle fiber lengths
auxdata.w1 = 1000;                      % Weight objective function
auxdata.w2 = 0.01; 
auxdata.Topt = 150;                     % Scaling factor: reserve actuators

% ADiGator works with 2D: convert 3D arrays to 2D structure (moment arms)
for i = 1:auxdata.Ndof
    auxdata.MA(i).Joint(:,:) = DatStore.dM(:,i,:);  % moment arms
end
auxdata.DOFNames = DatStore.DOFNames;   % names of dofs

tau_act = 0.015; auxdata.tauAct = tau_act * ones(1,auxdata.NMuscles);       % activation time constant (activation dynamics)
tau_deact = 0.06; auxdata.tauDeact = tau_deact * ones(1,auxdata.NMuscles);  % deactivation time constant (activation dynamics)
auxdata.b = 0.1;   

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
e_min = 0; e_max = 1;                   % bounds on muscle excitation
a_min = 0; a_max = 1;                   % bounds on muscle activation
vMtilde_min = -1; vMtilde_max = 1;      % bounds on normalized muscle fiber velocity
lMtilde_min = 0.2; lMtilde_max = 1.8;   % bounds on normalized muscle fiber length

% Time bounds
t0 = DatStore.time(1); tf = DatStore.time(end);
bounds.phase.initialtime.lower = t0; bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower = tf; bounds.phase.finaltime.upper = tf;
% Controls bounds
umin = e_min*ones(1,auxdata.NMuscles); umax = e_max*ones(1,auxdata.NMuscles);
vMtildemin = vMtilde_min*ones(1,auxdata.NMuscles); vMtildemax = vMtilde_max*ones(1,auxdata.NMuscles);
aTmin = -1*ones(1,auxdata.Ndof); aTmax = 1*ones(1,auxdata.Ndof);
bounds.phase.control.lower = [umin aTmin vMtildemin]; bounds.phase.control.upper = [umax aTmax vMtildemax];
% States bunds
actMin = a_min*ones(1,auxdata.NMuscles); actMax = a_max*ones(1,auxdata.NMuscles);
lMtildemin = lMtilde_min*ones(1,auxdata.NMuscles); lMtildemax = lMtilde_max*ones(1,auxdata.NMuscles);
bounds.phase.initialstate.lower = [actMin, lMtildemin]; bounds.phase.initialstate.upper = [actMax, lMtildemax];
bounds.phase.state.lower = [actMin, lMtildemin]; bounds.phase.state.upper = [actMax, lMtildemax];
bounds.phase.finalstate.lower = [actMin, lMtildemin]; bounds.phase.finalstate.upper = [actMax, lMtildemax];
% Integral bounds
bounds.phase.integral.lower = 0; bounds.phase.integral.upper = 10000*(tf-t0);

% Path constraints
HillEquil = zeros(1, auxdata.NMuscles);
ID_bounds = zeros(1, auxdata.Ndof);
bounds.phase.path.lower = [ID_bounds,HillEquil]; bounds.phase.path.upper = [ID_bounds,HillEquil];

% Eventgroup
% Impose mild periodicity
pera_lower = -1 * ones(1, auxdata.NMuscles); pera_upper = 1 * ones(1, auxdata.NMuscles);
perlMtilde_lower = -1*ones(1,auxdata.NMuscles); perlMtilde_upper = 1*ones(1,auxdata.NMuscles);
bounds.eventgroup.lower = [pera_lower perlMtilde_lower]; bounds.eventgroup.upper = [pera_upper perlMtilde_upper];

% Initial guess
N = length(DatStore.time);
guess.phase.time = DatStore.time;
% Static optimization results as initial guess
% guess.phase.control = [DatStore.SoAct DatStore.SoRAct./150 0.01*ones(N,auxdata.NMuscles)];
% guess.phase.state =  [DatStore.SoAct ones(N,auxdata.NMuscles)];
% Random initial guess
guess.phase.control = [0.2*ones(N,auxdata.NMuscles) zeros(N,auxdata.Ndof) 0.2*ones(N,auxdata.NMuscles)];
guess.phase.state =  [0.2*ones(N,auxdata.NMuscles) ones(N,auxdata.NMuscles)];
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
opti = casadi.Opti();

% Collocation scheme
d = 3; % degree of interpolating polynomial
method = 'radau'; % other option is 'legendre' (collocation scheme)
[tau_root,C,D,B] = CollocationScheme(d,method);
% Load CasADi function
CasADiFunctions

% Discretization
N = round((tf-t0)*Misc.Mesh_Frequency);
h = (tf-t0)/N;

% Interpolation
step = (tf-t0)/(N);
time_opt = t0:step:tf;
LMTinterp = zeros(length(time_opt),auxdata.NMuscles);
for m = 1:auxdata.NMuscles
    [LMTinterp(:,m),~,~] = SplineEval_ppuval(auxdata.LMTSpline(m),time_opt,1);
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
SoActInterp = interp1(DatStore.time,DatStore.SoAct,time_opt);
SoRActInterp = interp1(DatStore.time,DatStore.SoRAct,time_opt);

guess.phase.control = [SoActInterp(1:N,:) SoRActInterp(1:N,:)./150 0.01*ones(N,auxdata.NMuscles)];
guess.phase.state =  [SoActInterp ones(N+1,auxdata.NMuscles)];

% Random
% guess.phase.control = [zeros(N,auxdata.NMuscles) zeros(N,auxdata.Ndof) 0.01*ones(N,auxdata.NMuscles)];
% guess.phase.state =  [0.2*ones(N+1,auxdata.NMuscles) ones(N+1,auxdata.NMuscles)];

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
opti.set_initial(a,SoActInterp');
opti.set_initial(amesh,(reshape(permute(repmat(SoActInterp(1:N,:),1,1,d),[3,1,2]),d*N,auxdata.NMuscles)'));
% Muscle fiber lengths
lMtilde = opti.variable(auxdata.NMuscles,N+1);
lMtildemesh = opti.variable(auxdata.NMuscles,d*N);
opti.subject_to(lMtilde_min < lMtilde < lMtilde_max);
opti.subject_to(lMtilde_min < lMtildemesh < lMtilde_max);
opti.set_initial(lMtilde, ones(N+1,auxdata.NMuscles)');
opti.set_initial(lMtildemesh, ones(d*N,auxdata.NMuscles)');

% Controls
% Muscle excitations
e = opti.variable(auxdata.NMuscles,N);
opti.subject_to(e_min < e < e_max);
opti.set_initial(e, SoActInterp(1:N,:)')
% Reserve actuators
aT = opti.variable(auxdata.Ndof,N);
opti.subject_to(-1 < aT < 1);
opti.set_initial(aT,SoRActInterp(1:N,:)'/150)
% Time derivative of muscle-tendon forces (states)
vMtilde = opti.variable(auxdata.NMuscles,N);
opti.subject_to(vMtilde_min < vMtilde < vMtilde_max);
opti.set_initial(vMtilde,0.01);

% Loop over mesh points
for k=1:N
    
    ak = a(:,k); lMtildek = lMtilde(:,k);
    ak_colloc = [ak amesh(:,(k-1)*d+1:k*d)]; lMtildek_colloc = [lMtildek lMtildemesh(:,(k-1)*d+1:k*d)];
    vMtildek = vMtilde(:,k); aTk = aT(:,k); ek = e(:,k);

    % Loop over collocation points
    for j=1:d
        % Expression of the state derivatives at the collocation points
        ap = ak_colloc*C(:,j+1);
        lMtildep = lMtildek_colloc*C(:,j+1);
        
        % Append collocation equations
        % Activation dynamics (explicit formulation)  
        opti.subject_to(h*f_ActivationDynamics(ek,ak_colloc(:,j+1)) - ap == 0);

        % Contraction dynamics (implicit formulation)    
        opti.subject_to(h*vMtildek.*auxdata.scaling.vMtilde - lMtildep == 0)   
        % Add contribution to the quadrature function
        J = J + ...
            B(j+1)*f_ssNMuscles(ek')*h + ...   
            auxdata.w1*B(j+1)*f_ssNdof(aTk')*h + ...
            auxdata.w2*B(j+1)*f_ssNMuscles(vMtildek)*h;

    end
    
    % State continuity at mesh transition
    opti.subject_to(a(:,k+1)== ak_colloc*D);  
    opti.subject_to(lMtilde(:,k+1) == lMtildek_colloc*D);
    
    % Get muscle-tendon forces and derive Hill-equilibrium
    [Hilldiffk,FTk] = f_forceEquilibrium_lMtildeState(ak,lMtildek,vMtildek,LMTinterp(k,:)'); 
    % Add path constraints
    % Moment constraints
    for dof = 1:auxdata.Ndof
        T_exp = IDinterp(k,dof);    
        index_sel = (dof-1)*(auxdata.NMuscles)+1:(dof-1)*(auxdata.NMuscles)+auxdata.NMuscles;
        T_sim = f_spNMuscles(MAinterp(k,index_sel),FTk) + auxdata.Topt*aTk(dof);   
        opti.subject_to(T_exp - T_sim == 0);
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
diary('DynamicOptimization_lMtildeState_CasADi_Opti.txt'); 
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
lMtilde_opt = sol.value(lMtilde)';
lMtildemesh_opt = sol.value(lMtildemesh)';
% Muscle excitations
e_opt = sol.value(e)';
% Reserve actuators
aT_opt = sol.value(aT)';
% Time derivatives of muscle-tendon forces
vMtilde_opt = sol.value(vMtilde)';

% Variables at collocation points
% Muscle activations
a_opt_ext = zeros(N*(d+1)+1,auxdata.NMuscles);
a_opt_ext(1:(d+1):end,:) = a_opt;
a_opt_ext(2:(d+1):end,:) = amesh_opt(1:d:end,:);
a_opt_ext(3:(d+1):end,:) = amesh_opt(2:d:end,:);
a_opt_ext(4:(d+1):end,:) = amesh_opt(3:d:end,:);
% Muscle-tendon forces
lMtilde_opt_ext = zeros(N*(d+1)+1,auxdata.NMuscles);
lMtilde_opt_ext(1:(d+1):end,:) = lMtilde_opt;
lMtilde_opt_ext(2:(d+1):end,:) = lMtildemesh_opt(1:d:end,:);
lMtilde_opt_ext(3:(d+1):end,:) = lMtildemesh_opt(2:d:end,:);
lMtilde_opt_ext(4:(d+1):end,:) = lMtildemesh_opt(3:d:end,:);


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
lMtildeopt.meshPoints = lMtilde_opt;
lMtildeopt.collocationPoints = lMtilde_opt_ext;  
lM.meshPoints = lMtilde_opt.*repmat(DatStore.lOpt,length(Time.meshPoints),1);
lM.collocationPoints = lMtilde_opt_ext.*repmat(DatStore.lOpt,length(Time.collocationPoints),1);
MExcitation.meshPoints = e_opt;
RActivation.meshPoints = aT_opt*auxdata.Topt;
MuscleNames = DatStore.MuscleNames;
OptInfo = output;
% Tendon forces from lMtilde
lMTinterp.meshPoints = interp1(DatStore.time,DatStore.LMT,Time.meshPoints);
[TForcetilde.meshPoints,TForce.meshPoints] = TendonForce_lMtilde(lMtildeopt.meshPoints,auxdata.params,lMTinterp.meshPoints,auxdata.Atendon,auxdata.shift);
lMTinterp.collocationPoints = interp1(DatStore.time,DatStore.LMT,Time.collocationPoints);
[TForcetilde.collocationPoints,TForce.collocationPoints] = TendonForce_lMtilde(lMtildeopt.collocationPoints,auxdata.params,lMTinterp.collocationPoints,auxdata.Atendon,auxdata.shift);
end
