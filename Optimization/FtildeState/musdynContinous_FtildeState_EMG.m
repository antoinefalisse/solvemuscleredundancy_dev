function phaseout = musdynContinous_FtildeState_EMG(input)

% Get input data
NMuscles        = input.auxdata.NMuscles;
Ndof            = input.auxdata.Ndof;
tauAct          = input.auxdata.tauAct;
tauDeact        = input.auxdata.tauDeact;
params          = input.auxdata.params;
splinestruct    = input.auxdata.splinestruct;
numColPoints    = size(input.phase.state,1);

% Get controls
e   = input.phase.control(:,1:NMuscles);
aT  = input.phase.control(:,NMuscles+1:NMuscles+Ndof);
dFtilde  = 10*input.phase.control(:,NMuscles+Ndof+1:end);

% Get states
a       = input.phase.state(:,1:NMuscles);
Ftilde = input.phase.state(:,NMuscles+1:end);

% PATH CONSTRAINTS
% Hill-equilibrium constraint

if input.auxdata.Par_Elastic == 1 && input.auxdata.FL_Relation == 1
    [Hilldiff,F] = ForceEquilibrium_FtildeState(a,Ftilde,dFtilde,splinestruct.LMT,splinestruct.VMT,params,...
        input.auxdata.Fvparam,input.auxdata.Fpparam,input.auxdata.Faparam,input.auxdata.Atendon,input.auxdata.shift);
elseif input.auxdata.Par_Elastic == 0
    [Hilldiff,F] = ForceEquilibrium_FtildeState_v2(a,Ftilde,dFtilde,splinestruct.LMT,splinestruct.VMT,params,...
        input.auxdata.Fvparam,input.auxdata.Fpparam,input.auxdata.Faparam,input.auxdata.Atendon,input.auxdata.shift);
elseif input.auxdata.FL_Relation == 0 && input.auxdata.Par_Elastic==0
    [Hilldiff,F] = ForceEquilibrium_FtildeState_v3(a,Ftilde,dFtilde,splinestruct.LMT,splinestruct.VMT,params,...
        input.auxdata.Fvparam,input.auxdata.Fpparam,input.auxdata.Faparam,input.auxdata.Atendon,input.auxdata.shift);
else
    error('Error: There is currently no option to account for a parralel elastic element but not for the force-length force-velocity properties');
end



% Moments constraint
Topt = 150;
Tdiff = zeros(numColPoints,Ndof);
for dof = 1:Ndof
    T_exp=splinestruct.ID(:,dof);
    index_sel=(dof-1)*(NMuscles)+1:(dof-1)*(NMuscles)+NMuscles;
    T_sim=sum(F.*splinestruct.MA(:,index_sel),2) + Topt*aT(:,dof);
    Tdiff(:,dof) =  (T_exp-T_sim);
end

% EMG constraints
Scale_EMG   = input.phase.parameter;
EMG         = splinestruct.EMG;
e_EMG       = e(:,input.auxdata.EMGindices);
EMG_error   = e_EMG - Scale_EMG.*EMG;


% outputs
phaseout.path = [Tdiff Hilldiff EMG_error];

% DYNAMIC CONSTRAINTS
% Activation dynamics
dadt = ones(numColPoints,NMuscles);
for m = 1:NMuscles
    dadt(:,m) = ActivationDynamics(e(:,m),a(:,m),tauAct(m),tauDeact(m),input.auxdata.b);
end

% Contraction dynamics is implicit

phaseout.dynamics = [dadt dFtilde];

% OBJECTIVE FUNCTION
w1 = 1000;
w2 = 0.01;
phaseout.integrand = sum(e.^2,2)+ w1.*sum(aT.^2,2)+ w2*sum((dFtilde/10).^2,2);





