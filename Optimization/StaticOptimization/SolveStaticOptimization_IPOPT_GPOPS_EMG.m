function DatStore = SolveStaticOptimization_IPOPT_GPOPS_EMG(DatStore)

% STEP 1. Inputs
% --------------

time = DatStore.time;
N = length(time);
M = DatStore.nMuscles;
nDof = DatStore.nDOF;

load('ActiveFVParameters.mat','ActiveFVParameters');
load('PassiveFLParameters','PassiveFLParameters');
load('Faparam.mat','Faparam');

act = ones(N,M);
FMltilde = ones(N,M);
FMvtilde = ones(N,M);
Fpe = ones(N,M);
cos_alpha = ones(N,M);
for m = 1:M
    pp_y = spline(time,DatStore.LMT(:,m));
    [LMTg,vMTg,~] = SplineEval_ppuval(pp_y,time,1);
    [~, ~, FMltilde(:,m), FMvtilde(:,m), Fpe(:,m), cos_alpha(:,m)] = ...
        HillModel_RigidTendon(act(:,m),LMTg,vMTg,DatStore.params(:,m),...
        ActiveFVParameters,PassiveFLParameters,Faparam);
    clear pp_y 
end

if DatStore.Par_Elastic == 0 && DatStore.FL_Relation == 1
    Fpe = zeros(size(Fpe));
elseif DatStore.Par_Elastic == 0 && DatStore.FL_Relation == 0
    FMltilde = ones(size(FMltilde));
    FMvtilde = ones(size(FMltilde));
    Fpe = zeros(size(Fpe));
end

FMo = ones(size(act,1),1)*DatStore.Fiso;
Fpas = FMo.*Fpe.*cos_alpha;
Fact = FMo.*FMltilde.*FMvtilde.*cos_alpha;

% Add optimal force for reserve torques
Topt = 1;%150/sqrt(1000);
Fpas = [Fpas zeros(N,nDof)];
Fact = [Fact Topt*ones(N,nDof)];



ID_data = DatStore.T_exp;

I = N*(M+nDof);
MomentArms = zeros(I,nDof);
temp = zeros(N,M);
for i = 1:nDof    
    temp(:,:) = DatStore.dM(:,i,:);
    MomentArms(:,i) = ...
        reshape([temp zeros(N,i-1) ones(N,1) zeros(N,nDof-i)]',I,1);    
end

% STEP 2. Optimization
% --------------------

% EMG information
MaxScale    = DatStore.MaxScale;
BoundMin    = min(DatStore.EMGbounds);
BoundMax    = max(DatStore.EMGbounds);
nEMG        = DatStore.nEMG;        % update this based on input information
EMGindices  = DatStore.EMGindices;
actEMG      = DatStore.actEMGintSO;

% Initial guess
x0 = [repmat(0.01*ones(M+nDof,1),N,1); ones(nEMG,1)];

% Bounds
options.lb = [repmat([zeros(M,1); -1500000*ones(nDof,1)],N,1); zeros(nEMG,1)];
options.ub = [repmat([ones(M,1); 1500000*ones(nDof,1)],N,1); ones(nEMG,1)*MaxScale];
options.cl = [repmat(zeros(nDof,1),N,1); repmat(ones(nEMG,1)*BoundMin,N,1)];
options.cu = [repmat(zeros(nDof,1),N,1); repmat(ones(nEMG,1)*BoundMax,N,1)];

% Auxiliary data.
options.auxdata = { M N nDof reshape(Fact', I, 1)  reshape(Fpas', I, 1) ...
    ID_data MomentArms nEMG EMGindices actEMG};
options.ipopt.mu_strategy      = 'adaptive';
options.ipopt.max_iter         = 2000;
options.ipopt.tol              = 1e-8;
options.ipopt.hessian_approximation = 'limited-memory';

% Callback functions
funcs.objective         = @objective_SO;
funcs.constraints       = @constraints_SO;
funcs.gradient          = @gradient_SO;
funcs.jacobian          = @jacobian_SO;
funcs.jacobianstructure = @jacobianstructure_SO;

[x,~] = ipopt_auxdata(x0,funcs,options);
x_opt = reshape(x(1:end-nEMG), M+nDof, N)';

act = x_opt(:,1:M);
eT = x_opt(:, M+1:M+nDof)*Topt;
EMGscale = x((M+nDof)*N+1:end);

DatStore.SoAct = act;
DatStore.SoRAct = eT;
SoForce = FMo.*(act.*FMltilde.*FMvtilde + Fpe); 
DatStore.SoForce = SoForce;
DatStore.cos_alpha = cos_alpha;
DatStore.EMGscale  = EMGscale;

figure();
for i=1:length(EMGindices)
    subplot(4,ceil(length(EMGindices)./4),i)
    plot(act(:,EMGindices(i))); hold on;
    plot(actEMG(:,i).*repmat(EMGscale(i)',length(actEMG),1),'--k');
    title(DatStore.EMGselection{i});
end
suptitle('Results EMG constrained static optimization');
% ------------------------------------------------------------------
function f = objective_SO (x, auxdata)
[M, N, nDof, Fmax, Fpas, ID_data, MomentArm, nEMG, EMGindices, actEMG, ] = deal(auxdata{:});
  f     = 0.5 * sum(x(1:end-nEMG).^2);
  
% ------------------------------------------------------------------
function c = constraints_SO (x, auxdata)
  [M, N, nDof, Fmax, Fpas, ID_data, MomentArm, nEMG, EMGindices, actEMG] = deal(auxdata{:});
  
  F = Fmax .* x(1:end-nEMG) + Fpas;
  
  c = zeros(nDof*N + nEMG*N,1);
  
  for k = 1:nDof
      F_matrix = reshape(F, M+nDof, N)';
      MomentArm_matrix = reshape(MomentArm(:,k), M+nDof, N)';
      c(k:nDof:nDof*N) = sum(F_matrix.*MomentArm_matrix, 2) - ID_data(:,k); 
  end
  
  act = reshape(x(1:end-nEMG), M+nDof, N)';
  s = x(end-nEMG + 1:end);
  for m = 1:nEMG
     c(nDof*N + (m-1)*N + 1:nDof*N + m*N) = act(:,EMGindices(m)) - s(m) * actEMG(:,m); 
  end
  

% ------------------------------------------------------------------
function g = gradient_SO (x, auxdata)
  [M, N, nDof, Fmax, Fpas, ID_data, MomentArm, nEMG, EMGindices, actEMG] = deal(auxdata{:});
  g = [x(1:end - nEMG); zeros(nEMG,1)];

% ------------------------------------------------------------------
function J = jacobianstructure_SO (auxdata)  
  [M, N, nDof, Fmax, Fpas, ID_data, MomentArm, nEMG, EMGindices, actEMG] = deal(auxdata{:});
  
  nA = M + nDof; % number of actuators
  J = zeros(nDof*N + nEMG*N,(nA)*N +nEMG );
  for i = 1:N
      for k = 1:nDof
          J(k+(i-1)*nDof,nA*(i-1)+1:nA*i) = MomentArm((i-1)*nA+1:i*nA,k)';
      end
  end
  
   for m = 1:nEMG
      for i = 1:N
          J(nDof*N + (m-1)*N + i, (M+nDof)*(i-1) + EMGindices(m)) = 1;
          J(nDof*N + (m-1)*N + i, (M+nDof)*N + m) = - actEMG(i,m);
      end
   end
  
  
  J = sparse(J);
    
% ------------------------------------------------------------------
function J = jacobian_SO (x, auxdata)  
  [M, N, nDof, Fmax, Fpas, ID_data, MomentArm, nEMG, EMGindices, actEMG] = deal(auxdata{:});
  
  nA = M + nDof; % number of actuators
  J = zeros(nDof*N + nEMG*N,(nA)*N +nEMG );
  for i = 1:N
      for k = 1:nDof
          J(k+(i-1)*nDof,nA*(i-1)+1:nA*i) = ...
              Fmax((i-1)*nA+1:i*nA)'.*MomentArm((i-1)*nA+1:i*nA,k)';
      end
  end
  
  for m = 1:nEMG
      for i = 1:N
          J(nDof*N + (m-1)*N + i, (M+nDof)*(i-1) + EMGindices(m)) = 1;
          J(nDof*N + (m-1)*N + i, (M+nDof)*N + m) = - actEMG(i,m);
      end
  end
  
  
  J = sparse(J);     
  
% ------------------------------------------------------------------
function H = hessianstructure_SO (auxdata)
  H = speye(auxdata{1});

% ------------------------------------------------------------------
function H = hessian_SO (x, sigma, lambda, auxdata)
  H = speye(auxdata{1});
  
