function DatStore = SolveStaticOptimization_IPOPT(DatStore)
import casadi.*

% STEP 1. Inputs
% --------------
opti = casadi.Opti(); % Create opti instance


    
time = DatStore.time;
N = length(time);
M = DatStore.nMuscles;
nDOF = DatStore.nDOF;


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
FMo = ones(size(act,1),1)*DatStore.Fiso;
Fpas = FMo.*Fpe.*cos_alpha;
Fact = FMo.*FMltilde.*FMvtilde.*cos_alpha;


ID_data = DatStore.T_exp;
x = SX.sym('x',N,M + nDOF);

% Add optimal force for reserve torques
Topt = 150/sqrt(1000);
Fpas = [Fpas zeros(N,nDOF)];
Fact = [Fact Topt*ones(N,nDOF)];



I = N*(M+nDOF);
MomentArms = zeros(I,nDOF);
temp = zeros(N,M);
for i = 1:nDOF    
    temp(:,:) = DatStore.dM(:,i,:);
    MomentArms(:,i) = ...
        reshape([temp zeros(N,i-1) ones(N,1) zeros(N,nDOF-i)]',I,1);    
end



F = Fact .* x + Fpas;
F_matrix = reshape(F, M+nDOF, N)';

c = SX(nDOF*N,1);
  
for k = 1:nDOF
  MomentArm_matrix = reshape(MomentArms(:,k), M+nDOF, N)';
  c(k:nDOF:end) = sum(F_matrix.*MomentArm_matrix, 2) - ID_data(:,k); 
end

f_IDEquilibrium = Function('f_IDEquilibrium',{x},{c});




    
    
    
act = opti.variable(M,N);
opti.subject_to(0 < act < 1);           % Bounds
opti.set_initial(act,0.2);                      % Initial guess (naive)

aT = opti.variable(nDOF,N);
% opti.subject_to(-150 < aT < 150);

opti.minimize(1000*sumsqr(aT) + sumsqr(act));






c = f_IDEquilibrium([act; aT]');
opti.subject_to(c == 0);

optionssol.ipopt.nlp_scaling_method = 'gradient-based';
optionssol.ipopt.linear_solver = 'mumps';
optionssol.ipopt.tol = 1e-6;;
optionssol.ipopt.max_iter = 1000;

opti.solver('ipopt',optionssol);

sol = opti.solve();


% STEP 2. Optimization
% --------------------

% Initial guess
x0 = repmat(0.01*ones(M+nDOF,1),N,1);

% Bounds
options.lb = repmat([zeros(M,1); -1500000*ones(nDOF,1)],N,1);
options.ub = repmat([ones(M,1); 1500000*ones(nDOF,1)],N,1);
options.cl = repmat(zeros(nDOF,1),N,1);
options.cu = repmat(zeros(nDOF,1),N,1);

% Auxiliary data.
options.auxdata = { M N nDOF reshape(Fact', I, 1)  reshape(Fpas', I, 1) ...
    ID_data MomentArms};
options.ipopt.mu_strategy      = 'adaptive';
options.ipopt.max_iter         = 100;
options.ipopt.tol              = 1e-8;
options.ipopt.hessian_approximation = 'limited-memory';

% Callback functions
funcs.objective         = @objective_SO;
funcs.constraints       = @constraints_SO;
funcs.gradient          = @gradient_SO;
funcs.jacobian          = @jacobian_SO;
funcs.jacobianstructure = @jacobianstructure_SO;

[x,~] = ipopt_auxdata(x0,funcs,options);
x_opt = reshape(x, M+nDOF, N)';

act = x_opt(:,1:M);
eT = x_opt(:, M+1:M+nDOF)*Topt;

DatStore.SoAct = act;
DatStore.SoRAct = eT;
SoForce = FMo.*(act.*FMltilde.*FMvtilde + Fpe); 
DatStore.SoForce = SoForce;
DatStore.cos_alpha = cos_alpha;

% ------------------------------------------------------------------
function f = objective_SO (x, auxdata)
  f     = 0.5 * sum(x.^2);
  
% ------------------------------------------------------------------
function c = constraints_SO (x, auxdata)
  [M, N, nDOF, Fmax, Fpas, ID_data, MomentArm] = deal(auxdata{:});
  
  F = Fmax .* x + Fpas;
  
  c = zeros(nDOF*N,1);
  
  for k = 1:nDOF
      F_matrix = reshape(F, M+nDOF, N)';
      MomentArm_matrix = reshape(MomentArm(:,k), M+nDOF, N)';
      c(k:nDOF:end) = sum(F_matrix.*MomentArm_matrix, 2) - ID_data(:,k); 
  end

% ------------------------------------------------------------------
function g = gradient_SO (x, auxdata)
  g = x;

% ------------------------------------------------------------------
function J = jacobianstructure_SO (auxdata)  
  [M, N, nDOF, Fmax, Fpas, ID_data, MomentArm] = deal(auxdata{:});
  
  nA = M + nDOF; % number of actuators
  J = zeros(nDOF*N,(nA)*N);
  for i = 1:N
      for k = 1:nDOF
          J(k+(i-1)*nDOF,nA*(i-1)+1:nA*i) = MomentArm((i-1)*nA+1:i*nA,k)';
      end
  end
  
  J = sparse(J);
    
% ------------------------------------------------------------------
function J = jacobian_SO (x, auxdata)  
  [M, N, nDOF, Fmax, Fpas, ID_data, MomentArm] = deal(auxdata{:});
  
  nA = M + nDOF; % number of actuators
  J = zeros(nDOF*N,(nA)*N);
  for i = 1:N
      for k = 1:nDOF
          J(k+(i-1)*nDOF,nA*(i-1)+1:nA*i) = ...
              Fmax((i-1)*nA+1:i*nA)'.*MomentArm((i-1)*nA+1:i*nA,k)';
      end
  end
  
  J = sparse(J);     
  
% ------------------------------------------------------------------
function H = hessianstructure_SO (auxdata)
  H = speye(auxdata{1});

% ------------------------------------------------------------------
function H = hessian_SO (x, sigma, lambda, auxdata)
  H = speye(auxdata{1});
  
