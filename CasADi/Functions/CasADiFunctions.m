% This script contains several CasADi-based functions that are
% used when solving the OCPs
import casadi.*

%% Functions for normalized sum of squared values
% # muscles
e_ss = SX.sym('etemp_NMuscles',auxdata.NMuscles);
J_ss = 0;
for i=1:length(e_ss)
    J_ss = J_ss + e_ss(i).^2;
end
f_ssNMuscles = Function('f_ssNMuscles',{e_ss},{J_ss});
% # dofs
e_ss = SX.sym('e_Ndof',auxdata.Ndof);
J_ss = 0;
for i=1:length(e_ss)
    J_ss = J_ss + e_ss(i).^2;
end
f_ssNdof = Function('f_ssNdof',{e_ss},{J_ss});

%% Function for sum of products 
% # muscles
ma_sp = SX.sym('ma_NMuscles',auxdata.NMuscles);
ft_sp = SX.sym('ft_NMuscles',auxdata.NMuscles);
J_sp = 0;
for i=1:length(ma_sp)
    J_sp = J_sp + ma_sp(i,1)*ft_sp(i,1);    
end
f_spNMuscles = Function('f_spNMuscles',{ma_sp,ft_sp},{J_sp});

%% Muscle contraction dynamics
% States 
FTtilde = SX.sym('FTtilde',auxdata.NMuscles);    % Normalized tendon force
a       = SX.sym('a',auxdata.NMuscles); % Muscle activations
% Controls 
dFTtilde = SX.sym('dFTtilde',auxdata.NMuscles);  % Derivative tendon force
% Results
Hilldiff    = SX(auxdata.NMuscles,1);
FT          = SX(auxdata.NMuscles,1);
lMT = SX.sym('lMT',auxdata.NMuscles); % Muscle-tendon length
vMT = SX.sym('vMT',auxdata.NMuscles); % Muscle-tendon velocity
for m = 1:auxdata.NMuscles 
    [Hilldiff(m),FT(m)] = ForceEquilibrium_FtildeState_CasADi(a(m),FTtilde(m),dFTtilde(m),lMT(m),vMT(m),auxdata.params(:,m),auxdata.Fvparam,auxdata.Fpparam,auxdata.Faparam);        
end
f_forceEquilibrium_FtildeState = Function('f_forceEquilibrium_FtildeState',{a,FTtilde,dFTtilde,lMT,vMT,},{Hilldiff,FT});
% Test function
% atest = rand(auxdata.NMuscles,1);
% FTtildetest = rand(auxdata.NMuscles,1);
% dFTtildetest = rand(auxdata.NMuscles,1);
% lMTtest = rand(auxdata.NMuscles,1);
% vMTtest = rand(auxdata.NMuscles,1);
% for m = 1:auxdata.NMuscles
%     [Hilldifftest(m,1),FTtest(m,1)] = ForceEquilibrium_FtildeState_CasADi(...
%         atest(m),FTtildetest(m),dFTtildetest(m),lMTtest(m),...
%         vMTtest(m),auxdata.params(:,m),auxdata.Fvparam,auxdata.Fpparam,auxdata.Faparam);
% end
% [Hilldifftest2,FTtest2] = f_forceEquilibrium_FtildeState(atest,FTtildetest,dFTtildetest,lMTtest,vMTtest);
% assertHill = max(abs(Hilldifftest-Hilldifftest2));
% assertF = max(abs(FTtest-FTtest2));
