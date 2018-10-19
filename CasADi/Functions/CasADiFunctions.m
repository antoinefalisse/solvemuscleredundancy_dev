% This script contains several CasADi-based functions that are
% used when solving the OCPs
import casadi.*

%% Functions for normalized sum of squared values
% 9
etemp9 = SX.sym('etemp9',9);
Jtemp9 = 0;
for i=1:length(etemp9)
    Jtemp9 = Jtemp9 + etemp9(i).^2;
end
f_J9 = Function('f_J9',{etemp9},{Jtemp9});
% 3 
etemp3 = SX.sym('etemp3',3);
Jtemp3 = 0;
for i=1:length(etemp3)
    Jtemp3 = Jtemp3 + etemp3(i).^2;
end
f_J3 = Function('f_J3',{etemp3},{Jtemp3});

%% Function for sum of products 
% 9
ma_temp9 = SX.sym('ma_temp9',9);
ft_temp9 = SX.sym('ft_temp9',9);
J_sptemp9 = 0;
for i=1:length(ma_temp9)
    J_sptemp9 = J_sptemp9 + ma_temp9(i,1)*ft_temp9(i,1);    
end
f_T9 = Function('f_T9',{ma_temp9,ft_temp9},{J_sptemp9});

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
