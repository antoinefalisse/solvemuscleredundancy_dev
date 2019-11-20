Parameters = [2.5;4.5;6.5;8.5;10.5];
lMT = [1.5;3.5;5.5;7.5;9.5];
vMT = [1.5;3.5;5.5;7.5;9.5];
a = [1.5;3.5;5.5;7.5;9.5];
load ActiveFVParameters;
load PassiveFLParameters;
load Faparam;

[FM, lMtilde, FMactFL, FMactFV, FMpas, cos_alpha] = ...
    HillModel_RigidTendon(a,lMT,vMT,Parameters,ActiveFVParameters,...
    PassiveFLParameters,Faparam);
