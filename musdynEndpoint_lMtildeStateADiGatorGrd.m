% This code was generated using ADiGator version 1.3
% ©2010-2014 Matthew J. Weinstein and Anil V. Rao
% ADiGator may be obtained at https://sourceforge.net/projects/adigator/ 
% Contact: mweinstein@ufl.edu
% Bugs/suggestions may be reported to the sourceforge forums
%                    DISCLAIMER
% ADiGator is a general-purpose software distributed under the GNU General
% Public License version 3.0. While the software is distributed with the
% hope that it will be useful, both the software and generated code are
% provided 'AS IS' with NO WARRANTIES OF ANY KIND and no merchantability
% or fitness for any purpose or application.

function output = musdynEndpoint_lMtildeStateADiGatorGrd(input)
global ADiGator_musdynEndpoint_lMtildeStateADiGatorGrd
if isempty(ADiGator_musdynEndpoint_lMtildeStateADiGatorGrd); ADiGator_LoadData(); end
Gator1Data = ADiGator_musdynEndpoint_lMtildeStateADiGatorGrd.musdynEndpoint_lMtildeStateADiGatorGrd.Gator1Data;
% ADiGator Start Derivative Computations
q.dv = input.phase.integral.dv; q.f = input.phase.integral.f;
%User Line: q = input.phase.integral;
output.objective.dv = q.dv; output.objective.f = q.f;
%User Line: output.objective = q;
NMuscles = input.auxdata.NMuscles;
%User Line: NMuscles = input.auxdata.NMuscles;
%User Line: % Initial and end states
cada1f1 = 1:NMuscles;
a_end.dv = input.phase.finalstate.dv(Gator1Data.Index1);
a_end.f = input.phase.finalstate.f(cada1f1);
%User Line: a_end = input.phase.finalstate(1:NMuscles);
cada1f1 = NMuscles + 1;
cada1f2 = length(input.phase.finalstate.f);
cada1f3 = cada1f1:cada1f2;
lMtilde_end.dv = input.phase.finalstate.dv(Gator1Data.Index2);
lMtilde_end.f = input.phase.finalstate.f(cada1f3);
%User Line: lMtilde_end = input.phase.finalstate(NMuscles+1:end);
cada1f1 = 1:NMuscles;
a_init.dv = input.phase.initialstate.dv(Gator1Data.Index3);
a_init.f = input.phase.initialstate.f(cada1f1);
%User Line: a_init = input.phase.initialstate(1:NMuscles);
cada1f1 = NMuscles + 1;
cada1f2 = length(input.phase.initialstate.f);
cada1f3 = cada1f1:cada1f2;
lMtilde_init.dv = input.phase.initialstate.dv(Gator1Data.Index4);
lMtilde_init.f = input.phase.initialstate.f(cada1f3);
%User Line: lMtilde_init = input.phase.initialstate(NMuscles+1:end);
%User Line: % Constraints - mild periodicity
cada1td1 = zeros(18,1);
cada1td1(Gator1Data.Index5) = a_end.dv;
cada1td1(Gator1Data.Index6) = cada1td1(Gator1Data.Index6) + -a_init.dv;
pera.dv = cada1td1;
pera.f = a_end.f - a_init.f;
%User Line: pera = a_end - a_init;
cada1td1 = zeros(18,1);
cada1td1(Gator1Data.Index7) = lMtilde_end.dv;
cada1td1(Gator1Data.Index8) = cada1td1(Gator1Data.Index8) + -lMtilde_init.dv;
perlMtilde.dv = cada1td1;
perlMtilde.f = lMtilde_end.f - lMtilde_init.f;
%User Line: perlMtilde = lMtilde_end - lMtilde_init;
cada1td1 = zeros(36,1);
cada1td1(Gator1Data.Index9) = pera.dv;
cada1td1(Gator1Data.Index10) = perlMtilde.dv;
output.eventgroup.event.dv = cada1td1;
output.eventgroup.event.f = [pera.f perlMtilde.f];
%User Line: output.eventgroup.event = [pera perlMtilde];
output.eventgroup.event.dv_size = [18,39];
output.eventgroup.event.dv_location = Gator1Data.Index11;
output.objective.dv_size = 39;
output.objective.dv_location = Gator1Data.Index12;
end


function ADiGator_LoadData()
global ADiGator_musdynEndpoint_lMtildeStateADiGatorGrd
ADiGator_musdynEndpoint_lMtildeStateADiGatorGrd = load('musdynEndpoint_lMtildeStateADiGatorGrd.mat');
return
end