close all; clear; clc;

poleLength = 0.3; % m
poleMass = .2; % kg
cartMass = 0.5; % kg
g = 9.8; %m/s^2

% Cart Pole Fuzzy Inference System
cpFIS = mamfis('NumInputs', 4, 'NumInputMFs', 2, 'NumOutputs', 1, ...
    'NumOutputMFs', 4, 'AddRule', 'none');

% Inputs
cpFIS.Inputs(1).Name = 'Theta';
cpFIS.Inputs(1).Range = [-pi, pi];
cpFIS.Inputs(1).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(1).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(1).MembershipFunctions(1).Parameters = [-0.5, 0.5];
cpFIS.Inputs(1).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(1).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(1).MembershipFunctions(2).Parameters = [-0.5, 0.5];

cpFIS.Inputs(2).Name = 'Theta_dot';
cpFIS.Inputs(2).Range = [-5, 5];
cpFIS.Inputs(2).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(2).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(2).MembershipFunctions(1).Parameters = [-5, 5];
cpFIS.Inputs(2).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(2).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(2).MembershipFunctions(2).Parameters = [-5, 5];

cpFIS.Inputs(3).Name = 'Cart_position';
cpFIS.Inputs(3).Range = [-2, 2];
cpFIS.Inputs(3).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(3).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(3).MembershipFunctions(1).Parameters = [-1, 1];
cpFIS.Inputs(3).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(3).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(3).MembershipFunctions(2).Parameters = [-1, 1];

cpFIS.Inputs(4).Name = 'Cart_velocity';
cpFIS.Inputs(4).Range = [-5, 5];
cpFIS.Inputs(4).MembershipFunctions(1).Name = 'Negative';
cpFIS.Inputs(4).MembershipFunctions(1).Type = 'zmf';
cpFIS.Inputs(4).MembershipFunctions(1).Parameters = [-5, 5];
cpFIS.Inputs(4).MembershipFunctions(2).Name = 'Positive';
cpFIS.Inputs(4).MembershipFunctions(2).Type = 'smf';
cpFIS.Inputs(4).MembershipFunctions(2).Parameters = [-5, 5];

plotmf(cpFIS, 'input', 1, 1000);


%% Outputs
cpFIS.Outputs(1).Name = 'Force';
cpFIS.Outputs(1).Range = [-20, 20];
cpFIS.Outputs(1).MembershipFunctions(1).Name = 'NM'; % Negative Medium
cpFIS.Outputs(1).MembershipFunctions(1).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(1).Parameters = [5, 2, -12];
cpFIS.Outputs(1).MembershipFunctions(2).Name = 'PM'; % Positive Medium
cpFIS.Outputs(1).MembershipFunctions(2).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(2).Parameters = [5, 2, 12];

cpFIS.Outputs(1).MembershipFunctions(3).Name = 'NL'; % Negative Large
cpFIS.Outputs(1).MembershipFunctions(3).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(3).Parameters = [5, 2, -20];
cpFIS.Outputs(1).MembershipFunctions(4).Name = 'PL'; % Positive Large
cpFIS.Outputs(1).MembershipFunctions(4).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(4).Parameters = [5, 2, 20];

cpFIS.Outputs(1).MembershipFunctions(5).Name = 'NS'; % Negative Small
cpFIS.Outputs(1).MembershipFunctions(5).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(5).Parameters = [2, 2, -2];
cpFIS.Outputs(1).MembershipFunctions(6).Name = 'PS'; % Positive Small
cpFIS.Outputs(1).MembershipFunctions(6).Type = 'gbellmf';
cpFIS.Outputs(1).MembershipFunctions(6).Parameters = [2, 2, 2];

plotmf(cpFIS, 'output', 1, 1000);


% Specify Rules
rules = ["If Theta is Negative then Force is NM", ...
    "If Theta is Positive then Force is PM", ...
    "If Theta_dot is Negative then Force is NL", ...
    "If Theta_dot is Positive then Force is PL", ...
    "If Cart_position is Negative then Force is PS", ...
    "If Cart_position is Positive then Force is NS", ...
    "If Cart_velocity is Negative then Force is NM", ...
    "If Cart_velocity is Positive then Force is PM"];

cpFIS = addRule(cpFIS, rules);