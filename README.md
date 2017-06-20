## Flappy Bird Controller
Automatic control of Flappy Bird

The game was developed by Mingjing Zhang and posted on Mathworks file exchange. We modified the control of the game and interfaced it with three controllers as described below.

1. For the original code, see Mingjing Zhang’s code here: https://www.mathworks.com/matlabcentral/fileexchange/45795-roteaugen-flappybird-for-matlab

2. To understand the automatic controller we created, please see paper, “HOW TO BEAT FLAPPY BIRD:  A MIXED-INTEGER MODEL PREDICTIVE CONTROL APPROACH” by Matthew Piper, Pranav A. Bhounsule, and Krystel Castillo, ASME Dynamics Systems and Controls Conference 2017

### USAGE

(1) flappybird_heuristics.m - manual controller (Method 1 in the above paper)

(2) flappybird2.m - optimization based controller (Method 2 in the above paper)
Requires intlinprog in the optimization toolbox

(3) flappBirdMPC2 - Model predictive control (Method 3 in the above paper)
Requires Gurobi optimization
