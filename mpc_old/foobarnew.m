%% create MPC controller object with sample time
cv_mpc = mpc(cv_mpc_plant_C, 0.1);
%% specify prediction horizon
cv_mpc.PredictionHorizon = 2;
%% specify control horizon
cv_mpc.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
cv_mpc.Model.Nominal.U = [0;0;0];
cv_mpc.Model.Nominal.Y = [0;0;0;0;0;0;0];
%% specify constraints for MV and MV Rate
cv_mpc.MV(1).Min = -0.785398163397448;
cv_mpc.MV(1).Max = 0.785398163397448;
cv_mpc.MV(2).Min = 0;
cv_mpc.MV(2).Max = 1;
cv_mpc.MV(3).Min = 0;
cv_mpc.MV(3).Max = 1;
%% specify constraints for OV
cv_mpc.OV(6).Min = 0;
cv_mpc.OV(6).Max = 4;
%% specify weights
cv_mpc.Weights.MV = [0 0.066079 0.066079];
cv_mpc.Weights.MVRate = [0.302668018583816 0.151334009291908 0.151334009291908];
cv_mpc.Weights.OV = [0 0 0 3.30395 0 0.66079 0];
cv_mpc.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
sim(cv_mpc, 101, cv_mpc_RefSignal, cv_mpc_MDSignal, options);
