%% create MPC controller object with sample time
mpc2 = mpc(ss_model, 0.01);
%% specify prediction horizon
mpc2.PredictionHorizon = 10;
%% specify control horizon
mpc2.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc2.Model.Nominal.U = [0;0];
mpc2.Model.Nominal.Y = [0;0;0;0;0];
%% specify constraints for MV and MV Rate
mpc2.MV(1).Min = -0.785398163397448;
mpc2.MV(1).Max = 0.785398163397448;
mpc2.MV(1).RateMin = -0.1;
mpc2.MV(1).RateMax = 0.1;
%% specify constraint softening for MV and MV Rate
mpc2.MV(1).RateMinECR = 0.01;
mpc2.MV(1).RateMaxECR = 0.01;
%% specify weights
mpc2.Weights.MV = [0 0];
mpc2.Weights.MVRate = [0.1 0.1];
mpc2.Weights.OV = [5 0 1 0 10];
mpc2.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
