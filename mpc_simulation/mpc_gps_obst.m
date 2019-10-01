%% create MPC controller object with sample time
mpc_gps_o = mpc(ss_gps_o, 0.01);
%% specify prediction horizon
mpc_gps_o.PredictionHorizon = 5;
%% specify control horizon
mpc_gps_o.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc_gps_o.Model.Nominal.U = [0;0;0];
mpc_gps_o.Model.Nominal.Y = [0;0;0;0;0];
%% specify constraints for MV and MV Rate
mpc_gps_o.MV(1).Min = -0.785398163397448;
mpc_gps_o.MV(1).Max = 0.785398163397448;
mpc_gps_o.MV(1).RateMin = -0.1;
mpc_gps_o.MV(1).RateMax = 0.1;
mpc_gps_o.MV(2).Min = 0;
mpc_gps_o.MV(2).Max = 1.0;
mpc_gps_o.MV(3).Min = 0;
mpc_gps_o.MV(3).Max = 1.0;
%% specify constraints for OV
mpc_cv_o.OV(4).Min = 0;
%% specify constraint softening for MV and MV Rate
mpc_gps_o.MV(1).RateMinECR = 0.01;
mpc_gps_o.MV(1).RateMaxECR = 0.01;
%% specify weights
mpc_gps_o.Weights.MV = [0.1 0.2 0.2];
mpc_gps_o.Weights.MVRate = [0.1 0.1 0.1];
mpc_gps_o.Weights.OV = [2 0 0 1, 10];
mpc_gps_o.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
