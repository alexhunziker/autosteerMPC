%% create MPC controller object with sample time
mpc_gps_s = mpc(ss_gps_s, 0.01);
%% specify prediction horizon
mpc_gps_s.PredictionHorizon = 5;
%% specify control horizon
mpc_gps_s.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc_gps_s.Model.Nominal.U = [0;0;0];
mpc_gps_s.Model.Nominal.Y = [0;0;0;0];
%% specify constraints for MV and MV Rate
mpc_gps_s.MV(1).Min = -0.785398163397448;
mpc_gps_s.MV(1).Max = 0.785398163397448;
mpc_gps_s.MV(1).RateMin = -0.1;
mpc_gps_s.MV(1).RateMax = 0.1;
mpc_gps_s.MV(2).Min = 0;
mpc_gps_s.MV(2).Max = 1.0;
mpc_gps_s.MV(3).Min = 0;
mpc_gps_s.MV(3).Max = 1.0;
%% specify constraint softening for MV and MV Rate
mpc_gps_s.MV(1).RateMinECR = 0.01;
mpc_gps_s.MV(1).RateMaxECR = 0.01;
%% specify weights
mpc_gps_s.Weights.MV = [0.1 0.1 0.1];
mpc_gps_s.Weights.MVRate = [0.1 0.1 0.1];
mpc_gps_s.Weights.OV = [2 0 0 1];
mpc_gps_s.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
