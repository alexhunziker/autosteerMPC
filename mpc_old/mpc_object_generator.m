%% create MPC controller object with sample time
mpc1 = mpc(ss_model_discrete, 0.1);
%% specify prediction horizon
mpc1.PredictionHorizon = 6;
%% specify control horizon
mpc1.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = [0;0];
mpc1.Model.Nominal.Y = [0;0;0;0];
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -0.785398163397448;
mpc1.MV(1).Max = 0.785398163397448;
mpc1.MV(2).Min = -1;
mpc1.MV(2).Max = 0.5;

mpc1.OV(4).Min = 0;
mpc1.OV(4).Max = 4;
mpc1.OV(3).Min = -pi;
mpc1.OV(3).Max = pi;
%% specify overall adjustment factor applied to weights
beta = 0.96079;
%% specify weights
mpc1.Weights.MV = [0 0]*beta;
mpc1.Weights.MVRate = [0.2 0.1]/beta;
mpc1.Weights.OV = [0 1 0 0]*beta;
mpc1.Weights.ECR = 100000;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
%% run simulation
%sim(mpc1, 101, mpc1_RefSignal_1, mpc1_MDSignal_1, options);
