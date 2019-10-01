%% create MPC controller object with sample time
mpc1 = mpc(ss_model, 0.01);
%% specify prediction horizon
mpc1.PredictionHorizon = 5;
%% specify control horizon
mpc1.ControlHorizon = 2;
%% specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = [0;0;0;0];
mpc1.Model.Nominal.Y = [0;0;0;0;0;0];
%% specify constraints for MV and MV Rate
mpc1.MV(1).Min = -0.785398163397448;
mpc1.MV(1).Max = 0.785398163397448;
mpc1.MV(1).RateMin = -0.1;
mpc1.MV(1).RateMax = 0.1;
mpc1.MV(3).Min = -1.0;
mpc1.MV(3).Max = 1.0;
mpc1.MV(4).Min = -1.0;
mpc1.MV(4).Max = 1.0;
%% specify overall adjustment factor applied to weights
beta = 1.0408;
%% specify weights
mpc1.Weights.MV = [0 0]*beta;
mpc1.Weights.MVRate = [0.1 0]/beta;
mpc1.Weights.OV = [2 0 1 0 0]*beta;
mpc1.Weights.ECR = 100000;
%% specify overall adjustment factor applied to estimation model gains
alpha = 0.91201;
%% adjust default output disturbance model gains
setoutdist(mpc1, 'model', getoutdist(mpc1)*alpha);
%% adjust default measurement noise model gains
mpc1.Model.Noise = mpc1.Model.Noise/alpha;
%% specify simulation options
options = mpcsimopt();
options.RefLookAhead = 'off';
options.MDLookAhead = 'off';
options.Constraints = 'on';
options.OpenLoop = 'off';
