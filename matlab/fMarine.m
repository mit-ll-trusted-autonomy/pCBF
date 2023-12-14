%
% DISTRIBUTION STATEMENT A. Approved for public release. Distribution is unlimited.
%
% This material is based upon work supported by the Under Secretary of Defense for Research and Engineering under Air Force Contract No. FA8702-15-D-0001.
% Any opinions, findings, conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of
% the Under Secretary of Defense for Research and Engineering.
%
% © 2023 Massachusetts Institute of Technology.
%
% The software/firmware is provided to you on an As-Is basis
%
% Delivered to the U.S. Government with Unlimited Rights, as defined in DFARS Part 252.227-7013 or 7014 (Feb 2014).
% Notwithstanding any copyright notice, U.S. Government rights in this work are defined by DFARS 252.227-7013 or DFARS 252.227-7014 as detailed above.
% Use of this work other than as specifically authorized by the U.S. Government may violate any copyrights that exist in this work.
%

function outvector = fMarine(state,u,amax, turn_loss,lse_param, softabs_param, turn_rate)
%{
Approximate Nonlinear dynamics for uSimMarine.

!!! ATTENTION! This particular file/commit uses normal conventions for
\theta. Specifically,
    * 0 degrees is along the X axis ("due East")
    * 90 degrees is along the Y axis
    * Right-hand rule

Mike Benjamin uses the marine coordinate frame for \theta:
    * 0 Degrees is due North
    * 90 Degrees is due East

Ranges of control inputs:

    * Thrust: [0,100]
    * Rudder: [-100,100]

Makes the following assumptions/approximations:

    * Assumes default thrust map
    * Smoothly approximates max() with logsumexp() and abs() with the following SE
    method:
        https://math.stackexchange.com/a/1286162/452944
    * 
%}


%     x = state(1);
%     y = state(2);
    v = state(3);
    theta = state(4);

    thrust = 10*logsumexp([0,u(1)]/10,lse_param); % Softmax; No negative acceleration.
    rudder = u(2); % Range: [-100,100]

%     outvector = zeros(4,1);

    outvector(1,1) = v*cos(theta);
    outvector(2,1) = v*sin(theta);
%     outvector(3,1) = softminimum(amax, thrust*(1-turn_loss/100*softabs(rudder,softabs_param)));
    outvector(3,1) = thrust*(1-turn_loss/100*softabs(rudder,softabs_param));

%     outvector(4) = rudder*sign(rudder)*(1+(abs(thrust)+50)/50);
    outvector(4,1) = turn_rate/5000*u(1)*u(2);

end