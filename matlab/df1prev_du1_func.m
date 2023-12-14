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

function df1prev_du1 = df1prev_du1_func(in1,in2,amax,turn_loss,lse_param,softabs_param,turn_rate)
%DF1PREV_DU1_FUNC
%    DF1PREV_DU1 = DF1PREV_DU1_FUNC(IN1,IN2,AMAX,TURN_LOSS,LSE_PARAM,SOFTABS_PARAM,TURN_RATE)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    19-Sep-2022 16:50:43

u1_prev1 = in2(1,:);
u1_prev2 = in2(2,:);
t2 = softabs_param.^2;
t3 = u1_prev2.^2;
t5 = (lse_param.*u1_prev1)./1.0e+1;
t4 = t2+t3;
t6 = exp(t5);
t7 = t6+1.0;
df1prev_du1 = reshape([0.0,0.0,(t6.*((turn_loss.*(softabs_param-sqrt(t4)))./1.0e+2+1.0))./t7,(turn_rate.*u1_prev2)./5.0e+3,0.0,0.0,(1.0./sqrt(t4).*turn_loss.*u1_prev2.*log(t7).*(-1.0./1.0e+1))./lse_param,(turn_rate.*u1_prev1)./5.0e+3],[4,2]);
