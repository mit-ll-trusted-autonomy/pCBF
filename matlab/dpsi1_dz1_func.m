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

function dpsi1_dz1 = dpsi1_dz1_func(in1,in2,delta)
%DPSI1_DZ1_FUNC
%    DPSI1_DZ1 = DPSI1_DZ1_FUNC(IN1,IN2,DELTA)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    19-Sep-2022 16:50:44

z11 = in1(1,:);
z12 = in1(2,:);
z13 = in1(3,:);
z14 = in1(4,:);
z21 = in2(1,:);
z22 = in2(2,:);
z23 = in2(3,:);
z24 = in2(4,:);
t2 = cos(z14);
t3 = sin(z14);
t4 = z11.*2.0;
t5 = z12.*2.0;
t6 = z21.*2.0;
t7 = z22.*2.0;
t8 = -t6;
t9 = -t7;
t10 = t4+t8;
t11 = t5+t9;
dpsi1_dz1 = [-delta.*t10-t2.*z13.*2.0+z23.*cos(z24).*2.0,-delta.*t11-t3.*z13.*2.0+z23.*sin(z24).*2.0,-t2.*t10-t3.*t11,-t2.*t11.*z13+t3.*t10.*z13];