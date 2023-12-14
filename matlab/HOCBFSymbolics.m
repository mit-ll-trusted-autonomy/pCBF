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


% Generate symbolic expression for Double Integrator Unicycle HOCBF

x1 = sym('x1');
y1 = sym('y1');
v1 = sym('v1');
theta1 = sym('theta1');

x2 = sym('x2');
y2 = sym('y2');
v2 = sym('v2');
theta2 = sym('theta2');

% z1 = [x1; y1; v1; theta1];
% z2 = [x2; y2; v2; theta2];

z1 = sym('z1', [4,1]);
z2 = sym('z2', [4,1]);

u1 = sym('u1', [2,1]);
u2 = sym('u2', [2,1]);

R = sym('R');
delta = sym('delta');

psi0 = h(z1, z2, R);

dh_dz1 = jacobian(psi0, z1);
dh_dz2 = jacobian(psi0, z2);

psi1 = dh_dz1*f(z1) + dh_dz2*f(z2) + alpha(psi0, delta);

dpsi1_dz1 = jacobian(psi1, z1);
dpsi1_dz2 = jacobian(psi1, z2);

psi2 = dpsi1_dz1*(f(z1) + g(z1)*u1) + dpsi1_dz2*(f(z2) + g(z2)*u2) + alpha(psi1, delta);


avec = dpsi1_dz1*g(z1);
bval = -(alpha(psi1, delta) + dpsi1_dz1*f(z1) + dpsi1_dz2*(f(z2) + g(z2)*u2));

avec_latex = latex(avec);
bval_latex = latex(bval);


% Generate MATLAB Function

a_func = matlabFunction(avec, vars={z1, z2, delta}, file='a_func.m');

b_func = matlabFunction(bval, vars={z1, z2, delta, u2, R}, file='b_func.m');


% Generate C++ Function



% Functions
function outvector = f(z)
    outvector = [sin(z(4))*z(3); cos(z(4))*z(3); 0; 0];
end

function outvector = g(z)
    outvector = [0, 0; 0,0; 1, -1; 0, z(3)];
end

function outscalar = h(z1, z2, R)
    outscalar = R^2 - transpose(z1 - z2)*(z1-z2);
end

function outscalar = alpha(z,delta)
    outscalar = delta*z;
end



