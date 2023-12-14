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
u1_prev = sym('u1_prev', [2,1]);
u2 = sym('u2', [2,1]);

R = sym('R');
delta = sym('delta');
syms amax turn_loss lse_param softabs_param turn_rate

C = [eye(2) zeros(2)];

f1 = fMarine(z1,u1,amax,turn_loss,lse_param,softabs_param,turn_rate);
f2 = fMarine(z2,u2,amax,turn_loss,lse_param,softabs_param,turn_rate);

f1_func = matlabFunction(f1, vars={z1, u1, amax, turn_loss, lse_param, softabs_param, turn_rate});

df1_dz1 = jacobian(f1,z1); % Approximation of f(z1)
df1_du1 = jacobian(f1,u1); % Approximation of g(z1)
df2_dz2 = jacobian(f2,z2); % Approximation of f(z2)
df2_du2 = jacobian(f2,u2); % Approximation of g(z2)

df1_du1_func = matlabFunction(df1_du1, vars={z1,u1,amax,turn_loss,lse_param, softabs_param, turn_rate}, file='df1_du1_func.m');

% Linearize about the previous command input u
% x_{k+1} = f(x_k,u_{k-1}) - df_du@{x_k,u_{k-1}}*u_{k-1} + df_du@{x_k,u_{k-1}}*u_k
f1_prev = f1_func(z1, u1_prev, amax, turn_loss, lse_param, softabs_param, turn_rate);
df1prev_du1 = df1_du1_func(z1, u1_prev, amax, turn_loss, lse_param, softabs_param, turn_rate);



% Do we use this? Or do we simply use f(x2,u2)?
% f2_prev = f2_func(z2, u2_prev, amax, turn_loss, lse_param, softabs_param);
% df2prev_du2 = df2_du1_func(z2, u2_prev, amax, turn_loss, lse_param, softabs_param);

psi0 = h(C*z1, C*z2, R);

dh_dz1 = jacobian(psi0, z1);
dh_dz2 = jacobian(psi0, z2);

% dh_dz1_func = matlabFunction(dh_dz1_func, vars={})

psi1 = dh_dz1*(f1_prev - df1prev_du1*u1_prev) + dh_dz2*(f2) + alpha(psi0, delta);

dpsi1_dz1 = jacobian(psi1, z1);
dpsi1_dz2 = jacobian(psi1, z2);

psi2 = dpsi1_dz1*(f1_prev - df1prev_du1*u1_prev + df1prev_du1*u1) + dpsi1_dz2*(f2) + alpha(psi1, delta);


avec = dpsi1_dz1*df1prev_du1;
bval = -(alpha(psi1, delta) + dpsi1_dz1*(f1_prev - df1prev_du1*u1_prev) + dpsi1_dz2*(f2));

avec_latex = latex(avec);
bval_latex = latex(bval);


% Generate MATLAB Function

a_func = matlabFunction(avec, vars={z1, z2, u1_prev, amax, turn_loss, lse_param, softabs_param, turn_rate}, file='a_func.m');
b_func = matlabFunction(bval, vars={z1, z2, u1_prev, u2, delta, R, amax, turn_loss, lse_param, softabs_param, turn_rate}, file='b_func.m');

f1_prev_func = matlabFunction(f1_prev, vars={z1,u1_prev,amax,turn_loss,lse_param,softabs_param,turn_rate}, file='f1_prev_func.m');
df1prev_du1_func = matlabFunction(df1prev_du1, vars={z1, u1_prev, amax, turn_loss, lse_param, softabs_param,turn_rate}, file='df1prev_du1_func.m');

dh_dz1_func = matlabFunction(dh_dz1, vars={z1,z2}, file='dh_dz1_func.m');
dh_dz2_func = matlabFunction(dh_dz2, vars={z1,z2}, file='dh_dz2_func.m');

psi1_func = matlabFunction(psi1, vars={z1,z2,delta,R}, file='psi1_func.m');

dpsi1_dz1_func = matlabFunction(dpsi1_dz1, vars={z1,z2,delta}, file='dpsi1_dz1_func.m');
dpsi1_dz2_func = matlabFunction(dpsi1_dz2, vars={z1,z2,delta}, file='dpsi1_dz2_func.m');

% psi2_func = 




% Functions
function outvector = f(z)
    outvector = [sin(z(4))*z(3); cos(z(4))*z(3); 0; 0];
end

function outvector = g(z)
    outvector = [0, 0; 0,0; 1, -1; 0, z(3)];
end







