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

function outdata = simHOCBF(steps,dt)

%{
Simulates HOCBF for marine DI unicycles from uSimMarine

state: [x, y, v, theta]

control inputs: [thrust, rudder]

%}



%{
!!! WARNING! Mike Benjamin uses the marine coordinate frame for \theta:
    * 0 Degrees is due North
    * 90 Degrees is due East

Simulation is performed in marine coordinate frame; convert to normal
coordinate frame for plotting (marine2normal_rad function)
%}

state = zeros(4,steps);
state(4,1:2) = pi/2;
u = zeros(2,steps);
u(1,1) = 1;
u_nom = zeros(2,1);

obstacle_pos = [5,30,0,0;
                -30,60,0,0]';

n_obs = size(obstacle_pos,2);

% Parameters
amax = 1;
turn_loss = 0.85;
turn_rate = 70;
lse_param = 10;
softabs_param = 0.1;

delta = 1; % alpha(x) = delta*x
R = 10; % Safety radius


% Optimization
options = optimoptions('quadprog','Algorithm','active-set','Display','none');
options_infeas = optimoptions('quadprog','Display','none');
lpoptions = optimoptions('linprog','Display','none');
ws = optimwarmstart(zeros(2,1),options);
A = zeros(n_obs,2);
bvec = zeros(n_obs,1);

% Control input constraints
lb = [0;-100];
ub = [100;100];

A_history = zeros([size(A),steps]);
bvec_history = zeros([size(bvec),steps]);

% Simulation loop
for tt=2:steps-1

    % Nominal u()
    u_nom(1) = max(0,1-state(3,tt)); % Constant speed; no reverse thrust
    u_nom(2) = 5;

    for jj=1:n_obs
        if h(state(1:2,tt), obstacle_pos(1:2,jj), R) > 0
            ;
        end
    end
    % Modify u() via CBF
    P = eye(2);
    q = -2*u_nom;
    for jj=1:n_obs
        A(jj,:) = a_func(state(:,tt), obstacle_pos(:,jj), u(:,tt-1), amax, turn_loss, lse_param, softabs_param,turn_rate);
        bvec(jj) = b_func(state(:,tt), obstacle_pos(:,jj), u(:,tt-1), zeros(2,1), delta, R, amax, turn_loss, lse_param, softabs_param,turn_rate);
    end

    try
        [wsout, fval, exitflag, output, lambda] = quadprog(P,q,A,bvec,[],[],lb,ub,ws);
    catch
        ;
    end

    if exitflag > 0
        u(:,tt) = wsout.X;
        ws = wsout;
    else
        %{ 
        Find closest feasible solution to safe set of controls.
        
        min ||u-z||^2
        s.t.    A*z <= b        % Constrain z to be in safe set of controls
                lb <= u <= ub   % Control constraints

        Variable vector is [z; u]
        %}
        disp("Infeasible; solving LP approximation");
        Ainfeas = [A zeros(size(A,1),2)];
        bvec_infeas = bvec;
        lb_infeas = [-Inf(2,1); lb];
        ub_infeas = [Inf(2,1); ub];

        % Objective is norm(u-z)^2
        P_infeas = [eye(2), -eye(2); -eye(2), eye(2)];

        [x,fval,exitflag] = quadprog(P_infeas, [],Ainfeas,bvec_infeas,[],[],lb_infeas,ub_infeas,[],options_infeas);
        u(:,tt) = x(end-1:end); % Extract u, ignore z
    end

    % Update state
    state(:,tt+1) = state(:,tt) + fMarine(state(:,tt),u(:,tt),amax, turn_loss, lse_param, softabs_param,turn_rate)*dt;
    state(4,tt+1) = mod(state(4,tt+1),2*pi);

    if any(state(:,tt+1) == Inf)
    ;
    end

    % Record A, bvec history
    A_history(:,:,tt) = A;
    bvec_history(:,tt) = bvec;
end


outdata = struct();

outdata.state = state;
outdata.u = u;
outdata.obstacle_pos = obstacle_pos;
outdata.amax = amax;
outdata.turn_loss = turn_loss; 
outdata.lse_param = lse_param;
outdata.softabs_param = softabs_param;
outdata.turn_rate = turn_rate;
outdata.R = R;
outdata.delta = delta;
outdata.A_history = A_history;
outdata.bvec_history = bvec_history;

end






