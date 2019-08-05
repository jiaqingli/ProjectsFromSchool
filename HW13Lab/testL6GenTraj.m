syms l0 l1 l2 l3 l4 a1 a2 a3 a4 a5

g1 = SE3([0;0;l0],SE3.RotZ(a1));
g2 = SE3([0;0;0], SE3.RotX(a2));
g3 = SE3([0;0;l1], SE3.RotX(a3));
g4 = SE3([0;0;l2],SE3.RotX(a4));
g5 = SE3([0;0;l3],SE3.RotZ(a5));
g6 = SE3([0;0;l4],eye(3));

% configuration
g= g1 *g2 * g3 * g4 * g5 * g6;
ee_pos = getTranslation(g);
g = [getRotationMatrix(g), getTranslation(g); 0,0,0,1];

% initial and final configuration
mm2in   = 1/25.4;
linklen = [110.6 120 120 130 20]*mm2in;
alphai = [-pi/6 -pi/3 pi/6 -pi/2 0];
alphaf = [pi/6 0 -pi/12 pi/3 -pi/3];
gi = double(subs(g, [l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alphai(1:end)]));
gf = double(subs(g, [l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alphaf(1:end)]));
posi = double(subs(ee_pos,[l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alphai(1:end)]));
posf = double(subs(ee_pos,[l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alphaf(1:end)]));
tspan = 8;

% constant v = (xdot, ydot, zdot) in world
v_w = (posf - posi)/tspan;
v_w = [v_w;1;1;1];
% assign ptraj
traj.pvec = [alphai,0];
traj.pdot = v_w;
traj.tspan = tspan;
%% start 
l6 = lynx6(load('lynxparmsG2-06.mat'));
l6.gotoSleep();
l6.gotoHome();
l6.setArm(([alphai,0]*diag([180/pi, 180/pi, 180/pi, 180/pi, 180/pi, 1]))')
disp('At initial position, press return to start ...');
pause();

%% joint angles
% get joint angles
jTraj = l6.genPositionTrajectory(traj);
alphas = jTraj.alpha * diag([180/pi, 180/pi, 180/pi, 180/pi, 180/pi, 1]);

jTraj.alpha = alphas;

figure();
hold on

l6.displayJointTrajectory(jTraj, tspan);

hold off


%% follow joint angles
l6.followJointTrajectory(jTraj,[0,8]);

%% plot it
alphaf_generated = jTraj.alpha(end, :);
alphaf_generated = alphaf_generated(1:5)*pi/180;
gf_generated = double(subs(g, [l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alphaf_generated(1:end)]))
gerr = gf_generated\gf
eps_err = logm(gerr);
eps_err = SE3.unhat(eps_err)
eps_err_norm = norm(eps_err(1:5))
posf_gen = double(subs(ee_pos, [l0 l1 l2 l3 l4 a1 a2 a3 a4 a5], [linklen(1:end) alphaf_generated]))
posf

%% end

pause();
l6.gotoSleep();
l6.shutdown();
