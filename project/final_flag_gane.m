clear all;close all;clc
addpath('./Polytope-bounded-Voronoi-diagram-master' ); 
addpath('../');

%Run the simulation for a specific number of iterations
dt=0.01;                   % numerical steplength
max_iter = 3000;

%% Set up the Robotarium object
% Get the number of available agents from the Robotarium
global N_protect;
N_protect = 3;
global N;
N = 4;
global leader_ind;
leader_ind = 0;
% randomly generate a flag position
flag_opponent = [-0.73; -0.45]; %-[0.6*rand(1);0.4*rand(1)]-[0.6;0.4];  
global flag_home;
flag_home = [0.6; .05];
global dflag;
dflag=(0.8*sqrt(3))/3; %distance agents should keep from the flag

% Initialize robotarium
%global rb;
%rb = RobotariumBuilder();
global rbtm;
rbtm = Robotarium('NumberOfRobots', N+N_protect, 'ShowFigure', true);
%rbtm = rb.set_number_of_agents(N_protect+N).set_save_data(false).build();
title('Formation Control');
global h1;
h1 = plot(flag_opponent(1),flag_opponent(2),'Marker','*', 'markersize',10);
plot(flag_home(1), flag_home(2), 'Marker', '*', 'markersize',10);
global h3;
h3 = plot(flag_home(1), flag_home(2), 'Marker', '*', 'markersize', 10, 'Color', 'g');
set(h3, 'Visible', 'off');

global si_to_uni_dyn;
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 1, 'AngularVelocityLimit', 2);
% Single-integrator barrier certificates
global si_barrier_cert;
si_barrier_cert = create_si_barrier_certificate('SafetyRadius', .09);
% Single-integrator position controller
global si_pos_controller;
si_pos_controller = create_si_position_controller();

% Initialize robots
xuni = rbtm.get_poses();                                    % States of real unicycle robots
x = xuni(1:2,N_protect+1:N_protect+N);                                            % x-y positions only
rbtm.set_velocities(N_protect+1:N_protect+N, zeros(2,N));                       % Assign dummy zero velocity
rbtm.step();                                                % Run robotarium step
 
%% define variables
bnd_w = 1.6;
bnd_h = 1;
bnd0 = [-bnd_w,-bnd_h;bnd_w,-bnd_h;-bnd_w,bnd_h;bnd_w,bnd_h];
K = convhull(bnd0);
bnd_pnts = bnd0(K,:);   % take boundary points from vertices of convex polytope formed with the boundary point-candidates
detect_threshold =sqrt(1+1.6^2)/2;
global protect_event;
protect_event = 0;      %protect_event = 0 => go to target positions, = 1 => cyclic 
global protect_event2;
protect_event2 = 0;

%% Algorithm
% start the iteration
% first plot initial voronoi diagram
[vornb,vorvx] = polybnd_voronoi(x',bnd_pnts);
plot_vor(vorvx,x,bnd_pnts)
for i = 1:max_iter
    % Get new data and initialize new null velocities
    xuni = rbtm.get_poses();                                % Get new robots' states     
    x = xuni(1:2,N_protect+1:N_protect+N);                  % Extract single integrator states
    u_explore=zeros(2,N);                                   % Initialize velocities to zero
    u = zeros(2,N+N_protect);

    % protect the flag
    u_protect = protection(xuni(1:2,1:N_protect));
    if norm(u_protect,Inf) < 0.04
        protect_event = 1;
    end
    % try to detect the opponent's flag
    leader_ind = find_flag(detect_threshold,x,flag_opponent);
    
    [vornb,vorvx] = polybnd_voronoi(x',bnd_pnts);
   
    for j = 1:N
        % compute centroid of voronoi region    
        vx = vorvx{j}';
        num_vx = size(vx,2)-1;    % the first and last pts of vx are the same
        M_V = 0;
        C_Vx = 0;
        C_Vy = 0;
        for k = 1:num_vx
            M_V = M_V+(vx(1,k)*vx(2,k+1)-vx(2,k)*vx(1,k+1));
            C_Vx = C_Vx+ (vx(1,k)+vx(1,k+1))*(vx(1,k)*vx(2,k+1)-vx(2,k)*vx(1,k+1));
            C_Vy = C_Vy+ (vx(2,k)+vx(2,k+1))*(vx(1,k)*vx(2,k+1)-vx(2,k)*vx(1,k+1));            
        end
        M_V = M_V/2;
        C_Vx = C_Vx/6/M_V;
        C_Vy = C_Vy/6/M_V;
        % compute the velocity
        u_explore(:,j) = -2*M_V*(x(:,j)-[C_Vx;C_Vy]);
    end
    u = [u_protect,u_explore];
    norms = arrayfun(@(x) norm(u(:, x)), 1:N+N_protect);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    u(:, to_thresh) = threshold*u(:, to_thresh)./norms(to_thresh);
   
    dx = si_barrier_cert(u, xuni);
    dxu = si_to_uni_dyn(dx, xuni);             % Convert single integrator inputs into unicycle inputs
    rbtm.set_velocities(1:N_protect+N, dxu); rbtm.step();              % Set new velocities to robots and update
    % stop when velocities are too small
    if (norm(u_explore,Inf) < 0.04) && protect_event
        break;
    end
end
xuni = rbtm.get_poses();                                % Get new robots' states
rbtm.step();  
x = xuni(1:2,N_protect+1:N_protect+N);     
[vornb,vorvx] = polybnd_voronoi(x',bnd_pnts);
%plot_vor(vorvx, x, bnd_pnts);
%% plot Vornoi
plotVoronoi = plot([vorvx{1}(:, 1)], [vorvx{1}(:,2)],...
    'LineWidth',2, 'Color',[0.3,0.3,0.3]);

for k = 1:4
    set(plotVoronoi, 'Xdata', vorvx{k}(:, 1), 'Ydata', vorvx{k}(:, 2));

    pause(.5);
end
set(plotVoronoi, 'Xdata',0, 'Ydata', 0);

    %% fetch and end
fetch(flag_opponent, leader_ind, N);
rbtm.debug();

%%%%%%%%%%%%%%%%%%%%%%%%  Help function   %%%%%%%%%%%%%%%%%%%%%%%%

%% plot
function plot_vor(vorvx,pos, bnd_pnts) 
    %global rbtm;
    pos = pos';
    for i = 1:size(vorvx,2)
        col(i,:)= rand(1,3);
    end

    figure('position',[0 0 600 600],'Color',[1 1 1]);
    hold on;
    title('Voronoi Partition');
    for i = 1:size(pos,1)
     plot(vorvx{i}(:,1),vorvx{i}(:,2),'-r', 'LineWidth',3);
     hold on;
    end
    plot(bnd_pnts(:,1),bnd_pnts(:,2),'-');
    hold on;
    plot(pos(:,1),pos(:,2),'Marker','o','MarkerFaceColor',[0 .75 .75],'MarkerEdgeColor','k','LineStyle','none');
    %axis('equal')
    axis([-2 2 -2 2]);
%     set(gca,'xtick',[0 1]);
%     set(gca,'ytick',[0 1]);   
    %pause(.5);
    %delete(c1);
end

%% detection
function leader_index = find_flag(detect_threshold,pos,flag_opponent)
diff = pos - flag_opponent.*ones(1,size(pos,2));
diff_norm = sqrt(diff(1,:).^2+diff(2,:).^2);
if find(diff_norm<=detect_threshold)
    [min_dis,leader_index] = min(diff_norm);
else
    leader_index = 0;
end

end

%% fetch function
function fetch(flag_pos_adv, leader_indx, N)
% global variables
global rbtm;
global si_to_uni_dyn;
global si_barrier_cert;
global si_pos_controller;
global N_protect;
global flag_home;
global protect_event;
protect_event = 0;
global leader_ind;
global h1;
global h3;
% Initialization
Lnew = -completeGL(N);
%Initialize velocity vector
dxi = zeros(2, N);
%State for leader
state = 0;
%edges
E_fetch = [1,2; 2,4; 4,1; 3,4; 2,3; 3,1];
E_protect = [1,2; 2,3; 3,1];
E_protect_f = [];
for ii = 1:N+N_protect
   jj = mod(ii +1, N+N_protect+1); 
   jj_next = mod(jj + 1, N + N_protect+1);
   if jj == 0
      jj = 1; 
   end
   if jj_next == 0
      jj_next = 1; 
   end
   if ii ~= (leader_indx+N_protect) && jj ~= (leader_indx+N_protect)
      E_protect_f = [E_protect_f; [ii, jj]]; 
   end
   if jj == leader_indx + N_protect
       E_protect_f = [E_protect_f; [ii, jj_next]];
   end 
end
E_protect_f([6, 1:5], :) = E_protect_f([1:6], :);
%E_protect_f = [1,2;2,3;3,4;4,5;5,6;6,7;7,1];
x = rbtm.get_poses();
x_fetch = x(:, N_protect+1:N_protect+N);
x_protect = x(:, 1:N_protect);
edgFetchPlot = plot([x_fetch(1,E_fetch(:,1)),x_fetch(1,E_fetch(:,2))],...
    [x_fetch(2,E_fetch(:,1)),x_fetch(2,E_fetch(:,2))],... 
    'LineWidth',2, 'Color',[0.3,0.3,0.3]);
edgProtectPlot = plot([x_protect(1,E_protect(:,1)),x_protect(1,E_protect(:,2))] , ...
    [x_protect(2,E_protect(:,1)),x_protect(2,E_protect(:,2))],... 
    'LineWidth',2, 'Color',[0.3,0.3,0.3]);

rbtm.step();
% These are gains for our formation control algorithm
formation_control_gain = 10.0;
leader_gain = 1;
leader_gain_home = .1;
desired_distance = .4;
dd = sqrt(3) * desired_distance;

% weight
W = zeros(N,N);
W = [0, dd, dd, desired_distance;...
    dd, 0, dd, desired_distance;...
    dd, dd, 0, desired_distance;...
    desired_distance, desired_distance, desired_distance, 0];
if(N ~= leader_indx)
    W([N,leader_ind], :) = W([leader_ind, N], :);
    W(:, [N, leader_ind]) = W(:, [leader_ind, N]);
    Lnew([N,leader_ind], :) = Lnew([leader_ind, N], :);
    Lnew(:, [N, leader_ind]) = Lnew(:, [leader_ind, N]);
    
end

final_protection = false;
final_protection_first_time = false;
cnt_final_protection = 0;
% flags, cnt = formation time
flag_pos_opponent = flag_pos_adv;
flag_pos_team = flag_home;
cnt = 0;
edgProtectFPlot = plot(0, ...
            0,...
            'LineWidth',2, 'Color',[0,1,0]);
for t = 1:4000
    % poses
    x = rbtm.get_poses();
    
    x_fetch = x(:, N_protect+1:N_protect+N);
    x_protect = x(:, 1:N_protect);
    % plot edges
    if final_protection_first_time
        edgProtectFPlot = plot([x(1,E_protect_f(:,1)),x(1,E_protect_f(end,2))] , ...
            [x(2,E_protect_f(:,1)),x(2,E_protect_f(end,2))],... 
            'LineWidth',2, 'Color',[1,0,0]);
        
    elseif final_protection && ~final_protection_first_time
        set(edgFetchPlot,'Xdata',[0,0],...
        'Ydata',[0,0])
        set(edgProtectPlot,'Xdata',[0,0],...
        'Ydata',[0,0])
        set(edgProtectFPlot, 'Xdata',[x(1,E_protect_f(:,1)),x(1,E_protect_f(end,2))],...
        'Ydata',[x(2,E_protect_f(:,1)),x(2,E_protect_f(end,2))])
    else
        set(edgFetchPlot,'Xdata',[x_fetch(1,E_fetch(:,1)),x_fetch(1,E_fetch(:,2))],...
        'Ydata',[x_fetch(2,E_fetch(:,1)),x_fetch(2,E_fetch(:,2))])
        set(edgProtectPlot,'Xdata',[x_protect(1,E_protect(:,1)),x_protect(1,E_protect(:,2))],...
        'Ydata',[x_protect(2,E_protect(:,1)),x_protect(2,E_protect(:,2))])
    end
    % protection inputs
    u_protect = zeros(2, N_protect);
    if norm(x_protect(1:2, :) - flag_home) < 0.7
        protect_event = 1;
    else
        protect_event = 0;
    end
    
    % fetch Fllowers Dynamics Update
    for i = 1:N
                
            dxi(:, i) = [0 ; 0];
            neighbors = topological_neighbors(Lnew, i);
            if i ~= leader_indx
                for j = neighbors
                    dxi(:, i) = dxi(:, i) + ...
                    formation_control_gain*(norm(x_fetch(1:2, j) - x_fetch(1:2, i))^2 -  W(i,j)^2)*...
                    (x_fetch(1:2, j)-x_fetch(1:2, i));
                end
            end
        
    end
    
    % Make the leader travel between waypoints
    switch state
        % go to adv flag
        case 0
            d_norm = norm(x_fetch(1:2, leader_indx)-flag_pos_adv);
            dxi(:, leader_indx) =  leader_gain*(flag_pos_adv-x_fetch(1:2, leader_indx))/(d_norm);
            if(norm(x_fetch(1:2, leader_indx) - flag_pos_opponent) <= 0.1)
                state = 2;
                set(h1, 'Visible', 'off');
            end
            % protect the flag
            u_protect = protection(x(1:2,1:N_protect));
            dx = [u_protect, dxi];
            
        % go to home    
        case 1
            d_norm = norm(x_fetch(1:2, leader_indx)-flag_pos_team);
            dxi(:, leader_indx) =  leader_gain * (flag_pos_team-x_fetch(1:2, leader_indx))/(d_norm);
            u_protect = protection(x(1:2,1:N_protect));
            dx = [u_protect, dxi];

            % protect agents perform decoy
            if (norm(x_fetch(1:2, leader_indx) - flag_pos_team) <= .3)
                if cnt_final_protection < 1
                    final_protection_first_time = true;
                else
                    final_protection_first_time = false;
                end
                %final_protection_first_time = false;
                final_protection = true;
                cnt_final_protection = cnt_final_protection +1;
                
                u_pro = protection_final(x(1:2,:),leader_ind);          
                dx = [u_pro(:,1:N_protect+leader_ind-1),dxi(:,leader_indx),...
                    u_pro(:,N_protect+leader_ind:end)];
                set(h3, 'Visible', 'on');
            end
            
        % formation and wait for leader
        case 2
            dxi(:, leader_indx) = [0;0];
           
            cnt = cnt + 1;
            if(cnt >= 500)
                state = 1;
            end
            % protect the flag
            u_protect = protection(x(1:2,1:N_protect));
            dx = [u_protect, dxi];
            
           
    end
    
    % Use barrier certificate and convert to unicycle dynamics
    
    norms = arrayfun(@(x) norm(dx(:, x)), 1:N+N_protect);
    threshold = rbtm.max_linear_velocity/2;
    to_thresh = norms > threshold;
    dx(:, to_thresh) = threshold*dx(:, to_thresh)./norms(to_thresh);
    
    dx = si_barrier_cert(dx, x(:,1:N_protect+N));
    dxu = si_to_uni_dyn(dx, x(:, 1:N_protect+N));
    
    %Set velocities
    rbtm.set_velocities(1:N_protect+N, dxu);
    
    %Iterate experiment
    rbtm.step();
end

end

%% protect flag function
function u_protect = protection(pos)
global N_protect;
global protect_event;
global flag_home;
pflag = flag_home;

u_protect = zeros(2,N_protect);
kp=0.1;
center=pflag;
radius = 0.25;%(norm(center-pos(:,1))+norm(center-pos(:,2))+norm(center-pos(:,3)))/3;
interAgentDistance = radius*2*sin(pi/N_protect);
w1 = 3;
w2 = 0.4;
angles = (0:N_protect-1)*2*pi/N_protect;
if protect_event == 0    
    % go to the target
    circularTargets = radius*[ cos(angles) ;sin(angles)]+center;
    errorToInitialPos = pos - circularTargets;                % Error
    errorNorm = sqrt([1,1]*(errorToInitialPos.^2));               % Norm of error
    if max( errorNorm ) > 0.05
        u_protect = -0.3.*errorToInitialPos;
    else
        protect_event = 1;
        u_protect = zeros(2,N_protect);        
    end
elseif protect_event == 1
    % cyclic pursuit
    A = diag(ones(N_protect-1,1),1) + diag(ones(N_protect-1,1),-1) ;
    A(1,N_protect) = 1; A(N_protect,1) = 1;
    L = diag(sum(A)) - A;
    for i = 1:N_protect
        for j=1:N_protect
            alpha = pi/N_protect + w1*(interAgentDistance-norm(pos(:,j)-pos(:,i)));
            R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
            u_protect(:,i) = u_protect(:,i) + kp*A(i,j)*(R*(pos(:,j)-pos(:,i))-w2*(pos(:,i)-center));
        end
    end
    
end

end

%% disperse defense
function u_disperse = disperse(pos, leader_goal)
    global si_pos_controller;
    u_disperse = zeros(2, 3);
    global flag_home;
    L_protect = [0 0 0; -1 2 -1; -1 -1 2];
    goal_distance = .01;
    %leader_goal = [-.8; 0.7];
    for i = 2:3
        neighbors = topological_neighbors(L_protect, i);
        for j = neighbors
                u_disperse(:, i) = u_disperse(:, i) + ...
                    10*(norm(pos(1:2, j) - pos(1:2, i))^2 -  goal_distance^2)*(pos(1:2, j) - pos(1:2, i));
        end
    end
    u_disperse(:, 1) = u_disperse(:,1) + .1*(leader_goal - pos(1:2, 1));
end


%% final protection
function u_pro = protection_final(x,leader_ind)
% global variables
global N_protect;
global flag_home;
global N;
global protect_event2;

% initialize local variables
N_total = N_protect+N;
N_pro2 = N_total-1;
A = diag(ones(N_pro2-1,1),-1);
A(1,N_pro2) = 1; 
%A(N_pro2,1) = 1;
L = diag(sum(A)) - A;
center = flag_home;
radius = 0.4;
interAgentDistance = radius*2*sin(pi/(N_pro2));
angles = (0:N_pro2-1)*2*pi/N_pro2;

x_pro = [x(:,1:N_protect+leader_ind-1),x(:,N_protect+leader_ind+1:end)];
u_pro = zeros(2,N_pro2);

kp=0.1;
w1 = 3;
w2 = 0.4;

if protect_event2 == 0
    circularTargets = radius*[ cos(angles) ;sin(angles)]+center;
    errorToInitialPos = x_pro - circularTargets;                % Error
    errorNorm = sqrt([1,1]*(errorToInitialPos.^2));               % Norm of error
    if max( errorNorm ) > 0.05
        u_pro = -0.3.*errorToInitialPos;        
    else
        protect_event2 = 1;
        u_pro = zeros(2,N_pro2);        
    end
   
elseif protect_event2 == 1
    u_pro = zeros(2,N_pro2);
    for i = 1:N_pro2                
        for j = find(A(:,i))'
            if ~isempty(j)
                alpha = pi/N_pro2 + w1*(interAgentDistance - norm(x_pro(:,j)-x_pro(:,i)) );
                R = [cos(alpha), sin(alpha); -sin(alpha) cos(alpha)];
                u_pro(:,i) = u_pro(:,i) + R*( x_pro(:,j)-x_pro(:,i) ) - w2*( x_pro(:,i) - center );
            end
        end
    end
    
end
end

%% before Fetch
