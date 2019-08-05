 %% Formation control utilizing edge tension energy with a static, undirected
%communication topology
%Paul Glotfelter 
%3/24/2016

%% Set up Robotarium object

N = 4;
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true);

%% Set up constants for experiment

%Gains for the transformation from single-integrator to unicycle dynamics


% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 2000;

% Communication topology for the desired formation.  We need 2 * N - 3 = 9
% edges to ensure that the formation is rigid.
Lnew = -completeGL(N);

formation_control_gain = .5;
leader_gain = .1;
desired_distance = .4;
dd = sqrt(3) * desired_distance;

% weight
W = zeros(N,N);
W = [0, dd, dd, desired_distance;...
    dd, 0, dd, desired_distance;...
    dd, dd, 0, desired_distance;...
    desired_distance, desired_distance, desired_distance, 0];
    
% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

%% Grab tools for converting to single-integrator dynamics and ensuring safety 

si_barrier_cert = create_si_barrier_certificate('SafetyRadius', 1.5*r.robot_diameter);
si_to_uni_dyn = create_si_to_uni_mapping2('LinearVelocityGain', 0.5, ... 
    'AngularVelocityLimit', 0.75*r.max_angular_velocity);

% Iterate for the previously specified number of iterations
for t = 0:iterations
    
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    %% Algorithm
    
    %This section contains the actual algorithm for formation control!
    
    %Calculate single integrator control inputs using edge-energy consensus
    for i = 1:N
        dx(:, i) = [0 ; 0];
        if(true)        
            
            neighbors = topological_neighbors(Lnew, i);
            for j = neighbors
                
                    dx(:, i) = dx(:, i) + ...
                    formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  W(i,j)^2)*...
                    (x(1:2, j)-x(1:2, i));
                
                
            end
        end
    end
    dx(1:2, 4) = [0;0];
    % Transform the single-integrator dynamics to unicycle dynamics using a provided utility function
    dx = si_barrier_cert(dx, x);
    dx = si_to_uni_dyn(dx, x);  
    
    % Set velocities of agents 1:N
    r.set_velocities(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();   
end
r.get_poses()
% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
