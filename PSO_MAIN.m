%_________________________________________________________________________________%
%  Spherical Vector-based Particle Swarm Optimization (SPSO) source codes demo 1.0%
%                                                                                 %
%  Developed in MATLAB 2020b                                              %
%                                                                         %
%  Author and programmer: Manh Duong Phung                                %
%                                                                         %
%         e-Mail: duongpm@gmail.com                                       %
%                 duongpm@vnu.edu.vn                                      %
%                                                                         %
%       Homepage: https://uet.vnu.edu.vn/~duongpm/                        %
%                                                                         %
%   Main paper: Manh Duong Phung, Quang Phuc Ha                           %
%               "Safety-enhanced UAV Path Planning with                   %
%                Spherical Vector-based Particle Swarm Optimization",     %
%               Applied soft computing                                    %
%                                                                         %
%                                                                         %
%_________________________________________________________________________%

%
% Find a path that maximizes the probability of finding object
% 

clc;
clear;
close all;

%% Problem Definition

model = CreateModel(); % Create search map and parameters

CostFunction=@(x) MyCost(x,model);    % Cost Function

nVar=model.n;       % Number of Decision Variables = searching dimension of PSO = number of path nodes

VarSize=[1 nVar];   % Size of Decision Variables Matrix

% Lower and upper Bounds of particles (Variables)
VarMin.x=model.xmin;           
VarMax.x=model.xmax;           
VarMin.y=model.ymin;           
VarMax.y=model.ymax;           
VarMin.z=model.zmin;           
VarMax.z=model.zmax;                 

% Inclination (elevation) 


% Lower and upper Bounds of velocity
alpha=0.5;
VelMax.x=alpha*(VarMax.x-VarMin.x);    
VelMin.x=-VelMax.x;                    
VelMax.y=alpha*(VarMax.y-VarMin.y);    
VelMin.y=-VelMax.y;                    
VelMax.z=alpha*(VarMax.z-VarMin.z);    
VelMin.z=-VelMax.z;                    

%% PSO Parameters

MaxIt=200;          % Maximum Number of Iterations

nPop=500;           % Population Size (Swarm Size)

w=1;                % Inertia Weight
wdamp=0.98;         % Inertia Weight Damping Ratio
c1=1.5;             % Personal Learning Coefficient
c2=1.5;             % Global Learning Coefficient

%% Initialization

% Create Empty Particle Structure
empty_particle.Position=[];
empty_particle.Velocity=[];
empty_particle.Cost=[];
empty_particle.Best.Position=[];
empty_particle.Best.Cost=[];

% Initialize Global Best
GlobalBest.Cost=inf; % Minimization problem

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle=repmat(empty_particle,nPop,1);

% Initialization Loop
isInit = false;
while (~isInit)
        disp("Initialising...");
   for i=1:nPop

        % Initialize Position
        particle(i).Position=CreateRandomSolution_d(VarSize,VarMin,VarMax);

        % Initialize Velocity
        particle(i).Velocity.x=zeros(VarSize);
        particle(i).Velocity.y=zeros(VarSize);
        particle(i).Velocity.z=zeros(VarSize);

        % Evaluation
        particle(i).Cost= CostFunction(particle(i).Position);

        % Update Personal Best
        particle(i).Best.Position=particle(i).Position;
        particle(i).Best.Cost=particle(i).Cost;

        % Update Global Best
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest=particle(i).Best;
            isInit = true;
        end
    end
end

% Array to Hold Best Cost Values at Each Iteration
BestCost=zeros(MaxIt,1);

%% PSO Main Loop
tic
for it=1:MaxIt

    % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;

    for i=1:nPop          
        % x Part
        % Update Velocity
        particle(i).Velocity.x = w*particle(i).Velocity.x ...
            + c1*rand(VarSize).*(particle(i).Best.Position.x-particle(i).Position.x) ...
            + c2*rand(VarSize).*(GlobalBest.Position.x-particle(i).Position.x);

        % Update Velocity Bounds
        particle(i).Velocity.x = max(particle(i).Velocity.x,VelMin.x);
        particle(i).Velocity.x = min(particle(i).Velocity.x,VelMax.x);

        % Update Position
        particle(i).Position.x = particle(i).Position.x + particle(i).Velocity.x;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle(i).Position.x<VarMin.x | particle(i).Position.x>VarMax.x);
        particle(i).Velocity.x(OutOfTheRange)=-particle(i).Velocity.x(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.x = max(particle(i).Position.x,VarMin.x);
        particle(i).Position.x = min(particle(i).Position.x,VarMax.x);


        % y Part

        % Update Velocity
        particle(i).Velocity.y = w*particle(i).Velocity.y ...
            + c1*rand(VarSize).*(particle(i).Best.Position.y-particle(i).Position.y) ...
            + c2*rand(VarSize).*(GlobalBest.Position.y-particle(i).Position.y);

        % Update Velocity Bounds
        particle(i).Velocity.y = max(particle(i).Velocity.y,VelMin.y);
        particle(i).Velocity.y = min(particle(i).Velocity.y,VelMax.y);

        % Update Position
        particle(i).Position.y = particle(i).Position.y + particle(i).Velocity.y;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.y<VarMin.y | particle(i).Position.y>VarMax.y);
        particle(i).Velocity.y(OutOfTheRange)=-particle(i).Velocity.y(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.y = max(particle(i).Position.y,VarMin.y);
        particle(i).Position.y = min(particle(i).Position.y,VarMax.y);

        % z part
        % Update Velocity
        particle(i).Velocity.z = w*particle(i).Velocity.z ...
            + c1*rand(VarSize).*(particle(i).Best.Position.z-particle(i).Position.z) ...
            + c2*rand(VarSize).*(GlobalBest.Position.z-particle(i).Position.z);

        % Update Velocity Bounds
        particle(i).Velocity.z = max(particle(i).Velocity.z,VelMin.z);
        particle(i).Velocity.z = min(particle(i).Velocity.z,VelMax.z);

        % Update Position
        particle(i).Position.z = particle(i).Position.z + particle(i).Velocity.z;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.z<VarMin.z | particle(i).Position.z>VarMax.z);
        particle(i).Velocity.z(OutOfTheRange)=-particle(i).Velocity.z(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.z = max(particle(i).Position.z,VarMin.z);
        particle(i).Position.z = min(particle(i).Position.z,VarMax.z);

        % Evaluation
        particle(i).Cost=CostFunction(particle(i).Position);

        % Update Personal Best
        if particle(i).Cost < particle(i).Best.Cost

            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;

            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest=particle(i).Best;
            end

        end

    end
    if it==85
        toc;
    end
    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);

end

%% Plot results
% Best solution
BestPosition = GlobalBest.Position;
disp("Best solution...");
disp(BestPosition);
smooth = 0.8;
PlotSolution(BestPosition,model,smooth);

% Best cost  
figure;
plot(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
