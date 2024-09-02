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
AngleRange = pi/2; % Limit the angle range for better solutions

VarMax.theta1=AngleRange;           
VarMin.theta1=-AngleRange;

VarMin.theta2=-AngleRange;            
VarMax.theta2=AngleRange;          


% Azimuth 
% Determine the angle of vector connecting the start and end points
VarMin.theta3=-AngleRange;           
VarMax.theta3=AngleRange;           


% Lower and upper Bounds of velocity
alpha=0.5;
VelMax.theta1=alpha*(VarMax.theta1-VarMin.theta1);    
VelMin.theta1=-VelMax.theta1;                    
VelMax.theta2=alpha*(VarMax.theta2-VarMin.theta2);    
VelMin.theta2=-VelMax.theta2;                    
VelMax.theta3=alpha*(VarMax.theta3-VarMin.theta3);    
VelMin.theta3=-VelMax.theta3;                    

%% PSO Parameters

MaxIt=200;          % Maximum Number of Iterations

nPop=500;           % Population Size (Swarm Size)

w=1;                % Inertia Weight
wdamp=0.98;         % Inertia Weight Damping Ratio
c1=2;             % Personal Learning Coefficient
c2=2;             % Global Learning Coefficient

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
        particle(i).Position=CreateRandomSolution_t(VarSize,VarMin,VarMax);

        % Initialize Velocity
        particle(i).Velocity.theta1=zeros(VarSize);
        particle(i).Velocity.theta2=zeros(VarSize);
        particle(i).Velocity.theta3=zeros(VarSize);

        % Evaluation
        particle(i).Cost= CostFunction(thetatoxyz(particle(i).Position,model));

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
tic;
for it=1:MaxIt

    % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;

    for i=1:nPop          
        % theta1 Part
        % Update Velocity
        particle(i).Velocity.theta1 = w*particle(i).Velocity.theta1 ...
            + c1*rand(VarSize).*(particle(i).Best.Position.theta1-particle(i).Position.theta1) ...
            + c2*rand(VarSize).*(GlobalBest.Position.theta1-particle(i).Position.theta1);

        % Update Velocity Bounds
        particle(i).Velocity.theta1 = max(particle(i).Velocity.theta1,VelMin.theta1);
        particle(i).Velocity.theta1 = min(particle(i).Velocity.theta1,VelMax.theta1);

        % Update Position
        particle(i).Position.theta1 = particle(i).Position.theta1 + particle(i).Velocity.theta1;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle(i).Position.theta1<VarMin.theta1 | particle(i).Position.theta1>VarMax.theta1);
        particle(i).Velocity.theta1(OutOfTheRange)=-particle(i).Velocity.theta1(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.theta1 = max(particle(i).Position.theta1,VarMin.theta1);
        particle(i).Position.theta1 = min(particle(i).Position.theta1,VarMax.theta1);


        % theta2 Part

        % Update Velocity
        particle(i).Velocity.theta2 = w*particle(i).Velocity.theta2 ...
            + c1*rand(VarSize).*(particle(i).Best.Position.theta2-particle(i).Position.theta2) ...
            + c2*rand(VarSize).*(GlobalBest.Position.theta2-particle(i).Position.theta2);

        % Update Velocity Bounds
        particle(i).Velocity.theta2 = max(particle(i).Velocity.theta2,VelMin.theta2);
        particle(i).Velocity.theta2 = min(particle(i).Velocity.theta2,VelMax.theta2);

        % Update Position
        particle(i).Position.theta2 = particle(i).Position.theta2 + particle(i).Velocity.theta2;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.theta2<VarMin.theta2 | particle(i).Position.theta2>VarMax.theta2);
        particle(i).Velocity.theta2(OutOfTheRange)=-particle(i).Velocity.theta2(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.theta2 = max(particle(i).Position.theta2,VarMin.theta2);
        particle(i).Position.theta2 = min(particle(i).Position.theta2,VarMax.theta2);

        % theta3 part
        % Update Velocity
        particle(i).Velocity.theta3 = w*particle(i).Velocity.theta3 ...
            + c1*rand(VarSize).*(particle(i).Best.Position.theta3-particle(i).Position.theta3) ...
            + c2*rand(VarSize).*(GlobalBest.Position.theta3-particle(i).Position.theta3);

        % Update Velocity Bounds
        particle(i).Velocity.theta3 = max(particle(i).Velocity.theta3,VelMin.theta3);
        particle(i).Velocity.theta3 = min(particle(i).Velocity.theta3,VelMax.theta3);

        % Update Position
        particle(i).Position.theta3 = particle(i).Position.theta3 + particle(i).Velocity.theta3;

        % Velocity Mirroring
        OutOfTheRange=(particle(i).Position.theta3<VarMin.theta3 | particle(i).Position.theta3>VarMax.theta3);
        particle(i).Velocity.theta3(OutOfTheRange)=-particle(i).Velocity.theta3(OutOfTheRange);

        % Update Position Bounds
        particle(i).Position.theta3 = max(particle(i).Position.theta3,VarMin.theta3);
        particle(i).Position.theta3 = min(particle(i).Position.theta3,VarMax.theta3);

        % Evaluation
        particle(i).Cost=CostFunction(thetatoxyz(particle(i).Position,model));

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
    if it==76
        toc;
    end
    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);

end

%% Plot results
% Best solution
BestPosition = thetatoxyz(GlobalBest.Position,model);
disp("Best solution...");
BestPosition
smooth = 0.95;
PlotSolution(BestPosition,model,smooth);

% Best cost  
figure;
plot(BestCost,'LineWidth',2);
xlabel('Iteration');
ylabel('Best Cost');
grid on;
