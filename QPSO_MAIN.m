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

%% QPSO Parameters

MaxIt=200;          % Maximum Number of Iterations

nPop=500;           % Population Size (Swarm Size)

w1=0.5;                % Inertia Weight
w2=1.0;         % Inertia Weight Damping Ratio
c1=2.05;             % Personal Learning Coefficient
c2=2.05;             % Global Learning Coefficient

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
        %Mbest.x=zeros(1,nVar);
        %Mbest.y=zeros(1,nVar);
        %Mbest.z=zeros(1,nVar);
        
   for i=1:nPop

        % Initialize Position
        particle(i).Position=CreateRandomSolution_d(VarSize,VarMin,VarMax);
        %Mbest.x=Mbest.x+particle(i).Position.x/nPop;
        %Mbest.y=Mbest.y+particle(i).Position.y/nPop;
        %Mbest.z=Mbest.z+particle(i).Position.z/nPop;
        
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
tic;
for it=1:MaxIt

    % Update Best Cost Ever Found
    BestCost(it)=GlobalBest.Cost;
    beta=(w2-w1)*(MaxIt-it)/MaxIt+w1;
    
    mbest_x=zeros(VarSize);
    mbest_y=zeros(VarSize);
    mbest_z=zeros(VarSize);
    for i=1:nPop
        mbest_x=mbest_x+(particle(i).Best.Position.x)/nPop;
        mbest_y=mbest_y+(particle(i).Best.Position.y)/nPop;
        mbest_z=mbest_z+(particle(i).Best.Position.z)/nPop;
    end
    %mbest=sum(partical.Best)/nPop;

    for i=1:nPop          

        % position
        % x part
        fi=rand(1,nVar);
        p=fi.*particle(i).Best.Position.x+(1-fi).*GlobalBest.Position.x;
        u=rand(1,nVar);
        b=beta*abs(mbest_x-particle(i).Position.x);
        v=-log(u);
        
        particle(i).Position.x=p+((-1).^ceil(0.5+rand(1,nVar))).*b.*v;

        % Update Position Bounds
        particle(i).Position.x = max(particle(i).Position.x,VarMin.x);
        particle(i).Position.x = min(particle(i).Position.x,VarMax.x);
        
        
        
        % y part
        fi=rand(1,nVar);
        p=fi.*particle(i).Best.Position.y+(1-fi).*GlobalBest.Position.y;
        u=rand(1,nVar);
        b=beta*abs(mbest_y-particle(i).Position.y);
        v=-log(u);
        
        particle(i).Position.y=p+((-1).^ceil(0.5+rand(1,nVar))).*b.*v;

        % Update Position Bounds
        particle(i).Position.y = max(particle(i).Position.y,VarMin.y);
        particle(i).Position.y = min(particle(i).Position.y,VarMax.y);        
        
        
        
        % z part
        fi=rand(1,nVar);
        p=fi.*particle(i).Best.Position.z+(1-fi).*GlobalBest.Position.z;
        u=rand(1,nVar);
        b=beta*abs(mbest_z-particle(i).Position.z);
        v=-log(u);
        
        particle(i).Position.z=p+((-1).^ceil(0.5+rand(1,nVar))).*b.*v;

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
    if it==98
        toc;
    end

    % Inertia Weight Damping
    %w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);

end
%%

%% Plot results
% Best solution
BestPosition = GlobalBest.Position;
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
