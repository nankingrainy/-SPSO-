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
VarMax.r=2*norm(model.start-model.end)/nVar;           
VarMin.r=0;

% Inclination (elevation)
AngleRange = pi/4; % Limit the angle range for better solutions
VarMin.psi=-AngleRange;            
VarMax.psi=AngleRange;          


% Azimuth 
% Determine the angle of vector connecting the start and end points
dirVector = model.end - model.start;
phi0 = atan2(dirVector(2),dirVector(1));
VarMin.phi=phi0 - AngleRange;           
VarMax.phi=phi0 + AngleRange;           


% Lower and upper Bounds of velocity
alpha=0.5;
VelMax.r=alpha*(VarMax.r-VarMin.r);    
VelMin.r=-VelMax.r;                    
VelMax.psi=alpha*(VarMax.psi-VarMin.psi);    
VelMin.psi=-VelMax.psi;                    
VelMax.phi=alpha*(VarMax.phi-VarMin.phi);    
VelMin.phi=-VelMax.phi;           

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
GlobalBest_pso.Cost=inf; % Minimization problem

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle_pso=repmat(empty_particle,nPop,1);

% Initialization Loop
isInit = false;
while (~isInit)
        disp("Initialising...");
   for i=1:nPop

        % Initialize Position
        particle_pso(i).Position=CreateRandomSolution_d(VarSize,VarMin,VarMax);

        % Initialize Velocity
        particle_pso(i).Velocity.x=zeros(VarSize);
        particle_pso(i).Velocity.y=zeros(VarSize);
        particle_pso(i).Velocity.z=zeros(VarSize);

        % Evaluation
        particle_pso(i).Cost= CostFunction(particle_pso(i).Position);

        % Update Personal Best
        particle_pso(i).Best.Position=particle_pso(i).Position;
        particle_pso(i).Best.Cost=particle_pso(i).Cost;

        % Update Global Best
        if particle_pso(i).Best.Cost < GlobalBest_pso.Cost
            GlobalBest_pso=particle_pso(i).Best;
            isInit = true;
        end
    end
end

% Array to Hold Best Cost Values at Each Iteration
BestCost_pso=zeros(MaxIt,1);

%% PSO Main Loop

for it_pso=1:MaxIt

    % Update Best Cost Ever Found
    BestCost_pso(it_pso)=GlobalBest_pso.Cost;

    for i=1:nPop          
        % x Part
        % Update Velocity
        particle_pso(i).Velocity.x = w*particle_pso(i).Velocity.x ...
            + c1*rand(VarSize).*(particle_pso(i).Best.Position.x-particle_pso(i).Position.x) ...
            + c2*rand(VarSize).*(GlobalBest_pso.Position.x-particle_pso(i).Position.x);

        % Update Velocity Bounds
        particle_pso(i).Velocity.x = max(particle_pso(i).Velocity.x,VelMin.x);
        particle_pso(i).Velocity.x = min(particle_pso(i).Velocity.x,VelMax.x);

        % Update Position
        particle_pso(i).Position.x = particle_pso(i).Position.x + particle_pso(i).Velocity.x;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle_pso(i).Position.x<VarMin.x | particle_pso(i).Position.x>VarMax.x);
        particle_pso(i).Velocity.x(OutOfTheRange)=-particle_pso(i).Velocity.x(OutOfTheRange);

        % Update Position Bounds
        particle_pso(i).Position.x = max(particle_pso(i).Position.x,VarMin.x);
        particle_pso(i).Position.x = min(particle_pso(i).Position.x,VarMax.x);


        % y Part

        % Update Velocity
        particle_pso(i).Velocity.y = w*particle_pso(i).Velocity.y ...
            + c1*rand(VarSize).*(particle_pso(i).Best.Position.y-particle_pso(i).Position.y) ...
            + c2*rand(VarSize).*(GlobalBest_pso.Position.y-particle_pso(i).Position.y);

        % Update Velocity Bounds
        particle_pso(i).Velocity.y = max(particle_pso(i).Velocity.y,VelMin.y);
        particle_pso(i).Velocity.y = min(particle_pso(i).Velocity.y,VelMax.y);

        % Update Position
        particle_pso(i).Position.y = particle_pso(i).Position.y + particle_pso(i).Velocity.y;

        % Velocity Mirroring
        OutOfTheRange=(particle_pso(i).Position.y<VarMin.y | particle_pso(i).Position.y>VarMax.y);
        particle_pso(i).Velocity.y(OutOfTheRange)=-particle_pso(i).Velocity.y(OutOfTheRange);

        % Update Position Bounds
        particle_pso(i).Position.y = max(particle_pso(i).Position.y,VarMin.y);
        particle_pso(i).Position.y = min(particle_pso(i).Position.y,VarMax.y);

        % z part
        % Update Velocity
        particle_pso(i).Velocity.z = w*particle_pso(i).Velocity.z ...
            + c1*rand(VarSize).*(particle_pso(i).Best.Position.z-particle_pso(i).Position.z) ...
            + c2*rand(VarSize).*(GlobalBest_pso.Position.z-particle_pso(i).Position.z);

        % Update Velocity Bounds
        particle_pso(i).Velocity.z = max(particle_pso(i).Velocity.z,VelMin.z);
        particle_pso(i).Velocity.z = min(particle_pso(i).Velocity.z,VelMax.z);

        % Update Position
        particle_pso(i).Position.z = particle_pso(i).Position.z + particle_pso(i).Velocity.z;

        % Velocity Mirroring
        OutOfTheRange=(particle_pso(i).Position.z<VarMin.z | particle_pso(i).Position.z>VarMax.z);
        particle_pso(i).Velocity.z(OutOfTheRange)=-particle_pso(i).Velocity.z(OutOfTheRange);

        % Update Position Bounds
        particle_pso(i).Position.z = max(particle_pso(i).Position.z,VarMin.z);
        particle_pso(i).Position.z = min(particle_pso(i).Position.z,VarMax.z);

        % Evaluation
        particle_pso(i).Cost=CostFunction(particle_pso(i).Position);

        % Update Personal Best
        if particle_pso(i).Cost < particle_pso(i).Best.Cost

            particle_pso(i).Best.Position=particle_pso(i).Position;
            particle_pso(i).Best.Cost=particle_pso(i).Cost;

            % Update Global Best
            if particle_pso(i).Best.Cost < GlobalBest_pso.Cost
                GlobalBest_pso=particle_pso(i).Best;
            end

        end

    end

    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it_pso) ': Best Cost = ' num2str(BestCost_pso(it_pso))]);

end

%%

%%

% Initialize Global Best
GlobalBest_spso.Cost=inf; % Minimization problem

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle_spso=repmat(empty_particle,nPop,1);

% Initialization Loop
isInit = false;
while (~isInit)
        disp("Initialising...");
   for i=1:nPop

        % Initialize Position
        particle_spso(i).Position=CreateRandomSolution(VarSize,VarMin,VarMax);

        % Initialize Velocity
        particle_spso(i).Velocity.r=zeros(VarSize);
        particle_spso(i).Velocity.psi=zeros(VarSize);
        particle_spso(i).Velocity.phi=zeros(VarSize);

        % Evaluation
        particle_spso(i).Cost= CostFunction(SphericalToCart(particle_spso(i).Position,model));

        % Update Personal Best
        particle_spso(i).Best.Position=particle_spso(i).Position;
        particle_spso(i).Best.Cost=particle_spso(i).Cost;

        % Update Global Best
        if particle_spso(i).Best.Cost < GlobalBest_spso.Cost
            GlobalBest_spso=particle_spso(i).Best;
            isInit = true;
        end
    end
end

% Array to Hold Best Cost Values at Each Iteration
BestCost_spso=zeros(MaxIt,1);

%% SPSO Main Loop

for it_spso=1:MaxIt

    % Update Best Cost Ever Found
    BestCost_spso(it_spso)=GlobalBest_spso.Cost;

    for i=1:nPop          
        % r Part
        % Update Velocity
        particle_spso(i).Velocity.r = w*particle_spso(i).Velocity.r ...
            + c1*rand(VarSize).*(particle_spso(i).Best.Position.r-particle_spso(i).Position.r) ...
            + c2*rand(VarSize).*(GlobalBest_spso.Position.r-particle_spso(i).Position.r);

        % Update Velocity Bounds
        particle_spso(i).Velocity.r = max(particle_spso(i).Velocity.r,VelMin.r);
        particle_spso(i).Velocity.r = min(particle_spso(i).Velocity.r,VelMax.r);

        % Update Position
        particle_spso(i).Position.r = particle_spso(i).Position.r + particle_spso(i).Velocity.r;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle_spso(i).Position.r<VarMin.r | particle_spso(i).Position.r>VarMax.r);
        particle_spso(i).Velocity.r(OutOfTheRange)=-particle_spso(i).Velocity.r(OutOfTheRange);

        % Update Position Bounds
        particle_spso(i).Position.r = max(particle_spso(i).Position.r,VarMin.r);
        particle_spso(i).Position.r = min(particle_spso(i).Position.r,VarMax.r);


        % psi Part

        % Update Velocity
        particle_spso(i).Velocity.psi = w*particle_spso(i).Velocity.psi ...
            + c1*rand(VarSize).*(particle_spso(i).Best.Position.psi-particle_spso(i).Position.psi) ...
            + c2*rand(VarSize).*(GlobalBest_spso.Position.psi-particle_spso(i).Position.psi);

        % Update Velocity Bounds
        particle_spso(i).Velocity.psi = max(particle_spso(i).Velocity.psi,VelMin.psi);
        particle_spso(i).Velocity.psi = min(particle_spso(i).Velocity.psi,VelMax.psi);

        % Update Position
        particle_spso(i).Position.psi = particle_spso(i).Position.psi + particle_spso(i).Velocity.psi;

        % Velocity Mirroring
        OutOfTheRange=(particle_spso(i).Position.psi<VarMin.psi | particle_spso(i).Position.psi>VarMax.psi);
        particle_spso(i).Velocity.psi(OutOfTheRange)=-particle_spso(i).Velocity.psi(OutOfTheRange);

        % Update Position Bounds
        particle_spso(i).Position.psi = max(particle_spso(i).Position.psi,VarMin.psi);
        particle_spso(i).Position.psi = min(particle_spso(i).Position.psi,VarMax.psi);

        % Phi part
        % Update Velocity
        particle_spso(i).Velocity.phi = w*particle_spso(i).Velocity.phi ...
            + c1*rand(VarSize).*(particle_spso(i).Best.Position.phi-particle_spso(i).Position.phi) ...
            + c2*rand(VarSize).*(GlobalBest_spso.Position.phi-particle_spso(i).Position.phi);

        % Update Velocity Bounds
        particle_spso(i).Velocity.phi = max(particle_spso(i).Velocity.phi,VelMin.phi);
        particle_spso(i).Velocity.phi = min(particle_spso(i).Velocity.phi,VelMax.phi);

        % Update Position
        particle_spso(i).Position.phi = particle_spso(i).Position.phi + particle_spso(i).Velocity.phi;

        % Velocity Mirroring
        OutOfTheRange=(particle_spso(i).Position.phi<VarMin.phi | particle_spso(i).Position.phi>VarMax.phi);
        particle_spso(i).Velocity.phi(OutOfTheRange)=-particle_spso(i).Velocity.phi(OutOfTheRange);

        % Update Position Bounds
        particle_spso(i).Position.phi = max(particle_spso(i).Position.phi,VarMin.phi);
        particle_spso(i).Position.phi = min(particle_spso(i).Position.phi,VarMax.phi);

        % Evaluation
        particle_spso(i).Cost=CostFunction(SphericalToCart(particle_spso(i).Position,model));

        % Update Personal Best
        if particle_spso(i).Cost < particle_spso(i).Best.Cost

            particle_spso(i).Best.Position=particle_spso(i).Position;
            particle_spso(i).Best.Cost=particle_spso(i).Cost;

            % Update Global Best
            if particle_spso(i).Best.Cost < GlobalBest_spso.Cost
                GlobalBest_spso=particle_spso(i).Best;
            end

        end

    end

    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it_spso) ': Best Cost = ' num2str(BestCost_spso(it_spso))]);

end
%%

%%

w1=0.5;                % Inertia Weight
w2=1.0;         % Inertia Weight Damping Ratio
c1=2.05;             % Personal Learning Coefficient
c2=2.05;             % Global Learning Coefficient

% Initialize Global Best
GlobalBest_qpso.Cost=inf; % Minimization problem

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle_qpso=repmat(empty_particle,nPop,1);

% Initialization Loop
isInit = false;
while (~isInit)
        disp("Initialising...");
        %Mbest.x=zeros(1,nVar);
        %Mbest.y=zeros(1,nVar);
        %Mbest.z=zeros(1,nVar);
        
   for i=1:nPop

        % Initialize Position
        particle_qpso(i).Position=CreateRandomSolution_d(VarSize,VarMin,VarMax);
        %Mbest.x=Mbest.x+particle(i).Position.x/nPop;
        %Mbest.y=Mbest.y+particle(i).Position.y/nPop;
        %Mbest.z=Mbest.z+particle(i).Position.z/nPop;
        
        % Evaluation
        particle_qpso(i).Cost= CostFunction(particle_qpso(i).Position);

        % Update Personal Best
        particle_qpso(i).Best.Position=particle_qpso(i).Position;
        particle_qpso(i).Best.Cost=particle_qpso(i).Cost;

        % Update Global Best
        if particle_qpso(i).Best.Cost < GlobalBest_qpso.Cost
            GlobalBest_qpso=particle_qpso(i).Best;
            isInit = true;
        end
    end
end

% Array to Hold Best Cost Values at Each Iteration
BestCost_qpso=zeros(MaxIt,1);

%% QPSO Main Loop

for it_qpso=1:MaxIt

    % Update Best Cost Ever Found
    BestCost_qpso(it_qpso)=GlobalBest_qpso.Cost;
    beta=(w2-w1)*(MaxIt-it_qpso)/MaxIt+w1;
    
    mbest_x=zeros(VarSize);
    mbest_y=zeros(VarSize);
    mbest_z=zeros(VarSize);
    for i=1:nPop
        mbest_x=mbest_x+(particle_qpso(i).Best.Position.x)/nPop;
        mbest_y=mbest_y+(particle_qpso(i).Best.Position.y)/nPop;
        mbest_z=mbest_z+(particle_qpso(i).Best.Position.z)/nPop;
    end
    %mbest=sum(partical.Best)/nPop;

    for i=1:nPop          

        % position
        % x part
        fi=rand(1,nVar);
        p=fi.*particle_qpso(i).Best.Position.x+(1-fi).*GlobalBest_qpso.Position.x;
        u=rand(1,nVar);
        b=beta*abs(mbest_x-particle_qpso(i).Position.x);
        v=-log(u);
        
        particle_qpso(i).Position.x=p+((-1).^ceil(0.5+rand(1,nVar))).*b.*v;

        % Update Position Bounds
        particle_qpso(i).Position.x = max(particle_qpso(i).Position.x,VarMin.x);
        particle_qpso(i).Position.x = min(particle_qpso(i).Position.x,VarMax.x);
        
        
        
        % y part
        fi=rand(1,nVar);
        p=fi.*particle_qpso(i).Best.Position.y+(1-fi).*GlobalBest_qpso.Position.y;
        u=rand(1,nVar);
        b=beta*abs(mbest_y-particle_qpso(i).Position.y);
        v=-log(u);
        
        particle_qpso(i).Position.y=p+((-1).^ceil(0.5+rand(1,nVar))).*b.*v;

        % Update Position Bounds
        particle_qpso(i).Position.y = max(particle_qpso(i).Position.y,VarMin.y);
        particle_qpso(i).Position.y = min(particle_qpso(i).Position.y,VarMax.y);        
        
        
        
        % z part
        fi=rand(1,nVar);
        p=fi.*particle_qpso(i).Best.Position.z+(1-fi).*GlobalBest_qpso.Position.z;
        u=rand(1,nVar);
        b=beta*abs(mbest_z-particle_qpso(i).Position.z);
        v=-log(u);
        
        particle_qpso(i).Position.z=p+((-1).^ceil(0.5+rand(1,nVar))).*b.*v;

        % Update Position Bounds
        particle_qpso(i).Position.z = max(particle_qpso(i).Position.z,VarMin.z);
        particle_qpso(i).Position.z = min(particle_qpso(i).Position.z,VarMax.z);
      

        % Evaluation
        particle_qpso(i).Cost=CostFunction(particle_qpso(i).Position);

        % Update Personal Best
        if particle_qpso(i).Cost < particle_qpso(i).Best.Cost

            particle_qpso(i).Best.Position=particle_qpso(i).Position;
            particle_qpso(i).Best.Cost=particle_qpso(i).Cost;

            % Update Global Best
            if particle_qpso(i).Best.Cost < GlobalBest_qpso.Cost
                GlobalBest_qpso=particle_qpso(i).Best;
            end

        end

    end

    % Inertia Weight Damping
    %w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it_qpso) ': Best Cost = ' num2str(BestCost_qpso(it_qpso))]);

end
%%

%%
w=1;                % Inertia Weight
wdamp=0.98;         % Inertia Weight Damping Ratio
c1=2;             % Personal Learning Coefficient
c2=2;             % Global Learning Coefficient

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


% Initialize Global Best
GlobalBest_tpso.Cost=inf; % Minimization problem

% Create an empty Particles Matrix, each particle is a solution (searching path)
particle_tpso=repmat(empty_particle,nPop,1);

% Initialization Loop
isInit = false;
while (~isInit)
        disp("Initialising...");
   for i=1:nPop

        % Initialize Position
        particle_tpso(i).Position=CreateRandomSolution_t(VarSize,VarMin,VarMax);

        % Initialize Velocity
        particle_tpso(i).Velocity.theta1=zeros(VarSize);
        particle_tpso(i).Velocity.theta2=zeros(VarSize);
        particle_tpso(i).Velocity.theta3=zeros(VarSize);

        % Evaluation
        particle_tpso(i).Cost= CostFunction(thetatoxyz(particle_tpso(i).Position,model));

        % Update Personal Best
        particle_tpso(i).Best.Position=particle_tpso(i).Position;
        particle_tpso(i).Best.Cost=particle_tpso(i).Cost;

        % Update Global Best
        if particle_tpso(i).Best.Cost < GlobalBest_tpso.Cost
            GlobalBest_tpso=particle_tpso(i).Best;
            isInit = true;
        end
    end
end

% Array to Hold Best Cost Values at Each Iteration
BestCost_tpso=zeros(MaxIt,1);

%% theta-PSO Main Loop

for it_tpso=1:MaxIt

    % Update Best Cost Ever Found
    BestCost_tpso(it_tpso)=GlobalBest_tpso.Cost;

    for i=1:nPop          
        % theta1 Part
        % Update Velocity
        particle_tpso(i).Velocity.theta1 = w*particle_tpso(i).Velocity.theta1 ...
            + c1*rand(VarSize).*(particle_tpso(i).Best.Position.theta1-particle_tpso(i).Position.theta1) ...
            + c2*rand(VarSize).*(GlobalBest_tpso.Position.theta1-particle_tpso(i).Position.theta1);

        % Update Velocity Bounds
        particle_tpso(i).Velocity.theta1 = max(particle_tpso(i).Velocity.theta1,VelMin.theta1);
        particle_tpso(i).Velocity.theta1 = min(particle_tpso(i).Velocity.theta1,VelMax.theta1);

        % Update Position
        particle_tpso(i).Position.theta1 = particle_tpso(i).Position.theta1 + particle_tpso(i).Velocity.theta1;

        % Velocity Mirroring
        % If a particle moves out of the range, it will moves backward next
        % time
        OutOfTheRange=(particle_tpso(i).Position.theta1<VarMin.theta1 | particle_tpso(i).Position.theta1>VarMax.theta1);
        particle_tpso(i).Velocity.theta1(OutOfTheRange)=-particle_tpso(i).Velocity.theta1(OutOfTheRange);

        % Update Position Bounds
        particle_tpso(i).Position.theta1 = max(particle_tpso(i).Position.theta1,VarMin.theta1);
        particle_tpso(i).Position.theta1 = min(particle_tpso(i).Position.theta1,VarMax.theta1);


        % theta2 Part

        % Update Velocity
        particle_tpso(i).Velocity.theta2 = w*particle_tpso(i).Velocity.theta2 ...
            + c1*rand(VarSize).*(particle_tpso(i).Best.Position.theta2-particle_tpso(i).Position.theta2) ...
            + c2*rand(VarSize).*(GlobalBest_tpso.Position.theta2-particle_tpso(i).Position.theta2);

        % Update Velocity Bounds
        particle_tpso(i).Velocity.theta2 = max(particle_tpso(i).Velocity.theta2,VelMin.theta2);
        particle_tpso(i).Velocity.theta2 = min(particle_tpso(i).Velocity.theta2,VelMax.theta2);

        % Update Position
        particle_tpso(i).Position.theta2 = particle_tpso(i).Position.theta2 + particle_tpso(i).Velocity.theta2;

        % Velocity Mirroring
        OutOfTheRange=(particle_tpso(i).Position.theta2<VarMin.theta2 | particle_tpso(i).Position.theta2>VarMax.theta2);
        particle_tpso(i).Velocity.theta2(OutOfTheRange)=-particle_tpso(i).Velocity.theta2(OutOfTheRange);

        % Update Position Bounds
        particle_tpso(i).Position.theta2 = max(particle_tpso(i).Position.theta2,VarMin.theta2);
        particle_tpso(i).Position.theta2 = min(particle_tpso(i).Position.theta2,VarMax.theta2);

        % theta3 part
        % Update Velocity
        particle_tpso(i).Velocity.theta3 = w*particle_tpso(i).Velocity.theta3 ...
            + c1*rand(VarSize).*(particle_tpso(i).Best.Position.theta3-particle_tpso(i).Position.theta3) ...
            + c2*rand(VarSize).*(GlobalBest_tpso.Position.theta3-particle_tpso(i).Position.theta3);

        % Update Velocity Bounds
        particle_tpso(i).Velocity.theta3 = max(particle_tpso(i).Velocity.theta3,VelMin.theta3);
        particle_tpso(i).Velocity.theta3 = min(particle_tpso(i).Velocity.theta3,VelMax.theta3);

        % Update Position
        particle_tpso(i).Position.theta3 = particle_tpso(i).Position.theta3 + particle_tpso(i).Velocity.theta3;

        % Velocity Mirroring
        OutOfTheRange=(particle_tpso(i).Position.theta3<VarMin.theta3 | particle_tpso(i).Position.theta3>VarMax.theta3);
        particle_tpso(i).Velocity.theta3(OutOfTheRange)=-particle_tpso(i).Velocity.theta3(OutOfTheRange);

        % Update Position Bounds
        particle_tpso(i).Position.theta3 = max(particle_tpso(i).Position.theta3,VarMin.theta3);
        particle_tpso(i).Position.theta3 = min(particle_tpso(i).Position.theta3,VarMax.theta3);

        % Evaluation
        particle_tpso(i).Cost=CostFunction(thetatoxyz(particle_tpso(i).Position,model));

        % Update Personal Best
        if particle_tpso(i).Cost < particle_tpso(i).Best.Cost

            particle_tpso(i).Best.Position=particle_tpso(i).Position;
            particle_tpso(i).Best.Cost=particle_tpso(i).Cost;

            % Update Global Best
            if particle_tpso(i).Best.Cost < GlobalBest_tpso.Cost
                GlobalBest_tpso=particle_tpso(i).Best;
            end

        end

    end

    % Inertia Weight Damping
    w=w*wdamp;

    % Show Iteration Information
    disp(['Iteration ' num2str(it_tpso) ': Best Cost = ' num2str(BestCost_tpso(it_tpso))]);

end

%% Plot results
% Best solution

BestPosition_pso = GlobalBest_pso.Position;
BestPosition_spso = SphericalToCart(GlobalBest_spso.Position,model);
BestPosition_qpso = GlobalBest_qpso.Position;
BestPosition_tpso = thetatoxyz(GlobalBest_tpso.Position,model);
disp("Best solution...");
disp(BestPosition_pso);
disp(BestPosition_spso);
disp(BestPosition_qpso);
disp(BestPosition_tpso);
PlotSolution_all(BestPosition_pso,BestPosition_spso,BestPosition_qpso,BestPosition_tpso,model);

maker_idx = 1:25:200;
% Best cost  
figure;
%plot(BestCost_pso,'r',BestCost_spso,'g',BestCost_qpso,'b',BestCost_tpso,'y');
plot(BestCost_pso,'s-','LineWidth',2,'MarkerIndices',maker_idx);
hold on
plot(BestCost_spso,'x-','LineWidth',2,'MarkerSize',10,'MarkerIndices',maker_idx);
hold on
plot(BestCost_qpso,'+-','LineWidth',2,'MarkerIndices',maker_idx);
hold on
plot(BestCost_tpso,'*-','LineWidth',2,'MarkerIndices',maker_idx);
hold on
xlabel('Iteration');
ylabel('Best Cost');
legend('PSO','RPSO','QPSO','θPSO')
grid on; 
