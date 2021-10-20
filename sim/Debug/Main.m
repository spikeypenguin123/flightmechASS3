% Main Simulation Script

%Initialise Data
[X0_init U0_init FlightData] = Initialise;

X0_init = X0_init+0.1;
%Convert X0 to Quaternions
Q_init = InitQuat(X0_init);
X0 = [X0_init(1:6); Q_init; X0_init(10:12)]
U0 = U0_init+0.1


%Simulation Parameters
DT = 0.01;
TF = 60;    %End time for simulation
T0 = DT;

n_pts = TF/DT;

%Initialise Data Matrices
X = zeros(13,n_pts);
U = zeros(4,n_pts);
T = zeros(1,n_pts);


%Store Initial Conditions
X(:,1) = X0;
U(:,1) = U0;
T(1) = T0;


%Main Simulation Loop
for ii = 2:n_pts
    T(ii) = DT*ii; %Create Time Vector
    
    %Set Controls
    if T(ii) > 1
        U(:,ii-1) = U0+[0;0;0;0];
    else
        U(:,ii-1) = U0;
    end
    
    %Integrate X
    X_out = RK4(X(:,ii-1),U(:,ii-1),DT,FlightData);
    
    %Normalise Quaternions
    quat_norm = QuatNormalise(X_out(7:10));
    X(:,ii) = [X_out(1:6); quat_norm; X_out(11:13)];

    return
    
end