

clear all
clc



fprintf("--------------------------------------------------------------------------------------------------------------------------------\n")

fprintf("Kalman Filter Code\n\n")



% ----------------------------------------MINIMAL STATE VECTOR---------------------------------------------------------------------------------------------------------------------------------

%{

What is the minimal state vector for the Kalman filter (so that the resulting system is Markovian)?

For a car moving in a linear environment, we understand that the system is
moving in a 1 direction, x in this case. So x is the first entry in our
state vector. We don not have y or z because it is in 1D...

The other entry in our state vector is the velocity, and we will call it
v in this code.
 
As for acceleration, it is given that acceleration changes at every time
stamp, so we will assume it is constant throughout t. So a_t will be a
a constant random number at each time step, and does not depend on t-1. Therefore,
it is NOT included in the state vector...

 -- Minimal state vector is composed of x and v, we will call it big X

        State Vector = X = [x; v]

But what are their corresponding values? The answer lies in the dynamic
model equation of any linear 1D system, that is:
    
        x_t = x_prev + v_prev * delta_t + 0.5 * a_t * delta_t ^2
        v_t = v_prev + a_t * delta_t^2

      - Note that x_t means x at time t, and x_prev means x at time t-1 (x at the
        previous time stamp)


%}

fprintf("(a) The minimal state vector is composed of two states, x and v, where State Vector = X = [x; v]\n");

%_________________________________________________________________________________________________________________________




% ------------------------------------------STATE TRANSITION PROBABILITY---------------------------------------------------------------------------------------------------------

%{


The Kalman filter relies on two prediction steps to calculate the prior
belief, let's start with the first one which is calculating the State
prediction step:

X = A * X_t-1 + B * U_t + Uncertainty In State Transition.
 

Two points that should be clarified:
  - The Uncertainty in State Transition Matrix is Mainly caused by the
    uncertainty in acceleration.
  - Their is no control input U ( I will not model acceleration as a
  control rather I considered it as a factor affecting the state Vector X).

So U_t = 0;
B is therefore not important, it can be B = [1 1] or just 0.


So equation simplifies to 


X = A * X_t-1 + Uncertainty In State Transition.

X is made up of two rows and should resemble these values:

        x_t = x_prev + v_prev * delta_t + 0.5 * a_t * delta_t ^2
        v_t = v_prev + a_t * delta_t^2

  Comparing these two values of X, we reach this form:
      
    X =   x_t  =    1  delta_t   *   x_prev   +  0.5 * delta_t*2  * a
          v_t       0     1          v_prev       1 * delta_t   

So, A = [1 delta_t; 0 1];
    B = 0

Given that delta_t = 1 Then:

standard deviations of uncertainties coming from a are sigma_x = 0.5 and sigma_v = 1;
 
R_t is the covariance matrix that includes the variances of the noises
affecting each state in the state vector. So:

R_t = [sigma_x ^2  sigma_x * sigma_v;sigma_v*sigma_x sigma_v.^2];
       

%}

delta_t = 1;
fprintf("\n(b) The matrices A, B, and R")
A = [1 delta_t; 0 1]
B = 0


sigma_x = 0.5;
sigma_v = 1;
R_t = [sigma_x ^2  sigma_x*sigma_v; sigma_v*sigma_x sigma_v.^2]
   

% ---------------------------------PREDICTION STEP FOR t=1 THROUGH 5--------------------------------------------------------------------------------------------------
%{

Implement the state prediction step of the Kalman filter.

Compute the state distributions for times 1, 2, ..., 5.

%}   

% Initialize x, v and a according to given:
x_prev = 0;
v_prev = 0;
a_0=0;

% Initialize Matrices for later plotting
x_Matrix =[];
v_Matrix = [];
Covar_Matrix=[];



% Kalfam filter considers the mean of the state...  so The mean at t=0 is
% zero. Since a initial is equal to zero, then no initial uncertainties
% exist and covat at t=0 is zero..

mean_0 = [x_prev; v_prev];
Covar_0 = [0 0; 0 0];

% now for mean at t, knowing that mean_0 is zero, and then propagating
% forward with a mean of a also equal to zero (as given), then the mean of
% the state vector will always be equal to zero.
fprintf("The mean at all timestamps is equal to zero\n");

mean_t = [0;0]

% For the covariance matrix, it accounts for uncertaintis caused in the
% state transition. From Kalman filter algorithm, we know that covariance
% equation is:    Covar_t = A * Covar_prev * A'+ R_t;
fprintf("\n(c) Implementing the state prediction step: \n")

fprintf("At time t=0, The covariance is");

Covar_prev = Covar_0

% Set the number of iterations, or Timestamps
TimeStamps = 5;

for i = 1: TimeStamps
   
x_t = x_prev + v_prev + sigma_x * randn;
v_t = v_prev + sigma_v * randn;

x_Matrix = [x_Matrix x_t];
v_Matrix = [v_Matrix v_t];

fprintf('At time stamp %d',i);

Covar_t = A * Covar_prev * A'+ R_t
 if eig(Covar_t) >0
   error_ellipse(Covar_t)
   hold on
   pause(0.5)
   fprintf("Plotting the Uncertainty Ellipses (Fig 1)");
 end

Covar_prev = Covar_t;
x_prev = x_t;
v_prev = v_t;

end
figure
fprintf("\n(d) Plotting the posterior after the %.4f the Timestamp (Figure2) \n", i)
scatter(x_Matrix,v_Matrix, 'b');
xlabel('Position  x'); ylabel('Velocity v');



% --------------------------------MEASUREMENT CORRECTION-------------------------------------------------------------------------------------------------------------

%{

We will now add measurements to our Kalman filter. Suppose at time  t , 
we can receive a noisy observation of  x . In expectation, our sensor measures the true location.
However this measurement is corrupted by Gaussian noise with covariance  σ2=10 .


the measurement is denoted as z, and is calculated using the following
equation:

z_t = C * x_t + Uncertainty in measurement (i.e. noise). Where z is basically the noisy measurement 
we retrieved from our sensor.
 
%}

fprintf("--------------------------------------------------------------------------------------------------------------------------------\n")

fprintf("\n\n")

fprintf("Measurement Correction:\n\n")

fprintf("(a) C matrix is \n");

C=[1 0]

%{
Regarding matrix Q, it is the covariance of the noise in the measurement.

%}
variance_z = 10;

fprintf("Q matrix is \n");
Q = [variance_z 0;0 variance_z]

I = [1 0;0 1];

% At the 5th timstamp, we had an observation of z = 5


figure('Name', 'Uncertainty before and After measurement')

z = 5;
    
mean_t = [0; 0];    
Covar_t = A * Covar_prev * A'+ R_t;

KG = Covar_t * C' * inv(C * Covar_t * C' + variance_z);

mean_corrected = mean_t + KG * (z - C * mean_t);
Covar_corrected = (I - KG * C) * Covar_t;


fprintf("Plotting the Uncertainty Ellipses before measurement");
   error_ellipse(Covar_t)
   hold on
   pause(0.5)
   error_ellipse(Covar_corrected)
legend('Old Uncertainty', 'After measurement Uncertainty')

Covar_prev = Covar_corrected;



fprintf("(b) Before KF is the prediction step based on the dynamic mode, and After KF is the Correction step based on Measurement\n")

mean_t

mean_corrected

Covar_t

Covar_corrected

fprintf("We can notice that The Covariance (Uncertainty) was significantly reduced after incorporating the measurement\n\n")

% ------------------------------------------------------------------------------------------------------------------------------------------------------
fprintf("(e) As time increases, the correlation decreases ie the ellipse grows bigger, uncertainties increase n")

