# ppr
Polynomial-Polynomial Regulator (PPR) design 

## Syntax
`[v,K] = ppr(f,g,q,r,d)`

`[v,K,options] = ppr(f,g,q,r,degree,options)`

## Description
`[v,K] = ppr(f, g, q, r, d)` calculates the optimal gain coefficients `K` and the first `d` value function coefficients `v` of the associated Hamilton-Jacobi-Bellman equation using the continuous-time state-space model defined by `f` and `g`. `q` and `r` are the weight coefficients for states and inputs, respectively. 
All of these quantities are cell arrays containing matrix coefficients defining polynomials (feedback law, value function, dynamics, cost function, etc.).

<!-- example -->

## Examples
<!-- collapse all -->
<!-- ### LQR Control for Inverted Pendulum Model -->

<details open>
<summary>PPR Control for Inverted Pendulum on a Cart Model</summary>
<br>

`getSystem22()` returns the Taylor approximation of the 4D state-space model of an inverted pendulum on a cart based on [5-7].
The outputs are the cart displacement $x$ and the pendulum angle $\theta$. 
The control input $u$ is the horizontal force on the cart.
The nonlinear equations of motion are

$`
        \begin{bmatrix}
        \dot{x} \\
        \ddot{x} \\
        \dot{\theta} \\
        \ddot{\theta}
    \end{bmatrix}
    =              
    \begin{bmatrix}
         x_2 \\ 
         \frac{ - 0.27 x_2 - 0.18 x_4^2 \sin(x_3)  + 4 \cos(x_3) \sin(x_3)}
         {1.8 - 0.44\cos(x_3)^2}\\ 
         x_4 \\ 
         \frac{40 \sin(x_3) - 0.67 x_2 \cos(x_3) - 0.44 x_4^2 \cos(x_3)\sin(x_3)}
         {1.8 - 0.44\cos(x_3)^2}
    \end{bmatrix}
    +            
    \begin{bmatrix}
         0 \\ 
         \frac{2.7 }
         {1.8 - 0.44\cos(x_3)^2}\\ 
         0 \\ 
         \frac{6.7 \cos(x_3)}
         {1.8 - 0.44\cos(x_3)^2}
    \end{bmatrix} u
`$

$`
    y =
    \begin{bmatrix}
        1 & 0 & 0 & 0 \\
        0 & 0 & 1 & 0
    \end{bmatrix}
    \begin{bmatrix}
        x \\
        \dot{x} \\
        \theta \\
        \dot{\theta}
    \end{bmatrix}
    +
    \begin{bmatrix}
        0 \\
        0
    \end{bmatrix} u
`$

The linearized dynamics are then

$`
    \begin{bmatrix}
        \dot{x} \\
        \ddot{x} \\
        \dot{\theta} \\
        \ddot{\theta}
    \end{bmatrix}
    =
    \begin{bmatrix}
        0 & 1 & 0 & 0 \\
        0 & -0.2 & 3 & 0 \\
        0 & 0 & 0 & 1 \\
        0 & -0.5 & 30 & 0
    \end{bmatrix}
    \begin{bmatrix}
        x \\
        \dot{x} \\
        \theta \\
        \dot{\theta}
    \end{bmatrix}
    +
    \begin{bmatrix}
        0 \\
        2 \\
        0 \\
        5
    \end{bmatrix} u
`$

which is almost identical to the model used in the LQR function documentation, except for the (2,2) entry which is -0.1 there. 
`getSystem22()` uses the helper function `approxPolynomialDynamics()` to compute the polynomial coefficient arrays `f`, `g`, and `h` for the dynamics using the symbolic nonlinear dynamics.

Run the example with `runExample22()` after running `setKroneckerToolsPath`, `addpath('examples')`, and `addpath('utils')`. Here is a breakdown of key steps in the example script:

First, load a degree 7 approximation to the state-space model to the workspace. `f` and `g` are cell arrays containing the Taylor approximations to the nonlinear dynamics, and `xdot` is a symbolic function capturing the full nonlinear dynamics for use in simulations.
```
[f, g, ~, xdot] = getSystem22(7);
```

Define the initial condition of interest.
Neither PPR nor LQR can do full swing-up control currently, but there are conditions where PPR succeeds while LQR fails.
A simple case is when the cart is already in the up position, but shifted to the right 10 units and we wish to move it to the left to the origin without tipping the pendulum over.
```
x0 = [12;0;0;0];
```
Another interesting case is `x0 = [0;0;-pi/3*1.04;0];`, which physically corresponds to the pendulum starting from a tilted position. 
This case is just beyond the region where LQR succeeds; however, PPR still succeeds at this condition.

Next, we can define and compute the control laws.
Let's use a simple quadratic cost function defined by $Q$ and $R$ matrices: 
```
Q = [1,0,0,0;...
    0,0,0,0;...
    0,0,1,0;...
    0,0,0,0];
R = 1;
```
Find the gain matrices $K_i$ using `ppr`. 

```
[~, K] = ppr(f, g, Q, R,8)
```
```console
    K = 1×7 cell array

        {1×4 double}    {1×16 double}    {1×64 double}    {1×256 double}    {1×1024 double}    {1×4096 double}    {1×16384 double}
```

Unlike `lqr`, which returns a single gain matrix $K$, `ppr` return several matrices in a cell array. 
The first entry in the array $K_1$ is precisely the 1x4 LQR gain matrix, hence an LQR controller can be computed by calling `ppr` with `degree` set to 2, which computes a quadratic value function approximation and a linear feedback. 
The remaining $K_i$ are the higher order gain matrices corresponding to quadratic, cubic, etc. feedback terms. 

A feedback law can be defined using the gain coefficient cell array `K` using an anonymous function based on the utility function `kronPolyEval`. 
Here we use the function to define both the LQR and PPR controllers, along with an "open-loop" controller corresponding to the uncontrolled system: 
```
uOpenLoop = @(x) zeros(1,1);
uLQR = @(x) (kronPolyEval(K, x, 1));
uPPR = @(x) (kronPolyEval(K, x));
```

Using this format, we can easily simulate the closed-loop performance of the different controllers. 
Define the system dynamics $\dot{x} = f(x,u)$ using an anonymous function based on the `xdot` function from earlier:
```
FofXU = @(x,u) xdot(x,u);
```

The closed-loop systems can be simulated with your favorite ode solver; for example, 
```
% Time vector
tmax = 10; dt = 0.001; t = 0:dt:tmax-dt;

% Simulate with ode45
[t, Xunc] = ode45(@(t, x) FofXU(x,uOpenLoop(x)),t, x0);
[t, XLQR] = ode45(@(t, x) FofXU(x,uLQR(x)),t, x0);
[t, XPPR] = ode45(@(t, x) FofXU(x,uPPR(x)),t, x0);

% Plot the results 
figure; 
for i=1:4; subplot(2,2,i); hold on; plot(t,Xunc(:,i)); end
for i=1:4; subplot(2,2,i); hold on; plot(t,XLQR(:,i)); end
for i=1:4; subplot(2,2,i); hold on; plot(t,XPPR(:,i)); end

subplot(2,2,1); xlabel('time'); ylabel('z'); title('cart position'); ylim([-2 15])
subplot(2,2,2); xlabel('time'); ylabel('zdot'); title('cart velocity'); ylim([-10 5])
subplot(2,2,3); xlabel('time'); ylabel('theta'); title('pendulum angle'); ylim([-2*pi 2*pi])
subplot(2,2,4); xlabel('time'); ylabel('thetadot'); title('pendulum velocity'); ylim([-3 4])
legend('Uncontrolled system','LQR','PPR','Location','southoutside')
```

`runExample22()` does essentially this, but also compares with nonlinear MPC based on Matlab documentation's example.
To make the comparison easier, an explicit Euler time integrator is used, which slightly changes the results, but they are qualitatively similar. 
Below is the plot of the results comparing the controller performances: 

<img src="https://cnick1.github.io/images/invertedPendulumExample.png" width="600">
<img src="https://cnick1.github.io/images/invertedPendulumExampleControls.png" width="400">

This is not necessarily the optimal MPC performance that can be achieved with modern solvers; this is just based on modifying the simple example in the Matlab documentation. 
It is worth noting that the controller costs here are 180.642 for PPR and 204.601 for MPC, so PPR performs better. 
However, PPR is significantly faster online, because the control law is given as a simple polynomial, whereas MPC must solve an optimization problem repeatedly. 
Better performance can of course be achieved by modifying the settings used for MPC, but then the computations become more expensive and take even longer. 
It is also worth noting that MPC can do swing-up control, whereas PPR and LQR currently fail.

</details>