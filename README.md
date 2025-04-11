# MAE 143B 4th Hour Problem Sessions
Repository for Matlab coding problems

Helpful tip: you can always type `help (command)` to get information on a command or function

## Examples

<details open>
<summary>Session 2, Problem 1: Step response for a "car"</summary>
<br>

### Part a)
We are going to model a car as a simple first-order system with transfer function

$`
    G(s)
    =
    \frac{0.04}{s+0.1}
`$

The input $u(t)$ to the car is the gas pedal position, in millimeters (mm), and the output $y(t)$ is the speed of the car.

Use the Matlab `tf` command to create a car: 
```
car = tf(0.04, [1 .1])
```
Next, use the `step` command to simulate the step response of the car model: 
```
[y,t] = step(car);
```
Make sure to follow the command with a semicolon to suppress writing a long output vector.
Now we can plot the step response corresponding to a 100mm depression of the gas pedal: 
```
plot(t,y*100,'LineWidth',2)
title('Car step response')
xlabel('time (s)')
ylabel('speed (m/s)')
legend('speed response')
grid on
```
Here is what I get 

![Car step response](Session2/problem1a.png)

### Part b) P-control
Create a proportional controller transfer function with, say, $K_P=10$:
```
Kp=10; pcontrol = tf(Kp,1);
```
Put this into a unity feedback loop (with the negative sign as appropriate for error feedback): 
```
pcloop = feedback(pcontrol*car,1);
```
This is the closed-loop transfer function from reference speed $r(t)$ to output $y(t)$.
We can also compute the closed-loop transfer function from reference speed $r(t)$ to gas pedal position in mm:
```
uloop = feedback(pcontrol, car);
```
Now we can use `step` to simulate and then plot the response of the car for different values of $K_P$. 
We expect that all of the responses are exponentials.
Here is what I get for various different gains: 
![alt text](Session2/problem1b.png)

Remember that this is a feedback control problem where we are trying to track the reference $r(t) = 1$ m/s. 
How well does p-control work for this problem?

### Part c) PI-Control 
Create a PI-controller with, say, $K_P=10$ and $K_I=5$:
```
Kp=10; Ki=5; picontrol = tf([Kp Ki],[1 0])
```
Create the PI unity feedback closed loop:
```
picloop = feedback(picontrol*car,1);
```
Create the control transfer function:
```
cpiloop = feedback(picontrol, car)
```
Try out the Matlab logical function `isstable`
```
isstable(picloop)
```

Now we can again use `step` to simulate and then plot the response of the car for different values of $K_P$ and $K_I$. 
Note that now we have two poles in the closed-loop transfer functions. 

Here is what I get for various gain parameters: 
![alt text](Session2/problem1c.png)

Again, remember that the `step` function is simulating the response to a unit step reference input, so we are trying to track a reference of $r(t) =  1$ m/s. 
Comparing the PI controller performance with the P controller performance, do you notice any differences?

</details>