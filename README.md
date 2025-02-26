Download link :https://programming.engineering/product/lab-2-ece311-introduction-to-linear-control-systems/


# Lab-2-ECE311-Introduction-to-Linear-Control-Systems
Lab 2 – ECE311 Introduction to Linear Control Systems
Control of a Magnetically Levitated Ball

MAIN CONCEPTS OF THIS LAB

How to linearize nonlinear control systems in Matlab/Simulink

How to check stability of LTI systems in Matlab

Design of a lead controller for magnetic levitation of a steel ball

INTRODUCTION

In this lab you will learn how to define and linearize nonlinear control systems in Matlab/Simulink, and how to check the stability of the linearization. For this, we will investigate a basic magnetic levitation problem, whereby the current in an electromagnet is controlled so as to levitate a steel ball at a fixed distance from the bottom face of the electromagnet. The setup is depicted in the figure below.

ia

+

u

y

M

The system can be modelled as an RL circuit representing the electromagnet winding, and a mass M subject to gravity and the force imparted by the electromagnet. The control input u is the voltage applied at the electromagnet terminals, while the output y is the distance of the ball from the bottom face of the

electromagnet. Denoting by Ra and La the resistance and inductance of the winding, the circuit equation is

La

dia

+ Ra ia = u.

(1)

dt

The force imparted by the electromagnet on the steel ball, directed upward, is1 Fm = km i2a /y2, where km is a positive constant. Newton’s equation for the steel ball is

My¨ = −Fm + Mg = −km

i2a

+ Mg,

(2)

y2

where g is the acceleration due to gravity. The numerical values of various constants in the model are found in the table below. The model above is only valid if y > 0, i.e., if the steel ball is placed below the electromagnet.

Parameter

Description

Numerical value

La

Winding inductance

0.05 H

Ra

Winding resistance

3 ohms

M

Mass of the steel ball

0.1

Kg

km

Coefficient of electromagnetic force

0.1

N m2/A2

g

Acceleration due to gravity

9.81 m/sec2

Matlab code. Your code should be clean and readable, and it should contain abundant commentary, so that the instructors may follow your development and check its correctness. This component of the mark will be based on the correctness of the code and its readability, and it will be assigned as follows:

When the lab document asks you to comment on something, strive to provide meaningful, insightful commentary. This commentary should be an expanded version of the comments you make in person when you meet the lab TA. Make sure you display an understanding of what the lab is about. For exam-ple, if you are asked to comment on the experimental observation that a linear controller fails to stabilize a nonlinear system far from the equilibrium, you need to give a convincing explanation grounded in the concepts that you’ve learned in class, starting from the fact that the linearization constitutes an approximation of a nonlinear function. This component of the mark will be assigned as follows:


Output 1. • Write in your report the equilibrium x¯ ∈ R3 and the control u¯ that you determined above. These quantities should contain y¯, and M, La , Ra , g, km as parameters.

Write the linearization of (3) at x¯ with control u¯, defining all error variables.

Double-check carefully your work. Verify that f (x¯, u¯) = 0 and that the various partial derivatives were computed correctly.

NUMERICAL LINEARIZATION AND STABILITY ASSESSMENT

In this section you will create a Simulink diagram representing the model (3), and you will then numer-ically linearize it using Matlab. You will compare the numerical linearization matrices to the theoretical ones that you computed by hand in Section 3. You will also test the stability of the linearized model.

SIMULINK BLOCKS (INSIDE THE LIBRARY BROWSER)

Ports & Subystems → Subsystem creates a block containing a subsystem

Sources → In1 input port

Sinks → Terminator used to terminate output signals

Sinks → Out1 output port

Continuous → Integrator integrates input signal

User-defined functions → Interpreted MATLAB Fcn computes values using a Matlab function

MATLAB COMMANDS

[A,B,C,D]=linmod(’mod’,xbar,ubar)

Computes the numerical linearization of model mod at the equi-

librium xbar with control ubar

norm(M)

Computes the norm of a matrix M (or also a vector)

eig(M)

Computes the eigenvalues of a square matrix M

Open Simulink and the Library Browser. Create a blank diagram called lab2_1.slx.

Using the blocks listed above, draw a Simulink diagram like the one depicted below, where magball is a block containing a subsystem that you will define in a moment. Label all quantities as in the figure, and in particular label the subsystem block magball.


1 y

1

u

state

u

magball

Open the input port u on the left end side of the block diagram above. Click on Signal Attributes and in the box labelled Port dimensions write 1 in place of the default -1.

Open the magball subsystem block, and in it draw the diagram depicted below, labelling various quantities as in the figure.

Interpreted

MATLAB Fcn

compute yddot

ydot

y

1

state

Interpreted

MATLAB Fcn

compute idot

i

1

u

There are two Interpreted MATLAB Fcn blocks in the figure. These blocks take in input the four-dimensional vector x1 x2 x3 u ⊤, and compute, respectively, y¨ (upper block) and dia /dt (lower block) using the expressions for dia /dt and y¨ found in (1) and (2). Open these blocks and enter the expressions for y¨ and dia /dt. In doing so, note the Matlab convention that the input vector to the Function block is called u, so if you need to refer to, say, x3, you will write u(3), and the control input u will be u(4) according to our ordering. You will soon define the physical constants M, La , Ra , g, km in a script. Inside the Interpreted MATLAB Fcn blocks, refer to them simply as

M, La, Ra, g, km.

The output of the upper block representing y¨ is integrated twice to produce the states x1 = y and x2 = y˙, while the output of the lower block is integrated once to produce the state x3 = ia . The three states so defined, together with the input u coming from the input port are collected in a vector by the Mux block. This vector is fed back to the Interpreted MATLAB Fcn blocks. This subsystem with input u will output the state x, formed using a second Mux block.

Create a Matlab script named lab2.m. Begin by declaring variables M, La, Ra, g, km with numerical values given in the Table of Section 1.

Declare a variable ybar representing the equilibrium value of y, and set this variable equal to 0.1 metres. Then, declare a 3 × 1 vector xbar and a scalar ubar containing the expressions for x¯ and u¯ that you found in Section 3.

Note: By setting y¯ = 0.1, we are declaring that we are interested in stabilizing the ball at a distance of 10 cm from the magnet face.

Using the command linmod calling the Simulink model lab2_1.slx, find the matrices A, B, C, D of the numerical linearization at xbar,ubar.

Declare matrices A1,B1,C1,D1 containing the expressions for the linearization matrices that you found in Section 3, using the parameters M, La, Ra, g, km just declared. Using the command norm(A-A1), compute the error between theoretical and numerically derived matrices. Repeat for B,B1.

Note: The command norm computes the norm of a matrix, which roughly speaking is a measure of how large the entries of the matrix are. The norm of a matrix is zero if and only if the matrix is zero.

Using the matrices A1,B1,C1,D1, and using the technique your learned in lab 1, find the transfer function of the linearized model and extract the list of its poles. Name this transfer function G.

Using the command eig, compute the eigenvalues of A1.

Output 2. • Print the numerically derived matrices A,B and their theoretically derived counterparts,

A1,B1.

Print the errors described above, and comment on the accuracy of the numerical approximation performed by Matlab/Simulink.

Print the eigenvalues of A1 and the poles of G. Note that the poles of the transfer function coincide with the eigenvalues of the matrix A1. Is the linearized model internally stable? Is it BIBO stable? Comment on how your findings about stability conform with physical intuition.

FEEDBACK CONTROL OF THE MAGNETIC LEVITATION SYSTEM

You have computed the transfer function of the linearized model of the magnetic levitation system. Note that the input of the transfer function is u˜ = u − u¯, and the output is y˜ = y − y¯. In this section you’ll design a so-called lead controller to stabilize the linearized plant, and you’ll test the controller on the nonlinear Simulink model magball that you’ve developed in Section 4. More precisely, our objective is to make y(t) converge to y¯, or equivalently make y˜(t) converge to zero. We are therefore asking the error output y˜ to track a zero reference signal.

Before we begin, we need to understand how to implement a linearization-based controller, given

that the input of this latter is y˜, and the output is u˜. From the definition, we have

u = u¯ + u˜, y˜ = y − y¯,

so the block diagram of our controller looks like this.

u¯

y¯

_

y˜

u˜

+

u

y

+

+

+

C(s)

magball

+

Note that the input and output of the nonlinear model magball are u and y. The diagram uses y and y¯ to compute y˜, which is then fed to the controller. The controller computes u˜, to which we add the bias u¯ and get u, which is then fed to the nonlinear control system model magball.

SIMULINK BLOCKS (INSIDE THE LIBRARY BROWSER)

Control System Toolbox → LTI System Block containing LTI system object

Create a blank diagram called lab2_2.slx.

Cut and paste the subsystem magball from lab2_1.slx in the new diagram, and draw the feedback loop depicted below.

ubar

In Section 4 you should have determined that the linearized model is unstable (both internally and from the input-output point of view). It turns out that the stabilization problem for this system is much harder than the speed control problem for the DC motor that you investigated in lab 1. For this reason, we will suggest a specific controller structure, and you’ll simply tune a parameter to achieve stability.

The controller we propose is called a lead controller, and has the form

C(s) = K

s + z

,

(4)

s + p

where K, z, p > 0 are design parameters such that z < p.

Return to your Matlab script lab2.m and do the following.

Create an LTI system object named CONTROLLER representing the transfer function (4). Choose parameters z=10, p=100, K=70 for this transfer function.

Note: Do not name the controller C, as this symbol was already used earlier to define the output matrix of the linearization.

The poles of the closed-loop system transfer function (using the linearized plant), are the roots of 1 − C(s)G(s). Note the minus sign, due to definition of y˜ which led us to flipping the signs in the summing block of the feedback loop. Using the tools you learned in lab 1, determine the zeroes of 1-CONTROLLER*G and verify that the system is unstable.

Repeat the above operation with increasing values of K and find a value of K for which the closed-loop system is BIBO stable (try increasing K by increments of 10).

Optional: If you want to try a more advanced tool, look at the Matlab help for the command rlocus (root locus). The root locus of a transfer function T(s) is the plot in the complex plane of the roots of 1 + KT(s) as K ranges from 0 to ∞. In our case, we are interested in the roots of 1-CONTROLLER*G, and hence in the root locus of -CONTROLLER*G.

Now you’ll test your controller on the nonlinear control system. Return to the Simulink diagram lab2_2.slx. Edit the magball block and, in it, edit the integrator block which outputs y. Set the initial condition of this block to 0.15 metres. You are initializing the ball at 15 cm from the magnet face, or 5 cm away from the desired distance. Run the diagram and check whether the output converges to 0.1 metres. If it doesn’t, tune the parameter K.

Having found a value of K for which your controller work, you will now roughly determine how large is the range of initial ball positions y(0) for which the controller works. Without changing K, change the initial condition of the integrator block above to find rough lower and upper limits of y(0) for which the controller manages to stabilize the ball.

This block requires the Control Systems Toolbox. If you don’t have this toolbox installed, you can use a transfer function block calling numerator and denominator arrays in the workspace, as you did in lab 1.


Output 3. • Print the value of K you found above. Produce a figure of the output y(t) when y(0) = 0.15 m.

Describe the procedure you followed in finding K, and any observations you made along the way.

Print the range of initial conditions y(0) for which your controller succeeds in stabilizing the ball, and comment on why your controller does not work for initial conditions that are far from the equilibrium.

10
