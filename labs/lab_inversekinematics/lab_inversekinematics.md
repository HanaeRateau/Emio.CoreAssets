# Lab Inverse Kinematics, Quadratic Programming

::: highlight

##### Overview

In this lab, we will explore the principles of inverse kinematics (IK) as applied to soft robots. 
The kinematics of soft robots has been, and continues to be, a subject of extensive scientific research. 
Here, the proposed approach relies on the computation of mechanical models that consider the material properties, 
specifically utilizing compliance projected into the space of actuators and end effectors.

By the end of this lab, you will have a better understanding of the concept of mechanical compliance and 
the formulation of an optimization problem that allows to get the inverse kinematics from the mechanical
compliance model of soft robots.

:::

::::::: collapse Coupling of the legs
## Coupling of the legs

As described in the previous lab, the robot's legs are modeled as deformable structures. These legs can be grouped 
in various configurations using connectors. The aim of this lab is to place the legs in two different 
positions on the motor (clockwise or counterclockwise), and test different effectors. 
Certain configurations may result in mechanical instabilities, primarily due to leg buckling. 
This hands-on experiment will enable you to observe that some configurations are mechanically more stable than others 
and that, sometimes, several position of the effector $\mathbf{y}_{e}$ can be found for the same motor positions $\mathbf{u}_{a}$.

Next, we propose to introduce the mathematical modeling of leg coupling using three different modeling approaches. 
In each case, the objective is to constrain the degrees of freedom (DOF) of the leg ends to match those of the effector. 
For simplicity, we assume in the following that the robot has only two actuated legs, although in reality it has four. 

### Case 1: Absolute Coordinates 

To enforce this equality, the simplest case arises when we work with the DOF in absolute coordinates in space. 
In such a case, the idea is to select the degrees of freedom at the end of the leg (in this case,
$\mathbf{I}_1$ and $\mathbf{I}_2$) and equalize them together with the end-effector motion at each step.

$$
 \left\{ 
 \begin{array}{l}
 \mathbf{A}_1 d\mathbf{q}_1 = \mathbf{b}_1  \ \ \text{leg 1}  \\
 \mathbf{A}_2 d\mathbf{q}_2 = \mathbf{b}_2  \ \ \text{leg 2}  \\
 \text{with} \\
 \underbrace{\left [ 0 ... 0 \ I \  0 ... 0  \right ]}_{\mathbf{I}_1} d\mathbf{x}_1 = 
 \underbrace{\left [ 0 ... 0 \ I \  0 ... 0  \right ]}_{\mathbf{I}_2}
 d\mathbf{x}_2 = d\mathbf{y}_e
 \end{array}
 \right.
$$

In this case, we can reorder the equations to equalize the components of $\mathbf{q}_1$ and $\mathbf{q}_2$ corresponding 
to the end DOF of the legs that need to be matched.

$$
\underbrace{
    \left[  
    \begin{array}{lll}
    \mathbf{A}_1 & \mathbf{A}_1 & 0 \\
    0 & \mathbf{A}_1 + \mathbf{A}_2 & \mathbf{A}_2 \\
    0 & \mathbf{A}_2 & \mathbf{A}_2  
    \end{array}
    \right]
}_{\mathbf{A}}
\underbrace{
    \left[  
    \begin{array}{l}
        d\mathbf{q}_1^{\neq} \\
        d\mathbf{q}_1^{=} = d\mathbf{q}_2^{=} \\
        d\mathbf{q}_2^{\neq}
    \end{array}
    \right]
}_{d\mathbf{q}}    
    =
\underbrace{
    \left[  
    \begin{array}{l}
        \mathbf{b}_1 \\
        \mathbf{b}_1^{=} + \mathbf{b}_2^{=} \\
        \mathbf{b}_2 
    \end{array}
    \right]
}_{\mathbf{b}}    
$$

In practice this case is only implemented with FEM beams.

### Case 2: Lagrange Multipliers

In the second and third cases, the absolute position of each leg end, $\mathbf{x}_1$ and $\mathbf{x}_2$, is derived kinematically from the DOF $\mathbf{q}_1$ and $\mathbf{q}_2$, respectively with a kinematic function $\mathbf{x}_1 = \boldsymbol{\delta}_1(\mathbf{q}_1)$ and $\mathbf{x}_2 = \boldsymbol{\delta}_2(\mathbf{q}_2)$.
This allows us to write the equality in terms of the effector's position $\mathbf{y}_e = \mathbf{x}_1  = \mathbf{x}_2$.
In $\boldsymbol{\delta}_1$ and $\boldsymbol{\delta}_2$ we take into account the offset of the end of the leg in relation to the center of the end effector.

$$
\left\{ 
\begin{array}{l}
d\mathbf{x}_1  = \mathbf{H}_1 d\mathbf{q}_1  = d\mathbf{y}_e \\
d\mathbf{x}_2  = \mathbf{H}_2 d\mathbf{q}_2 = d\mathbf{y}_e
\end{array}
\right.
$$

with $\mathbf{H}_1  = \frac{\partial  \delta_1}{\partial \mathbf{q}_1}$ and $\mathbf{H}_2  = \frac{\partial  \delta_2}{\partial \mathbf{q}_2}$.

so we need to impose the kinematic constraint:
$$
 \mathbf{H}_1 d\mathbf{q}_1 = 
 \mathbf{H}_2 d\mathbf{q}_2 
$$

In Case 2, Lagrange multipliers are used to enforce position constraints, which increases the system's size.

$$
\underbrace{
    \left[  
    \begin{array}{lll}
    \mathbf{A}_1 & 0 & \mathbf{H}_1^T \\
    0 & \mathbf{A}_2 & -\mathbf{H}_2^T \\
    \mathbf{H}_1 & -\mathbf{H}_2^T & 0  
    \end{array}
    \right]
}_{\mathbf{A}}
\underbrace{    
    \left[  
    \begin{array}{l}
        d\mathbf{q}_1 \\
        d\mathbf{q}_2 \\
        \boldsymbol{\lambda}
    \end{array}
    \right]
}_{d\mathbf{q}}    
    =
\underbrace{
    \left[  
    \begin{array}{l}
        \mathbf{b}_1 \\
        \mathbf{b}_2 \\
        0
    \end{array}
    \right]
}_{\mathbf{b}}   
$$

### Case 3: Penalty Approach

In Case 3, a penalty method is applied. It consists of creating a coupling forces to impose the kinematic constraint.

$$
 \left\{ 
 \begin{array}{l}
 \mathbf{A}_1 d\mathbf{q}_1 = \mathbf{b}_1 + \mathbf{f}_1 \ \ \text{leg 1} \\
 \mathbf{A}_2 d\mathbf{q}_2 = \mathbf{b}_2 + \mathbf{f}_2 \ \ \text{leg 2} \\ 
 \mathbf{H}_1 k(\mathbf{H}_1 d\mathbf{q}_1 - 
 \mathbf{H}_2 d\mathbf{q}_2 )  = -\mathbf{f}_1 \\
 \mathbf{H}_2 k(\mathbf{H}_1 d\mathbf{q}_1 - 
 \mathbf{H}_2 d\mathbf{q}_2 )  = \mathbf{f}_2
 \end{array}
 \right.
$$

While this approach keeps the system size constant, additional coupling terms may appear in the matrix.

$$
\underbrace{
    \left[  
    \begin{array}{ll}
    \mathbf{A}_1 + \mathbf{H}_1^T k \mathbf{H}_1 & -\mathbf{H}_1^T k \mathbf{H}_2 \\
    -\mathbf{H}_1^T k \mathbf{H}_2 & \mathbf{A}_2 + \mathbf{H}_2^T k \mathbf{H}_2 
    \end{array}
    \right]
}_{\mathbf{A}}
\underbrace{   
    \left[  
    \begin{array}{l}
        d\mathbf{q}_1 \\
        d\mathbf{q}_2 
    \end{array}
    \right]
}_{d\mathbf{q}}    
    =
\underbrace{
    \left[  
    \begin{array}{l}
        \mathbf{b}_1 \\
        \mathbf{b}_2 
    \end{array}
    \right]
}_{\mathbf{b}}       
$$

In all three cases, the result is a linear system that must be solved at each time step, similar to the static equilibrium 
equations used for individual leg kinematics. Lagrange Multiplier and Penalty approach are more generic as it does not 
require to use absolute coordinate system, like for the proposed Cosserat Model. 

:::::::

::::::: collapse Kinematics of Emio
## Kinematics of Emio

In this section, we will explore the kinematic model of Emio, depending on the configurations of the legs.
After modeling each leg of the robot and the mechanical coupling between these legs (depending on the effector connector), 
we obtained the static calculations of the robot.
To observe the kinematics, you will test different configurations of Emio. 

As explained above, the kinematics of the robot can be expressed as a function that gives the position of the end-effector 
$\textcolor{darkgreen}{\mathbf{y}_{e}}$ based on the commanded motor position $\textcolor{red}{\mathbf{u}_{a}}$. 
To compute this function, we will modify the static force calculations, incorporate the coupling of the four legs, 
and impose the motor motion.

In the algorithm, the four values of the actuation torque are introduced into the simulation as a vector of 
Lagrange multiplier $\boldsymbol{\lambda}_{\mathrm{a}}$. Furthermore, the motor positions in the simulation, 
$\boldsymbol{\delta}_{\mathrm{a}}$, is a function of the robot's position $\mathbf{q}$ 
(which is a concatenation of the leg positions: $\mathbf{q}_1,  ..., \mathbf{q}_4$).

![](assets/data/images/lab2-algorithm2.png){width=65%, .center}

To compute Equation 15 (in the algorithm above), it is more efficient to proceed with an indirect solution. 
We will decompose the movement at each time step by separating the contributions from the force $\mathbf{b}$, which is 
related to internal forces, external forces, and gravity (whose values we can compute), and the forces $\mathbf{H}_{\mathrm{a}}^T$ 
related to actuation (whose values are unknown and depend on the force required to satisfy the constraints).

$$
\mathbf{A}d\mathbf{q} = \mathbf{b} + \mathbf{H}_{\mathrm{a}}^T \boldsymbol{\lambda}_{\mathrm{a}} 
\Leftrightarrow
\left \{
\begin{array}{l}
d\mathbf{q} = d\mathbf{q}^{\mathrm{free}}  + d\mathbf{q}^{\lambda} \\ 
\mathrm{with}: \\
\mathbf{A}d\mathbf{q}^{\mathrm{free}} =  \mathbf{b} \leftrightarrow d\mathbf{q}^{\mathrm{free}} =  \mathbf{A}^{-1}\mathbf{b} \\
\mathbf{A}d\mathbf{q}^{\lambda} = \mathbf{H}_{\mathrm{a}}^T 
\boldsymbol{\lambda}_{\mathrm{a}} 
\leftrightarrow d\mathbf{q}^{\lambda}  = \mathbf{A}^{-1}\mathbf{H}_{\mathrm{a}}^T 
\boldsymbol{\lambda}_{\mathrm{a}}
\end{array}
\right .
$$

Thus, we can rewrite the kinematic constraint, 
$\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q} = \textcolor{red}{\mathbf{u}_{a}}$, 
as directly depending on the actuation force:

$$
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}\left(d\mathbf{q}^{\mathrm{free}} + d\mathbf{q}^{\lambda}\right) = \textcolor{red}{\mathbf{u}_{a}} \Longleftrightarrow
$$

$$
\underbrace{
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}}}_{\boldsymbol{\delta}_{\mathrm{a}}^{\mathrm{free}}} + 
\underbrace{\mathbf{H}_{\mathrm{a}}\mathbf{A}^{-1}\mathbf{H}_{\mathrm{a}}^T}_{\mathbf{W}_{\mathrm{aa}}} \boldsymbol{\lambda}_{\mathrm{a}} = \textcolor{red}{\mathbf{u}_{a}}
$$

This equation expresses the coupling of the actuation motion by the various torques via the compliance matrix $\mathbf{W}_{\mathrm{aa}}$, 
which represents the projection of the inverse matrix in the space of motor constraints.

The same way, we can rewrite 

$$
\textcolor{darkgreen}{\mathbf{y}_{e}} = \boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{e}} d\mathbf{q} 
$$

$$
\textcolor{darkgreen}{\mathbf{y}_{e}} =
\underbrace{
\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}}}_{\boldsymbol{\delta}_{\mathrm{a}}^{\mathrm{free}}} +
\underbrace{\mathbf{H}_{\mathrm{e}}\mathbf{A}^{-1}\mathbf{H}_{\mathrm{a}}^T}_{\mathbf{W}_{\mathrm{ea}}} \boldsymbol{\lambda}_{\mathrm{a}}
$$

Combining equations above, we obtain a reduced formula of the linearized kinematics:

$$
\textcolor{darkgreen}{\mathbf{y}_{e}} = \boldsymbol{\delta}_{\mathrm{e}}^{\mathrm{free}} + \mathbf{W}_{\mathrm{ea}}\mathbf{W}_{\mathrm{aa}}^{-1} ( \textcolor{red}{\mathbf{u}_{a}} - \boldsymbol{\delta}_{\mathrm{a}}^{\mathrm{free}})
$$

$\mathbf{J}_{\mathbf{SR}} = \mathbf{W}_{\mathrm{ea}}\mathbf{W}_{\mathrm{aa}}^{-1}$ being the jacobian of the soft robot. 



:::::: exercise

**Exercise 1:**


Set up your robot, try different configuration of the legs and different connectors.

::::: group-grid 

**Motor n°0**
![](assets/data/images/legs/blueleg-counterclockwisedown.png){data-condition="exo1motor1orientation==counterclockwisedown"}
![](assets/data/images/legs/blueleg-clockwisedown.png){data-condition="exo1motor1orientation==clockwisedown"}
:::: select exo1motor1orientation
::: option clockwisedown
::: option counterclockwisedown
::::

**Motor n°1**
![](assets/data/images/legs/blueleg-counterclockwisedown.png){data-condition="exo1motor2orientation==counterclockwisedown"}
![](assets/data/images/legs/blueleg-clockwisedown.png){data-condition="exo1motor2orientation==clockwisedown"}
:::: select exo1motor2orientation
::: option clockwisedown
::: option counterclockwisedown
::::

**Motor n°2**
![](assets/data/images/legs/blueleg-counterclockwisedown.png){data-condition="exo1motor3orientation==counterclockwisedown"}
![](assets/data/images/legs/blueleg-clockwisedown.png){data-condition="exo1motor3orientation==clockwisedown"}
:::: select exo1motor3orientation
::: option clockwisedown
::: option counterclockwisedown
::::

**Motor n°3**
![](assets/data/images/legs/blueleg-counterclockwisedown.png){data-condition="exo1motor4orientation==counterclockwisedown"}
![](assets/data/images/legs/blueleg-clockwisedown.png){data-condition="exo1motor4orientation==clockwisedown"}
:::: select exo1motor4orientation
::: option clockwisedown
::: option counterclockwisedown
::::

:::::

::::: group-grid 
**Connector**
![](assets/data/images/centerparts/bluepart.png){data-condition="exo1centerpart==bluepart"}
![](assets/data/images/centerparts/yellowpart.png){ data-condition="exo1centerpart==yellowpart"}
:::: select exo1centerpart
::: option bluepart
::: option yellowpart
::::
:::::

For each configuration, launch the corresponding simulation and apply movements to the motors. Observe the movement of the end effector. 
Specifically, analyze the difference between the model's predicted position and the robot's actual position. 
Also observe if and when some combinations of legs and connector configuration lead to mechanical instabilities.

1. What explains the instabilities?
2. What is the most *stable* configuration of the robot you have found (i.e. on which there is no mechanical instabilities)?

#runsofa-button("assets/labs/lab_inversekinematics/lab_inversekinematics.py", "--legsName", "blueleg-direct", "--legsModel", "beam", "--legsPositionOnMotor", "exo1motor1orientation" "exo1motor2orientation" "exo1motor3orientation" "exo1motor4orientation" "--centerPartName" "exo1centerpart")
::::::
:::::::

::::: collapse Inverse Kinematics
## Inverse Kinematics

The goal of the inverse kinematics process is to find the inverse of the previously described relationship, i.e., to compute the motor command positions $\textcolor{red}{\mathbf{u}_{\mathrm{a}}} = \boldsymbol{f}^{-1}(\textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})$. This means determining the motor inputs that result in the desired end-effector position.
There are several challenges in solving this inverse problem:

- **Non-uniqueness of the inverse**: The robot's structure is deformable, and as a result, the inverse relationship $\boldsymbol{f}^{-1}$ is not unique. Different motor positions $\mathbf{u}_{\mathrm{a}}$ can lead to the same end-effector position $\mathbf{y}_{\mathrm{e}}$, depending on the deformation of the robot's legs.
- **Internal forces and lack of analytical model**: The robot's kinematic model is based on internal forces within the deformable structure, and there is no general analytical model for $\boldsymbol{f}(\mathbf{u}_{\mathrm{a}})$. This makes it difficult to derive a closed-form expression for the inverse function, particularly because $\boldsymbol{f}(\mathbf{u}_{\mathrm{a}})$ is highly nonlinear.
- **Nonlinearity of the system**: As mentioned, $\boldsymbol{f}(\mathbf{u}_{\mathrm{a}})$ is a nonlinear function. Therefore, solving for the inverse kinematics requires setting up an optimization process that provides motor positions $\mathbf{u}_{\mathrm{a}}$ to minimize the distance with the desired end-effector position $\mathbf{y}_{\mathrm{e}}$.

To handle these challenges, we typically employ QP optimization techniques. 

![](assets/data/images/lab2-algorithm3.png){width=65%, .center}

With the indirect solving, the optimization presented in equation 20 (in the algorithm above) can be rewriten:

$$
\left \{
\begin{array}{l}
\mathbf{A}d\mathbf{q}^{\mathrm{free}}  =  \mathbf{b} \\
%%
\underset{\boldsymbol{\lambda}_{\mathrm{a}}}{min}
\frac{1}{2}(\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + 
\mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}} 
+ \mathbf{W}_{\mathrm{ea}} \lambda_{\mathrm{a}}  
- \textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})^2 \\
%%
\mathbf{A}d\mathbf{q}^{\lambda} = \mathbf{H}_{\mathrm{a}}^T 
\boldsymbol{\lambda}_{\mathrm{a}} 
\end{array}
\right .
$$

The advantage is that the optimization algorithm corresponds to convex optimization (i.e. Quadratic Programming - QP) 
on small matrices $\mathbf{W}_{\mathrm{ea}}$. If we develop the equation above, we obtain:

$$
\underset{\boldsymbol{\lambda}_{\mathrm{a}}}{min} (
\frac{1}{2} \lambda_{\mathrm{a}} \mathbf{W}_{\mathrm{ea}}^T \mathbf{W}_{\mathrm{ea}} \lambda_{\mathrm{a}}
+ \mathbf{W}_{\mathrm{ea}}^T(\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + 
\mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}}   
- \textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})\lambda_{\mathrm{a}}
)
$$

Remember that the relation between the motors torque and displacement 
is given by $\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}} = \textcolor{red}{\mathbf{u}_{a}}$. Thus, to limit the course of the actuators we can add the following constraint to the QP:

$$
\textcolor{red}{\mathbf{u}_{min}} <=
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}}
<= \textcolor{red}{\mathbf{u}_{max}}
$$

and to constrain the actuators to a position $\textcolor{red}{\mathbf{u}_{0}}$ we can add the following constraint to the QP:

$$
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}}
= \textcolor{red}{\mathbf{u}_{0}}
$$

Each line of the two equations above constrains one of the actuators. 

In the case where the number of end-effectors is smaller than the number 
of actuators, we can have several solutions, but we can add a term $\epsilon \mathbf{W}_{\mathrm{aa}}$ to optimize the 
deformation energy [[Coevoet17]](https://inria.hal.science/hal-01649355/document) and achieve a unique solution.

In the tutorial, we propose you to implement the inverse kinematics using this QP optimization approach in Python. 
You will gain hands-on experience with solving real-world, nonlinear robotics problems. This exercise is valuable as 
it teaches how to handle constraints, explore a numerical optimization method, and deal with the complexities of soft 
robots in a practical context, fostering understanding of robot control and kinematics.

### Hands-on: Implement your own Optimization Program

To solve the inverse kinematics of Emio, we propose to write a Quadratic Program (QP). In this section you will
learn to write the QP system, understand singularity problems, and add constraints to the QP. Finally, you will use a solver
provided by the python library [qpsolvers](https://qpsolvers.github.io/qpsolvers/quadratic-programming.html#qpsolvers.solve_qp).

At this stage, the matrices $\mathbf{W}$ and vectors $d\mathbf{q}^{\mathrm{free}}$ and $\boldsymbol{\delta}(\mathbf{q}^{i-1})$ have been computed, and we want to solve the following optimization problem:

$$
\left \{
\begin{array}{l}
\underset{\boldsymbol{\lambda}_{\mathrm{a}}}{min} (
\frac{1}{2} \lambda_{\mathrm{a}} \mathbf{W}_{\mathrm{ea}}^T \mathbf{W}_{\mathrm{ea}} \lambda_{\mathrm{a}}
+ \mathbf{W}_{\mathrm{ea}}^T(\boldsymbol{\delta}_{\mathrm{e}}(\mathbf{q}^{i-1}) + 
\mathbf{H}_{\mathrm{e}} d\mathbf{q}^{\mathrm{free}}   
- \textcolor{darkgreen}{\mathbf{y}_{\mathrm{e}}})\lambda_{\mathrm{a}}
) \\
%%
\textrm{(optional)} \quad \textcolor{red}{\mathbf{u}_{min}} <=
\boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}}
<= \textcolor{red}{\mathbf{u}_{max}} \\
%%
\textrm{(optional)} \quad \boldsymbol{\delta}_{\mathrm{a}}(\mathbf{q}^{i-1}) + \mathbf{H}_{\mathrm{a}}d\mathbf{q}^{\mathrm{free}} + \mathbf{W}_{\mathrm{aa}}\boldsymbol{\lambda}_{\mathrm{a}}
= \textcolor{red}{\mathbf{u}_{0}} 
\end{array}
\right .
$$

You will be asked to correctly identify the matrices from the system above to implement your own QP. 

:::: exercise

::: collapse {open} Set up Emio 

Take four <span style="color:blue">*blue legs*</span> and put them on each motor, as shown on the image. 
Pay a special attention to the orientation of the legs, it should be (counterclockwise / clockwise / counterclockwise / clockwise). 
Next, attach the <span style="color:blue">*blue connector*</span> at the tip of each leg, then fix
one <span style="color:green">*green marker*</span> on the center of the connector (as shown on the image).

![](assets/data/images/lab2-exercice2-emio.png){width=75% .center}

:::

**Exercise 2:**

Implement your own QP. Open the file `myQP_lab_inversekinematics.py` by clicking the *open* button, and follow the instructions step 
by step (todos):

![](assets/data/images/lab2-exercice2.png)

At each step, try your implementation by clicking the *SOFA* button (for this exercise, we won't connect the robot). Each time
you change the file `myQP_lab_inversekinematics.py`, you will need to close and relaunch the simulation for the changes to be taking into account.

#open-button("assets/labs/lab_inversekinematics/myQP_lab_inversekinematics.py")

#runsofa-button("assets/labs/lab_inversekinematics/lab_inversekinematics.py" "--legsName" "blueleg" "--legsModel" "beam" "--legsPositionOnMotor" "counterclockwisedown" "clockwisedown" "counterclockwisedown" "clockwisedown" "--centerPartName" "bluepart")

::::
:::::

:::::: collapse Models Comparison
## Models Comparison

In this section we propose to observe again the behavior of the *white leg*.
With a new configuration of the robot and using the solver provided by SOFA to solve its inverse kinematics, 
you will compare again the models, and conclude on the advantages and disadvantages of each approach. 

::::: exercise

<!-- Camera Calibration -->
#include(assets/labs/modules/camera_calibration.md)

::: collapse {open} Set up Emio 

Take four *white legs* and put them on each motor as shown on the image.
The orientations are the same as in exercise 1 and 2. 
Next, attach again the <span style="color:blue">*blue connector*</span> at the tip of each leg, and place
one <span style="color:green">*green marker*</span> on the top of the connector.

![](assets/data/images/lab2-exercice3-emio.png){width=75% .center}
:::

**Exercise 3:**

Try the three models with this setup of Emio. Move the effector target in the *x* direction. 

:::: select exo3model
::: option beam
::: option cosserat
::: option tetra
::::

1. What differences do you observe between the models?

Connect the simulation to the real robot and look at the error, i.e. the difference
between the two green and red spheres (you can also use the *Plotting* tab).

2. Which model gives the best simulation to real results?

#runsofa-button("assets/labs/lab_inversekinematics/lab_inversekinematics.py" "--legsName" "whiteleg" "--legsModel" "exo3model" "--legsPositionOnMotor" "counterclockwisedown" "clockwisedown" "counterclockwisedown" "clockwisedown" "--centerPartName" "bluepart")

:::::
::::::
