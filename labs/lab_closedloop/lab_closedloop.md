# Lab Closed Loop

::: highlight
##### Overview

This lab is dedicated to control. Its goals are to make you understand:

1. When and why a closed loop control scheme is required for a given application.
2. What are the effects of the different parameters of the controller.
3. The limitation of the proposed approach and what has to be done to overcome it.
4. Give some insight of what could be achieved with more time.
:::

::: collapse {open} Set up Emio  for the Lab
## Set up Emio

In this lab session, we will use only the following configuration: Emio with the <span style="color:blue">*blue legs*</span>,
the <span style="color:blue">*blue connector*</span>, and the <span style="color:green">*green marker*</span>:

![](assets/data/images/lab2-exercice2-emio.png){width=75% .center}

In the provided simulation, we now consider the effect of a disturbance acting at the tip of the robot. This force can be simulated in the provided scene.
:::

::::: collapse Open Loop Control
## Open Loop Control

**Open loop accuracy.** 
The idea of inverse model based control is to rely on Inverse Kinematic method to simplify the control loop. The principle is depicted in the following figure:

|  ![](assets/data/images/OL-principle.png)   | 
|:-------------------------------------------:|
| **Inverse-model-based Open loop principle** |

If we consider the following quasi-static model:
$$q_{k+1}=q_k + \delta_{free}(q_k)+W(q_k)\lambda_k$$
where $q_k$ is the effector positon and $\lambda_k$ the efforts. 
If we consider the following linearising control:
$$\lambda_k=W(q_k)^{-1}(q_t-q_k - \delta_{free}(q_k)))$$
the results is given by:
$$q_{k+1}=q_t$$

For the implementation, instead of using the inverse of the matrix we will prefer to solve an optimisation problem (as seen in previous labs) which allows to solve the problem even if $W$ is not square and allows us to add constrains.

In order to implement that controller, one needs to write the inverse problem `assets/labs/lab_closedloop/myQP_lab_closedloop_.py`  and to write the open loop controller `assets/labs/lab_closedloop/myControl.py`. The connexions are depicted here:

|  ![](assets/data/images/OL-implementation.png)   | 
|:------------------------------------------------:| 
| **Inverse-model-based Open loop implementation** |


:::: exercise
**Exercise 1:**

Open the script `assets/labs/lab_closedloop/myControl.py` by clicking on the *open* button below, and have
a glance at the content. Read the descriptions and try to understand what the script does 
and how it works. 

#open-button("assets/labs/lab_closedloop/myControl.py")
::::

:::: exercise
**Exercise 2:**

Write the QP optimization problem. 

#open-button("assets/labs/lab_closedloop/myQP_lab_closedloop.py")
::::

:::: exercise
**Exercise 3:**

Choose some target points and try to reach them. Then: 
1. Observe the error on the plots.
2. Conclude about the accuracy of the open loop controller.
3. Try to reach points outside the workspace of the robot.

#runsofa-button("assets/labs/lab_closedloop/lab_closedloop.py")
::::

:::::

::::: collapse Proportional Controller Closed Loop 
## Proportional Controller Closed Loop

**Inverse model-based control principle.** 
The problem of the open loop controller is that the model is just an approximation of the reality. Some uncertainties and disturbances makes this type of control innacurate and sensitive to the environement.

To solve this problem, on can use to close the control loop by exploiting a sensor feedback in the control scheme:

|   ![](assets/data/images/CL-principle.png)    | 
|:---------------------------------------------:|
| **Inverse-model-based Closed loop principle** |

This schemes allows to take new actions according to what happens in reality.
We will keep the idea of inverse model based control to simplify the behavior of the robot:
$$q_{k+1}=q_t + f(d_k)$$
where $d_k$ is the disturbance (badly modeled things and interactions with the environment) that is represented by a force applied at the effector.

The controller that we will implement will check the error between the user target and sensor feedback and will add a shift to $q_t$ each time an error is detected (proportial action in the direction of the error):
$$ q_t = q_{user} + k_p * \varepsilon $$

This controller is composed of a feedforward $q_{user}$ and a feedback part $k_p * \varepsilon$. $varepsilon$ is the error between the target provided by the user and the current location of the effector.

|   ![](assets/data/images/CL-implementation.png)    | 
|:--------------------------------------------------:| 
| **Inverse-model-based Closed loop implementation** |

::: highlight
#icon("warning") **Warning:** Be careful with these controllers as they can be unstable if they are badly implemented or tunned.
**A good practice is to check in simulation first.**
:::

:::: exercise

**Exercise 4:**

1.  Try to implement this type of control **in simulation first** and then test it on the robot, observe the error. 
2.  Try to disturb the robot during the experiment, observe the reaction of the robot for different values of the control gain. 
3.  Try to reach points outside the workspace of the robot. 
4.  Try to increase the control gain. 
5.  Conclude on what to do next. 
  
#open-button("assets/labs/lab_closedloop/myControl.py")

#runsofa-button("assets/labs/lab_closedloop/lab_closedloop.py")
::::

:::::



::::: collapse Integral Controller Closed Loop 

## Integral Controller Closed Loop 

**Integral action principle.** 
In order to remove completely the steady state error, it is usual to incorporate an integral action. For sake of 
simplicity, we propose to implement this integral action alone (remove the previous controller).

The principle is to give a velocity to the Qp target $q_t$ in the direction of the error so that the $q_t$ will 
'search' for the right value that compensate the steady state error. The initial value of the $q_t$ should be an 
admissible value for the robot otherwise the robot might go crazy.

:::: exercise

**Exercise 5:**

1.  Try to implement this type of control **in simulation first** and then test it on the robot, observe the error. 
2.  Try to disturb the robot during the experiment, observe the reaction of the robot for different values of the control gain. 
3.  Try to reach points outside the workspace of the robot. 
4.  Try to increase the control gain. 
5.  Conclude on what to do next. 
  
#open-button("assets/labs/lab_closedloop/myControl.py")

#runsofa-button("assets/labs/lab_closedloop/lab_closedloop.py")
::::

:::::


::::: collapse Improve the Integral Controller

## Improve the Integral Controller 

**Making the closed loop working.**
The main problem you encountered in the last step is called integrator windup.

It happens when the control cannot make the error converge to 0 because of physical limitation (workspace of the robot). 

To avoid that we need to detect the windup effect and prevent it to diverge. An easy way to do that is to compute the 
distance between the theoretical effector position and the target provided by the controller to the inverse model. 
When this difference is too big, recompute the target point to a fixed distance but the same relative direction 
from the effector position.

:::: exercise
**Exercise 6:**

1.  Try to implement this type of control **in simulation first**. 
2.  Try to reach points outside the workspace of the robot. 
3.  If everything goes well, try on the robot. 
4.  Write an open loop controller that relies on the QP solver. 
  
#open-button("assets/labs/lab_closedloop/myControl.py")

#runsofa-button("assets/labs/lab_closedloop/lab_closedloop.py")
::::
:::::