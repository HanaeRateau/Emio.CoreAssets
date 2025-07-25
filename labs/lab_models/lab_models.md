# Lab Models

::: highlight
##### Overview 

This lab aims to familiarize with certain deformable models used in the soft robotics community, particularly 
beam models and volumetric finite element models. We will briefly describe three models and provide references 
for a deeper understanding. 

This lab consists of understanding these models and their parameters, and experimentally analyzing their behavior. 
To do so, different legs with various geometries are provided with the robot. Each leg leading to different 
mechanical behaviors. You will be able to confront the models with the legs one by one.
:::

<!-- Camera Calibration -->
#include(assets/labs/modules/camera_calibration.md)

:::::: collapse Static Analysis


## Static Analysis

Our approach is grounded in structural mechanics, which enables to account for both the geometric and material properties 
of the robot. To achieve a good level of generality, all models are derived from mechanical energy principles. 
We assume that whatever the configuration, the robot exhibits elastic behavior, meaning the energy considered is potential energy. 
Additionally, we assume that the robot moves slowly enough that kinetic energy can be neglected for the time being. 
This energy-based formulation is particularly well-suited to the weak formulations commonly used in continuum mechanics.  

Whatever the model, we suppose that $\mathbf{q}$ represents the parameters of the motion (motion of the nodes in FEM, 
strains for Cosserat models) and $\mathcal{W}(\mathbf{q})$ is the potential energy of the deformation for the structure. 
The variation of the energy at the structural level can be discretized, meaning it can be decomposed as a sum of the 
energy variations at the level of each element $e$ (or each discrete rod), with $\sigma$ being the stress and $\mathbf{E}$ 
being the strain. 

$$
\partial \mathcal{W}(\mathbf{q}) = \int_{D} \boldsymbol{\sigma}^T  \partial \mathbf{E} \,dV
\approx \sum_e \int_e \boldsymbol{\sigma}^T(\mathbf{q}) \partial \mathbf{E}(\mathbf{q}) \,dV
$$

The internal forces $\mathbf{F}(\mathbf{q}) = \frac{\partial \mathcal{W}(\mathbf{q})}{\partial \mathbf{q} }$ are obtained by the derivative of this deformation potential energy.
We can make a parallel with the principle of virtual work, as the work created by $\partial \mathbf{q}$ needs to be equal to the variation of the strain energy: $\partial \mathcal{W}(\mathbf{q}) = \mathbf{F}(\mathbf{q})^T \partial \mathbf{q}$.

The configuration of the robot, given by $\mathbf{q}$ is obtained by solving the static equilibrium between these internal forces  $\mathbf{F}(\mathbf{q})$ and the external loads $\mathbf{F}_{ext}$, including the efforts exerted by the actuators. We can also add the forces created by the gravity $\mathbf{M}\mathbf{g}$. 

$$
\mathbf{F}(\mathbf{q}) + \mathbf{M}\mathbf{g} + \mathbf{F}_{ext} = \mathbf{0}
$$

In practice $\mathbf{F}(\mathbf{q})$ being non-linear, the computation of the equilibrium position is obtained by iterative resolution (see algorithm 1 below).

![](assets/data/images/lab1-algorithm1.png){width=65%, .center}

To model the various legs and their deformations, we propose three types of modeling of these internal forces:

- an FEM beam model, computed in global coordinates
- a Cosserat rod model, computed in local coordinates (strain space)
- a volume FEM with corotational linear tetrahedral elements

::::::


:::::: collapse Beam Models

## Beam FEM Model

This model was presented in [[Bieze21]](https://hal.science/hal-03028723/file/IJRR_EchelonIII_corrected%20(2).pdf), 
where a soft robot is created from a lattice of deformable beams. In this model, each beam connects two reference frames: 
two nodes whose position is given by a 3-dimensional vector (in the inertial frame), and orientation is given 
by a quaternion.

![](assets/data/images/beammodel.png){width=50%, .center}

Each of these nodes can have a rigid transformation with respect to its reference *DOF* frame. This facilitates 
the creation of lattices where two or more beams can be connected to the same DOF frame.

The proposed beams have a geometric support based on degree 3 splines between their nodes. This allows for good
geometric continuity and the calculation of their reference frame. Indeed, we use a corotational approach which
involves calculating an average frame for each element (here, we calculate it using a frame placed in the middle
of the beam). The deformation movements of each beam are first analyzed in this local frame to calculate the linear 
elastic forces. Then, these forces are placed back into the global frame; if there is no uniqueness between nodes 
and DOFs, we transfer the forces to the DOFs to assemble them with the forces of other beams on the same DOFs.

## Cosserat Model 
This model is an implementation of [[Renda16]](https://ieeexplore.ieee.org/document/7759808), where continuously deformable robots are studied
(see also [[Adagolodjo21]](https://hal.science/hal-03192168/document)). One of the
unique aspects of the model is that the motion parameters are calculated using a local parametric space, 
of the strain type. Thus, we parameterize the deformation movements of the rod by rates of bending, torsion,
elongation, or shearing. This allows us to easily decide not to simulate elongation and shearing, and 
such deformations will not appear on the rods. Each rod thus has 3 to 6 DOFs plus an additional 6 DOFs 
to define its base frame. We can have multiple rods in series starting from the same base. In the basic 
implementation in SOFA, the number of DOFs depends on the number of elementary rods arranged in series. 

In a more advanced implementation, we can use polynomial interpolation along a series of rods to further
reduce the number of DOFs. Once the movements of these DOFs are known, we can reconstruct the configuration 
of the rods through the calculation of the direct geometric model. 
However, since inertial forces and contact forces are more easily expressed in the global frame, this model 
requires bringing these values back through a recurrence from the end of the rod to its base.

## Parameters

[Young's modulus](https://en.wikipedia.org/wiki/Young%27s_modulus) (unit of pressure Pa), also called the modulus of elasticity, is a mechanical property of solid materials that 
quantifies their tensile or compressive stiffness when subjected to lengthwise force. It represents the modulus of 
elasticity for tension or axial compression. Young's modulus is defined as the ratio of the applied stress (force per unit area) 
to the resulting axial strain (displacement or deformation) within the material's linear elastic region.

![](assets/data/images/poissonRatio.svg){width=19% align=right style=margin:15px}

[Poisson's ratio](https://en.wikipedia.org/wiki/Poisson%27s_ratio) measures the deformation (expansion or contraction) of a material in directions perpendicular 
to the applied load. For small deformations, Poisson's ratio is the ratio of transverse elongation to axial compression. 
Most materials have Poisson's ratio values between in 0.0 and 0.5.

Soft materials, like rubber, with a bulk modulus 
much higher than the shear modulus, have Poisson's ratios near 0.5. Open-cell polymer foams have Poisson's ratios 
near zero due to cell collapse under compression. Typical solids have Poisson's ratios in the range of 0.2 to 0.3.

## Hands-on

Now you will play with the simulation and compare the deformations with the ones of the real device. 
You will see which parameters influence the most the beam and Cosserat models, 
and finally estimate the parameters that improve the models' fidelity.


::::: exercise
::: collapse {open} Set up Emio 
Take the <span style="color:blue">*blue leg*</span> and put it on the *motor n°0* (clockwise down as shown on the image). 
Next, attach the <span style="color:grey">*grey cube*</span> at the tip of the leg, then place
two <span style="color:green">*green markers*</span> on the leg: one in the middle of the leg and the second at the tip,
just above the cube.

![](assets/data/images/lab1-exercice1-leg.png){.center width=50%}

:::


**Exercise 1:**

Once setup, choose a model between *beam* and *cosserat* in the drop-down menu below:

:::: select modelsexo1
::: option beam 
::: option cosserat 
::: option tetra
::::

Launch the simulation, make sure that the markers you put on the leg match the 
markers in the simulation, then change the motor's position. Observe the deformation differences 
between the simulation and the real leg. Adjust the parameters until the simulation accurately 
mirrors the deformation of the real device.

1. Which parameters influences the most the deformation of the leg in this configuration?  
2. Which parameters did you choose to change and why?

After determining the appropriate values, save them in the file `myparameters.py`. Click the *open* button, enter 
the values, and save the changes (ctrl + s).

#open-button("assets/labs/lab_models/myparameters.py")

#runsofa-button("assets/labs/lab_models/lab_models.py", "blueleg", "modelsexo1")

:::::
::::::

:::::: collapse Corotational Volume FEM Model

## Corotational Volume FEM Model

The corotational model aims to separate rigid deformations from purely elastic deformations. 
It is particularly useful for simulations where objects undergo significant rotations but the elastic 
deformations are relatively small. The corotational model relies on a principle of deformation separation: 
the total displacement of an element is decomposed into a rigid rotation followed by a purely elastic 
deformation. For the rigid rotation, the rotational part of the nodal displacements of a tetrahedron
is extracted using a technique such as Singular Value Decomposition (SVD) to obtain the rotation matrix. 
For elastic deformations, once the rotation is extracted, the remaining deformations are treated as small 
deformations within the framework of linear elasticity in a 'local' frame associated with each element. 
Finally, the contributions of the individual elements are assembled into the global system of the equation
of motion.

The corotational model allows for the use of a simple linear elasticity formulation while handling large
rotations. It is computationally efficient because it avoids the complex calculations required in other 
full nonlinear approaches. In the context of soft robotics, it is useful for managing significant rotations 
of moving structures that undergo relatively small elastic deformations.

## Hands-on

Now, you will experiment with the volume model and compare it to the beam and Cosserat models using a new leg (the white one). 
This exercise will help you understand the advantages and disadvantages of each method, as well as determine 
the best scenarios for their application.

Note that, similar to many other numerical methods, with FEM, there are concerns about the quality of the discretization, 
in addition to the resolution algorithm itself. The principle is that as the mesh becomes finer, 
the solutions should converge to the solution of the original partial differential equation.
Note that it is common to use a smaller value for the Young's modulus when using a coarse mesh.
See the below image for an example of the influence of the mesh resolution on the solution.
                                                          
|                                                                        ![](assets/data/images/volumemodel.png){width=75%}                                                                                                                                                                                                           |                                                                                                                            
|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| **In this simulation the leg is attached at its base and is subjected to gravity. The illustration shows a comparison between the beam model (represented by frames) and the volume model (the blue meshes) with different mesh size factor: (a) 114 nodes, (b) 252 nodes, (c) 1932 nodes, (d) 11008 nodes, (e) 41692 nodes.** |

::::: exercise

:::: collapse {open} Set up Emio 

Take the *white leg* and put it on the *motor n°0* (clockwise down as shown on the image). 
Next, attach the <span style="color:grey">*grey cube*</span> at the tip of the leg, then place
two <span style="color:green">*green markers*</span> on the leg: one in the middle of the leg and the second at the tip,
just above the cube. 

![](assets/data/images/lab1-exercice2-leg.png){width=50% .center}

::::

**Exercise 2:**

Run the following simulation and observe the behavior of the leg.

:::: select modelsexo2
::: option tetra 
::: option beam 
::: option cosserat 
::::

Try the simulation using the *beam*, *cosserat*, and *tetra* models. 

What differences do you observe?

#runsofa-button("assets/labs/lab_models/lab_models.py", "whiteleg", "modelsexo2")

:::::

::: highlight
#icon("info-circle")  **Note:** Beam models (local & global) for calculating linear elasticity in large displacements present several 
advantages and disadvantages.
:::
::::::

:::::: collapse Comparison of Approaches
## Comparison of Approaches

::::: exercise
**Question 1:**
:::: quiz
::: question What are the advantages of using beam models, compared to volume models?
Beam models simplify calculations compared to full three-dimensional models, reducing computation time 
and resources needed.
They are widely used in many civil and mechanical engineering applications, facilitating the analysis 
and design of structures such as bridges and buildings. In our case, beam models are well-suited to 
predict the behavior of a continuum robot, especially when the model is used for control purposes.
:::
::::
:::::

::::: exercise
**Question 2:**
:::: quiz
::: question What are the disadvantages of using beam models, compared to volume models?
Beam models rely on assumptions that may not be valid for all situations, such as the assumption that 
cross-sections are undeformable and remain flat and perpendicular to the neutral axis (Bernoulli-Euler 
hypothesis). Timoshenko's theory accounts for shear deformation, but not all deformations of the 
cross-section are considered.
Beams are often not appropriate for structures with complex geometries or significant three-dimensional 
effects, where local deformations and stresses play a crucial role.
:::
::::
:::::

::::: exercise
**Question 3:**
:::: quiz
::: question What can you tell about local and global parametrization?
Using a local parametrization (rates of bending, torsion, elongation) for beams allows for a more 
intuitive and compact modeling of internal deformations while parameterizing movement in a linear space. 
This approach can offer faster calculations and be closer to sensor information that could be placed on 
the beam (which would locally measure bending, torsion, or elongation). This can be compared to the
parametrization of articulated rigid robots in local coordinates.

Using a global parameterization (position of the nodes in space) simplifies the modeling of complex 
structures that connect a mesh of beams. This type of parameterization is often easier to implement 
for global structural analyses, contact management, or multi-physics coupling with other phenomena.
:::
::::
:::::

::::: exercise
**Question 4:**
:::: quiz 
::: question Which model is the best suited for the blue leg? 
- [X] Cosserat
- [ ] Volume
- [X] Beam
:::
::::
:::::

::::::