# Introduction

::: highlight
##### Overview 

Before we begin the lab sessions, let's take some time to explore Emio and its software GUI (Graphical User Interface).
:::

:::: collapse Emio
## Emio

Emio is a parallel deformable robot developed by [Compliance Robotics](https://compliance-robotics.com/). It features a structure composed of four servo motor-actuated deformable 
legs connected together. The robot includes various sets of legs and connectors, conveniently stored in a drawer,
as well as a depth camera. 

|                                      ![](assets/data/images/emio-drawer-camera.png)                                      |                                           ![](assets/data/images/emio-motors.png)                                            | 
|:-------------------------------------------------------------------------------------------------------------------:|:-----------------------------------------------------------------------------------------------------------------------:| 
| **(1) The drawer in which you will find Emio's accessories. (2) The depth camera that can be oriented up or down.** | **The four motors with their corresponding identification number (n°0, n°1, n°2, n°3), which we'll use in the following labs.** |

| ![](assets/data/images/accessories.jpg){width="75%"} | 
|:----------------------------------------------------:|
|                   **Accessories**                    |


As depicted in the images above, the robot features two distinct configurations. In the first, or _extended_
configuration, Emio's legs point downward, enabling it to perform pick-and-place tasks. 
The second configuration is _compact_, with the legs pointing upward, making it easier to interact with the robot.

Emio comes with a software to pilot and program the robot. You will find a USB cable in the drawer to connect the robot 
to your computer. The robot also has a power supply and a switch. 

|          ![](assets/data/images/emio-connections.png){width="50%"}           | 
|:----------------------------------------------------------------------------:| 
| **(1) USB connection. (2) Power connection. (3) Switch to power the robot.** | 


Each motor is equipped with a drum and a cap for connecting a leg. 
You can take a leg from the drawer and attach it to a motor. 
Simply rotate the cap until you can set the leg into the desired orientation (as shown in the images). 
The zero position of the motor is indicated by the orange marker pointing upward.

|            ![](assets/data/images/motor-cap1.png)             |        ![](assets/data/images/motor-cap2.png)        |              ![](assets/data/images/motor-cap3.png)               |         ![](assets/data/images/motor-cap4.png)         | 
|:-------------------------------------------------------------:|:----------------------------------------------------:|:-----------------------------------------------------------------:|:------------------------------------------------------:|
| **(1) Motor's zero position (orange marker pointing upward)** | **(2) Rotate the cap until the leg can be attached** | **(3) Once the leg is in place, rotate the cap again to lock it** | **(4) The leg can be adjusted to different positions** |      
                             
| ![](assets/data/images/legs/blueleg-clockwiseup.png)    | ![](assets/data/images/legs/blueleg-counterclockwiseup.png) | ![](assets/data/images/legs/blueleg-clockwisedown.png)   | ![](assets/data/images/legs/blueleg-counterclockwisedown.png)   | 
|:--------------------------------------------------:|:------------------------------------------------------:|:---------------------------------------------------:|:----------------------------------------------------------:|
|         **(1) The blue leg clockwise up**          |       **(2) The blue leg counter clockwise up**        |         **(3) The blue leg clockwise down**         |        **(4) The blue leg counter clockwise down**         |

::: highlight
#icon("info-circle") **Note:** Throughout the lab sessions, you will be instructed to configure Emio into specific setups. Simply follow the 
provided instructions by clicking on *Set up Emio*. We will use colors to refer to the legs and connectors, and numbers to identify the motors.
:::
::::

:::: collapse Software GUI
## Software GUI

We use the simulation framework [SOFA](https://www.sofa-framework.org/) to model, simulate, and solve the inverse kinematics of Emio. The GUI, developed by Compliance Robotics on top of SOFA, 
enables intuitive piloting and programming of the robot.

The GUI features two main components: a simulation 3D viewport, where you can visualize the simulated robot, and a series of tabs that
provide various functionalities. These tabs allow you to program the robot or directly control its movements:

1. **My Robot**: Access information and settings related to the simulation and Emio.
2. **Move**: Directly control the TCP target or adjust the position of motors.
3. **Program**: Develop robot programs by adding waypoints on a timeline that corresponds to simulation time.
4. **Plotting**: Some labs include plotting data for analysis purposes.

| ![](assets/data/images/emio-simulationgui.png) |
|:----------------------------------------------:|
|           **Screenshot of the GUI**            |

The *Simulation* button toggles the connection between the simulation and the physical robot. In simulation mode, the robot remains
stationary, providing a safe environment to test your programs. Before deploying your programs on the real robot, ensure they are
thoroughly tested in simulation mode to avoid any potential issues.

|![](assets/data/images/simulation-toggle.png){width="20%" class="center"}|
|:-----------------------------------------------------------------------:|
|                 **Simulation / Robot switch button**                    |

::: highlight
#icon("info-circle")  **Note:** As previously mentioned, throughout the lab sessions, you will receive instructions to set up Emio in specific configurations.
Some exercises use the simulation software described above. You will need to select options to configure Emio; 
these selections affect the simulation setup. Make sure that the simulation matches the real robot.
:::
::::

:::: collapse Let's Try Emio

## Let's Try Emio

**1. Set up Emio:**
Take four <span style="color:blue">*blue legs*</span> and put them on each motor, as shown on the following image. 
Pay a special attention to the orientation of the legs, it should be from n°0 to n°3: counterclockwise, clockwise, counterclockwise, clockwise. 
Next, attach the <span style="color:blue">*blue connector*</span> at the tip of each leg. Plug the robot's USB cable to your computer.

![](assets/data/images/lab2-exercice2-emio.png){width="50%" class="center"}

**2. Run the simulation:**
Launch the simulation GUI by clicking the *SOFA* button below.   
On the GUI, click the *Play* button (center top) to start the simulation. 

|![](assets/data/images/play-pause-buttons.png){width="30%" class="center"}|
|:------------------------------------------------------------------------:|
|                 **Play / Pause and Step buttons**                        |

Once you're ready, toggle the *Simulation* button, which is above the *Play* button, to send 
the command to the robot.

**3. Pilot the robot:**
Navigate to the *Move* tab, and use the sliders to move the effector's target.

#runsofa-button("assets/labs/introduction/introduction.py", "-ln", "blueleg", "-lm", "beam", "-lp", "counterclockwisedown", "clockwisedown", "counterclockwisedown", "clockwisedown", "-cn", "bluepart")

::: collapse Troubleshooting

1. If you connect the robot to your computer and still get the following error message `[ERROR] No serial port found with manufacturer = FTDI`. 
Try to install the [FTDI drivers](https://ftdichip.com/drivers/vcp-drivers/).

2. On Ubuntu, when trying to connect the real robot, if you get a `[Errno 13] Permission denied: '/dev/ttyUSB0'` message. Run the following command in a terminal:
    ```console
    sudo chmod 777 /dev/ttyUSB0
    ```
   Make sure that the name of the USB port matches the one from the error message.
   
:::

::::