import os
import sys

sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/../../")


def createScene(rootnode):
    from utils.header import addHeader, addSolvers
    from parts.emio import Emio
    from parts.controllers.assemblycontroller import AssemblyController

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    addSolvers(simulation)

    rootnode.dt = 0.01
    rootnode.gravity = [0., -9810., 0.]
    rootnode.VisualStyle.displayFlags.value = ["hideBehavior"]

    # Add Emio to the scene
    emio = Emio(name="Emio",
                legsName=["blueleg"],
                legsModel=["beam"],
                legsPositionOnMotor=["counterclockwisedown"],
                centerPartName="bluepart",
                centerPartType="rigid",
                extended=True)
    if not emio.isValid():
        return

    simulation.addChild(emio)
    emio.attachCenterPartToLegs()
    emio.addObject(AssemblyController(emio))

    # Add effector
    emio.effector.addObject("MechanicalObject", template="Rigid3", position=[0, 0, 0, 0, 0, 0, 1])
    emio.effector.addObject("RigidMapping", index=0)

    # Target
    effectorTarget = modelling.addChild('Target')
    effectorTarget.addObject('EulerImplicitSolver', firstOrder=True)
    effectorTarget.addObject('CGLinearSolver', iterations=50, tolerance=1e-10, threshold=1e-10)
    effectorTarget.addObject('MechanicalObject', template='Rigid3',
                             position=[0, -150, 40, 0, 0, 0, 1],
                             showObject=True, showObjectScale=20)

    # Add inverse components and GUI
    emio.addInverseComponentAndGUI(effectorTarget.getMechanicalState().position.linkpath)

    # Components for the connection to the real robot 
    emio.addConnectionComponents()

    return rootnode
