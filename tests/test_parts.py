from tests import partTest
import os


def test_leg():
    partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/leg.py")
    for leg in ["blueleg", "whiteleg"]:
        for model in ["cosserat", "beam", "tetra"]:
            partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/leg.py", "-n," + leg + ",-m," + model)


def test_centerpart():
    partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/centerpart.py")


def test_emio():
    partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/emio.py", "--no-connection")
    for leg in ["blueleg", "whiteleg"]:
        for model in ["beam", "tetra"]:
            partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/emio.py", "-ln," + leg + ",-lm," + model + ",--no-connection")


def test_motor():
    partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/motor.py")


def test_camera():
    partTest(os.path.dirname(os.path.abspath(__file__)) + "/../parts/camera.py")

