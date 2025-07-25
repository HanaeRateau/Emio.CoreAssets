import os
import subprocess


def partTest(filename, argv=None, nbIterations=10):
    if os.path.isfile(filename):
        args = ["runSofa", filename, "-l", "SofaPython3,SofaImGui", "-g", "batch", "-n", str(nbIterations)]
        if argv is not None:
            args += ["--argv", argv]
        result = subprocess.run(args, capture_output=True, text=True)
        result.check_returncode()
        print(result.args)
        print(result.stdout)
        assert str(nbIterations) + " iterations done" in result.stdout
        print(result.stderr)
        assert "[ERROR]" not in result.stderr
        # assert "[WARNING]" not in result.stdout # todo: fix warning "Implicit object registration is deprecated since v24.12. Check #4429 for more information."
    else:
        raise FileNotFoundError(filename)
