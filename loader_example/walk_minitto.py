import crocoddyl as croc
import numpy as np
from humanoids_loader import loadBipedalPlateform
from V0_walk_param import WalkV0Params
from V0_sidewalk_param import SideWalkV0Params
# Local imports
import sobec
import sobec.walk_without_think.plotter
import os

import meshcat
from pinocchio.visualize import MeshcatVisualizer


### load 6d model
walkParams = WalkV0Params()

cwd=os.getcwd()
robot = loadBipedalPlateform()


assert len(walkParams.stateImportance) == robot.model.nv * 2
assert len(walkParams.stateTerminalImportance) == robot.model.nv * 2

contactPattern = walkParams.contactPattern




viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
viz.viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
viz.clean()
viz.loadViewerModel(rootNodeName="universe")
viz.display(robot.x0[: robot.model.nq])



# #####################################################################################
# ### DDP #############################################################################
# #####################################################################################

ddp = sobec.wwt.buildSolver(robot, contactPattern, walkParams, solver='FDDP')
problem = ddp.problem
x0s, u0s = sobec.wwt.buildInitialGuess(ddp.problem, walkParams)
ddp.setCallbacks([croc.CallbackVerbose(), croc.CallbackLogger()])

Us=np.array(u0s)


croc.enable_profiler()
ddp.solve(init_xs=x0s, init_us=u0s, maxiter=800, init_reg=1e-6, is_feasible=False)

sol = sobec.wwt.Solution(robot, ddp)
plotter = sobec.wwt.plotter.WalkPlotter(robot.model, robot.contactIds)
plotter.setData(contactPattern, sol.xs, sol.us, sol.fs0)
# plotter.plotJointTorques()
# plt.show()
sol = sobec.wwt.Solution(robot, ddp)




while input("Press q to quit the visualisation") != "q":
    viz.play(np.array(ddp.xs)[:, : robot.model.nq], walkParams.DT)


stop

import numpy as np
import time
import imageio.v2 as imageio

fps = 1.0 / walkParams.DT

with imageio.get_writer(
    "/tmp/lerobothumanoid.mp4",
    fps=fps,
    format="FFMPEG",
    codec="libx264",
    pixelformat="yuv420p",
    macro_block_size=None,   # avoids “size not divisible by 16” problems
) as w:
    for x in ddp.xs:
        viz.display(x[:robot.model.nq])
        time.sleep(0.05)
        frame = np.asarray(viz.viewer.get_image())[:, :, :3]  # RGB
        w.append_data(frame)

print("wrote /tmp/lerobothumanoid.mp4")