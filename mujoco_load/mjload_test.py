import mujoco
import mujoco.viewer
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent

stop
xml_path="model/nipedal_platefrorm/mjcf/scene.xml"
xml_path="lerobot-humanoid-model/models/lerobot-humanoide/mjcf/scene.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
   while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
