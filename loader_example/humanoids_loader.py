
import pinocchio as pin
import numpy as np
import sobec
import os
import yaml
from yaml.loader import SafeLoader
from example_parallel_robots.loader_tools import completeRobotLoader
from toolbox_parallel_robots.mounting import closedLoopMountProximal,closedLoopMountScipy,closedLoopMountCasadi
pin.SE3.__repr__ = pin.SE3.__str__

CWD = os.path.dirname(os.path.abspath(__file__))



import meshcat
from pinocchio.visualize import MeshcatVisualizer














def loadBipedalPlateform():
    model,constraint_models,actuation_model,visual_model,colision_model = completeRobotLoader(CWD + '/../urdf/bipedal_plateform/urdf', freeflyer=True)

    for c in constraint_models:
        c.corrector.Kp[:]=np.ones(6)*10
        c.corrector.Kd[:]=np.ones(6)*2
    model.armature[actuation_model.mot_ids_v]=[3400*8*1e-7,1477*18*1e-7,1477*18*1e-7,1477*6*1e-7,1477*6*1e-7,1477*18*1e-7]*2 # Note : adjust the armature values according to the robot's specifications

    data=model.createData()
    cdata=[c.createData() for c in constraint_models]

    
    entraxe=-0.105
    foot_id=[model.getFrameId(f) for f in ["foot_right","foot_left"]]
    Lcontact_frame =[]
    for fid in foot_id:
        f=model.frames[fid]
        if "right" in f.name:
            placement=pin.SE3.Identity()
            placement.translation[1]=entraxe
            placement.rotation=pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])
        else:
            placement=pin.SE3.Identity()
            placement.translation[1]=-entraxe
            placement.rotation= pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])


    base_height= 0.65
    Lcontact_frame =[]
    for fid in foot_id:
        f=model.frames[fid]
        if "left" in f.name:
            placement=pin.SE3.Identity()
            placement.translation[1]=-entraxe
            placement.rotation=pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])
        else:
            placement=pin.SE3.Identity()
            placement.translation[1]=entraxe
            placement.rotation= pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])

    
    torso_name="torso"
    torso_placement=pin.SE3.Identity()
    torso_placement.translation[2]=base_height
    torso_placement.translation[0]=0.0
    id_torso=model.getFrameId(torso_name)
    Lcontact_frame.append([model.frames[id_torso],torso_placement])

    nconstraint_model=[]
    for f1,placement in Lcontact_frame[:]:
        nconstraint_model.append(pin.RigidConstraintModel(pin.ContactType.CONTACT_6D,model,f1.parentJoint,f1.placement,0,placement,pin.ReferenceFrame.LOCAL))

    ncdata=[c.createData() for c in nconstraint_model]
    

    q0 = closedLoopMountProximal(model,data,constraint_models+nconstraint_model[:],cdata+ncdata[:])


    viz = MeshcatVisualizer(model, visual_model, visual_model)
    viz.viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    viz.clean()
    viz.loadViewerModel(rootNodeName="universe")
    q0 = closedLoopMountProximal(model,data,constraint_models+nconstraint_model[:],cdata+ncdata[:])
    viz.display(q0)




    model.referenceConfigurations["half_sitting"] = q0

    print([f.name for f in model.frames])
    idfoot=[model.getFrameId(n) for n in ["foot_left","foot_right"]]
    for idf in idfoot:
        model.frames[idf].name += "48646"



    robot = sobec.wwt.RobotWrapper(model, contactKey="48646", closed_loop=True)
    robot.collision_model = visual_model
    robot.visual_model = visual_model
    robot.actuationModel = actuation_model
    robot.loop_constraints_models = constraint_models
    assert len(robot.contactIds) == 2


    return(robot)







def loadHumanoidRobot():
    model,constraint_models,actuation_model,visual_model,colision_model = completeRobotLoader(CWD + '/../urdf/lerobot_humanoide/urdf', freeflyer=True)

    for c in constraint_models:
        c.corrector.Kp[:]=np.ones(6)*10
        c.corrector.Kd[:]=np.ones(6)*2
    model.armature[actuation_model.mot_ids_v]=[3400*8*1e-7,1477*18*1e-7,1477*18*1e-7,1477*6*1e-7,1477*6*1e-7,1477*18*1e-7]*2

    data=model.createData()
    cdata=[c.createData() for c in constraint_models]

    
    entraxe=-0.105
    foot_id=[model.getFrameId(f) for f in ["foot_right","foot_left"]]
    Lcontact_frame =[]
    for fid in foot_id:
        f=model.frames[fid]
        if "right" in f.name:
            placement=pin.SE3.Identity()
            placement.translation[1]=entraxe
            placement.rotation=pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])
        else:
            placement=pin.SE3.Identity()
            placement.translation[1]=-entraxe
            placement.rotation= pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])


    base_height= 0.65
    Lcontact_frame =[]
    for fid in foot_id:
        f=model.frames[fid]
        if "left" in f.name:
            placement=pin.SE3.Identity()
            placement.translation[1]=-entraxe
            placement.rotation=pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])
        else:
            placement=pin.SE3.Identity()
            placement.translation[1]=entraxe
            placement.rotation= pin.utils.rotate('z',np.deg2rad(0))
            Lcontact_frame.append([f,placement.copy()])

    
    torso_name="torso"
    torso_placement=pin.SE3.Identity()
    torso_placement.translation[2]=base_height
    torso_placement.translation[0]=0.0
    id_torso=model.getFrameId(torso_name)
    Lcontact_frame.append([model.frames[id_torso],torso_placement])

    nconstraint_model=[]
    for f1,placement in Lcontact_frame[:]:
        nconstraint_model.append(pin.RigidConstraintModel(pin.ContactType.CONTACT_6D,model,f1.parentJoint,f1.placement,0,placement,pin.ReferenceFrame.LOCAL))

    ncdata=[c.createData() for c in nconstraint_model]
    

    q0 = closedLoopMountProximal(model,data,constraint_models+nconstraint_model[:],cdata+ncdata[:])



    viz = MeshcatVisualizer(model, visual_model, visual_model)
    viz.viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
    viz.clean()
    viz.loadViewerModel(rootNodeName="universe")
    q0 = closedLoopMountProximal(model,data,constraint_models+nconstraint_model[:],cdata+ncdata[:])
    viz.display(q0)




    model.referenceConfigurations["half_sitting"] = q0

    print([f.name for f in model.frames])
    idfoot=[model.getFrameId(n) for n in ["foot_left","foot_right"]]
    for idf in idfoot:
        model.frames[idf].name += "48646"



    robot = sobec.wwt.RobotWrapper(model, contactKey="48646", closed_loop=True)
    robot.collision_model = visual_model
    robot.visual_model = visual_model
    robot.actuationModel = actuation_model
    robot.loop_constraints_models = constraint_models
    assert len(robot.contactIds) == 2


    return(robot)


