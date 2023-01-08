# Copyright 2022 CNRS
# Author: Florent Lamiraux
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:

# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from math import pi, sqrt
from hpp.corbaserver import loadServerPlugin, shrinkJointRange
from hpp.corbaserver.manipulation import Robot, \
    createContext, newProblem, ProblemSolver, ConstraintGraph, \
    ConstraintGraphFactory, Rule, Constraints, CorbaClient, SecurityMargins
from hpp.gepetto.manipulation import ViewerFactory
from agimus_demos.tools_hpp import RosInterface
from hpp.corbaserver import wrap_delete

class Box:
    urdfFilename = "package://agimus_demos/franka/manipulation/urdf/small_box.urdf"
    srdfFilename = "package://agimus_demos/franka/manipulation/srdf/small_box.srdf"
    rootJointType = "freeflyer"

connectedToRos = False
try:
    import rospy
    Robot.urdfString = rospy.get_param('robot_description')
    print("reading URDF from ROS param")
    connectedToRos = True
except:
    print("reading generic URDF")
    from hpp.rostools import process_xacro, retrieve_resource
    Robot.urdfString = process_xacro\
      ("package://agimus_demos/franka/manipulation/urdf/demo.urdf.xacro")
Robot.srdfString = ""

defaultContext = "corbaserver"
loadServerPlugin (defaultContext, "manipulation-corba.so")
newProblem()

robot = Robot("robot", "pandas", rootJointType="anchor")
shrinkJointRange(robot, [f'pandas/panda1_joint{i}' for i in range(1,8)],0.95)
ps = ProblemSolver(robot)

ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
ps.setParameter('SimpleTimeParameterization/order', 2)
ps.setParameter('SimpleTimeParameterization/maxAcceleration', .5)
ps.setParameter('SimpleTimeParameterization/safety', 0.95)

# Add path projector to avoid discontinuities
ps.selectPathProjector ("Progressive", .05)
ps.selectPathValidation("Graph-Progressive", 0.01)
vf = ViewerFactory(ps)
vf.loadRobotModel (Box, "box")
robot.setJointBounds('box/root_joint', [-1., 1., -1., 1., -0.8, 1.5])
print("Part loaded")

robot.client.manipulation.robot.insertRobotSRDFModel\
    ("pandas", "package://agimus_demos/franka/manipulation/srdf/demo.srdf")

q0 = [0, -pi/4, 0, -3*pi/4, 0, pi/2, pi/4, 0.001, 0.001, -0.15, 0.2, 0.87, 0, sqrt(2)/2, 0, sqrt(2)/2]

r = robot.rankInConfiguration['box/root_joint']
q0[r+2] = 0.86
q0[r+3:r+7] = [0, sqrt(2)/2, 0, sqrt(2)/2]

if connectedToRos:
    ri = RosInterface(robot)
    q = ri.getCurrentConfig(q0)
else:
    q = q0[:]

handles = ps.getAvailable('handle')
grippers = ps.getAvailable('gripper')
boxContactSurfaces = list(filter(lambda s:s.startswith('box/'),
                                 ps.getAvailable('RobotContact')))
envContactSurfaces = list(filter(lambda s:s.startswith('pandas/'),
                                 ps.getAvailable('RobotContact')))
graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers(grippers)
factory.setObjects(["box",], [handles], [boxContactSurfaces])
factory.environmentContacts(envContactSurfaces)
factory.generate()
SecurityMargins.separators.append('_')
sm = SecurityMargins(ps, factory, ["pandas/panda1", "box"])
sm.setSecurityMarginBetween("pandas/panda1", "box", 0.03)
sm.setSecurityMarginBetween("pandas/panda1", "pandas/panda1", 0)
sm.defaultMargin = 0.05
sm.apply()
## deactivate collision between gripper and contact surfaces on transition that
#  go to or come from contact.
edges = ['pandas/panda1_gripper > box/handle | f_12',
         'pandas/panda1_gripper < box/handle | 0-0_21',
         'pandas/panda1_gripper > box/handle | f_23',
         'pandas/panda1_gripper < box/handle | 0-0_32',
         'pandas/panda1_gripper > box/handle2 | f_12',
         'pandas/panda1_gripper < box/handle2 | 0-1_21',
         'pandas/panda1_gripper > box/handle2 | f_23',
         'pandas/panda1_gripper < box/handle2 | 0-1_32',]
joints = sm.gripperToJoints['pandas/panda1_gripper']
for edge in edges:
    for j in joints:
        graph.setSecurityMarginForEdge(edge, 'universe', j, 0)
# Lock gripper in open position.
ps.createLockedJoint('locked_finger_1', 'pandas/panda1_finger_joint1', [0.035])
ps.createLockedJoint('locked_finger_2', 'pandas/panda1_finger_joint2', [0.035])
graph.addConstraints(graph=True,
                     constraints = Constraints(numConstraints =
                        ['locked_finger_1', 'locked_finger_2']))
graph.initialize()

res, q_init,err = graph.applyNodeConstraints('free', q)

q_goal = q_init[:]
q_goal[r+1] = 0

ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
