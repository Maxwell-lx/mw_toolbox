import pybullet as p


class pybullet_robot():
    def __init__(self, physics_client, id):
        self.id = id
        self.name = physics_client.getBodyInfo(id)
        self.joint_num = physics_client.getNumJoints(id)
        self.link_num = self.joint_num
        self.visual_shape_data = physics_client.getVisualShapeData(id)
        self.joints = []
        self.links = []
        for i in range(self.joint_num):
            self.joints.append(pybullet_joint(p.getJointInfo(id, i)))
            self.links.append(pybullet_link(p.getLinkState(id, i), i))

    def print(self, detail=[]):
        print('Robot info:')
        print('RobotId:', self.id, 'BaseName:', self.name[0], ' FileName:', self.name[1])
        self._print_joint_info(detail)
        self._print_link_info()
        print(self.visual_shape_data)
        # for i in range(len(self.visual_shape_data)):
        #     self._print_vsd_info(self.visual_shape_data[i])

    def _print_joint_info(self, detail):
        print('joint_info:')
        for i in range(self.joint_num):
            if 'all' in detail:
                print(' Index:', self.joints[i].index,
                      ' Name:', self.joints[i].name,
                      ' Type:', _joint_type(int(self.joints[i].type)),
                      ' qIndex:', self.joints[i].qIndex,
                      ' uIndex:', self.joints[i].uIndex,
                      ' flag:', self.joints[i].flag,
                      ' Damping:', self.joints[i].damping,
                      ' Friction:', self.joints[i].friction,
                      ' LowerLimit:', self.joints[i].lowerLimit,
                      ' UpperLimit:', self.joints[i].upperLimit,
                      ' MaxForce:', self.joints[i].maxForce,
                      ' MaxVelocity:', self.joints[i].maxVelocity,
                      ' LinkName:', self.joints[i].linkName,
                      ' JointAxis:', self.joints[i].jointAxis,
                      ' ParentFramePos:', self.joints[i].parentFramePos,
                      ' ParentFrameOrn:', self.joints[i].parentFrameOrn,
                      ' ParentIndex:', self.joints[i].parentIndex)
            else:
                print(' Index:', self.joints[i].index,
                      ' Name:', self.joints[i].name,
                      ' Type:', _joint_type(int(self.joints[i].type)),
                      ' LinkName:', self.joints[i].linkName,
                      ' ParentIndex:', self.joints[i].parentIndex,end='')
                if 'dynamic' in detail:
                    print(' uIndex:', self.joints[i].uIndex,
                          ' Damping:', self.joints[i].damping,
                          ' Friction:', self.joints[i].friction,
                          ' MaxForce:', self.joints[i].maxForce,
                          ' MaxVelocity:', self.joints[i].maxVelocity,end='')
                if 'kinematic' in detail:
                    print(' qIndex:', self.joints[i].qIndex,
                          ' LowerLimit:', self.joints[i].lowerLimit,
                          ' UpperLimit:', self.joints[i].upperLimit,
                          ' JointAxis:', self.joints[i].jointAxis,
                          ' ParentFramePos:', self.joints[i].parentFramePos,
                          ' ParentFrameOrn:', self.joints[i].parentFrameOrn,end='')
                print('')

    def _print_link_info(self):
        print('link_info:')
        for i in range(self.link_num):
            print('linkWorldPosition:', self.links[i].linkWorldPosition,
                  'linkWorldOrientation:', self.links[i].linkWorldOrientation,
                  'localInertialFramePosition:', self.links[i].localInertialFramePosition,
                  'localInertialFrameOrientation:', self.links[i].localInertialFrameOrientation,
                  'worldLinkFramePosition:', self.links[i].worldLinkFramePosition,
                  'worldLinkFrameOrientation:', self.links[i].worldLinkFrameOrientation)

    def _print_vsd_info(self):
        pass

class pybullet_joint():
    def __init__(self, joint_info):
        self.index = joint_info[0]
        self.name = joint_info[1]
        self.type = joint_info[2]
        self.qIndex = joint_info[3]
        self.uIndex = joint_info[4]
        self.flag = joint_info[5]
        self.damping = joint_info[6]
        self.friction = joint_info[7]
        self.lowerLimit = joint_info[8]
        self.upperLimit = joint_info[9]
        self.maxForce = joint_info[10]
        self.maxVelocity = joint_info[11]
        self.linkName = joint_info[12]
        self.jointAxis = joint_info[13]
        self.parentFramePos = joint_info[14]
        self.parentFrameOrn = joint_info[15]
        self.parentIndex = joint_info[16]


class pybullet_link():
    def __init__(self, link_info,index):
        self.index = index
        self.linkWorldPosition = link_info[0]
        self.linkWorldOrientation = link_info[1]
        self.localInertialFramePosition = link_info[2]
        self.localInertialFrameOrientation = link_info[3]
        self.worldLinkFramePosition = link_info[4]
        self.worldLinkFrameOrientation = link_info[5]


def _joint_type(num):
    if num == 0:
        return 'REV'
    elif num == 1:
        return 'PRI'
    elif num == 2:
        return 'SPH'
    elif num == 3:
        return 'PLA'
    elif num == 4:
        return 'FIX'
    else:
        return 'UNK'
