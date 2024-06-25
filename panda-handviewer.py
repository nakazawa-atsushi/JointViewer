# please see Panda3DTutorial for reference
# https://arsthaumaturgis.github.io/Panda3DTutorial.io/tutorial/tut_lesson01.html

import numpy as np
import pandas as pd
from math import pi, sin, cos
from direct.showbase.ShowBase import ShowBase
from direct.gui.OnscreenText import OnscreenText
from direct.task import Task
from panda3d.core import *
from procedural3d import *
from myutils import *

class HandViewer(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        properties = WindowProperties()
        properties.setSize(1000, 750)
        self.win.requestProperties(properties)       

        self.setBackgroundColor(.3,.3,.3)
        
        # taskを設定する
        self.updateTask = taskMgr.add(self.update, "update")

        # trackballを設定する
        self.useTrackball()
        self.trackball.node().setForwardScale(0.01)
        self.trackball.node().set_rel_to(self.cam)
        
        # 地面を描く        
        lines = LineSegs()
        lines.setThickness(3)         
        lines.setColor(1,0,0,1)        
        lines.moveTo(0,0,0)
        lines.drawTo(1,0,0)
        lines.setColor(0, 1, 0, 1)
        lines.moveTo(0,0,0)
        lines.drawTo(0,1,0)
        lines.setColor(0, 0, 1, 1)
        lines.moveTo(0,0,0)
        lines.drawTo(0,0,1)
        node = lines.create()
        np = NodePath(node)
        np.reparentTo(render)

        # ライティングを設定する
        light_node = PointLight("point_light")
        light_node.set_color((1., 1., 1., 1.))
        light = self.cam.attach_new_node(light_node)
        light.set_pos(100., -100., 100.)
        self.render.set_light(light)

        # on screen text object
        self.frame = 0
        self.maxframe = 0
        self.d_frame = 1
        self.ost = OnscreenText(text="", pos=(-1.0, 0.95), scale=0.07)
        
        # set default links
        self.links = []

        # keyboard functions
        self.accept('space', self.pause)

    def setlinks(self, links):
        self.links = links

    def pause(self):
        if self.d_frame == 1:
            self.d_frame = 0
        else:
            self.d_frame = 1

    def update(self, task):
        self.ost.text = f'frame:{self.frame} ({self.maxframe})'
        
        if self.frame < self.maxframe:
            self.frame += self.d_frame
        else:
            self.frame = 0
        
        # jointの座標をアップデートする
        for name in self.jnames:
            self.joints[name].setPos(self.data[name+'.x'][self.frame],
                                    self.data[name+'.y'][self.frame],
                                    self.data[name+'.z'][self.frame])

        # リンクの座標をアップデートする
        for link in self.links:
            for i in range(len(link)-1):
                name1 = link[i]
                name2 = link[i+1]
                p1 = [ self.data[name1+'.x'][self.frame], self.data[name1+'.y'][self.frame], self.data[name1+'.z'][self.frame]]
                p2 = [ self.data[name2+'.x'][self.frame], self.data[name2+'.y'][self.frame], self.data[name2+'.z'][self.frame]]
                self.__setbonepose(self.bones[name1+name2], p1, p2)

        return task.cont

    def __sphere(self, sz):
        maker = SphereMaker(
            center=(0,0,0), radius=sz,
            thickness=.45, inverted=False,            
            vertex_color=LVecBase4f(0.6, 0.1, 1.0, 1.0),
            has_uvs=True,
        )
        return maker

    def __cylinder(self, sz, LENGTH):
        cyl_maker = CylinderMaker(
            bottom_center=(-0.5*LENGTH,0,0), top_center=(0.5*LENGTH,0,0),
            radius=sz,
            thickness=.45, inverted=False,
            vertex_color=LVecBase4f(0.1, 0.6, 0.6, 1.0),
            has_uvs=True,
        )
        return cyl_maker

    def __setbonepose(self,bo,p1,p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        L,H,p,r = getLHPRFromVector(p2-p1)
        c = 0.5*p1 + 0.5*p2
        bo.setPos(c[0],c[1],c[2])
        bo.setHpr(H,p,r)
        
    def __createbone(self,p1,p2):
        L = np.linalg.norm(np.array(p1)-np.array(p2))
        bo = render.attach_new_node(self.__cylinder(0.005,L).generate())
        self.__setbonepose(bo,p1,p2)
        return bo

    def __createbones(self):
        for link in self.links:
            for i in range(len(link)-1):
                name1 = link[i]
                name2 = link[i+1]
                p1 = [ self.data[name1+'.x'][0], self.data[name1+'.y'][0], self.data[name1+'.z'][0]]
                p2 = [ self.data[name2+'.x'][0], self.data[name2+'.y'][0], self.data[name2+'.z'][0]]
                bo = self.__createbone(p1,p2)
                self.bones[name1+name2] = bo

    def loaddata(self,fname):
        self.fname = fname
        data = pd.read_csv(FILE)
        
        # データを辞書型に変換
        self.data = data.to_dict()

        # 関節の名前を抽出
        self.jnames = []
        for name in self.data.keys():
            if '.' in name:
                if name.split('.')[0] not in self.jnames:
                    self.jnames.append(name.split('.')[0])        
        
        # 最大フレームを取り出す
        self.maxframe = np.max(list(self.data[self.jnames[0] + '.x'].keys()))
        
        # 現在のフレーム
        self.frame = 0
        
        # for notification
        print('joint names:', self.jnames)
        print('maxframe :', self.maxframe)

        # 各関節に対応するノードを設定する
        self.joints = {}
        
        # sphereを作って格納する
        for name in self.jnames:
            joint = render.attach_new_node(self.__sphere(0.004).generate())
            self.joints[name] = joint
            self.joints[name].setPos(self.data[name+'.x'][self.frame],
                                    self.data[name+'.y'][self.frame],
                                    self.data[name+'.z'][self.frame])

        # boneを設定する
        self.bones = {}
        self.__createbones()

        # 視点を設定する
        self.cam.setPos(1, 1, 1)        
        self.cam.lookAt(
                self.data[self.jnames[0]+'.x'][0],
                self.data[self.jnames[0]+'.y'][0],
                self.data[self.jnames[0]+'.z'][0])



if __name__ == '__main__':
    # データファイルを読む
    FILE = 'HandBonePosition.csv'
    # link for bones (Hand model)
    links = [['Thumb1', 'Thumb2', 'Thumb3', 'ThumbTip'],
        ['Index1', 'Index2', 'Index3', 'IndexTip'],
        ['Middle1', 'Middle2', 'Middle3', 'MiddleTip'],
        ['Ring1', 'Ring2', 'Ring3', 'RingTip'],
        ['Pinky1', 'Pinky2', 'Pinky3', 'PinkyTip'],
        ['ForearmStub', 'Thumb0', 'Thumb1'],
        ['ForearmStub', 'Pinky0']]

    app = HandViewer()
    app.setlinks(links)

    app.loaddata(FILE)
    app.run()