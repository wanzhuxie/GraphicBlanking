# !/usr/bin/python
# -*- coding: utf-8 -*-

# -------------------------------------------
'''
===立方体背面消隐算法===
==mine方法==4ms
主要理论：只要面的一个点被遮挡，则该面完全被遮挡
主要方法：找到所有被遮挡的顶点，顶点的三个邻面均被遮挡
opengl X轴水平向右，Y竖直向上，Z垂直屏幕向外

==Robert方法==0.4ms
面的外法线方向与Z方向夹角<90可见
'''
# -------------------------------------------

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
import copy
import cv2
import time
import Geometry3D as g3d

def TwoPointDistance(mPoint1 , mPoint2):
    dist = np.sqrt(np.sum(np.square(np.array(mPoint1) - np.array(mPoint2))))
    return dist
# 判断线面是否相交
def GetLinePlaneIntersect(mLinePont1,mLineVector, mPlanePoint1, mPlanePoint2, mPlanePoint3):
    planeVector = np.cross(np.array(mPlanePoint2) - np.array(mPlanePoint1), np.array(mPlanePoint3) - np.array(mPlanePoint1))
    #print("planeVector",planeVector)
    planePoint = mPlanePoint1
    lineVector = mLineVector
    linePoint = mLinePont1

    vp1 = planeVector[0]
    vp2 = planeVector[1]
    vp3 = planeVector[2]
    n1 = planePoint[0]
    n2 = planePoint[1]
    n3 = planePoint[2]
    v1 = lineVector[0]
    v2 = lineVector[1]
    v3 = lineVector[2]
    m1 = linePoint[0]
    m2 = linePoint[1]
    m3 = linePoint[2]
    vpt = v1 * vp1 + v2 * vp2 + v3 * vp3
    # 首先判断直线是否与平面平行
    if vpt == 0:
        return None

    intersectionPoint = []
    t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt
    intersectionPoint.append( m1 + v1 * t)
    intersectionPoint.append( m2 + v2 * t)
    intersectionPoint.append( m3 + v3 * t)
    #print(intersectionPoint)
    return intersectionPoint
#空间3点三角形面积 面积S=根号 (p* (p-a)* (p-b)* (p-c)) 其中p= (a+b+c)/2
def TrangleArea(mPoint1,mPoint2,mPoint3):
    a=TwoPointDistance(mPoint1 , mPoint2)
    b=TwoPointDistance(mPoint2 , mPoint3)
    c=TwoPointDistance(mPoint3 , mPoint1)
    p=(a+b+c)/2
    dArea=np.sqrt(p* (p-a)* (p-b)* (p-c))
    return dArea

#mine
class FaceBlankingBox:
    def __init__(self,center=[0,0,0],sideLen=2):
        self.center=center
        self.sideLen=sideLen
        self._RotateX = 0.0
        self._RotateY = 0.0
        self._RotateZ = 0.0

        #8个顶点的坐标
        self.listVertexCoor=[]
        self.listVertexInitialCoor=[[-1,1,1],[-1,-1,1],[1,-1,1],[1,1,1],[1,1,-1],[1,-1,-1],[-1,-1,-1],[-1,1,-1]]
        # 立方体每个面的顶点序号 前方面的点序号1234 后方面的点序号5678
        # 面的序号：前0后1左2右3上4下5
        self.listEdgeVertex=[[0, 1], \
                             [1, 2], \
                             [2, 3], \
                             [3, 0], \
                             [4, 5], \
                             [5, 6],\
                             [6, 7],\
                             [7, 4],\
                             [0, 7],\
                             [1, 6],\
                             [2, 5],\
                             [3, 4]]
        self.listFaceVertex=[[0, 1, 2, 3], \
                             [4, 5, 6, 7], \
                             [7, 6, 1, 0], \
                             [3, 2, 5, 4], \
                             [7, 0, 3, 4], \
                             [1, 6, 5, 2]]

    #设置旋转角度
    def SetRotation(self,x,y,z):
        self._RotateX = x
        self._RotateY = y
        self._RotateZ = z

    #更新旋转后的顶点坐标
    def UpdateVertexCoor(self):
        self.listVertexCoor=copy.deepcopy(self.listVertexInitialCoor)
        #print("======================")
        #print(self.listVertexCoor)
        sinX=np.sin(self._RotateX/180*np.pi)
        sinY=np.sin(self._RotateY/180*np.pi)
        sinZ=np.sin(self._RotateZ/180*np.pi)
        cosX=np.cos(self._RotateX/180*np.pi)
        cosY=np.cos(self._RotateY/180*np.pi)
        cosZ=np.cos(self._RotateZ/180*np.pi)
        for point in self.listVertexCoor:
            #print("ori:", point)
            # 绕X旋转
            x,y,z=point[0],point[1],point[2]
            point[0]=x
            point[1]=y*cosX-z*sinX
            point[2]=y*sinX+z*cosX
            # 绕Y旋转
            x,y,z=point[0],point[1],point[2]
            point[0]=z*sinY+x*cosY
            point[1]=y
            point[2]=z*cosY-x*sinY
            # 绕Z旋转
            x,y,z=point[0],point[1],point[2]
            point[0]=x*cosZ-y*sinZ
            point[1]=x*sinZ+y*cosZ
            point[2]=z
            #print("new:",point)
        #print(self.listVertexCoor)

    # 获取隐藏面 算法：对于每个点，做垂直于屏幕向外的射线，若与任何拓扑面相交，则认为该点被隐藏，被隐藏的点的3个邻面均被隐藏
    def GetHideFaces(self):
        self.UpdateVertexCoor()

        listHidenVertex = []# 被隐藏的点
        listHidenEdges = []# 被隐藏的线
        listHidenFaces = []# 被隐藏的面

        #点
        for i in range(len(self.listVertexCoor)):#遍历每个顶点坐标
            for j in range (len(self.listFaceVertex)):#遍历每个面
                if i in self.listFaceVertex[j]:
                    continue
                if self.CheckLinePlaneIntersect(i,j):
                    listHidenVertex.append(i)

        #线
        listHidenEdges_Multi=[]
        for item in listHidenVertex:
            listHidenEdges_Multi.append(self.GetVertexNeighborEdges(item))
        nplistHidenEdges_Multi=np.array(listHidenEdges_Multi)
        nplistHidenEdges=nplistHidenEdges_Multi.reshape(1,-1)
        listHidenEdges=np.unique(nplistHidenEdges[0])

        #面
        listHidenFaces_Multi=[]
        for item in listHidenVertex:
            listHidenFaces_Multi.append(self.GetVertexNeighborFaces(item))
        nplistHidenFaces_Multi=np.array(listHidenFaces_Multi)
        nplistHidenFaces=nplistHidenFaces_Multi.reshape(1,-1)
        listHidenFaces=np.unique(nplistHidenFaces[0])

        #print(listHidenFaces)
        return listHidenVertex , listHidenEdges , listHidenFaces

    #检查顶点是否被面所遮挡
    def CheckLinePlaneIntersect(self,iVertexIndex,iFaceIndex):
        listVertexIndexOfFace=self.listFaceVertex[iFaceIndex]
        # 顶点与面接触则跳过
        if iVertexIndex in listVertexIndexOfFace:
            return False

        mFaceVertex0=self.listVertexCoor[listVertexIndexOfFace[0]]
        mFaceVertex1=self.listVertexCoor[listVertexIndexOfFace[1]]
        mFaceVertex2=self.listVertexCoor[listVertexIndexOfFace[2]]
        mFaceVertex3=self.listVertexCoor[listVertexIndexOfFace[3]]
        mCheckPoint=self.listVertexCoor[iVertexIndex]
        #交点
        mIntersectPoint=GetLinePlaneIntersect(np.array(mCheckPoint),np.array([0,0,1]),mFaceVertex0,mFaceVertex1,mFaceVertex2)
        if mIntersectPoint is None:
            return False
        #若交点Z值<检查点Z值，未被隐藏
        if mIntersectPoint[2]<mCheckPoint[2]:
            return False

        #检查交点是否在Face之外 即交点到4个边的距离是否等于四边形面积，等于则在内部(包含在边缘)，否则在外部
        dStdArea=TrangleArea(mFaceVertex0,mFaceVertex1,mFaceVertex2)+TrangleArea(mFaceVertex0,mFaceVertex3,mFaceVertex2)
        dCheckArea0=TrangleArea(mIntersectPoint,mFaceVertex0,mFaceVertex1)
        dCheckArea1=TrangleArea(mIntersectPoint,mFaceVertex1,mFaceVertex2)
        dCheckArea2=TrangleArea(mIntersectPoint,mFaceVertex2,mFaceVertex3)
        dCheckArea3=TrangleArea(mIntersectPoint,mFaceVertex3,mFaceVertex0)
        dCheckArea=dCheckArea0+dCheckArea1+dCheckArea2+dCheckArea3

        if dCheckArea-dStdArea<0.001:
            return True
        return False

    #根据顶点Index获取相邻的面index
    def GetVertexNeighborFaces(self, vertexIndex):
        listNeighborFaces=[]
        for eachFaceIndex in range (len(self.listFaceVertex)):
            if vertexIndex in self.listFaceVertex[eachFaceIndex]:
                listNeighborFaces.append(eachFaceIndex)
        return listNeighborFaces
    #根据顶点Index获取相邻的边线index
    def GetVertexNeighborEdges(self, vertexIndex):
        listNeighborEdges=[]
        for eachEdgeIndex in range (len(self.listEdgeVertex)):
            if vertexIndex in self.listEdgeVertex[eachEdgeIndex]:
                listNeighborEdges.append(eachEdgeIndex)
        return listNeighborEdges



#Robert methed
class FaceBlankingBox0:
    def __init__(self,center=[0,0,0],sideLen=2):
        self.center=center
        self.sideLen=sideLen
        self._RotateX = 0.0
        self._RotateY = 0.0
        self._RotateZ = 0.0

        #8个顶点的坐标
        self.listVertexCoor=[]
        self.listVertexInitialCoor=[[-1,1,1],[-1,-1,1],[1,-1,1],[1,1,1],[1,1,-1],[1,-1,-1],[-1,-1,-1],[-1,1,-1]]
        # 立方体每个面的顶点序号 前方面的点序号1234 后方面的点序号5678
        # 面的序号：前0后1左2右3上4下5
        self.listFaceVertex=[[0, 1, 2, 3], \
                             [4, 5, 6, 7], \
                             [7, 6, 1, 0], \
                             [3, 2, 5, 4], \
                             [7, 0, 3, 4], \
                             [1, 6, 5, 2]]

    #设置旋转角度
    def SetRotation(self,x,y,z):
        self._RotateX = x
        self._RotateY = y
        self._RotateZ = z

    #更新旋转后的顶点坐标
    def UpdateVertexCoor(self):
        self.listVertexCoor=copy.deepcopy(self.listVertexInitialCoor)
        #print("======================")
        #print(self.listVertexCoor)
        sinX=np.sin(self._RotateX/180*np.pi)
        sinY=np.sin(self._RotateY/180*np.pi)
        sinZ=np.sin(self._RotateZ/180*np.pi)
        cosX=np.cos(self._RotateX/180*np.pi)
        cosY=np.cos(self._RotateY/180*np.pi)
        cosZ=np.cos(self._RotateZ/180*np.pi)
        for point in self.listVertexCoor:
            #print("ori:", point)
            # 绕X旋转
            x,y,z=point[0],point[1],point[2]
            point[0]=x
            point[1]=y*cosX-z*sinX
            point[2]=y*sinX+z*cosX
            # 绕Y旋转
            x,y,z=point[0],point[1],point[2]
            point[0]=z*sinY+x*cosY
            point[1]=y
            point[2]=z*cosY-x*sinY
            # 绕Z旋转
            x,y,z=point[0],point[1],point[2]
            point[0]=x*cosZ-y*sinZ
            point[1]=x*sinZ+y*cosZ
            point[2]=z
            #print("new:",point)
        #print(self.listVertexCoor)

    # 面的外法线方向与Z方向夹角<90可见
    def GetHideFaces(self):
        self.UpdateVertexCoor()

        listHidenFaces = []
        for j in range (len(self.listFaceVertex)):#遍历每个面
            listFaceVertexIndex=self.listFaceVertex[j]
            mFaceVertex1 = self.listVertexCoor[listFaceVertexIndex[1]]
            mFaceVertex2 = self.listVertexCoor[listFaceVertexIndex[2]]
            mFaceVertex3 = self.listVertexCoor[listFaceVertexIndex[3]]

            planeVector = np.cross(np.array(mFaceVertex2) - np.array(mFaceVertex1),np.array(mFaceVertex3) - np.array(mFaceVertex1))
            if np.dot(planeVector,(0,0,1))<=0:
                listHidenFaces.append(j)
        #print(listHidenFaces)
        return listHidenFaces

