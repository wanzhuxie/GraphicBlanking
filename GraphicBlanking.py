# !/usr/bin/python
# -*- coding: utf-8 -*-

# -------------------------------------------
'''
===立方体背面消隐算法===
主要理论：只要面的一个点被遮挡，则该面完全被遮挡
主要方法：找到所有被遮挡的顶点，顶点的三个邻面均被遮挡
opengl X轴水平向右，Y竖直向上，Z垂直屏幕向外

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

class MathBox:
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

    # 获取隐藏面 算法：对于每个点，做垂直于屏幕向外的射线，若与任何拓扑面相交，则认为该点被隐藏，被隐藏的点的3个邻面均被隐藏
    def GetHideFaces(self):
        self.UpdateVertexCoor()

        listHidenVertex = []# 被隐藏的点
        for i in range(len(self.listVertexCoor)):#遍历每个顶点坐标
            for j in range (len(self.listFaceVertex)):#遍历每个面
                if i in self.listFaceVertex[j]:
                    continue
                if self.CheckLinePlaneIntersect(i,j):
                    listHidenVertex.append(i)



        #收集隐藏顶点对应的面，均是隐藏面
        listHidenFaces_Multi=[]
        for item in listHidenVertex:
            listHidenFaces_Multi.append(self.GetVertexNeighborFaces(item))

        nplistHidenFaces_Multi=np.array(listHidenFaces_Multi)
        nplistHidenFaces=nplistHidenFaces_Multi.reshape(1,-1)
        listHidenFaces=np.unique(nplistHidenFaces[0])
        #print(listHidenFaces)
        return listHidenFaces

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

    #根据顶点Index获取相邻的3个面index
    def GetVertexNeighborFaces(self, vertexIndex):
        listNeighborFaces=[]
        for eachFaceIndex in range (len(self.listFaceVertex)):
            if vertexIndex in self.listFaceVertex[eachFaceIndex]:
                listNeighborFaces.append(eachFaceIndex)
        return listNeighborFaces

    # 获取隐藏面 算法：深度最深的点肯定被隐藏，被隐藏的点的3个邻面均被隐藏
    # 被隐藏的点数量只有3种情况，1个（此时能看到3个面）2个（此时能看到2个面）4个（此时能看到1个面）
    #以上算法有误，比如先绕X轴旋转45°再绕Z轴旋转45°，此时有2个点为距离屏幕最远的点，但可以看到3个面
    def GetHideFaces0(self):
        self.UpdateVertexCoor()
        listVertexWithZ=[]#顶点序号，顶点Z坐标
        for i in range (len(self.listVertexCoor)):
            listVertexWithZ.append((i,self.listVertexCoor[i][2]))

        listVertexWithZ.sort(key=lambda x:x[1])#x[1]表示Z坐标
        z0=listVertexWithZ[0][1]
        z1=listVertexWithZ[1][1]
        z2=listVertexWithZ[2][1]
        listHidenVertex = []
        if np.abs(z0-z1)<0.00001 and np.abs(z0-z2)<0.00001:# 只会有一个面显示，即最前面的4个点所对应的面
            listHidenVertex.append(listVertexWithZ[0][0])
            listHidenVertex.append(listVertexWithZ[1][0])
            listHidenVertex.append(listVertexWithZ[2][0])
            listHidenVertex.append(listVertexWithZ[3][0])
        elif np.abs(z0-z1)<0.00001:
            listHidenVertex.append(listVertexWithZ[0][0])
            listHidenVertex.append(listVertexWithZ[1][0])
        else:
            listHidenVertex.append(listVertexWithZ[0][0])

        #收集隐藏顶点对应的面，均是隐藏面
        listHidenFaces_Multi=[]
        for item in listHidenVertex:
            listHidenFaces_Multi.append(self.GetVertexNeighborFaces(item))

        nplistHidenFaces_Multi=np.array(listHidenFaces_Multi)
        nplistHidenFaces=nplistHidenFaces_Multi.reshape(1,-1)
        listHidenFaces=np.unique(nplistHidenFaces[0])
        #print(listHidenFaces)
        return listHidenFaces



class BlankingVerification:
    def __init__(self):
        self.width=640
        self.height=640
        self.mathBox=MathBox()

        self.InitGL(self.width, self.height)

        #rotate around XYZ axes
        self._RotateX = 0.0
        self._RotateY = 0.0
        self._RotateZ = 0.0

    # InitGL
    def InitGL(self, width, height):
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(width, height)
        self.window = glutCreateWindow("ARVS")
        glutDisplayFunc(self.Draw)
        glutIdleFunc(self.Draw)

        glEnable(GL_TEXTURE_2D)
        glClearColor(1.0, 1.0, 1.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glShadeModel(GL_SMOOTH)

        # Back culling, blanking [the effect is not obvious]
        glEnable(GL_CULL_FACE)
        glCullFace(GL_BACK)
        glEnable(GL_POINT_SMOOTH)
        glEnable(GL_LINE_SMOOTH)
        glEnable(GL_POLYGON_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glHint(GL_POINT_SMOOTH_HINT, GL_NICEST)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST)
        glLoadIdentity()
        #gluPerspective(45.0, float(width) / float(height), 0.1, 100.0)
        glOrtho(-3,3,-3,3,-10,10)
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_DEPTH_TEST)

        self.LoadTexture()

    #贴图
    def LoadTexture(self):
        img1 = cv2.imread("image/0.jpg")
        img2 = cv2.imread("image/1.jpg")
        img3 = cv2.imread("image/2.jpg")
        img4 = cv2.imread("image/3.jpg")
        img5 = cv2.imread("image/4.jpg")
        img6 = cv2.imread("image/5.jpg")
        img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGBA)
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2RGBA)
        img3 = cv2.cvtColor(img3, cv2.COLOR_BGR2RGBA)
        img4 = cv2.cvtColor(img4, cv2.COLOR_BGR2RGBA)
        img5 = cv2.cvtColor(img5, cv2.COLOR_BGR2RGBA)
        img6 = cv2.cvtColor(img6, cv2.COLOR_BGR2RGBA)
        img1=cv2.flip(img1,0)
        img2=cv2.flip(img2,0)
        img3=cv2.flip(img3,0)
        img4=cv2.flip(img4,0)
        img5=cv2.flip(img5,0)
        img6=cv2.flip(img6,0)

        list_img=[img1,img2,img3,img4,img5,img6]
        for i in range(6):
            img = list_img[i]
            h1, w1, c1 = img.shape
            glGenTextures(2)
            glBindTexture(GL_TEXTURE_2D, i)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w1, h1, 0, GL_RGBA, GL_UNSIGNED_BYTE, img.data)
            glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S, GL_CLAMP)
            glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T, GL_CLAMP)
            glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T, GL_REPEAT)
            glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER, GL_NEAREST)
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)

    # draw main
    def Draw(self):
        timePoint1=time.perf_counter()

        #self.LoadTexture()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        self._RotateX += 0.1
        self._RotateY += 0.2
        self._RotateZ += 0.3

        timePoint3=time.perf_counter()
        self.mathBox.SetRotation(self._RotateX,self._RotateY,self._RotateZ)
        listHidenFace=self.mathBox.GetHideFaces()
        #print(listHidenFace)
        timePoint4=time.perf_counter()
        print ("mathBox:", "%.2f" % ((timePoint4-timePoint3)*1000), "ms")

        #Transform 注意opengl逆序变换
        glRotatef(self._RotateZ, 0, 0, 1)
        glRotatef(self._RotateY, 0, 1, 0)
        glRotatef(self._RotateX, 1, 0, 0)

        self.DrawBox()

        #Refresh screen
        glutSwapBuffers()

        timePoint2=time.perf_counter()
        #print ("UpdateView:", "%.2f" % ((timePoint2-timePoint1)*1000), "ms")
        #time.sleep(0.01)

    #绘制立方体
    def DrawBox(self):
        #纹理的四个顶点的坐标分别是(0.0f, 1.0f)、(0.0f, 0.0f)、(1.0f, 0.0f)、(1.0f, 1.0f)，分别对应左上、左下、右下、右上
        #face1
        glBindTexture(GL_TEXTURE_2D, 0)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0);        glVertex3f(-1,1,1)
        glTexCoord2f(0.0, 0.0);        glVertex3f(-1,-1,1)
        glTexCoord2f(1.0, 0.0);        glVertex3f(1,-1,1)
        glTexCoord2f(1.0, 1.0);        glVertex3f(1,1,1)
        glEnd()

        #face2
        glBindTexture(GL_TEXTURE_2D, 1)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0);        glVertex3f(1,1,-1)
        glTexCoord2f(0.0, 0.0);        glVertex3f(1,-1,-1)
        glTexCoord2f(1.0, 0.0);        glVertex3f(-1,-1,-1)
        glTexCoord2f(1.0, 1.0);        glVertex3f(-1,1,-1)
        glEnd()

        #face3
        glBindTexture(GL_TEXTURE_2D, 2)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0);        glVertex3f(-1,1,-1)
        glTexCoord2f(0.0, 0.0);        glVertex3f(-1,-1,-1)
        glTexCoord2f(1.0, 0.0);        glVertex3f(-1,-1,1)
        glTexCoord2f(1.0, 1.0);        glVertex3f(-1,1,1)
        glEnd()

        #face4
        glBindTexture(GL_TEXTURE_2D, 3)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0);        glVertex3f(1,1,1)
        glTexCoord2f(0.0, 0.0);        glVertex3f(1,-1,1)
        glTexCoord2f(1.0, 0.0);        glVertex3f(1,-1,-1)
        glTexCoord2f(1.0, 1.0);        glVertex3f(1,1,-1)
        glEnd()

        #face5
        glBindTexture(GL_TEXTURE_2D, 4)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0);        glVertex3f(-1,1,-1)
        glTexCoord2f(0.0, 0.0);        glVertex3f(-1,1,1)
        glTexCoord2f(1.0, 0.0);        glVertex3f(1,1,1)
        glTexCoord2f(1.0, 1.0);        glVertex3f(1,1,-1)
        glEnd()

        #face6
        glBindTexture(GL_TEXTURE_2D, 5)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0);        glVertex3f(-1,-1,1)
        glTexCoord2f(0.0, 0.0);        glVertex3f(-1,-1,-1)
        glTexCoord2f(1.0, 0.0);        glVertex3f(1,-1,-1)
        glTexCoord2f(1.0, 1.0);        glVertex3f(1,-1,1)
        glEnd()

    #MainLoop
    def MainLoop(self):
        glutMainLoop()

if __name__ == '__main__':
    w=BlankingVerification()
    w.MainLoop()

