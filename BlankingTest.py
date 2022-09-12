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
from FaceBlanking import *
from OpenGL.GLUT import *
import numpy as np
import copy
import cv2
import time
import Geometry3D as g3d


class BlankingVerification:
    def __init__(self):
        self.width=640
        self.height=640
        self.mathBox=FaceBlankingBox()

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

        #显示字符
        #  Fonts supported by GLUT are: GLUT_BITMAP_8_BY_13,
        #  GLUT_BITMAP_9_BY_15, GLUT_BITMAP_TIMES_ROMAN_10,
        #  GLUT_BITMAP_TIMES_ROMAN_24, GLUT_BITMAP_HELVETICA_10,
        #  GLUT_BITMAP_HELVETICA_12, and GLUT_BITMAP_HELVETICA_18.
        font_style = GLUT_BITMAP_TIMES_ROMAN_24
        glColor3f(0.0, 1.0, 0.0)
        info="Current back faces: "
        for i in range (len(listHidenFace)):
            if i==0:
                info += str(listHidenFace[i])
            else:
                info += "," + str(listHidenFace[i])

        glRasterPos2i(-3, 2)
        for i in info:
            glutBitmapCharacter(font_style, ord(i))

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

