#pitch, roll, yaw from https://en.wikipedia.org/wiki/Aircraft_principal_axes

import math

from PySide6 import *
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtOpenGL import *
from PySide6.QtOpenGLWidgets import *

from ctypes import util
try:
    from OpenGL.platform import win32
except AttributeError:
    pass

from OpenGL.GL import *
from OpenGL.GLU import *

from .Model3D import *


#--------------------------------------------------#
class Cube(object):
    def __init__(self, position, color):
        self.position = position
        self.color = color

        # Cube information
        self.num_faces = 6

        self.vertices = [(-1.0, -0.05, 0.5), (1.0, -0.05, 0.5),
                         (1.0, 0.05, 0.5), (-1.0, 0.05, 0.5),
                         (-1.0, -0.05, -0.5), (1.0, -0.05, -0.5),
                         (1.0, 0.05, -0.5), (-1.0, 0.05, -0.5)]

        self.normals = [
            (0.0, 0.0, +1.0),  # front
            (0.0, 0.0, -1.0),  # back
            (+1.0, 0.0, 0.0),  # right
            (-1.0, 0.0, 0.0),  # left
            (0.0, +1.0, 0.0),  # top
            (0.0, -1.0, 0.0)
        ]  # bottom

        self.vertex_indices = [
            (0, 1, 2, 3),  # front
            (4, 5, 6, 7),  # back
            (1, 5, 6, 2),  # right
            (0, 4, 7, 3),  # left
            (3, 2, 6, 7),  # top
            (0, 1, 5, 4)
        ]  # bottom

    def render(self):
        glColor(self.color)

        vertices = self.vertices

        glBegin(GL_QUADS)
        for face_no in range(self.num_faces):
            glNormal3dv(self.normals[face_no])
            v1, v2, v3, v4 = self.vertex_indices[face_no]
            glVertex(vertices[v1])
            glVertex(vertices[v2])
            glVertex(vertices[v3])
            glVertex(vertices[v4])
        glEnd()


#--------------------------------------------------#


class AttitudeWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        #super().__init__(QGLFormat(QGL.SampleBuffers), parent)

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self.obj = OBJ()
        self.obj_file = os.path.join("model", 'flight.obj')

    def minimumSizeHint(self):
        return QSize(70, 50)

    def sizeHint(self):
        return QSize(150, 120)

    def update_attitude(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self.update()

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.2, 0.2, 0.2, 0.2)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_BLEND)
        glEnable(GL_POLYGON_SMOOTH)
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        self.obj.load(self.obj_file)
        self.obj.initGL()
        glLightfv(GL_LIGHT0, GL_AMBIENT, (1.0, 1.0, 1.0, 1.0))

    def paintGL(self):

        y_angle = math.degrees(self.pitch)
        x_angle = math.degrees(self.roll)
        z_angle = math.degrees(self.yaw - math.pi / 2)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        #self.paintBox()

        glPushMatrix()

        glRotate(float(z_angle), 0, -1, 0)
        glRotate(float(y_angle), 0, 0, -1)
        glRotate(float(x_angle), -1, 0, 0)

        #self.cube.render()
        glCallList(self.obj.gl_list)

        glPopMatrix()

    def resizeGL(self, width, height):
        if (width == 0) or (height == 0):
            return

        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(20.0, float(width) / height, 0.001, 100.0)
        #gluPerspective(120.0, width/float(height), 0.001, 100.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        gluLookAt(0.0, 1.0, -5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        #
        #gluLookAt(0.0, 1.0, -5.0,
        #      0.0, 0.0, 0.0,
        #      0.0, 1.0, 0.0)

    def paintBox(self):

        glColor((1., 1., 1.))
        glLineWidth(0.3)

        glBegin(GL_LINES)

        for x in range(-20, 22, 2):
            glVertex3f(x / 10., -1, -1)
            glVertex3f(x / 10., -1, 1)

        for x in range(-20, 22, 2):
            glVertex3f(x / 10., -1, 1)
            glVertex3f(x / 10., 1, 1)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z / 10.)
            glVertex3f(2, -1, z / 10.)

        for z in range(-10, 12, 2):
            glVertex3f(-2, -1, z / 10.)
            glVertex3f(-2, 1, z / 10.)

        for z in range(-10, 12, 2):
            glVertex3f(2, -1, z / 10.)
            glVertex3f(2, 1, z / 10.)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y / 10., 1)
            glVertex3f(2, y / 10., 1)

        for y in range(-10, 12, 2):
            glVertex3f(-2, y / 10., 1)
            glVertex3f(-2, y / 10., -1)

        for y in range(-10, 12, 2):
            glVertex3f(2, y / 10., 1)
            glVertex3f(2, y / 10., -1)

        glEnd()


#--------------------------------------------------#
