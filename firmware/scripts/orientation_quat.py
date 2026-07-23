import serial
import struct
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np


# ==========================
# Serial configuration
# ==========================

PORT = "COM11"      # change this
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0)


# ==========================
# Quaternion functions
# ==========================

def quat_to_matrix(q):

    w, x, y, z = q

    norm = np.sqrt(w*w + x*x + y*y + z*z)

    if norm < 1e-6:
        return np.identity(4, dtype=np.float32)

    w /= norm
    x /= norm
    y /= norm
    z /= norm


    return np.array([
        [
            1 - 2*(y*y + z*z),
            2*(x*y - z*w),
            2*(x*z + y*w),
            0
        ],

        [
            2*(x*y + z*w),
            1 - 2*(x*x + z*z),
            2*(y*z - x*w),
            0
        ],

        [
            2*(x*z - y*w),
            2*(y*z + x*w),
            1 - 2*(x*x + y*y),
            0
        ],

        [0,0,0,1]

    ], dtype=np.float32)



def imu_to_opengl(q):

    R = quat_to_matrix(q)

    # STM32/IMU frame:
    #
    # X = forward
    # Y = right
    # Z = down
    #
    # OpenGL frame:
    #
    # X = right
    # Y = up
    # Z = toward viewer
    #

    C = np.array([

        [0,1,0,0],

        [1,0,0,0],

        [0,0,1,0],

        [0,0,0,-1]

    ], dtype=np.float32)


    return C @ R @ C.T



# ==========================
# Drawing
# ==========================

def cube(size):

    s = size

    glBegin(GL_QUADS)

    glColor3f(0.8,0.1,0.1)

    # top
    glVertex3f(-s,-s,s)
    glVertex3f(s,-s,s)
    glVertex3f(s,s,s)
    glVertex3f(-s,s,s)

    # bottom
    glVertex3f(-s,-s,-s)
    glVertex3f(-s,s,-s)
    glVertex3f(s,s,-s)
    glVertex3f(s,-s,-s)

    # front
    glVertex3f(-s,s,-s)
    glVertex3f(-s,s,s)
    glVertex3f(s,s,s)
    glVertex3f(s,s,-s)

    # back
    glVertex3f(-s,-s,-s)
    glVertex3f(s,-s,-s)
    glVertex3f(s,-s,s)
    glVertex3f(-s,-s,s)

    # left
    glVertex3f(-s,-s,-s)
    glVertex3f(-s,-s,s)
    glVertex3f(-s,s,s)
    glVertex3f(-s,s,-s)

    # right
    glVertex3f(s,-s,-s)
    glVertex3f(s,s,-s)
    glVertex3f(s,s,s)
    glVertex3f(s,-s,s)

    glEnd()



def arm():

    length = 1.5
    width = 0.07
    height = 0.04

    glColor3f(0.15,0.15,0.15)

    glBegin(GL_QUADS)

    glVertex3f(-length,-width,height)
    glVertex3f(length,-width,height)
    glVertex3f(length,width,height)
    glVertex3f(-length,width,height)

    glVertex3f(-length,-width,-height)
    glVertex3f(-length,width,-height)
    glVertex3f(length,width,-height)
    glVertex3f(length,-width,-height)

    glEnd()



def motor():

    glPushMatrix()

    glScalef(0.18,0.18,0.05)

    cube(1)

    glPopMatrix()



def draw_quad():

    # flight controller
    cube(0.25)


    for angle in [45,-45]:

        glPushMatrix()

        glRotatef(angle,0,0,1)

        arm()


        glPushMatrix()
        glTranslatef(1.5,0,0)
        motor()
        glPopMatrix()


        glPushMatrix()
        glTranslatef(-1.5,0,0)
        motor()
        glPopMatrix()


        glPopMatrix()



# ==========================
# OpenGL setup
# ==========================

pygame.init()

width = 1000
height = 800

pygame.display.set_mode(
    (width,height),
    DOUBLEBUF | OPENGL
)


glMatrixMode(GL_PROJECTION)

gluPerspective(
    45,
    width/height,
    0.1,
    100
)


glMatrixMode(GL_MODELVIEW)

glEnable(GL_DEPTH_TEST)



clock = pygame.time.Clock()


# ==========================
# Main loop
# ==========================

q = (1,0,0,0)

while True:


    for event in pygame.event.get():

        if event.type == pygame.QUIT:

            ser.close()
            pygame.quit()
            quit()



    # ----------------------
    # Read quaternion
    # ----------------------

    while ser.in_waiting >= 16:

        packet = ser.read(16)

        q = struct.unpack("<ffff", packet)

        print(
            "q:",
            "%.3f %.3f %.3f %.3f" % q
        )



    # ----------------------
    # Render
    # ----------------------

    glClear(
        GL_COLOR_BUFFER_BIT |
        GL_DEPTH_BUFFER_BIT
    )


    glLoadIdentity()

    glTranslatef(
        0,
        0,
        -6
    )


    R = imu_to_opengl(q)


    glMultMatrixf(
        R.T.flatten()
    )


    draw_quad()


    pygame.display.flip()

    # clock.tick(240)