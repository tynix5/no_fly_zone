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

PORT = "COM8"
# PORT = "COM7"         # this is for no_fly_zone
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0)


# ==========================
# Quaternion functions
# ==========================

def quat_to_matrix(q):
    """
    Converts a unit quaternion (w, x, y, z) into a 4x4 rotation matrix.
    Assumes the STM32 sends the quaternion in (w, x, y, z) order over
    serial as 4 little-endian floats. If your firmware sends
    (x, y, z, w) instead, swap the unpacking order in the main loop.
    """

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
        [0, 0, 0, 1]
    ], dtype=np.float32)


def imu_to_opengl(q):
    """
    Converts an orientation quaternion expressed in the STM32/IMU body
    frame into the equivalent rotation matrix in OpenGL's display frame.

    STM32/IMU body frame (FRD, right-handed):
        X = forward
        Y = right
        Z = down

    OpenGL display frame (right-handed):
        X = right
        Y = up
        Z = toward viewer (camera looks down -Z)

    Visual convention chosen here (a "chase view" from behind/above):
        right   (Y_body) -> right   (X_gl)
        down    (Z_body) -> down    (-Y_gl)
        forward (X_body) -> into screen (-Z_gl)

    C is the change-of-basis matrix mapping body-frame vectors to
    gl-frame vectors: v_gl = C @ v_body. Because both frames are
    right-handed, C must be a proper rotation (orthonormal, det = +1),
    NOT a reflection. Swapping two axes without a sign flip (det = -1)
    would mirror the model and invert rotation direction on screen.
    """

    R = quat_to_matrix(q)

    C = np.array([
        [0,  -1,  0, 0],   # X_gl = -Y_body   (right -> right)
        [-1,  0, 0, 0],   # Y_gl = -X_body  (down  -> down on screen)
        [0, 0,  1, 0],   # Z_gl = Z_body  (forward -> into screen)
        [0,  0,  0, 1]
    ], dtype=np.float32)

    # Similarity transform: express the body-frame rotation in gl-frame.
    # C is orthogonal, so C^-1 == C.T
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