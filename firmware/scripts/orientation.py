import serial
import pygame
import struct
import time
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

# -----------------------------
# Configuration
# -----------------------------
COM_PORT = "COM11"
BAUD = 115200

pitch = 0.0
roll = 0.0
yaw = 0.0


frames = 0
last = time.time()

# -----------------------------
# Serial
# -----------------------------
ser = serial.Serial(COM_PORT, BAUD, timeout=0.01)

# -----------------------------
# Pygame/OpenGL
# -----------------------------
pygame.init()

display = (1000, 800)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

gluPerspective(45, display[0] / display[1], 0.1, 100.0)

glTranslatef(0.0, 0.0, -8)

glEnable(GL_DEPTH_TEST)


def draw_motor():
    """Draw motor"""
    glColor3f(0.15, 0.15, 0.15)

    radius = 0.18
    h = 0.08

    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(0, 0, h)
    for i in range(31):
        a = 2 * np.pi * i / 30
        glVertex3f(radius * np.cos(a), radius * np.sin(a), h)
    glEnd()

    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(0, 0, -h)
    for i in range(31):
        a = -2 * np.pi * i / 30
        glVertex3f(radius * np.cos(a), radius * np.sin(a), -h)
    glEnd()


def draw_arm():
    glColor3f(0.2, 0.2, 0.2)

    w = 0.08
    l = 2.0
    h = 0.04

    glBegin(GL_QUADS)

    # top
    glVertex3f(-l, w, h)
    glVertex3f(l, w, h)
    glVertex3f(l, -w, h)
    glVertex3f(-l, -w, h)

    # bottom
    glVertex3f(-l, -w, -h)
    glVertex3f(l, -w, -h)
    glVertex3f(l, w, -h)
    glVertex3f(-l, w, -h)

    glEnd()


def draw_body():
    glColor3f(0.85, 0.15, 0.15)

    s = 0.32

    glBegin(GL_QUADS)

    # Top
    glVertex3f(-s, -s, s)
    glVertex3f(s, -s, s)
    glVertex3f(s, s, s)
    glVertex3f(-s, s, s)

    # Bottom
    glVertex3f(-s, s, -s)
    glVertex3f(s, s, -s)
    glVertex3f(s, -s, -s)
    glVertex3f(-s, -s, -s)

    # Front
    glVertex3f(-s, s, s)
    glVertex3f(s, s, s)
    glVertex3f(s, s, -s)
    glVertex3f(-s, s, -s)

    # Back
    glVertex3f(-s, -s, -s)
    glVertex3f(s, -s, -s)
    glVertex3f(s, -s, s)
    glVertex3f(-s, -s, s)

    # Left
    glVertex3f(-s, -s, -s)
    glVertex3f(-s, -s, s)
    glVertex3f(-s, s, s)
    glVertex3f(-s, s, -s)

    # Right
    glVertex3f(s, -s, s)
    glVertex3f(s, -s, -s)
    glVertex3f(s, s, -s)
    glVertex3f(s, s, s)

    glEnd()


def draw_axes():
    glLineWidth(3)

    glBegin(GL_LINES)

    # X (red)
    glColor3f(1, 0, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(3, 0, 0)

    # Y (green)
    glColor3f(0, 1, 0)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 3, 0)

    # Z (blue)
    glColor3f(0, 0, 1)
    glVertex3f(0, 0, 0)
    glVertex3f(0, 0, 3)

    glEnd()


def draw_quadcopter():

    draw_axes()

    # Body
    draw_body()

    # Arm 1
    glPushMatrix()
    glRotatef(45, 0, 0, 1)
    draw_arm()

    glPushMatrix()
    glTranslatef(2, 0, 0)
    draw_motor()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(-2, 0, 0)
    draw_motor()
    glPopMatrix()

    glPopMatrix()

    # Arm 2
    glPushMatrix()
    glRotatef(-45, 0, 0, 1)
    draw_arm()

    glPushMatrix()
    glTranslatef(2, 0, 0)
    draw_motor()
    glPopMatrix()

    glPushMatrix()
    glTranslatef(-2, 0, 0)
    draw_motor()
    glPopMatrix()

    glPopMatrix()


clock = pygame.time.Clock()

while True:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            ser.close()
            pygame.quit()
            quit()

    # -----------------------------
    # Read serial
    # -----------------------------
    try:
        # line = ser.readline().decode().strip()

        # if line:
        #     print(line)
        #     p, r, y = line.split(',')

        if ser.in_waiting >= 16:
            packet = ser.read(16)

            pitch, roll, yaw, null = struct.unpack("<ffff", packet)

            # pitch = float(p)
            # roll = float(r)
            # yaw = float(y)
            print(f"Pitch: {pitch:8.2f}\tRoll: {roll:8.2f}\tYaw: {yaw:8.2f}")

    except:
        pass

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glPushMatrix()

    #
    # Aerospace rotation:
    # yaw -> pitch -> roll
    #
    glRotatef(yaw, 0, 0, 1)
    glRotatef(pitch, 1, 0, 0)
    glRotatef(roll, 0, 1, 0)

    draw_quadcopter()

    glPopMatrix()

    pygame.display.flip()

    # frames += 1
    # now = time.time()

    # if now - last >= 1:
    #     print("FPS:", frames)
    #     frames = 0
    #     last = now

    # clock.tick(120)