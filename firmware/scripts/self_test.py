"""
Self-test for imu_to_opengl axis mapping.

No serial connection needed. Cycles through pure roll, pure pitch, and
pure yaw rotations (generated in code, not from hardware) so you can
confirm the render is correct without any hand-motion impurity or
sensor noise. An on-screen label + colored axis gizmo (attached to the
body) make it unambiguous which rotation is being shown.

Gizmo colors (drawn in body frame, so they rotate with the vehicle):
    RED   = body X (forward / roll axis)
    GREEN = body Y (right   / pitch axis)
    BLUE  = body Z (down    / yaw axis)
"""

import math
import time
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np


# ==========================
# Quaternion / frame conversion (same as imu_visualizer_fixed.py)
# ==========================

def quat_to_matrix(q):
    w, x, y, z = q
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-6:
        return np.identity(4, dtype=np.float32)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), 0],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w), 0],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y), 0],
        [0, 0, 0, 1]
    ], dtype=np.float32)


C = np.array([
    [0,  -1,  0, 0],
    [-1,  0, 0, 0],
    [0, 0,  1, 0],
    [0,  0,  0, 1]
], dtype=np.float32)


def imu_to_opengl(q):
    R = quat_to_matrix(q)
    return C @ R @ C.T


def quat_from_axis_angle(axis, angle_deg):
    theta = math.radians(angle_deg)
    w = math.cos(theta/2)
    x, y, z = (a * math.sin(theta/2) for a in axis)
    return (w, x, y, z)


# ==========================
# Drawing helpers
# ==========================

def cube(size):
    s = size
    glBegin(GL_QUADS)
    glColor3f(0.8,0.1,0.1)
    glVertex3f(-s,-s,s); glVertex3f(s,-s,s); glVertex3f(s,s,s); glVertex3f(-s,s,s)
    glVertex3f(-s,-s,-s); glVertex3f(-s,s,-s); glVertex3f(s,s,-s); glVertex3f(s,-s,-s)
    glVertex3f(-s,s,-s); glVertex3f(-s,s,s); glVertex3f(s,s,s); glVertex3f(s,s,-s)
    glVertex3f(-s,-s,-s); glVertex3f(s,-s,-s); glVertex3f(s,-s,s); glVertex3f(-s,-s,s)
    glVertex3f(-s,-s,-s); glVertex3f(-s,-s,s); glVertex3f(-s,s,s); glVertex3f(-s,s,-s)
    glVertex3f(s,-s,-s); glVertex3f(s,s,-s); glVertex3f(s,s,s); glVertex3f(s,-s,s)
    glEnd()


def arm():
    length, width, height = 1.5, 0.07, 0.04
    glColor3f(0.15,0.15,0.15)
    glBegin(GL_QUADS)
    glVertex3f(-length,-width,height); glVertex3f(length,-width,height)
    glVertex3f(length,width,height); glVertex3f(-length,width,height)
    glVertex3f(-length,-width,-height); glVertex3f(-length,width,-height)
    glVertex3f(length,width,-height); glVertex3f(length,-width,-height)
    glEnd()


def motor():
    glPushMatrix()
    glScalef(0.18,0.18,0.05)
    cube(1)
    glPopMatrix()


def draw_quad():
    cube(0.25)
    for angle in [45,-45]:
        glPushMatrix()
        glRotatef(angle,0,0,1)
        arm()
        glPushMatrix(); glTranslatef(1.5,0,0); motor(); glPopMatrix()
        glPushMatrix(); glTranslatef(-1.5,0,0); motor(); glPopMatrix()
        glPopMatrix()


def draw_body_axis_gizmo():
    """
    Draws RED/GREEN/BLUE lines along body X/Y/Z, expressed directly in
    BODY-frame coordinates. Since this is drawn using the SAME
    imu_to_opengl-transformed matrix as the vehicle, if the axis
    mapping is correct, the RED line should always point along the
    vehicle's nose, GREEN along its right side, BLUE toward its belly
    -- regardless of how the vehicle is rotated.
    """
    glLineWidth(4)
    glBegin(GL_LINES)
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(2.2,0,0)   # body X (forward)
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,2.2,0)   # body Y (right)
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,2.2)   # body Z (down)
    glEnd()
    glLineWidth(1)


# ==========================
# OpenGL setup
# ==========================

pygame.init()
width, height = 1000, 800
pygame.display.set_mode((width,height), DOUBLEBUF | OPENGL)
pygame.display.set_caption("Axis mapping self-test")

glMatrixMode(GL_PROJECTION)
gluPerspective(45, width/height, 0.1, 100)
glMatrixMode(GL_MODELVIEW)
glEnable(GL_DEPTH_TEST)

clock = pygame.time.Clock()


# ==========================
# Test sequence
# ==========================
# Each phase: (label, body-frame axis to rotate about, hold seconds)
phases = [
    ("IDENTITY (no rotation)", (0,0,0), 1.5),
    ("PURE ROLL  -- red gizmo axis should sweep, model should bank",  (1,0,0), 3.0),
    ("IDENTITY (reset)",       (0,0,0), 1.0),
    ("PURE PITCH -- green gizmo axis should sweep, nose should tip", (0,1,0), 3.0),
    ("IDENTITY (reset)",       (0,0,0), 1.0),
    ("PURE YAW   -- blue gizmo axis should sweep, model should turn", (0,0,1), 3.0),
]

start_time = time.time()
phase_idx = 0
phase_start = start_time

print("Watch the console for the current phase label.")
print("Watch which gizmo axis (RED/GREEN/BLUE) visibly sweeps for each phase.\n")

running = True
last_printed_phase = -1

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    now = time.time()
    elapsed_in_phase = now - phase_start

    label, axis, duration = phases[phase_idx]

    if last_printed_phase != phase_idx:
        print(f">>> {label}")
        last_printed_phase = phase_idx

    if elapsed_in_phase > duration:
        phase_idx = (phase_idx + 1) % len(phases)
        phase_start = now
        elapsed_in_phase = 0

    # oscillate angle 0 -> 40 -> 0 degrees over the phase duration
    t = elapsed_in_phase / duration
    angle_deg = 40 * math.sin(math.pi * t) if axis != (0,0,0) else 0

    q = quat_from_axis_angle(axis, angle_deg) if axis != (0,0,0) else (1,0,0,0)

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    # Oblique "chase cam": positioned above and to the side, looking at
    # the origin. A straight-on camera (staring down the nose) makes
    # roll and yaw visually indistinguishable on a flat, unlit shape --
    # this offset guarantees no single rotation axis lines up with the
    # camera's line of sight, so all three rotations read unambiguously.
    gluLookAt(
        4, 3, 6,   # eye position (up and to the side)
        0, 0, 0,   # look-at target (the vehicle)
        0, 1, 0    # up vector
    )

    R = imu_to_opengl(q)
    glMultMatrixf(R.T.flatten())

    draw_quad()
    draw_body_axis_gizmo()

    pygame.display.flip()
    clock.tick(60)

pygame.quit()