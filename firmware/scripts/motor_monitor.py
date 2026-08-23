"""
Reads 4 uint16_t motor speed values over serial and displays them.
 
Packet format: 4x little-endian uint16_t, 8 bytes total, order:
    front_left, front_right, back_left, back_right
Each value expected in range 0-2047.
 
Displays:
  1. A live terminal readout (always works, no extra deps beyond pyserial).
  2. A pygame top-down visualizer showing motor speed bars plus an
     estimated roll/pitch/yaw *tendency* derived from differential
     thrust between motor pairs.
 
IMPORTANT CAVEAT:
Motor speed alone does not determine true attitude -- that requires
real dynamics (mass, inertia, thrust curves, integration over time) or
an actual IMU/AHRS. The roll/pitch/yaw shown here are simplified
DIRECTIONAL TENDENCIES based on standard X-configuration mixing
assumptions:
 
    roll  tendency ~ (front_left + back_left) - (front_right + back_right)
    pitch tendency ~ (back_left + back_right) - (front_left + front_right)
    yaw   tendency ~ (front_left + back_right) - (front_right + back_left)
 
The yaw formula assumes the common diagonal-pairing convention (FL/BR
spin one direction, FR/BL spin the other). If your frame uses a
different spin/motor layout, the sign or pairing may need to change --
verify against your actual mixer before trusting the yaw indicator.
"""
 
import sys
import struct
import serial
import pygame
from pygame.locals import *
 
 
# ==========================
# Serial configuration
# ==========================
 
PORT = "COM7"      # change this
BAUD = 115200
MAX_SPEED = 2047
 
ser = serial.Serial(PORT, BAUD, timeout=0)
 
 
# ==========================
# Terminal display
# ==========================
 
BAR_WIDTH = 40
 
def speed_bar(value, max_value=MAX_SPEED, width=BAR_WIDTH):
    filled = int(width * value / max_value) if max_value else 0
    filled = max(0, min(width, filled))
    return "#" * filled + "-" * (width - filled)
 
 
def print_terminal(fl, fr, bl, br):
    # \033[H moves cursor home, \033[J clears from cursor down --
    # avoids terminal flicker vs. a full clear-screen call each frame.
    sys.stdout.write("\033[H\033[J")
    sys.stdout.write("Motor speeds (0-%d)\n" % MAX_SPEED)
    sys.stdout.write("-" * 60 + "\n")
    sys.stdout.write(f"Front Left  [{speed_bar(fl)}] {fl:5d}\n")
    sys.stdout.write(f"Front Right [{speed_bar(fr)}] {fr:5d}\n")
    sys.stdout.write(f"Back Left   [{speed_bar(bl)}] {bl:5d}\n")
    sys.stdout.write(f"Back Right  [{speed_bar(br)}] {br:5d}\n")
    sys.stdout.flush()
 
 
# ==========================
# Attitude tendency estimate
# ==========================
 
def estimate_tendencies(fl, fr, bl, br):
    """
    Returns (roll, pitch, yaw) each normalized to roughly [-1, 1].
    See module docstring for the assumptions behind these formulas.
    """
    span = 2 * MAX_SPEED  # max possible differential between two-motor sums
 
    roll = ((fl + bl) - (fr + br)) / span
    pitch = ((bl + br) - (fl + fr)) / span
    yaw = ((fl + br) - (fr + bl)) / span
 
    return roll, pitch, yaw
 
 
# ==========================
# Pygame visualizer
# ==========================
 
WIDTH, HEIGHT = 700, 700
CENTER = (WIDTH // 2, HEIGHT // 2)
ARM_LEN = 180
MOTOR_RADIUS_MAX = 45
 
BG_COLOR = (18, 18, 22)
ARM_COLOR = (90, 90, 100)
BODY_COLOR = (200, 200, 210)
TEXT_COLOR = (230, 230, 230)
TILT_COLOR = (240, 90, 90)
 
# Motor screen positions relative to center, in an X-configuration,
# using standard screen coordinates (y grows downward in pygame).
MOTOR_OFFSETS = {
    "FL": (-ARM_LEN, -ARM_LEN),
    "FR": (ARM_LEN, -ARM_LEN),
    "BL": (-ARM_LEN, ARM_LEN),
    "BR": (ARM_LEN, ARM_LEN),
}
 
 
def motor_color(speed):
    t = max(0.0, min(1.0, speed / MAX_SPEED))
    # blue (slow) -> green (mid) -> red (fast)
    if t < 0.5:
        k = t / 0.5
        return (int(30 + k * 30), int(80 + k * 150), int(220 - k * 150))
    else:
        k = (t - 0.5) / 0.5
        return (int(60 + k * 195), int(230 - k * 150), int(70 - k * 40))
 
 
def draw_frame(screen, font, fl, fr, bl, br, roll, pitch, yaw):
    screen.fill(BG_COLOR)
 
    # Arms (drawn as a fixed X shape -- this is a static top-down
    # frame outline, not itself tilted, since roll/pitch here are
    # differential-thrust indicators rather than a real 3D attitude).
    for dx, dy in MOTOR_OFFSETS.values():
        pygame.draw.line(screen, ARM_COLOR, CENTER,
                          (CENTER[0] + dx, CENTER[1] + dy), 6)
 
    # Body
    pygame.draw.circle(screen, BODY_COLOR, CENTER, 28)
 
    # Motors
    speeds = {"FL": fl, "FR": fr, "BL": bl, "BR": br}
    for label, (dx, dy) in MOTOR_OFFSETS.items():
        pos = (CENTER[0] + dx, CENTER[1] + dy)
        speed = speeds[label]
        radius = 12 + int(MOTOR_RADIUS_MAX * speed / MAX_SPEED)
        pygame.draw.circle(screen, motor_color(speed), pos, radius)
        pygame.draw.circle(screen, (0, 0, 0), pos, radius, 2)
 
        label_surf = font.render(f"{label} {speed}", True, TEXT_COLOR)
        rect = label_surf.get_rect(center=(pos[0], pos[1] + radius + 16))
        screen.blit(label_surf, rect)
 
    # Roll indicator: horizon-style line, tilted by roll tendency
    roll_angle = roll * 40  # degrees, scaled for visibility
    import math
    hx = math.cos(math.radians(roll_angle)) * 120
    hy = math.sin(math.radians(roll_angle)) * 120
    pygame.draw.line(screen, TILT_COLOR,
                      (CENTER[0] - hx, CENTER[1] - hy),
                      (CENTER[0] + hx, CENTER[1] + hy), 4)
 
    # Pitch indicator: vertical bar offset by pitch tendency
    pitch_y = CENTER[1] + int(pitch * 100)
    pygame.draw.line(screen, TILT_COLOR,
                      (CENTER[0] - 15, pitch_y), (CENTER[0] + 15, pitch_y), 4)
 
    # Yaw indicator: curved arrow direction shown as a rotated wedge
    yaw_text = font.render(
        f"roll {roll:+.2f}   pitch {pitch:+.2f}   yaw {yaw:+.2f}",
        True, TEXT_COLOR
    )
    screen.blit(yaw_text, (20, HEIGHT - 40))
 
    pygame.display.flip()
 
 
# ==========================
# Main loop
# ==========================
 
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Motor speed / attitude tendency visualizer")
    font = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()
 
    fl = fr = bl = br = 0
 
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
 
        # Read all fully-available 8-byte packets, keep the latest.
        while ser.in_waiting >= 8:
            packet = ser.read(8)
            fl, fr, bl, br = struct.unpack("<4H", packet)
 
        print_terminal(fl, fr, bl, br)
        roll, pitch, yaw = estimate_tendencies(fl, fr, bl, br)
        draw_frame(screen, font, fl, fr, bl, br, roll, pitch, yaw)
 
        clock.tick(60)
 
    ser.close()
    pygame.quit()
 
 
if __name__ == "__main__":
    main()