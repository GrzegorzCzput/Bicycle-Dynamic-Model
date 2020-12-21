import math
import numpy as np
import pygame
import pygame.freetype


done = False
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 200, 0)
RED = (255, 0, 0)


ratio = 10  # Ratio between calculated model postion and display model postion, calculated model ostion [x,y] display model postion [x*ratio,y*ratio]

screen = pygame.display.set_mode([1600, 800])
pygame.display.set_caption("PUT Motorsport Driverles Model")
clock = pygame.time.Clock()
pygame.font.init()
pygame.init()
myfont = pygame.font.SysFont('Comic Sans MS', 50)




max_steer = np.radians(30.0)  # [rad] 
max_velocity = 20.0  # [m/s] 
L = 2.5 # [m]
dt = 0.1
Lr = L / 2.0  # [m]
Lf = L - Lr
Cf = 1600.0 * 2.0  # Tire stiffness N/slip
Cr = 1700.0 * 2.0  # Tire stiffness N/slip
Iz = 5250.0  # kg/m2
m = 2500.0  # kg



 
class BicycleModel():
    def __init__(self, x=800.0/ratio, y=400.0/ratio, yaw=0.0, vx=0.4, vy=0, omega=0):
        self.x = x 
        self.y = y
        self.yaw = yaw
        self.vx = vx
        self.vy = vy
        self.Lf = Lf
        self.Lr = Lr
        self.omega = omega
        self.length = L
        self.delt = 0
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.10
        

    def update(self, throttle, delta):
        """
        Calculates model positon, velocity ... based on non-linear bicycle model 
        :param throttle: (float) [m/s2]
        :param delta: (float) [rad] 
        :return: none
        """
        delta = np.clip(delta, -max_steer, max_steer)
        self.delt = delta
        self.x = self.x + self.vx * math.cos(self.yaw) * dt - self.vy * math.sin(self.yaw) * dt
        self.y = self.y + self.vx * math.sin(self.yaw) * dt + self.vy * math.cos(self.yaw) * dt
        self.yaw = self.yaw + self.omega * dt
        self.yaw = normalize_angle(self.yaw)
        Ffy = -Cf * math.atan2(((self.vy + Lf * self.omega) / self.vx - delta), 1.0)
        Fry = -Cr * math.atan2((self.vy - Lr * self.omega) / self.vx, 1.0)
        R_x = self.c_r1 * self.vx
        F_aero = self.c_a * self.vx ** 2
        F_load = F_aero + R_x
        self.vx = self.vx + (throttle - Ffy * math.sin(delta) / m - F_load/m + self.vy * self.omega) * dt
        velocity = np.clip(self.vx, 0.4, max_velocity)
        self.vx = velocity
        self.vy = self.vy + (Fry / m + Ffy * math.cos(delta) / m - self.vx * self.omega) * dt
        self.omega = self.omega + (Ffy * Lf * math.cos(delta) - Fry * Lr) / Iz * dt

    def show_vehicle(self):
          pygame.draw.polygon(screen, GREEN, rect(self.x, self.y, self.yaw, self.length*ratio, 0.4*ratio))
          pygame.draw.polygon(screen, RED, rect(self.x + (self.Lf) * math.cos(self.yaw),
                                              self.y + (self.Lf) * math.sin(self.yaw),
                                              self.delt + self.yaw, 1*ratio, 0.5*ratio))
          pygame.draw.polygon(screen, RED, rect(self.x - (self.Lr) * math.cos(self.yaw), 
                                              self.y - (self.Lr)* math.sin(self.yaw),  
                                              self.yaw, 1*ratio, 0.5*ratio))
          pygame.draw.polygon(screen, BLUE, rect(self.x, self.y,  0.0, 4.0, 4.0))
          


def rect(x, y, angle, w, h):
    return [translate(x*ratio, y*ratio, angle, -w/2,  h/2),
            translate(x*ratio, y*ratio, angle,  w/2,  h/2),
            translate(x*ratio, y*ratio, angle,  w/2, -h/2),
            translate(x*ratio, y*ratio, angle, -w/2, -h/2)]


def translate(x, y, angle, px, py):
    x1 = x + px * math.cos(angle) - py * math.sin(angle)
    y1 = y + px * math.sin(angle) + py * math.cos(angle)
    return [int(x1), int(y1)]


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle    


def round_up(n, decimals=0):
    multiplier = 10 ** decimals
    return math.ceil(n * multiplier) / multiplier


B_Model = BicycleModel()


while not done:

    clock.tick(80)
    screen.fill(BLACK)
    pressed = pygame.key.get_pressed()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True


    if pressed[pygame.K_RIGHT]:
      steering = 0.2
    elif pressed[pygame.K_LEFT]:
      steering = -0.2
    else:
      steering = 0

    if pressed[pygame.K_UP]:
      acceleration = 0.8
      throttle_str = pygame.font.Font.render(myfont,'THROTTLE', 1, BLUE)
      screen.blit(throttle_str,(1360,100))
    elif pressed[pygame.K_DOWN]:
      acceleration = -0.8
      break_str = pygame.font.Font.render(myfont,'BREAK', 1, RED)
      screen.blit(break_str,(1390,150))
    else:
      acceleration = 0

    if pressed[pygame.K_SPACE]:
      B_Model.x = 800.0/ratio
      B_Model.y = 400.0/ratio
   
    velocity_string = str(round_up(B_Model.vx,2)) + ' m/s'
    speed = pygame.font.Font.render(myfont,velocity_string, 1, WHITE)
    screen.blit(speed,(1390,50))

    
    B_Model.update(acceleration, steering)
    B_Model.show_vehicle()
    pygame.display.flip()





pygame.quit()