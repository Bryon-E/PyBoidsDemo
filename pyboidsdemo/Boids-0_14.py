#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 10 18:32:58 2024

Boids App copied from previous prototype iteration
This build is focused on creating Quad-Trees. 
Goal is to add Quad-Trees, reducing time complexity from O(n^2) -> O(n*log(n))

Most parameters are constants and declared privately. To be improved as needed.
@author: bread



Boid visualiser. 
Creates a window that displays a set of boids, created at the center of the 
screen which follow simple SAC (Separation, Alignment, Cohesion) procedure.
All boids inherit from Boid which hosts the vector creation from position
parameter tuple (x, y), the 

"""
import pygame
import math as m
import random as r
import numpy as np
import time

from math import sqrt   

class QuadTreeNode:
    def __init__(self, x, y, width, height, points):
        self.x0 = x
        self.y0 = y
        self.width = width
        self.height = height
        self.points = points
        self.children = []
    
    
    
    
class QuadTree:
    def __init__(self, cutoff, boids, resolution: list):
        self.threshold = cutoff
        self.boids = boids
        # Create root node which hosts all other nodes
        self.root = QuadTreeNode(0, 0, resolution[0], resolution[1], self.boids)
        self.segments = 0
    
    def contains(self, x, y, w, h, points):
       pts = []
       for point in points:
           if point.position[0] >= x and point.position[0] <= x+w and point.position[1] >= y and point.position[1] <= y+h:
               pts.append(point)
        
       return pts    
   
    
    def recursive_subdivide(self, node):
      if len(node.points)<=self.threshold:
          return
      
      w_ = float(node.width/2)
      h_ = float(node.height/2)

      p = self.contains(node.x0, node.y0, w_, h_, node.points)
      x1 = QuadTreeNode(node.x0, node.y0, w_, h_, p)
      self.recursive_subdivide(x1)

      p = self.contains(node.x0, node.y0+h_, w_, h_, node.points)
      x2 = QuadTreeNode(node.x0, node.y0+h_, w_, h_, p)
      self.recursive_subdivide(x2)

      p = self.contains(node.x0+w_, node.y0, w_, h_, node.points)
      x3 = QuadTreeNode(node.x0 + w_, node.y0, w_, h_, p)
      self.recursive_subdivide(x3)

      p = self.contains(node.x0+w_, node.y0+h_, w_, h_, node.points)
      x4 = QuadTreeNode(node.x0+w_, node.y0+h_, w_, h_, p)
      self.recursive_subdivide(x4,)

      node.children = [x1, x2, x3, x4]    
       
      def find_children(self, node):
          if not node.children:
              return [node]
          csum = []
          for child in node.children:
              csum += self.find_children(node)
          return csum
      
      # query root node for all sub-nodes, call find_children for all 
      #def query(self, node):
          
     

class Boid:
    def __init__(self, pos: [int, int], _id: int, resolution, fixation = None):
        self.id = _id
        # Default parameters 
        self.separation = 25
        self.alignment = 75
        self.cohesion = 75
        self.position = pos
        self.velocity = [0,0]
        self.speed_limit = 15
        self.fixation = fixation
        self.bounds = [[self.separation, self.separation], [ u - self.separation for u in resolution ]]
        self.color = (r.randint(0, 255),r.randint(0, 255),r.randint(0, 255))
        
        
    # Set parameters after creation
    def set_parameters(self, **kwargs):
        if 'bounds' in kwargs:
            self.bounds = [[self.separation, self.separation], [ u - self.separation for u in kwargs['bounds'] ]]
        if 'limits' in kwargs:
            self.limits = kwargs['limits']
        if 'separation' in kwargs:
            self.separation = kwargs['separation']
        if 'alignment' in kwargs:
            self.alignment = kwargs['alignment']
        if 'cohesion' in kwargs:
            self.cohesion = kwargs['cohesion']    
        if 'position' in kwargs:
            self.position = kwargs['position']
        if 'velocity' in kwargs:
            self.velocity = kwargs['velocity']
        if 'accel' in kwargs:
            self.accel = kwargs['velocity']
        if 'color' in kwargs:
            self.color = kwargs['color']
        if 'fixation' in kwargs:
            self.fixation = kwargs['fixation']
    
    def __str__(self):
        return 'Boid ID: {}, Position: {}, Velocity: {}'.format(
            self.id,
            self.position,
            self.velocity
            
            )
    
    # Returns magnitude of a vector in a way that's efficient without numpy 
    # ||v|| = square root(vx**2 + vy**2)
    def magnitude(self, vector: list):
        mag = sqrt(sum([ i ** 2 for i in vector ]))
        return mag
    
    
    # Returns normalised vector indicating direction of velocity
    def direction(self, vector, magnitude=None):
        if magnitude == None:
            mag = self.magnitude(vector)
        else: 
            mag = magnitude
            
            
        if mag != 0:
            norm = []
            for i in range(len(vector)):
                norm.append(vector[i] / mag)
        
            return norm
        return [0, 0]
    
    # Returns Euclidean distance between two points: Dist = sqrt((x2-x1)^2 + (y2-y1)^2)
    def euc_distance(self, other):
        dx = (other.position[0] - self.position[0])**2
        dy = (other.position[1] - self.position[1])**2
        return sqrt(dx + dy)
    
    # Returns a vector between two points: Vector = [ (x2 - x1), (y2, y1) ]
    def vec_distance(self, other):
        dx = (other.position[0] - self.position[0])
        dy = (other.position[1] - self.position[1])
        return [ dx, dy ]
    
    '''
    Parameters: 
        - deltaaccel = []
            Computed as scaled by scalar deltatime during frame updates
            
    Prodedure:
        Adds deltavel vector to internal boid position, assigning to self.
        
        Once assigned, X and Y velocities are determined using the 
        Vector.x and Vector.y attributes.
        
    '''
    def move(self, boids, deltatime):
        self.scount, self.acount, self.ccount = 0, 0, 0
        sep_accel = [0, 0]
        ali_accel = [0, 0]
        coh_pos = [0, 0]
        for bj in boids:
            
            # Direct distance to boid
            eudist = self.euc_distance(bj)
            
            # Pure axis positional distance
            dist = [ bjp - bp for bjp, bp in zip(bj.position, self.position) ]
            
            # Assesses all rules for EVERY other boid                 
            sep_accel = np.add(sep_accel, self.separation_accel(bj, dist, eudist))
            ali_accel = np.add(ali_accel, self.alignment_accel(bj, eudist))
            coh_pos = np.add(coh_pos, self.cohesion_accel(bj, eudist))
            
        # Averages all relevant accelerations
        if self.scount != 0:
            sep_accel = [ val / self.scount for val in sep_accel ]
        # average and then reduce to change of 5%
        if self.acount != 0:
            ali_accel = [ (val / self.acount) * .18 for val in ali_accel ]
        # average position
        if self.ccount != 0:
            coh_pos = [ val / self.ccount for val in coh_pos ]
       
        # list comprehension renders the position a vector and then takes a percentage to pass as velocity
        coh_vec = [ (cp - sp) *.12 for cp, sp in zip(coh_pos, self.position) ]
        deltaaccel = np.add(sep_accel, coh_vec)
        
        # Allows for left mouse button to add a fixation point to boid movement
        if self.fixation != None:
            fixation_accel = np.add(deltaaccel, self.fixation_accel())
            deltaaccel = fixation_accel
        else:
            pass
       
        # Throttles velocity to speed limit
        throttled_vel = self.limit_speed(np.add( self.velocity, deltaaccel ))
        
        # Modifies speed to include distance from boundaries
        bounded_vel     = self.boundary_acceleration(throttled_vel)
        
        
        self.velocity = bounded_vel
        self.position = np.add(self.position, self.velocity)
        
        
        
        
     
    '''
    seperation_accel: function = inverse average distance from other boids                              /// complete, returns inverse vector of dist to nearby boids
        Distance marked by boid internal separation rule.
        -> Functionality of this rule is in identifying a vector betweeen 
        two boids based on euclidean distance.
        
        
    alignment_accel: function = vector averaging other boids velocities and adding proportion in        /// complete, returns velocity of nearby boids
        Average velocities amongst all other boids within alignment distance.
        -> Functionality of this rule averages velocities of all boids nearby
        
    cohesion_accel: function = vector averaging position to other boids and adding proportion in        /// complete, returns position of nearby boids
        Distance marked by boid internal cohesion distance
        -> Functionality of this rule identifies average position of all boids,
        identifies vector between current position and perceived center
    
    
    '''    
    # Adds velocity changes per velocity of respective boid
    def separation_accel(self, bj: 'Boid', dist: list, eudist: int):
        accel = [0, 0]
        
        # Boids are literally on top of each other (get a room)
        if eudist == 0:
            return [0, 0]
        
        # one or both velocity axes are within separation distance (not leaving space for jesus)
        # vector return containing x and y distance from target
        elif eudist < self.separation:
            accel = [ -d for d in dist ]
        
        
        
        if any([ x != 0 for x in accel]) :
            self.scount += 1
            
        return accel
    
    # Adds velocity changes per the perceived average direction of boids
    def alignment_accel(self, bj: 'Boid', eudist: int):
       # Check within distance, ticks counter, adds velocity of boid
       if eudist == 0:
           return [0, 0]
       
       if eudist < self.alignment:
           self.acount += 1
           return bj.velocity
           
       return [0, 0]
           
    # Sums all respective positions of boids
    def cohesion_accel(self, bj: 'Boid', eudist: int):
        # Perceived center added to running total
        if eudist == 0:
            return [0, 0]
        
        if eudist < self.cohesion:
            self.ccount += 1
            return bj.position
        
        return [0, 0]
    
    # Finds vector between boid and fixation point
    def fixation_accel(self):
        fixation_vec = [ (fp - bp) * .24 for fp, bp, in zip(self.fixation, self.position)]
        
        return fixation_vec
    
    
    '''
    Limits magnitude of boid, limiting maximum speed of boid.
    New (Δx/s**2, Δy/s**2) passed after magnitude throttled to speed and 
    new speed forward returned
    ''' 
    def limit_speed(self, deltaaccel):
        deltamag = self.magnitude(deltaaccel)
        deltanorm = self.direction(deltaaccel, magnitude = deltamag)
        
        if deltamag > self.speed_limit:
            deltamag = self.speed_limit
        
        throttled_accel = [ i * deltamag for i in deltanorm ]
        
        return throttled_accel
    
    # Adds inverse vector of distance to boundary domain and range
    def boundary_acceleration(self, deltavel):
        # determine new destination
        new_pos = [ p + dv for p, dv in zip(self.position, deltavel)]
        
        for i in range(len(new_pos)):
            if new_pos[i] + deltavel[i] < self.bounds[0][i]:
                deltavel[i] =  self.bounds[0][i] - new_pos[i]
            
            if new_pos[i] + deltavel[i] > self.bounds[1][i]:
                deltavel[i] = self.bounds[0][i] - new_pos[i]
        
        
        return deltavel
    
    
    # Velocity as a list (Δx/s, Δy/s) passed to limit and return velocity
    # within target parameters
    def limit_velocity(self, velocity, vel_lim):
        for i in range(len(velocity)):
            if abs(velocity[i]) >= vel_lim[i]:
                sign = abs(velocity[i]) / velocity[i]
                velocity[i] = vel_lim[i]*(sign)
        return velocity
    
    '''
    def throttle_vel(self):
        
        if self.vel >= self.limit:
            
        for i in (0, 1):
            if abs(dv[i]) > lim:
                lv[i] = (dv[i] / abs(dv[i])) * lim
        return lv
    '''

    
    
class Window:
    def __init__( self, resolution: [int, int], running: bool, num_boids):
        # Whether everything is initialized or not
        self.running = running
        self.resolution = resolution
        self.debug_text = []
        self.boids_total = num_boids
        self.posx = self.resolution[0] / self.boids_total
        self.posy = self.resolution[1] / self.boids_total
        
        # Define window parameters
        self.screen = pygame.display.set_mode(self.resolution)
        self.surface = pygame.display.get_surface()
        self.font = pygame.font.SysFont('Arial', 14)
    '''
    Alternative render strategy: Render individual boids when passed boids by 
    central App manager. 
    '''
        
    def render_boid_debug(self, boid):
        self.debug_text.append(self.font.render(str(boid), True, (0, 0, 0)))
        self.screen.blit(self.debug_text[boid.id], 
                         (self.posx, self.posy 
                          + (boid.id * 14) 
                          + (boid.id * 15)
                          ))  
    
    def render_boid(self, boid, shape='circle'):
        if shape == 'circle':
            pygame.draw.circle(self.screen, boid.color, boid.position, 5)
        elif shape == 'arrow':
            # arrow indicating direction at any given time
            pass
                

class App:
    def __init__(self, debug = False):
        # Start pygame module
        pygame.init()
        
        # Display renewal and deltatime constants
        self.clock = pygame.time.Clock()
        self.FPS = 60
        self.ANIMATION_DT = 1000
        self.running = True
        self.resolution = [1400, 800]
        self.boid_total = 50
        self.debug = debug
        self.window = Window(self.resolution, self.running, self.boid_total)
        self.focus_point = None
        
        # slows boids speeds down
        self.friction = 0

        
      
    def create_boids(self, amount: int, shape='default'):
        self.shape = shape
        if shape == 'default':
            self.boids = []
            for i in range(0, amount):
                x = r.randint(0, self.resolution[0])
                y = r.randint(0, self.resolution[1])
                boid = Boid((x,y), i, self.resolution)
                self.boids.append(boid)  
                self.window.render_boid(boid)
                if self.debug:
                    self.window.render_boid_debug(boid)
                    
    def run(self):
        self.create_boids(self.boid_total)
        self.window.surface.fill((146, 215, 245))
        
        while self.running:
            # Catch pygame events during runtime, starting with QUIT
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    pygame.quit()
                    break
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.dict['button'] == 1:
                    self.focus_point = list(pygame.mouse.get_pos())
                    continue
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.dict['button'] == 3:
                    self.focus_point = None
                    continue
                    
                
                    
                    
            self.window.screen.fill((146, 215, 245))
            self.clock.tick(self.FPS)
            dt = self.clock.get_time()/self.ANIMATION_DT 
            
            t0 = time.time()
            qtree = QuadTree(3, self.boids, self.resolution)
            qtree.recursive_subdivide(qtree.root)
            t1 = time.time()
            print(dt)
            print(f'QuadTree Recursive Subdivide Time: {t1-t0} sec')
            '''
            Find boids nearby based on quadtree positional evaluations and 
            pass boids into boid movement. Any boids within a boid's 
            rule limits and restrictions are passed in. 
            
            Modifies time complexity from O(n^2) to o(n*log(n)). 
            (For 1000 boids, operations per cycle goes from 1.000.000 to 13,000)
            
            '''
            
            if self.shape == 'default':
                for boid in self.boids:
                    if self.focus_point != boid.fixation:
                        boid.set_parameters(fixation = self.focus_point)
                        
                    try:
                        boid.move(self.boids, dt)
                    except Exception as error:
                        print(error)
                        pygame.quit()
                        
                    self.window.render_boid(boid)
                    if self.debug:
                        self.window.render_boid_debug(boid)
            
            
            pygame.display.flip()

        pygame.display.quit()
        
def main():
    app = App()
    app.run()
    
if __name__ == '__main__':
    main()
