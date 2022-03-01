import matplotlib.animation as anim
import time
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID
import math as ma
import sys

class PID_2D:
    ################## FONCTIONS DE BASE ##########################
    def __init__(self, Kx=(1,1,1), Ky=(1,1,1), cible=(0,0), temps_pause=None):
        '''
        Initialisation des 3 PID de l'espace -accessible via self.x, self.y et self.z- avec les paramètres suivants :
        - Kx, Ky, Kz de la forme Kx = (Kp, Ki, Kd) : gain proportionnel, intégral et dérivé pour chacun des 3 pid
        - cible = (x,y,z) : Position initiale de la cible.
        - temps_pause : Temps en secondes entre deux génération de valeur.
            Le PID fonctionne mieux quand il est appelé en continue, mais si besoin de discrétiser le temps c'est possible.
            Si temps_pause=None, le PID génère une nouvelle valeur à chaque fois qu'il est appelé.
        
        Plus de paramètres sont disponibles mais ne sont pas utiles dans notre cas d'utilisation, comme :
        - output_limits
        - auto_mode
        - proportional_on_measurement
        - error_map
        '''
        self.pid_x = PID(Kx[0], Kx[1], Kx[2], setpoint=cible[0], sample_time=temps_pause)
        self.pid_y = PID(Ky[0], Ky[1], Ky[2], setpoint=cible[1], sample_time=temps_pause)

        self.x = 0
        self.y = 0
        self.vx = 0
        self.vy = 0
        self.target = cible
    
    def set_target(self, target=(0,0)):
        self.pid_x.setpoint = target[0]
        self.pid_y.setpoint = target[1]
        self.target = target
    
    def set_position(self, position=(0,0)):
        self.x = position[0]
        self.y = position[1]
    
    def correction(self):
        '''
        Fonction PID 3D principale qui met a jour les variables de self selon la position du drone et de la cible.
        '''
        self.vx = self.pid_x(self.x)
        self.vy = self.pid_y(self.y)
        #print("pos_x = {0:.2f} | pos_y = {1:.2f} | pos_z = {2:.2f}  ||  v_x = {3:.2f} | v_y = {4:.2f} | v_z = {5:.2f} |".format(self.x, self.y, self.z, self.vx, self.vy, self.vz))

    ####################################################################
    ############################### TESTS ##############################
    ####################################################################
   
    def print_PID_2D(self, camera):
        '''
        Test du pid et affichage 3D du suivi de chemin=(target1=(x,y,z), target2, etc...)
        '''
        fig = plt.figure()
        ax = plt.axes()
        c = np.array(camera)
        ax.plot(c[:,0], c[:,1],label='Camera')
        ax.legend()
        plt.show()

    
