from this import d
import matplotlib.animation as anim
import time
import numpy as np
import matplotlib.pyplot as plt
from simple_pid import PID
import math as ma
import sys

class PID_3D:
    ################## FONCTIONS DE BASE ##########################
    def __init__(self, Kx=(1,1,1), Ky=(1,1,1), Kz=(1,1,1), cible=(0,0,0), temps_pause=None):
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
        self.pid_z = PID(Kz[0], Kz[1], Kz[2], setpoint=cible[2], sample_time=temps_pause)

        self.x = 0
        self.y = 0
        self.z = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.target = cible
        self.pasdetemps = temps_pause
        if temps_pause == None:
            self.pasdetemps = 0.01
    
    def set_target(self, target=(0,0,0)):
        self.pid_x.setpoint = target[0]
        self.pid_y.setpoint = target[1]
        self.pid_z.setpoint = target[2]
        self.target = target
    
    def set_position(self, position=(0,0,0)):
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
    
    def correction(self):
        '''
        Fonction PID 3D principale qui met a jour les variables de self selon la position du drone et de la cible.
        '''
        self.vx = self.pid_x(self.x)
        self.vy = self.pid_y(self.y)
        self.vz = self.pid_z(self.z)
        print("pos_x = {0:.2f} | pos_y = {1:.2f} | pos_z = {2:.2f}  ||  v_x = {3:.2f} | v_y = {4:.2f} | v_z = {5:.2f} |".format(self.x, self.y, self.z, self.vx, self.vy, self.vz))


    ####################################################################
    ############################### TESTS ##############################
    ####################################################################
    def Test_1D(self, xmax):
        '''
        Test du pid et affichage 1D du pid_x
        '''
        y = [0]
        x = [self.x]
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.plot(x, y)

        def update(i):
            self.correction()
            self.x = self.x + self.vx*self.pasdetemps
            x.append(self.x)
            y.append(i)
            ax.clear()
            ax.plot(y, x)
            plt.plot(range(200), [10 for i in range(200)])
            plt.plot(0,0, marker="o", markerfacecolor="green")

        a = anim.FuncAnimation(fig, update, frames=xmax, repeat=False)
        plt.show()
    
    def Test_2D(self, xmax):
        '''
        Test du pid et affichage 2D de pid_x et pid_y
        '''
        y = [self.y]
        x = [self.x]
        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)
        ax.plot(x, y)

        def update(i):
            self.correction()
            self.x = self.x + self.vx*self.pasdetemps
            x.append(self.x)
            self.y = self.y + self.vy*self.pasdetemps
            y.append(self.y)
            ax.clear()
            ax.plot(x, y)
            plt.plot(10, 10, marker="o", markeredgecolor="red", markerfacecolor="green")
            plt.plot(0,0, marker="o", markerfacecolor="green")

        a = anim.FuncAnimation(fig, update, frames=xmax, repeat=False)
        plt.show()
    
    def Test_3D(self, xmax):
        '''
        Test du pid et affichage 3D
        '''
        z = [self.z]
        y = [self.y]
        x = [self.x]
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(x, y)

        def update(i):
            self.correction()
            self.x = self.x + self.vx*self.pasdetemps
            self.y = self.y + self.vy*self.pasdetemps
            self.z = self.z + self.vz*self.pasdetemps
            x.append(self.x)
            y.append(self.y)
            z.append(self.z)
            ax.clear()
            ax.plot3D(x, y, z)
            ax.plot3D(10, 10, 10, marker="o", markeredgecolor="red", markerfacecolor="green")
            ax.plot3D(0,0,0, marker="o", markerfacecolor="green")

        a = anim.FuncAnimation(fig, update, frames=xmax, repeat=False)
        plt.show()

    def Test_3D_chemin(self, xmax, chemin):
        '''
        Test du pid et affichage 3D du suivi de chemin=(target1=(x,y,z), target2, etc...)
        '''
        self.set_target(chemin[0])
        print("target = ", self.target)
        z = [self.z]
        y = [self.y]
        x = [self.x]
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(x, y)

        def update(i):
            if ma.sqrt((self.target[0]-self.x)**2 + (self.target[1]-self.y)**2 + (self.target[2]-self.z)**2)<0.5:
                self.set_target(chemin[self.target[3]+1])
            self.correction()
            self.x = self.x + self.vx*self.pasdetemps
            self.y = self.y + self.vy*self.pasdetemps
            self.z = self.z + self.vz*self.pasdetemps
            x.append(self.x)
            y.append(self.y)
            z.append(self.z)
            ax.clear()
            ax.plot3D(x, y, z)
            ax.plot3D(10, 10, 10, marker="o", markeredgecolor="red", markerfacecolor="green")
            ax.plot3D(10, 20, 10, marker="o", markeredgecolor="red", markerfacecolor="green")
            ax.plot3D(20, 20, 10, marker="o", markeredgecolor="red", markerfacecolor="green")
            ax.plot3D(20, 10, 10, marker="o", markeredgecolor="red", markerfacecolor="green")
            ax.plot3D(0,0,0, marker="o", markerfacecolor="green")

        a = anim.FuncAnimation(fig, update, frames=xmax, repeat=False)
        plt.show()


def main():
    pid = PID_3D(Kx=(5,1,1), Ky=(5,1,1), Kz=(5,1,1), cible=(10,10,10), temps_pause=0.1)
    pid.Test_3D_chemin(100, [(10,10,10,0), (10,20,10,1), (20,20,10,2), (20,10,10,3), (0,0,0,4)]) #chemin = (x,y,z,i)


if __name__== "__main__":
    main()



class PID_3D:
    ################## FONCTIONS DE BASE ##########################
    def __init__(self, Kx=(1,1,1), Ky=(1,1,1), Kz=(1,1,1), cible=(0,0,0), temps_pause=None):
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
        self.pid_z = PID(Kz[0], Kz[1], Kz[2], setpoint=cible[2], sample_time=temps_pause)

        self.x = 0
        self.y = 0
        self.z = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.target = cible
        self.pasdetemps = temps_pause
        if temps_pause == None:
            self.pasdetemps = 0.01
    
    def set_target(self, target=(0,0,0)):
        self.pid_x.setpoint = target[0]
        self.pid_y.setpoint = target[1]
        self.pid_z.setpoint = target[2]
        self.target = target
    
    def set_position(self, position=(0,0,0)):
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
    
    def correction(self):
        '''
        Fonction PID 3D principale qui met a jour les variables de self selon la position du drone et de la cible.
        '''
        self.vx = self.pid_x(self.x)
        self.vy = self.pid_y(self.y)
        self.vz = self.pid_z(self.z)