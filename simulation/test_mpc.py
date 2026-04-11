import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.mpc import MPCPlanner, MPCConfig

def main():
    print("MPC Trajectory Tracking Simulation Started!!")
    
    config = MPCConfig()
    planner = MPCPlanner(config)
    
    # Referans Yol (Sinüs Dalgası)
    ref_x = np.linspace(0, 50, 100)
    ref_y = [2.0 * math.sin(ix / 5.0) for ix in ref_x]
    
    # Başlangıç durumu [x, y, v, yaw]
    x = np.array([0.0, 1.0, 2.0, 0.0]) 
    
    history_x, history_y = [], []
    
    for _ in range(200):
        # Basitleştirilmiş kontrol (Normalde burada bir QP solver çalışır)
        # Sadece ileriye dönük bir tahmin ufku çizelim
        u = np.array([[0.1], [0.1]]) # Sabit ivme ve direksiyon (örnek)
        
        # Bir adım ilerle (Motion Model)
        dt = 0.1
        x[0] += x[2] * math.cos(x[3]) * dt
        x[1] += x[2] * math.sin(x[3]) * dt
        x[3] += x[2] / 2.5 * math.tan(u[1, 0]) * dt
        
        history_x.append(x[0])
        history_y.append(x[1])

        if x[0] > 45: break

        # Animasyon
        plt.cla()
        plt.plot(ref_x, ref_y, "--k", label="Referans Yol")
        plt.plot(history_x, history_y, "-b", label="Araç İzleği")
        plt.plot(x[0], x[1], "or", label="Araç")
        
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.pause(0.01)

    plt.show()

if __name__ == '__main__':
    main()
