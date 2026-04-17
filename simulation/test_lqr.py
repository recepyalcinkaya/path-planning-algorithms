import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.lqr_controller import LQRController
# Araç fiziği için Stanley örneğinde yazdığımız State sınıfını kullanacağız
from algorithms.stanley_controller import State 

def main():
    print("LQR (Linear Quadratic Regulator) Simülasyonu Başlatıldı!!")

    # Referans Yol (Otoban virajı simülasyonu)
    cx = np.arange(0, 150, 0.5)
    cy = [math.sin(ix / 10.0) * (ix / 5.0) for ix in cx] # Giderek büyüyen bir yılan kavi
    cyaw = np.arctan2(np.gradient(cy), np.gradient(cx))

    target_speed = 30.0 / 3.6  # [m/s] Otoban hızı

    # Araç Durumu: Yolun 3 metre dışından başlasın
    state = State(x=0.0, y=-3.0, yaw=0.0, v=0.0)
    controller = LQRController()

    x_history, y_history = [state.x], [state.y]
    target_ind = 0

    # Simülasyon Döngüsü
    while target_ind < len(cx) - 1:
        # Basit ivme kontrolü
        accel = 1.0 * (target_speed - state.v)
        
        # LQR ile Optimal Direksiyon Kontrolü
        delta, target_ind, error = controller.lqr_steering_control(state, cx, cy, cyaw, target_ind)

        # Aracı güncelle
        state.update(accel, delta, controller.dt)

        x_history.append(state.x)
        y_history.append(state.y)

        # --- Animasyon ---
        plt.cla()
        plt.plot(cx, cy, "--k", label="Referans Rota")
        plt.plot(x_history, y_history, "-b", linewidth=2, label="LQR Takip Yörüngesi")
        plt.plot(cx[target_ind], cy[target_ind], "xg", label="Takip Noktası")
        plt.plot(state.x, state.y, "or", markersize=8, label="Otonom Araç")
        
        # Ön aks yönünü okla göster
        plt.arrow(state.x, state.y, math.cos(state.yaw)*5, math.sin(state.yaw)*5, color='r', width=0.2)
        
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"LQR Controller | Anlık Hata (e): {abs(error):.3f}m")
        plt.pause(0.001)

    print("Parkur başarıyla tamamlandı!")
    plt.show()

if __name__ == '__main__':
    main()
