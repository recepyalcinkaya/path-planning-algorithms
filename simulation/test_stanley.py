import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.stanley_controller import State, StanleyController

def main():
    print("Stanley Controller Yörünge İzleme Simülasyonu Başlatıldı!!")

    # Referans Yol (Hafif kıvrımlı bir parkur)
    ax = [0.0, 50.0, 100.0, 150.0]
    ay = [0.0, 10.0, 5.0, 20.0]
    
    # Yolu pürüzsüzleştirmek için basit bir interpolasyon
    cx = np.linspace(0, 150, 300)
    cy = np.interp(cx, ax, ay)
    cyaw = np.arctan2(np.gradient(cy), np.gradient(cx))

    target_speed = 20.0 / 3.6 # [m/s] 20 km/h

    # Araç Durumu: Yolun 5 metre uzağından ve yanlış açıyla başlasın
    state = State(x=0.0, y=5.0, yaw=math.radians(-20), v=0.0)
    controller = StanleyController()

    x_history, y_history = [state.x], [state.y]
    target_id = 0

    while target_id < len(cx) - 1:
        # Hız ve Direksiyon Komutları
        accel = controller.pid_control(target_speed, state.v)
        delta, target_id = controller.stanley_control(state, cx, cy, cyaw, target_id)

        # Aracı güncelle
        state.update(accel, delta, controller.dt)

        x_history.append(state.x)
        y_history.append(state.y)

        # --- Animasyon ---
        plt.cla()
        plt.plot(cx, cy, "--k", label="Referans Yol")
        plt.plot(x_history, y_history, "-b", label="Araç İzleği")
        plt.plot(cx[target_id], cy[target_id], "xg", label="Takip Noktası")
        plt.plot(state.x, state.y, "or", label="Otonom Araç")
        
        # Ön aks yönünü küçük bir çizgiyle göster
        plt.arrow(state.x, state.y, math.cos(state.yaw)*3, math.sin(state.yaw)*3, color='r')
        
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"Stanley Control | Hata (e): {abs(state.y - cy[target_id]):.2f}m")
        plt.pause(0.01)

    plt.show()

if __name__ == '__main__':
    main()
