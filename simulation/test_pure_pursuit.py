import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.pure_pursuit import State, PurePursuitController

def main():
    print(__file__ + " Pure Pursuit Yörünge Takip Simülasyonu Başlatıldı!!")

    # Referans Yörünge Oluştur (S şekilli keskin virajlı bir yol)
    cx = np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s] Hedef Hız

    # Sistemin Başlatılması
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)
    controller = PurePursuitController()

    lastIndex = len(cx) - 1
    time = 0.0
    x_history, y_history = [state.x], [state.y]
    
    target_ind, _ = controller.calc_target_index(state, cx, cy)

    # Simülasyon Döngüsü
    while lastIndex > target_ind:
        # Hız Kontrolü (Gaz/Fren)
        a = controller.proportional_control(target_speed, state.v)
        
        # Yön Kontrolü (Direksiyon)
        delta, target_ind = controller.pure_pursuit_steer_control(state, cx, cy, target_ind)

        # Aracın Fiziğini Güncelle
        state.update(a, delta, controller.dt)
        time += controller.dt

        x_history.append(state.x)
        y_history.append(state.y)

        # Animasyon Çizimi
        plt.cla()
        
        # Referans yol (Hedeflenen)
        plt.plot(cx, cy, ".gray", label="Planlanan Referans Yol")
        
        # Aracın izlediği gerçek yol
        plt.plot(x_history, y_history, "-b", label="Aracın Gerçek İzi")
        
        # Arabayı göster (Kırmızı nokta)
        plt.plot(state.x, state.y, "or", markersize=8, label="Otonom Araç")
        
        # Lookahead hedefini göster (Yeşil yıldız)
        plt.plot(cx[target_ind], cy[target_ind], "*g", markersize=12, label="Lookahead Hedefi")
        
        # Arabadan hedefe uzanan hayali bakış açısı çizgisi
        plt.plot([state.x, cx[target_ind]], [state.y, cy[target_ind]], "--g", alpha=0.5)

        plt.title(f"Pure Pursuit İzleme | Hız: {state.v * 3.6:.1f} km/h")
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.pause(0.001)

    print("Yörünge başarıyla tamamlandı!")
    plt.show()

if __name__ == '__main__':
    main()
