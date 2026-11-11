
# Controlador Reactivo Follow the Gap (FTG) - Pista Zandvoort
**Autor:** Kristopher Pacheco  
**Materia:** Vehículos Autónomos - Proyecto Primer Parcial (1ra Parte)  
**Pista asignada:** Zandvoort (Grupo 4)

## 1. Descripción del Enfoque Utilizado
Este proyecto implementa un algoritmo de navegación reactiva **Follow the Gap (FTG)** mejorado con un **Extensor de Disparidades (Disparity Extender)** y un **perfil de velocidad adaptativo** para recorrer de forma autónoma el circuito de Zandvoort sin colisiones:
1. **Recorte y Suavizado del LiDAR (`/scan`):** Se filtra el campo de visión frontal a $\pm 95^\circ$, se aplica un filtro de media móvil de 5 muestras para atenuar ruido y se limita el alcance máximo a $8.5\text{ m}$.
2. **Extensor de Disparidades y Burbuja de Seguridad:** Ante saltos bruscos de profundidad entre rayos consecutivos ($>0.35\text{ m}$), se extiende la distancia del obstáculo más cercano según el ancho físico del vehículo. Adicionalmente, se anulan a $0.0\text{ m}$ los rayos dentro de un radio de seguridad ($0.42\text{ m}$) alrededor del punto más próximo.
3. **Selección del Gap y Punto Objetivo:** Se identifica la secuencia continua más amplia de rayos libres que superen un umbral dinámico y se calcula el ángulo objetivo combinando un $65\%$ del punto más profundo del gap con un $35\%$ de su centro geométrico para evitar cierres prematuros en las horquillas de Zandvoort.
4. **Control Dinámico de Velocidad y Dirección (`/drive`):** La dirección emplea una ley proporcional-derivativa suave y la velocidad longitudinal se ajusta dinámicamente entre $1.9\text{ m/s}$ en curvas cerradas y $6.2\text{ m/s}$ en la recta principal en función del ángulo de giro y la distancia libre frontal.

## 2. Estructura del Código
El nodo `ftg_zandvoort.py` está organizado de forma modular dentro de la clase `FollowTheGapZandvoort`:
* `__init__()`: Configuración de hiperparámetros de navegación, variables de estado del cronómetro/contador de vueltas y creación de suscriptores (`/scan`, `/ego_racecar/odom`) y publicador (`/drive`).
* `odom_callback()` y `print_dashboard()`: Capturan la posición inicial del vehículo como línea de meta, contabilizan automáticamente cada vuelta completada con un filtro antirrebote temporal ($10\text{ s}$) y actualizan en pantalla el cronómetro en vivo, la última vuelta y el récord de vuelta rápida.
* `preprocess_lidar()` y `apply_safety_bubble()`: Ejecutan el filtrado espacial de los rangos del LiDAR, el extensor de disparidades en bordes de muros y la generación de la burbuja de exclusión.
* `find_max_gap()` y `find_best_point()`: Extraen los índices de inicio y fin del mayor espacio libre y determinan el índice óptimo de seguimiento.
* `scan_callback()`: Ciclo principal de control reactivo que integra percepción y cálculo de comandos `AckermannDriveStamped`.

## 3. Instrucciones de Ejecución
1. Copiar los archivos `Zandvoort_map.png` y `Zandvoort_map.yaml` dentro de `f1tenth_gym_ros/maps/` y configurar `map_path` en `f1tenth_gym_ros/config/sim.yaml`.
2. En una primera terminal, compilar e iniciar el simulador oficial F1Tenth:
   ```bash
   cd ~/f1tenth_ws
   colcon build
   source install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
