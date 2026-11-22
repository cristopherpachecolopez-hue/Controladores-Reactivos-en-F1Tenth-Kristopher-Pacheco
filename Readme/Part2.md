# Controlador Reactivo Follow the Gap (Parte 2) - Circuito Zandvoort

**Autor:** Kristopher Andrés Pacheco López  
**Curso:** Vehículos Autónomos (II PAO 2026)  
**Entorno:** F1Tenth Gym ROS (ROS 2 Humble)  
**Pista asignada:** Zandvoort (Grupo 4)

---

## 1. Descripción del proyecto

Implementación de un controlador reactivo **Follow the Gap (FTG)** con extensor de disparidades en Python sobre **ROS 2 Humble**, adaptado para superar las dos pruebas de la Segunda Parte del proyecto en el circuito de Zandvoort:

1. **Prueba 1 (Obstáculos estáticos):** Navegar el mapa modificado con **5 obstáculos estáticos** distribuidos a lo largo de la pista y completar **10 vueltas consecutivas sin colisión**.
2. **Prueba 2 (Obstáculo dinámico y rebase):** Incorporar un **segundo vehículo 5 metros adelante** del robot principal utilizando el mismo controlador base, lograr **rebasarlo** durante la carrera y completar **10 vueltas consecutivas sin colisión**.

Adicionalmente, el nodo integra contador automático de vueltas, cronómetro en tiempo real, cálculo de distancia recorrida por vuelta y visualización en **RViz** del HUD 3D y de la trayectoria reiniciada en cada vuelta.

---

## 2. Enfoque técnico

El controlador (`ftg_node.py`) funciona de forma parametrizada para operar tanto en el vehículo principal (`ego_racecar`) como en el vehículo oponente (`opp_racecar`), aplicando las siguientes etapas en cada lectura del LiDAR:

* **Recorte de campo de visión y extensor de disparidades:** Se filtra el barrido frontal del LiDAR a un campo de visión de $\pm 74^\circ$ (lo suficientemente amplio para ver las salidas de curvas sin mirar hacia atrás y evitar giros en U). Tras aplicar un filtro de media móvil de ventana 3, cada salto de profundidad mayor a `0.24 m` proyecta un margen lateral de `0.21 m` sobre la lectura más cercana para rodear esquinas internas, bloques estáticos y el chasis del vehículo rival.
* **Burbuja de seguridad compacta y selección de carril:** Cuando existe un obstáculo a menos de `1.9 m`, se aplica una burbuja de seguridad de radio `0.26 m` alrededor del punto más cercano. Sobre los espacios libres restantes se selecciona el hueco con mejor relación entre profundidad y ancho ($\text{profundidad} \cdot \sqrt{\text{ancho}}$) para identificar el carril de rebase más viable.
* **Dirección PD suavizada y guarda lateral anti-corte:** El ángulo de dirección se calcula con un controlador PD adaptativo y un limitador de cambio máximo por ciclo (`±0.095 rad`) para evitar volantazos bruscos. Si los rayos laterales ($32^\circ$ a $72^\circ$) detectan al vehículo emparejado a menos de `0.48 m`, se restringe el giro hacia ese costado hasta completar el adelantamiento.
* **Perfil de velocidad con modo rebase en recta:** El vehículo utiliza un corredor longitudinal frontal (`±0.26 m`) para frenar antes de entrar en horquillas o ante obstáculos cercanos. Cuando detecta al rival bloqueando el frente en una recta con un carril lateral despejado (`> 5.5 m`), activa el modo de rebase reduciendo la penalización de velocidad por giro para mantener impulso y adelantarlo limpiamente.
* **Telemetría y trayectoria por vuelta en RViz:** Mediante `/ego_racecar/odom`, el nodo detecta cruces por la línea de meta con histéresis espacial (`2.0 m`) y tiempo mínimo antirrebote (`6.5 s`), integra la distancia euclidiana recorrida por vuelta y publica la trayectoria en `/ftg_trajectory_marker` y `/ftg_path`. Al iniciar cada nueva vuelta, el trazo se reinicia y cambia de color sincronizado con el texto del HUD (`/ftg_hud_marker`).

---

## 3. Estructura del proyecto

```text
ftg_zandvoort/
├── ftg_zandvoort/
│   ├── __init__.py
│   └── ftg_node.py           # Controlador FTG multi-agente, telemetría y marcadores RViz
├── maps/
│   ├── Zandvoort_map.png     # Mapa original de Zandvoort (Prueba 2)
│   ├── Zandvoort_map.yaml    # Metadatos del mapa original (0.05109 m/px)
│   ├── Zandvoort_obs.png     # Mapa modificado con 5 obstáculos estáticos (Prueba 1)
│   └── Zandvoort_obs.yaml    # Metadatos del mapa con obstáculos
├── resource/
│   └── ftg_zandvoort
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

### Bloques principales en `ftg_node.py`
* **Percepción y control (`apply_disparity_extender`, `apply_safety_bubble`, `find_max_gap`, `find_best_point`, `scan_callback`):** Procesan `/scan` (o `/opp_scan`) y publican los comandos Ackermann en `/drive` (o `/opp_drive`).
* **Parámetros ROS 2 (`is_opponent`, `v_max`, `v_min`, `center_bias`, `lane_bias`):** Permiten reutilizar el mismo nodo para ambos vehículos desde consola ajustando velocidad y comportamiento de carril sin duplicar scripts.
* **Telemetría y visualización (`odom_callback`, `reset_lap_trajectory`, `append_trajectory_point`, `publish_hud_marker`, `publish_target_marker`, `print_dashboard`):** Gestionan el conteo de vueltas, cronómetro, distancia por vuelta y marcadores en RViz.

---

## 4. Mapa con obstáculos estáticos y configuración de 2 agentes

* **5 obstáculos estáticos (`Zandvoort_obs.png`):** Se generó una copia del mapa oficial de Zandvoort (`resolución: 0.05109 m/px`, `origen: [-19.7047, -57.9025, 0.0]`) colocando 5 bloques sólidos negros (`#000000`) de `10x10` píxeles (`~0.51 m x 0.51 m`) distribuidos en distintos tramos de la pista con margen transitable a un costado.
* **Posición del segundo vehículo (5 m adelante):** Con el vehículo principal en `(sx: 0.0, sy: 0.0, stheta: 1.22 rad)`, la posición del oponente 5 metros adelante sobre la recta principal resulta en:
  * `sx1 = 5 * cos(1.22) = 1.72`
  * `sy1 = 5 * sin(1.22) = 4.70`
  * `stheta1 = 1.22`

---

## 5. Instrucciones de instalación y ejecución

### Paso 1: Copiar los mapas y compilar el paquete
```bash
cp maps/Zandvoort_* ~/ros2_ws/src/f1tenth_gym_ros/maps/
cd ~/ros2_ws
colcon build --packages-select f1tenth_gym_ros ftg_zandvoort --symlink-install
source install/setup.bash
```

### Paso 2: Ejecutar Prueba 1 (5 obstáculos estáticos — 10 vueltas)
Editar `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml`:
```yaml
map_path: '/home/kristopher/ros2_ws/src/f1tenth_gym_ros/maps/Zandvoort_obs'
map_img_ext: '.png'
num_agent: 1
sx: 0.0
sy: 0.0
stheta: 1.22
```

* **Terminal 1 (Simulador y RViz):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ```
* **Terminal 2 (Controlador principal):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run ftg_zandvoort ftg_node
  ```

### Paso 3: Ejecutar Prueba 2 (2 robots a 5 m y rebase — 10 vueltas)
Editar `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml`:
```yaml
map_path: '/home/kristopher/ros2_ws/src/f1tenth_gym_ros/maps/Zandvoort_map'
map_img_ext: '.png'
num_agent: 2
sx: 0.0
sy: 0.0
stheta: 1.22
sx1: 1.72
sy1: 4.70
stheta1: 1.22
```

* **Terminal 1 (Simulador y RViz con 2 agentes):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ```
* **Terminal 2 (Robot oponente — obstáculo dinámico):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run ftg_zandvoort ftg_node --ros-args -r __node:=ftg_opp -p is_opponent:=true -p v_max:=5.4 -p v_min:=2.0 -p center_bias:=0.70 -p lane_bias:=0.08
  ```
* **Terminal 3 (Vehículo principal — rebase y telemetría):**
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 run ftg_zandvoort ftg_node --ros-args -p is_opponent:=false -p v_max:=8.2 -p v_min:=2.1 -p center_bias:=0.26
  ```

### Visualización en RViz
En el panel **Displays** agregar mediante **Add -> By topic**:
* `/ftg_hud_marker` (`Marker`): Contador de vueltas, cronómetro, distancia y velocidad sobre el vehículo.
* `/ftg_target_marker` (`Marker`): Esfera del punto objetivo seleccionado.
* `/ftg_trajectory_marker` (`Marker`): Línea de trayectoria reiniciada con cambio de color en cada vuelta (o activar `Path` en `/ftg_path`).

---

## 6. Resultados y videos de evidencia

| Prueba | Condición | Vueltas sin colisión | Mejor tiempo / Distancia por vuelta |
| :--- | :--- | :---: | :---: |
| **Prueba 1** | Zandvoort con 5 obstáculos estáticos | `10 / 10` | `[Completar con tu mejor tiempo y distancia]` |
| **Prueba 2** | Zandvoort con 2 robots (5 m + rebase) | `10 / 10` | `[Completar con tu mejor tiempo y distancia]` |

* **Video Prueba 1 (5 obstáculos estáticos):** `[Insertar enlace]`
* **Video Prueba 2 (Obstáculo dinámico y rebase):** `[Insertar enlace]`
