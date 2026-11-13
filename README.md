
# 🏎️ Controlador Reactivo Follow the Gap (FTG) con Extensor de Disparidades — Circuito Zandvoort (F1Tenth)

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)
![Simulator](https://img.shields.io/badge/F1Tenth-Gym_ROS-E10600?style=for-the-badge)
![Status](https://img.shields.io/badge/Pruebas-10_Vueltas_Completadas-2EA44F?style=for-the-badge)

* **Autor:** Kristopher Andrés Pacheco López
* **Asignatura:** Vehículos Autónomos — II PAO 2026
* **Evaluación:** Proyecto Primer Parcial (1ra Parte) — Competencia de Controladores Reactivos
* **Pista Asignada:** Zandvoort (Grupo 4)

---

## 📑 Tabla de Contenidos
1. [Descripción General y Objetivos del Proyecto](#-1-descripción-general-y-objetivos-del-proyecto)
2. [Análisis Geométrico del Circuito Zandvoort](#-2-análisis-geométrico-del-circuito-zandvoort)
3. [Arquitectura del Sistema en ROS 2](#-3-arquitectura-del-sistema-en-ros-2)
4. [Fundamento Matemático y Enfoque Algorítmico](#-4-fundamento-matemático-y-enfoque-algorítmico)
5. [Sistema de Telemetría: Contador de Vueltas, Cronómetro y HUD 3D](#-5-sistema-de-telemetría-contador-de-vueltas-cronómetro-y-hud-3d)
6. [Estructura Detallada del Código](#-6-estructura-detallada-del-código)
7. [Tabla de Hiperparámetros y Estrategia de Ajuste (Tuning)](#-7-tabla-de-hiperparámetros-y-estrategia-de-ajuste-tuning)
8. [Instrucciones de Instalación, Configuración y Ejecución](#-8-instrucciones-de-instalación-configuración-y-ejecución)
9. [Resultados Experimentales y Evidencia en Video](#-9-resultados-experimentales-y-evidencia-en-video)

---

## 🎯 1. Descripción General y Objetivos del Proyecto

Este repositorio contiene el diseño, implementación y validación de un **controlador reactivo autónomo** basado en el algoritmo **Follow the Gap (FTG)** mejorado con un **Extensor de Disparidades (*Disparity Extender*)**, control lateral **Proporcional-Derivativo (PD) adaptativo** y un **perfil longitudinal exponencial de velocidad**, desarrollado en Python sobre **ROS 2** para el simulador oficial **F1Tenth Gym ROS**.

El controlador fue diseñado para superar las dos pruebas de desempeño en pista y cumplir con los requerimientos de telemetría en tiempo real:
1. **Prueba de Resistencia y Confiabilidad:** Completar **10 vueltas consecutivas sin colisionar** con los muros del circuito.
2. **Prueba de Velocidad Pura (*Time Attack*):** Registrar el **menor tiempo de vuelta** válido dentro de la serie sin colisiones.
3. **Telemetría Integrada:** Conteo automático de vueltas completadas, cronómetro de precisión por vuelta en terminal y visualización de datos en tiempo real mediante marcadores 3D (`visualization_msgs/Marker`) directamente sobre el vehículo en **RViz**.

---

## 🗺️ 2. Análisis Geométrico del Circuito Zandvoort

El trazado asignado corresponde al circuito **Zandvoort** (`Zandvoort_map.png` / `Zandvoort_map.yaml`), caracterizado por combinar secciones de altísima velocidad con horquillas muy cerradas que exigen una gestión dinámica entre frenado anticipado y trazado cercano al vértice (*apex*):

| Parámetro del Mapa | Valor / Especificación | Impacto en el Controlador |
| :--- | :--- | :--- |
| **Resolución espacial** | `0.05109 m/píxel` | Permite detectar variaciones de borde de muro con precisión sub-decimétrica. |
| **Origen de coordenadas** | `[-19.7047, -57.9025, 0.0]` | Sitúa la pose inicial `(0.0, 0.0)` directamente sobre la recta principal izquierda. |
| **Orientación inicial (`stheta`)** | `1.22 rad` ($\approx 69.9^\circ$) | Alinea el eje longitudinal del vehículo con la recta principal en sentido horario. |
| **Sector 1 (Recta Principal)** | Recta larga de alta velocidad | Permite alcanzar velocidad punta (`10.2 m/s`) con atenuación de ganancia direccional. |
| **Sector 2 (Horquillas de $180^\circ$)** | Curvas cerradas de radio reducido | Requiere un campo de visión amplio ($\pm 88^\circ$) y frenado preventivo a `3.6 m` del muro frontal. |
| **Sector 3 (Chicanes rápidas)** | Enlaces derecha-izquierda fluidos | El extensor de disparidades evita que el chasis roce los vértices internos. |

---

## 🏗️ 3. Arquitectura del Sistema en ROS 2

El sistema opera bajo una arquitectura desacoplada de publicación/suscripción en **ROS 2**, donde el nodo `ftg_zandvoort_kristopher` procesa cada barrido del sensor LiDAR a alta frecuencia ($\approx 40\text{ Hz}$) y actualiza la odometría y visualización en tiempo real:

```text
+-----------------------------------+
|      F1Tenth Gym ROS Bridge       |
|  (Simulador Físico + Mapa 2D)     |
+--------+-----------------+--------+
         |                 |
         | /scan           | /ego_racecar/odom
         | (LaserScan)     | (Odometry)
         v                 v
+-----------------------------------+
|  Nodo: ftg_zandvoort_kristopher   |
|  (ftg_zandvoort/ftg_node.py)      |
|                                   |
|  1. Filtro LiDAR + Disparity Ext. |
|  2. Burbuja de Seguridad          |
|  3. Detección Max Gap + Target    |
|  4. Control PD + Perfil Velocidad |
|  5. Máquina de Estados Vueltas    |
+---+-------------+-------------+---+
    |             |             |
    | /drive      | /ftg_hud    | /ftg_target
    | (Ackermann) | (Marker 3D) | (Marker 3D)
    v             v             v
+--------+  +-----------------------+
| Vehíc. |  |         RViz 2        |
| Ego    |  | (Visualización y HUD) |
+--------+  +-----------------------+
```

### Interfaces de Comunicación (Tópicos ROS 2)

| Tópico | Tipo de Mensaje | Dirección | Función en el Sistema |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Suscripción | Recibe el arreglo de distancias del LiDAR 2D ($1080$ haces). |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Suscripción | Obtiene la posición $(x, y)$ del vehículo para detectar cruces por línea de meta. |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Publicación | Envía el ángulo de dirección (`steering_angle`) y velocidad (`speed`) al chasis. |
| `/ftg_hud_marker` | `visualization_msgs/msg/Marker` | Publicación | Renderiza el panel flotante 3D sobre el auto con vueltas, cronómetro y velocidad. |
| `/ftg_target_marker` | `visualization_msgs/msg/Marker` | Publicación | Dibuja una esfera verde en el punto objetivo calculado dentro del *gap* libre. |

---

## 🧮 4. Fundamento Matemático y Enfoque Algorítmico

A diferencia de un controlador *Follow the Gap* básico —que tiende a recortar demasiado las curvas internas o a oscilar en rectas largas—, esta implementación ejecuta un **pipeline de 7 etapas** por cada ciclo de LiDAR:

### Etapa 1: Recorte de Campo de Visión (FOV) y Suavizado Espacial
Se filtran los rayos del LiDAR para conservar únicamente el sector frontal comprendido entre $[-\theta_{\text{fov}}, +\theta_{\text{fov}}]$ con $\theta_{\text{fov}} = 88^\circ$, descartando lecturas posteriores irrelevantes para la navegación hacia adelante. Posteriormente, se reemplazan valores `NaN` o infinitos por el horizonte máximo $R_{\max} = 13.0\text{ m}$ y se aplica un filtro de convolución de media móvil de ventana $k = 3$:

$$r_{\text{smooth}}[i] = \frac{1}{3} \sum_{m=-1}^{1} r[i + m]$$

### Etapa 2: Extensor de Disparidades (*Disparity Extender*)
Cuando el vehículo se aproxima a una esquina o horquilla de Zandvoort, existe un salto brusco de profundidad entre un rayo que impacta el borde cercano del muro ($d_{\text{cerca}}$) y el rayo adyacente que apunta hacia el fondo de la curva. Si $\vert{}r[i+1] - r[i]\vert{} > \delta_{\text{disp}}$ (con $\delta_{\text{disp}} = 0.30\text{ m}$), se proyecta el semiancho físico del vehículo ($w_{\text{half}} = 0.24\text{ m}$) sobre la distancia mínima para extender virtualmente el obstáculo:

$$\Delta\theta = \arctan\left(\frac{w_{\text{half}}}{d_{\text{cerca}}}\right), \qquad N_{\text{muestras}} = \left\lceil \frac{\Delta\theta}{\Delta\alpha} \right\rceil$$

donde $\Delta\alpha$ es el incremento angular del LiDAR (`angle_increment`). Esto garantiza que el centro geométrico del vehículo nunca pase más cerca del muro de lo que permite su propio ancho.

### Etapa 3: Burbuja de Seguridad Dinámica (*Safety Bubble*)
Si el obstáculo más cercano se encuentra dentro de un umbral de proximidad crítica ($d_{\min} \le 2.8\text{ m}$), se localiza su índice $i_{\min}$ y se anulan a $0.0\text{ m}$ todos los rayos contenidos dentro de un radio de exclusión $R_{\text{bubble}} = 0.34\text{ m}$:

$$N_{\text{bubble}} = \left\lceil \frac{\arctan(R_{\text{bubble}} / \max(d_{\min}, 0.1))}{\Delta\alpha} \right\rceil$$

### Etapa 4: Detección del Mayor Espacio Libre (*Max Gap*) con Umbral Adaptativo
Para evitar quedar atrapado en grietas estrechas o lecturas ruidosas, se define un umbral dinámico de espacio navegable:

$$T_{\text{gap}} = \max\left(1.6,\; 0.38 \cdot \max_i(r_{\text{proc}}[i])\right)$$

Se construye una máscara booleana $M[i] = (r_{\text{proc}}[i] > T_{\text{gap}})$ y se extrae el intervalo contiguo $[i_{\text{start}}, i_{\text{end}}]$ de mayor longitud $(i_{\text{end}} - i_{\text{start}})$.

### Etapa 5: Selección Ponderada del Punto Objetivo (*Best Point*)
Apuntar exclusivamente al rayo más profundo del *gap* provoca que el auto roce el bordillo interior en curvas de $180^\circ$, mientras que apuntar solo al centro geométrico reduce excesivamente la velocidad de paso. Por ello, se calcula un índice híbrido ponderado:

$$i_{\text{best}} = \left\lfloor 0.68 \cdot i_{\text{deep}} + 0.32 \cdot i_{\text{center}} \right\rfloor$$

donde $i_{\text{deep}}$ es el promedio de los índices que alcanzan al menos el $94\%$ de la profundidad máxima dentro del *gap* y $i_{\text{center}} = \lfloor (i_{\text{start}} + i_{\text{end}})/2 \rfloor$. El ángulo objetivo final es $\theta_{\text{target}} = \text{angles}[i_{\text{best}}]$.

### Etapa 6: Control Lateral PD con Atenuación en Recta
El ángulo de dirección Ackermann $\delta$ se calcula mediante un controlador Proporcional-Derivativo que ajusta dinámicamente su ganancia proporcional en función de la distancia libre frontal ($d_{\text{front}}$) para evitar oscilaciones a más de $10\text{ m/s}$:

$$K_{p,\text{dyn}} = \begin{cases} 0.72 \cdot K_{p,\text{base}} & \text{si } d_{\text{front}} > 6.5\text{ m} \\ K_{p,\text{base}} & \text{en otro caso} \end{cases}$$

$$\delta = \text{clip}\Big(K_{p,\text{dyn}} \cdot \theta_{\text{target}} + K_d \cdot (\theta_{\text{target}} - \theta_{\text{prev}}),\; -0.4189,\; +0.4189\Big)$$

### Etapa 7: Perfil Longitudinal Exponencial y Frenado Preventivo
La velocidad comandada $v$ combina velocidad máxima en rectas despejadas, decaimiento exponencial según el ángulo de giro y frenado anticipado antes de las horquillas:

$$v = v_{\min} + (v_{\max} - v_{\min}) \cdot e^{-3.2 \vert{}\delta\vert{}} \cdot \min\left(1.0,\; \frac{\max(d_{\text{front}}, 0.85\,d_{\text{target}})}{7.5}\right)$$

Adicionalmente, si el vehículo detecta un muro frontal a menos de $3.6\text{ m}$ al final de una recta ($d_{\text{front}} < 3.6\text{ m}$), se aplica una limitación lineal de entrada a curva:

$$v \leftarrow \min\left(v,\; v_{\min} + 1.1 \cdot \frac{d_{\text{front}}}{3.6}\right)$$

---

## ⏱️ 5. Sistema de Telemetría: Contador de Vueltas, Cronómetro y HUD 3D

Para garantizar mediciones exactas durante la competencia y el video de evidencia, el nodo incorpora un módulo de telemetría sincronizado con `/ego_racecar/odom`:

* **Línea de Meta Automática:** Al recibir el primer mensaje de odometría, el nodo almacena las coordenadas iniciales $(x_0, y_0)$ como referencia de línea de meta.
* **Histéresis Espacial y Antirrebote Temporal:** Para evitar falsos conteos mientras el auto cruza el área de meta, se exige que el vehículo primero abandone el perímetro de salida ($d > 2.2 \cdot R_{\text{zona}}$, con $R_{\text{zona}} = 2.0\text{ m}$) y que haya transcurrido un tiempo mínimo de vuelta $t_{\min} = 5.5\text{ s}$ antes de validar el siguiente paso por meta ($d \le 2.0\text{ m}$).
* **Dashboard en Terminal y HUD 3D en RViz:** Cada $0.15\text{ s}$ se actualiza en consola y en el marcador 3D flotante (`/ftg_hud_marker`, tipo `TEXT_VIEW_FACING`) el número de vueltas completadas (`X/10`), el cronómetro en vivo, el tiempo de la última vuelta, el récord de vuelta rápida y la velocidad instantánea en $\text{m/s}$. Al completar las 10 vueltas, el texto del HUD cambia automáticamente de color cian a verde.

---

## 📂 6. Estructura Detallada del Código

El paquete ROS 2 `ftg_zandvoort` está estructurado modularmente dentro de la clase `FollowTheGapZandvoort(Node)` en `ftg_zandvoort/ftg_node.py`:

| Método / Componente | Responsabilidad Técnica | Entradas / Salidas |
| :--- | :--- | :--- |
| `__init__(self)` | Inicializa hiperparámetros de navegación, variables de estado del cronómetro, suscriptores (`/scan`, `/ego_racecar/odom`) y publicadores (`/drive`, `/ftg_hud_marker`, `/ftg_target_marker`). | Configuración interna del nodo ROS 2. |
| `odom_callback(self, msg)` | Ejecuta la máquina de estados de conteo de vueltas, calcula tiempos parciales/totales y dispara la actualización del HUD 3D. | Entrada: `Odometry` $\rightarrow$ Actualiza estado de carrera. |
| `publish_hud_marker(self, ...)` | Construye y publica el mensaje `Marker.TEXT_VIEW_FACING` anclado a `ego_racecar/base_link` a $0.9\text{ m}$ de altura. | Entrada: `current_lap_time` $\rightarrow$ Publica en `/ftg_hud_marker`. |
| `publish_target_marker(self, ...)` | Proyecta las coordenadas polares $(d_{\text{target}}, \theta_{\text{target}})$ a cartesianas $(x, y)$ para visualizar el objetivo en RViz. | Entrada: `target_dist, target_angle` $\rightarrow$ Publica en `/ftg_target_marker`. |
| `print_dashboard(self, ...)` | Formatea la salida en tiempo real en la terminal y muestra un banner resumen cada vez que se completa una vuelta. | Salida estándar (`stdout`) en consola. |
| `preprocess_lidar(self, ...)` | Limpia valores no numéricos, suaviza el barrido LiDAR y ejecuta el algoritmo de extensión de disparidades. | Entrada: `ranges, angle_inc` $\rightarrow$ Salida: `np.ndarray` filtrado. |
| `apply_safety_bubble(self, ...)` | Detecta el punto más cercano y anula los rayos dentro del radio de seguridad del vehículo. | Entrada: `proc_ranges, angle_inc` $\rightarrow$ Salida: `np.ndarray` con burbuja. |
| `find_max_gap(self, ...)` | Aplica umbralización adaptativa y encuentra los índices `[start_i, end_i]` del mayor segmento libre. | Entrada: `free_space_ranges` $\rightarrow$ Salida: tupla `(start_i, end_i)`. |
| `find_best_point(self, ...)` | Calcula el índice objetivo ponderando profundidad máxima ($68\%$) y centro del *gap* ($32\%$). | Entrada: `start_i, end_i, ranges` $\rightarrow$ Salida: entero `best_i`. |
| `scan_callback(self, data)` | Bucle principal de control reactivo: orquesta percepción, control PD de dirección, perfil de velocidad y publicación Ackermann. | Entrada: `LaserScan` $\rightarrow$ Publica en `/drive`. |
| `main(args=None)` | Inicializa `rclpy`, ejecuta `rclpy.spin(node)` y asegura que al detener el nodo (`Ctrl+C`) se envíe un comando de velocidad `0.0 m/s`. | Punto de entrada `console_scripts`. |

---

## ⚙️ 7. Tabla de Hiperparámetros y Estrategia de Ajuste (Tuning)

| Parámetro | Valor Actual | Unidad | Descripción y Efecto en Zandvoort |
| :--- | :---: | :---: | :--- |
| `fov_deg` | `88.0` | grados ($\pm^\circ$) | Apertura frontal; permite ver la salida de horquillas de $180^\circ$ sin perder enfoque en recta. |
| `max_lidar_range` | `13.0` | $\text{m}$ | Alcance máximo considerado para anticipar rectas largas. |
| `bubble_radius` | `0.34` | $\text{m}$ | Radio de exclusión alrededor del muro más cercano. |
| `disparity_threshold` | `0.30` | $\text{m}$ | Salto mínimo entre rayos contiguos para activar el extensor de disparidades. |
| `car_half_width` | `0.24` | $\text{m}$ | Semiancho virtual proyectado en esquinas internas. |
| `v_max` | `10.2` | $\text{m/s}$ | Velocidad máxima en recta principal (`10.2 m/s` equilibrado; `11.2 m/s` modo agresivo). |
| `v_min` | `2.65` | $\text{m/s}$ | Velocidad mínima en curvas cerradas para evitar subviraje. |
| `kp_base` | `0.72` | — | Ganancia proporcional base del lazo de dirección. |
| `kd_steer` | `0.16` | — | Ganancia derivativa para amortiguar giros bruscos. |

---

## 🚀 8. Instrucciones de Instalación, Configuración y Ejecución

### Requisitos Previos
* Sistema Operativo: **Ubuntu 22.04 LTS** con **ROS 2 Humble** instalado.
* Simulador oficial **F1Tenth Gym ROS** compilado en el workspace (`~/ros2_ws`).

### Paso 1: Copiar el Mapa de Zandvoort y Configurar el Simulador
1. Copie los archivos `maps/Zandvoort_map.png` y `maps/Zandvoort_map.yaml` dentro de la carpeta de mapas del simulador:
   ```bash
   cp maps/Zandvoort_map.* ~/ros2_ws/src/f1tenth_gym_ros/maps/
   ```
2. Edite el archivo `~/ros2_ws/src/f1tenth_gym_ros/config/sim.yaml` y configure la ruta del mapa y la pose inicial:
   ```yaml
   map_path: '/home/kristopher/ros2_ws/src/f1tenth_gym_ros/maps/Zandvoort_map'
   map_img_ext: '.png'
   num_agent: 1
   sx: 0.0
   sy: 0.0
   stheta: 1.22
   ```

### Paso 2: Clonar y Compilar este Paquete en el Workspace
```bash
cd ~/ros2_ws/src
git clone [https://github.com/](https://github.com/)<TU_USUARIO>/ftg_zandvoort.git
cd ~/ros2_ws
colcon build --packages-select f1tenth_gym_ros ftg_zandvoort --symlink-install
source install/setup.bash
```

### Paso 3: Ejecutar la Simulación y el Controlador Reactivo
1. **Terminal 1 — Iniciar el entorno F1Tenth y RViz:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch f1tenth_gym_ros gym_bridge_launch.py
   ```
2. **Configuración visual en RViz (opcional):**
   * En el panel **Views**, seleccione `Target Frame: ego_racecar/base_link` para que la cámara siga automáticamente al vehículo.
   * Haga clic en **Add -> By topic** y agregue `/ftg_hud_marker` (`Marker`) y `/ftg_target_marker` (`Marker`) para visualizar el cronómetro 3D y el punto objetivo en pista.
3. **Terminal 2 — Ejecutar el controlador Follow the Gap:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run ftg_zandvoort ftg_node
   ```

---

## 📊 9. Resultados Experimentales y Evidencia en Video

### Resumen de Desempeño en el Circuito Zandvoort (Serie de 10 Vueltas)

| Métrica Evaluada | Resultado Obtenido | Estado |
| :--- | :---: | :---: |
| **10 vueltas consecutivas sin colisión** | `10 / 10 vueltas` | ✅ Completado |
| **Mejor tiempo de vuelta registrado** | `43.471 s` | ✅ Completado |
| **Velocidad máxima alcanzada en recta** | `10.2 m/s` | ✅ Verificado |
| **Distancia recorrida por vuelta (línea de carrera)** | `~376.81 m / vuelta`| ✅ Verificado |
| **Velocidad punta alcanzada en recta principal** | `10.2 m/s` | ✅ Verificado |

### 🎥 Video de Evidencia
* **Enlace al video demostrativo (RViz + Contador de Vueltas + Cronómetro en vivo):** `[https://youtu.be/idIuEAnc6hQ]`
