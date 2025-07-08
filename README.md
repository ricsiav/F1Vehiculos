# 🏎️ F1TENTH - Follow the Gap, Lap Counter y Lap Timer

Este repositorio contiene tres nodos ROS2 implementados en Python para el simulador F1TENTH Gym. Los nodos permiten al vehículo autónomo detectar caminos libres (gaps), contar vueltas y medir el tiempo de cada vuelta.

---

## 📌 Descripción del enfoque

### 🔍 Follow the Gap

El algoritmo **Follow the Gap** es una técnica de navegación reactiva para vehículos autónomos basada en la información del LiDAR. La idea principal es:

* Identificar los espacios libres (gaps) en el entorno.
* Evitar obstáculos cercanos mediante una **burbuja de seguridad**.
* Elegir el gap más amplio disponible y dirigir el vehículo hacia su centro.
* Ajustar la **velocidad** según el ángulo de giro, reduciéndola en curvas y aumentándola en rectas.

### 🧼 Lap Counter y Lap Timer

Ambos nodos utilizan la posición del vehículo (`/odom`) para detectar cuándo pasa por una zona específica del mapa (un checkpoint circular):

* `lap_counter.py`: cuenta la cantidad de vueltas completadas.
* `lap_timer.py`: mide y registra el tiempo de cada vuelta.


---

## ⚙️ Instrucciones de ejecución

### 1. Requisitos previos

* ROS 2 (recomendado: Humble)
* `f1tenth_gym_ros` correctamente instalado
* Dependencias: `numpy`, `rclpy`, `sensor_msgs`, `ackermann_msgs`, `nav_msgs`

### 2. Clonar el repositorio

```bash
git clone https://github.com/tu_usuario/f1tenth-follow-gap.git
cd f1tenth-follow-gap
```

### 3. Construir el paquete

Asegúrate de que los archivos `.py` estén dentro de un paquete de ROS 2 válido.

```bash
colcon build
source install/setup.bash
```

### 4. Ejecutar el sistema

```bash
ros2 launch f1tenth_gym_ros follow_gap.launch.py
```

Este comando lanza los tres nodos:

* 🧐 `follow_gap_node`: controla el auto con el algoritmo Follow the Gap.
* 🔢 `lap_counter_node`: cuenta vueltas cuando el vehículo pasa por el checkpoint.
* ⏱️ `lap_timer_node`: registra el tiempo de cada vuelta.

---

## 📜 Detalles del código

### `follow_gap.py`

* Suscribe a `/scan` para datos del LiDAR.
* Detecta el obstáculo más cercano y crea una burbuja de seguridad alrededor.
* Encuentra los gaps (secuencias de lecturas válidas).
* Selecciona el gap más amplio y dirige el volante hacia su centro.
* Ajusta la velocidad automáticamente dependiendo del ángulo de giro.

### `lap_counter.py`

* Suscribe a `/ego_racecar/odom`.
* Detecta si el vehículo entra en una zona circular (`zone_x`, `zone_y`, `radius`).
* Aumenta el contador de vueltas y lo imprime en consola.

### `lap_timer.py`

* Igual que `lap_counter.py`, pero en vez de contar vueltas, mide el tiempo entre cada paso por el checkpoint.
* Imprime en consola el tiempo de cada vuelta.

---

## 🧪 Ejemplo de salida en consola

```bash
⏱️ Tiempo vuelta 0: 12.34 s
⏱️ Tiempo vuelta 1: 11.98 s
🚗 Vuelta completada: 1
🚗 Vuelta completada: 2
```


## 👨‍💻 Autor

Proyecto desarrollado para el simulador F1TENTH con ROS 2.
Espol / Vehiculos no tripulados / Adrian Siavichay.
