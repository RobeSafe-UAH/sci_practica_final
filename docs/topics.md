# Topics ROS

La interfaz utiliza **ROS 2 Jazzy**. El control externo publica órdenes de
movimiento y recibe los sensores; el simulador no incluye lógica de
navegación ni de adelantamiento.

## Conexión

El valor inicial de `ROS_DOMAIN_ID` es **42**. El control y el simulador
deben compartir dominio, disponer de conectividad DDS y tener permitido
el tráfico UDP correspondiente. Si se usa el reloj simulado, configurar
`use_sim_time=true` en los nodos que consumen los datos.

Los siguientes nombres corresponden a `robot.name: r0` en
`config/scenario.yaml`.

## Entrada de control

**Topic:** `/r0/cmd_vel` · **Tipo:** `geometry_msgs/msg/Twist`.

| Campo | Significado | Unidad |
|---|---|---|
| `linear.x` | Velocidad longitudinal; positiva hacia delante. | m/s |
| `angular.z` | Velocidad de giro del coche; positiva hacia la izquierda. | rad/s |

Los demás componentes no se utilizan. `angular.z` representa velocidad
angular, no el ángulo de las ruedas. El puente limita la orden según
`robot.max_speed`, `robot.wheelbase` y `robot.max_steer_deg`.
El coche Ackermann no gira sobre el sitio.

Los límites iniciales del escenario son **1 m/s** de velocidad longitudinal
y **24°** de dirección de las ruedas. Las órdenes son consignas: la
velocidad alcanzada se consulta en la odometría.

Se deben enviar órdenes periódicamente. Si transcurre **un segundo sin
una orden nueva**, se aplica una parada. Este plazo utiliza el reloj del
sistema y sigue activo aunque se detenga el reloj simulado. Una frecuencia
de envío de **10–20 Hz** mantiene la orden renovada.

## Sensores y estado

| Topic | Tipo ROS 2 | Contenido / frame |
|---|---|---|
| `/r0/scan` | `sensor_msgs/msg/LaserScan` | LiDAR, frame `r0/laser`. |
| `/r0/sonar0` … `/r0/sonar7` | `sensor_msgs/msg/Range` | Un topic por sonar, frame `r0/sonar0` … `r0/sonar7`. |
| `/r0/odom` | `nav_msgs/msg/Odometry` | Pose y velocidad en `r0/odom`; cuerpo `r0/base_link`. |
| `/r0/ground_truth/odom` | `nav_msgs/msg/Odometry` | Pose exacta del simulador en `map`. |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transformación dinámica de odometría al coche. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Transformaciones fijas de mapa, odometría y sensores. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Tiempo de simulación. |

Los topics bajo `/_sim/` son internos. La odometría procede de la pose
exacta del simulador; no incluye deriva de un estimador.

### LiDAR

La configuración inicial publica barridos a **20 Hz**, con **1081 rayos**,
un campo de visión de **270°** y alcance máximo de **12 m**. Los ángulos y
distancias de cada barrido se describen en los campos del propio mensaje
`LaserScan`.

### Ocho sonares

Cada sonar publica a **20 Hz**, con un cono de **15°** y un intervalo de
medida entre **0,04 y 5 m**. El eje `x` del coche apunta hacia delante y el
eje `y` hacia la izquierda.

| Índice | Topic | Ángulo | Orientación |
|---|---|---:|---|
| 0 | `/r0/sonar0` | 0° | Delante. |
| 1 | `/r0/sonar1` | 45° | Delante e izquierda. |
| 2 | `/r0/sonar2` | 90° | Izquierda. |
| 3 | `/r0/sonar3` | 135° | Detrás e izquierda. |
| 4 | `/r0/sonar4` | 180° | Detrás. |
| 5 | `/r0/sonar5` | −135° | Detrás y derecha. |
| 6 | `/r0/sonar6` | −90° | Derecha. |
| 7 | `/r0/sonar7` | −45° | Delante y derecha. |

El campo `range` contiene el retorno válido más cercano del cono, en
metros. `+Inf` indica que no hay retorno dentro del alcance y `-Inf` que
hay un objeto por debajo de la distancia mínima. Cada mensaje conserva
el instante de adquisición en `header.stamp`.
`variance=0` indica que la varianza de la medida es desconocida.

Los sonares se simulan mediante haces geométricos con ruido de alcance;
no modelan propagación acústica.

## QoS y transformaciones

La entrada de control y las salidas de sensores y odometría utilizan
**reliable**, durabilidad **volatile** y profundidad **10**. `/tf_static`
utiliza durabilidad **transient local**. El reloj `/clock` utiliza el QoS
del publicador nativo de MVSim.

Las transformaciones forman este árbol:

```text
map → r0/odom → r0/base_link → r0/laser
                           → r0/sonar0 … r0/sonar7
```
