# Topics ROS

## Conexión

Se utiliza **ROS 2 Jazzy** y `ROS_DOMAIN_ID=42`. El control y el simulador
deben compartir dominio y tener comunicación DDS. Para trabajar con el
reloj de la simulación, utilizar `use_sim_time=true`.

## Control

**`/r0/cmd_vel` · `geometry_msgs/msg/Twist`**

| Campo | Significado | Unidad |
|---|---|---|
| `linear.x` | Velocidad longitudinal; positiva hacia delante. | m/s |
| `angular.z` | Velocidad angular; positiva hacia la izquierda. | rad/s |

Los demás componentes no se utilizan. `angular.z` es la velocidad angular
del coche, no el ángulo de las ruedas. Los límites son **100 km/h (27,78 m/s)** de velocidad
y **24°** de dirección. El coche no gira sobre el sitio.

El límite de velocidad se configura con `robot.max_speed`, en m/s, en
`docker/config/scenario.yaml`. Las consignas que lo superan se recortan. La velocidad
efectiva depende de la aceleración y la dinámica del vehículo.

El primer mensaje válido, aunque sea cero, pone en marcha las cajas configuradas.
Después, las cajas siguen circulando aunque el coche esté parado.
Enviar las órdenes a **10–20 Hz**: tras **un segundo sin mensajes nuevos**,
la consigna del coche vuelve a cero.

## Colisiones

**`/r0/collision` · `std_msgs/msg/Bool`**

- `false`: no se ha detectado ninguna colisión durante esta sesión.
- `true`: el coche ha colisionado con una caja o con los límites de la pista.

Se publica a **20 Hz**. Una vez detectada una colisión, permanece en `true`
hasta reiniciar la simulación. Este topic informa del choque; no modifica
las órdenes ni activa maniobras para evitarlo.

## Sensores y estado

| Topic | Tipo | Información |
|---|---|---|
| `/r0/scan` | `sensor_msgs/msg/LaserScan` | Hokuyo: 270°, 1081 rayos, 40 Hz y alcance máximo de 30 m; frame `r0/laser`. |
| `/r0/sonar0` … `/r0/sonar7` | `sensor_msgs/msg/Range` | Media de cada uno de los ocho sectores del mismo LiDAR, a 40 Hz. |
| `/r0/odom` | `nav_msgs/msg/Odometry` | Pose y velocidad en `r0/odom`; cuerpo `r0/base_link`. |
| `/r0/ground_truth/odom` | `nav_msgs/msg/Odometry` | Pose exacta del coche en `map`. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Tiempo de simulación. |
| `/tf` y `/tf_static` | `tf2_msgs/msg/TFMessage` | Posiciones y orientaciones del coche y sus sensores. |

### Sonares

Cada sector abarca **33,75°**. El índice 0 está a la derecha y el 7 a la
izquierda. El ángulo cero apunta hacia delante.

| Sonar | Ángulo central |
|---|---:|
| 0 | −118,125° |
| 1 | −84,375° |
| 2 | −50,625° |
| 3 | −16,875° |
| 4 | 16,875° |
| 5 | 50,625° |
| 6 | 84,375° |
| 7 | 118,125° |

`range` es la media de los retornos finitos válidos del sector. `+Inf`
indica ausencia de retorno y `NaN`, ausencia de una medida válida.
Los límites y la marca de tiempo proceden del LiDAR. Los ocho conos tienen
su origen en el Hokuyo, a 0,27 m por delante y 0,15 m de altura respecto al
coche. Los 90° traseros quedan fuera de su cobertura.

Los topics de control, sensores, odometría y colisiones utilizan QoS
**reliable**, durabilidad **volatile** y profundidad **10**.
