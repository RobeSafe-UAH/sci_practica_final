---
hide:
  - toc
---
# Repositorio

## Archivos principales

1. **`docker/`**: imagen, contenedor y configuración del simulador.

    | Archivo | Uso |
    |---|---|
    | `config/scenario.yaml` | Circuito, vehículo, cajas y sensores. |
    | `.env` | Dominio ROS y puerto del navegador. |
    | `Dockerfile` | Dependencias y construcción de la imagen. |
    | `compose.yaml` | Servicio del simulador. |
    | `start.sh` | Arranque y cierre de la simulación y del escritorio web. |

2. **`src/`**: código y recursos de simulación.

    | Carpeta | Contenido |
    |---|---|
    | `sci_mvsim/definitions/` | Modelo físico del vehículo, Hokuyo y cajas. |
    | `sci_mvsim/models/` | Mallas 3D del coche. |
    | `sci_mvsim/tracks/` | Mapa de referencia del circuito. |
    | `sci_mvsim/launch/` | Lanzamiento ROS2. |
    | `sci_mvsim/sci_mvsim/` | Generación de la pista y publicación de sensores y colisiones. |
    | `sci_start_gate/` | Mantiene el coche y las cajas parados hasta la primera orden. |

3. **`matlab/`**: carpeta de trabajo del controlador y la evaluación.

    | Archivo | Uso |
    |---|---|
    | `ackerman_ROS2_controller.slx` | Modelo Simulink: lectura de sensores, controlador, publicación de órdenes y parada. |
    | `controller_ros2.m` | Configuración del dominio ROS2, ejecución del modelo y evaluación del resultado. |
    | `evaluate_controller.m` | Lectura de `data.mat` y figura con tiempo, recorrido, ocupación de carril y consignas. |

4. **`docs/`**: páginas de esta guía, configuración de MkDocs y dependencias de documentación.
5. **`Makefile`**: comandos `build`, `run` y `attach`.
6. **`README.md`**: acceso a la guía publicada. **`LICENSE`**: licencia del proyecto.

## Trabajar en MATLAB

1. Abrir MATLAB y seleccionar **`matlab/` dentro del repositorio** como carpeta actual.
2. Crear el controlador propio y guardarlo como **`fuzzy_controller.fis`** en esa carpeta. El modelo utiliza ocho entradas, de `sonar_0` a `sonar_7`, en metros, y dos salidas: `V`, en km/h, y `W`, como mando de dirección en grados.
3. Editar el dominio en **`controller_ros2.m`** para que coincida con `ROS_DOMAIN_ID` en `docker/.env`. En el modelo Simulink, abrir **ROS 2 → Configure ROS Network** y seleccionar el mismo **Domain ID**.
4. Con el simulador en ejecución, lanzar desde la ventana de comandos de MATLAB:

    ```matlab
    controller_ros2
    ```

5. El modelo se detiene al cruzar la línea de meta tras una vuelta o detectar una colisión. El registro se guarda en `data.mat` y se abre la figura de evaluación.
6. El repositorio versiona únicamente el modelo `.slx` y los dos scripts `.m`. El FIS y los archivos generados por MATLAB permanecen locales.

## Parámetros del simulador

1. Editar **`docker/config/scenario.yaml`**:

    | Parámetro | Uso |
    |---|---|
    | `simulation.rate_hz` | Frecuencia de la física, sensores y publicaciones ROS2. Por defecto, 20 Hz (paso de 0,05 s). |
    | `track.length`, `track.height` | Dimensiones de la línea central, en metros. |
    | `track.corner_radius` | Radio de las curvas, en metros. |
    | `track.width` | Anchura total de la carretera, dividida en dos carriles iguales. |
    | `robot.max_speed` | Velocidad máxima en m/s: 27,77777777777778 = 100 km/h. |
    | `robot.max_steer_deg` | Límite de dirección en grados: 45, para un rango de −45° a +45°. |
    | `traffic.count` | Número de cajas, de 0 a 6. |
    | `traffic.speed` | Velocidad de las cajas en m/s. |
    | `traffic.start_distance`, `traffic.spacing` | Distancia inicial y separación entre cajas, en metros. |
    | `traffic.box_length`, `traffic.box_width`, `traffic.box_height` | Dimensiones de las cajas, en metros. |
    | `lidar` | Posición, cobertura, número de rayos y alcance del Hokuyo. |

2. Guardar los cambios, detener la simulación con **Ctrl+C** y volver a lanzarla:

    ```bash
    make run
    ```

3. Si se ha cambiado código o alguna definición XML, construir la imagen antes de lanzarla:

    ```bash
    make build
    make run
    ```

4. Al cambiar `simulation.rate_hz`, ajustar también **Ctrl+E → Solver → Fixed-step size** en Simulink a `1 / simulation.rate_hz`. A 20 Hz corresponde un paso de **0,05 s**. Si se cambian las dimensiones del circuito, actualizar la geometría fija de `evaluate_controller.m`.
5. Los recursos del circuito se guardan en un volumen de Docker y se reutilizan mientras no cambien la configuración o los recursos del simulador.

## Funcionamiento

1. El primer mensaje válido en `/r0/cmd_vel`, incluso con velocidad cero, inicia la circulación de las cajas.
2. Enviar órdenes de forma continua: tras un segundo sin mensajes, la consigna del coche vuelve a cero.
3. Cada nueva ejecución restablece las posiciones y el estado de colisión.
