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

3. **`docs/`**: páginas de esta guía, configuración de MkDocs y dependencias de documentación.
4. **`Makefile`**: comandos `build`, `run` y `attach`.
5. **`README.md`**: acceso a la guía publicada. **`LICENSE`**: licencia del proyecto.

## Parámetros del simulador

1. Editar **`docker/config/scenario.yaml`**:

    | Parámetro | Uso |
    |---|---|
    | `track.length`, `track.height` | Dimensiones de la línea central, en metros. |
    | `track.corner_radius` | Radio de las curvas, en metros. |
    | `track.width` | Anchura total de la carretera, dividida en dos carriles iguales. |
    | `robot.max_speed` | Velocidad máxima en m/s: 27,77777777777778 = 100 km/h. |
    | `robot.max_steer_deg` | Límite de dirección en grados: 45, para un rango de −45° a +45°. |
    | `traffic.count` | Número de cajas, de 0 a 6. |
    | `traffic.speed` | Velocidad de las cajas en m/s. |
    | `traffic.start_distance`, `traffic.spacing` | Distancia inicial y separación entre cajas, en metros. |
    | `traffic.box_length`, `traffic.box_width`, `traffic.box_height` | Dimensiones de las cajas, en metros. |
    | `lidar` | Posición, cobertura, frecuencia y alcance del Hokuyo. |

2. Reiniciar con **Ctrl+C** y `make run` para aplicar los parámetros. Los recursos generados se guardan en un volumen de Docker y se reutilizan mientras no cambien la configuración o los recursos del simulador. Tras cambiar código o definiciones XML, ejecutar también `make build`.

## Funcionamiento

1. El primer mensaje válido en `/r0/cmd_vel`, incluso con velocidad cero, inicia la circulación de las cajas.
2. Enviar órdenes de forma continua: tras un segundo sin mensajes, la consigna del coche vuelve a cero.
3. Cada nueva ejecución restablece las posiciones y el estado de colisión.
