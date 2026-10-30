---
hide:
  - toc
---
# ROS2

## Conexión

1. Desde la carpeta `sci_practica_final`, abrir **`docker/.env`** y editar **`ROS_DOMAIN_ID`** con el dominio elegido. El controlador debe utilizar ese mismo dominio.
2. Si se cambia el dominio con el simulador en marcha, detener `make run` con **Ctrl+C**, salir de las terminales del contenedor y ejecutar de nuevo `make run`. El cambio se aplica al crear el nuevo contenedor.
3. Con la simulación en marcha, abrir otra terminal en la carpeta del proyecto y entrar en el contenedor. En Windows, utilizar otra terminal de **Ubuntu de WSL2**:

    ```bash
    make attach
    ```

4. Comprobar el dominio desde esa terminal:

    ```bash
    echo "$ROS_DOMAIN_ID"
    ```

    Debe coincidir con el valor de `docker/.env` y con el del controlador. `make attach` abre una terminal con el dominio del contenedor en ejecución. Cambiarlo solo con `export` en esa terminal no cambia el dominio del simulador.

Todos los comandos `ros2` de esta página se ejecutan **dentro de la terminal abierta con `make attach`**. Escribir `exit` para salir de ella; la simulación continúa en la terminal de `make run`.

## Comprobar los topics

Estos comandos permiten comprobar la conexión antes de enviar órdenes al coche:

```bash
ros2 topic list
ros2 topic info /r0/cmd_vel --verbose
```

En la información de `/r0/cmd_vel` debe aparecer el tipo **`geometry_msgs/msg/Twist`** y al menos una suscripción del simulador. Antes de lanzar un controlador puede haber cero publicadores; es normal. Si no aparecen los topics `/r0/...`, comprobar que `make run` sigue activo y que el dominio coincide.

Los topics públicos de esta guía usan QoS **reliable**, durabilidad **volatile** y una cola de **10 mensajes**. La consulta con `--verbose` permite ver esta configuración.

## Control

El coche recibe órdenes en **`/r0/cmd_vel`**, de tipo **`geometry_msgs/msg/Twist`**:

| Campo | Significado | Unidad |
|---|---|---|
| `linear.x` | Velocidad de avance, entre 0 y 13,89 m/s. | m/s |
| `angular.z` | Velocidad angular; positiva hacia la izquierda y negativa hacia la derecha. | rad/s |

Los límites son **0 a 50 km/h (0 a 13,89 m/s)** de velocidad y **−45° a +45°** de dirección de las ruedas. Las velocidades negativas se convierten en cero. `angular.z` es una velocidad angular, no un ángulo de dirección: el simulador la limita según la velocidad de avance y la geometría del coche. Con velocidad de avance cero, el coche permanece inmóvil y no puede girar sobre sí mismo.

### Enviar una orden de prueba

1. Mantener abierto el simulador en el navegador. Para esta prueba, dejar detenido cualquier otro controlador, incluido MATLAB, de modo que solo haya una fuente de órdenes.
2. Publicar una velocidad de **0,5 m/s**, sin giro, a **10 Hz**:

    ```bash
    ros2 topic pub --rate 10 /r0/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.5}, angular: {z: 0.0}}'
    ```

    El comando continúa publicando hasta pulsar **Ctrl+C**. Los campos del mensaje que no se indican toman el valor cero. Para probar un giro suave hacia la izquierda, se puede repetir con `angular: {z: 0.2}`; para girar hacia la derecha, utilizar un valor negativo.

3. Pulsar **Ctrl+C** en la terminal de `ros2 topic pub` para dejar de enviar órdenes. El coche se detiene cuando pasa **un segundo sin recibir una orden**. Este mecanismo exige que el controlador publique periódicamente; una orden aislada no mantiene el movimiento.
4. Para ordenar la parada de forma explícita, publicar una velocidad cero después de detener el publicador anterior:

    ```bash
    ros2 topic pub --once /r0/cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.0}, angular: {z: 0.0}}'
    ```

    `--once` envía un único mensaje y termina. Si el comando espera una suscripción, comprobar que la simulación está en marcha y que el dominio es el mismo.

Todos los coches permanecen parados hasta recibir el **primer mensaje válido** en `/r0/cmd_vel`, incluso si contiene velocidad cero. Ese primer mensaje inicia el tráfico: los coches **naranjas** circulan por el carril derecho. Las órdenes de parada y el tiempo de espera de un segundo afectan al coche del alumno; los coches de tráfico continúan circulando.

Para programar un controlador completo, consultar la pestaña [MATLAB](matlab.md).

## Colisiones

**`/r0/collision`**, de tipo **`std_msgs/msg/Bool`**, pasa de `false` a `true` al detectar un choque y mantiene ese valor hasta reiniciar la simulación. Consultar su estado:

```bash
ros2 topic echo /r0/collision --field data --once --qos-reliability reliable
```

Para reiniciar el estado de colisión y volver a la salida, detener `make run` con **Ctrl+C** y ejecutarlo de nuevo.

## Sensores

Los sensores se publican desde el arranque, también con el coche parado:

| Topic | Tipo | Información |
|---|---|---|
| `/r0/scan` | `sensor_msgs/msg/LaserScan` | Hokuyo: 270°, 1081 rayos, 20 Hz y alcance máximo de 30 m. |
| `/r0/sonar0` … `/r0/sonar7` | `sensor_msgs/msg/Range` | Media de cada sector del LiDAR, a 20 Hz. |
| `/r0/odom` | `nav_msgs/msg/Odometry` | Posición, orientación y velocidad del coche, a 20 Hz. |

Leer una medida del sonar 0, expresada en metros:

```bash
ros2 topic echo /r0/sonar0 --field range --once --qos-reliability reliable
```

Leer la posición del coche en metros, con las coordenadas `x`, `y` y `z`:

```bash
ros2 topic echo /r0/odom --field pose.pose.position --once --qos-reliability reliable
```

`--field` selecciona la parte del mensaje que se quiere ver y `--once` termina tras recibir una medida. Quitar `--once` para observar la evolución y pulsar **Ctrl+C** para dejar de leer. Para consultar la velocidad de avance, sustituir `pose.pose.position` por `twist.twist.linear.x`. Estos comandos de lectura no envían órdenes al coche.

La frecuencia predeterminada es **20 Hz**. Para cambiar la frecuencia común de la física, los sensores y las publicaciones ROS2, editar `simulation.rate_hz` en `docker/config/scenario.yaml`. Reiniciar la simulación con **Ctrl+C** y `make run` para aplicar el cambio.

## Sonares

1. Identificar los ocho sectores de **33,75°**. El ángulo cero apunta hacia delante.

<div class="sonar-layout" markdown="1">

<div markdown="1">

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

</div>

<div class="sonar-figure" markdown="1">

![Vista superior del coche: los ocho conos parten del Hokuyo, desde sonar 0 a la derecha hasta sonar 7 a la izquierda.](assets/sonares.svg)

</div>

</div>

2. Leer `range`: media de las distancias válidas; `+Inf` sin retorno y `NaN` sin medida válida. Los 90° traseros quedan fuera de cobertura.
