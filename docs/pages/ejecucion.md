# Ejecución

## Construir y lanzar

Desde la carpeta del proyecto, con Docker abierto:

```bash
make build
make run
```

`make build` construye la imagen. Repetirlo después de actualizar el proyecto.
`make run` ejecuta el simulador en la terminal actual y muestra sus registros.
**Mantenerla abierta. Al pulsar Ctrl+C, el contenedor se detiene y se elimina.**

## Abrir el navegador

Abrir [http://localhost:6080/vnc.html?autoconnect=true&resize=scale](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Si aparece la bienvenida de noVNC, pulsar **Connect**.

La cámara muestra una perspectiva 3D que sigue al Traxxas. La rueda del
ratón permite cambiar el zoom de la vista.

El coche y las cajas empiezan parados. El primer mensaje válido en
[`/r0/cmd_vel`](topics.md#control) inicia la circulación de las cajas,
incluso si la orden tiene velocidad cero. Después continúan dando vueltas.
El coche se controla con las órdenes ROS del estudiante.

## Comandos Docker

Ejecutar estos comandos en otra terminal mientras el simulador está abierto:

| Comando | Uso |
|---|---|
| `docker ps` | Ver los contenedores en ejecución. |
| `docker ps -a` | Ver también los contenedores detenidos. |
| `docker stop sci-practica-final` | Detener la simulación desde otra terminal. |
| `docker rm NOMBRE_O_ID` | Eliminar un contenedor detenido que aparezca en `docker ps -a`. |

El contenedor creado por `make run` se elimina automáticamente al terminar.
Para iniciar una sesión nueva, ejecutar otra vez `make run`.

## Escenario

| Elemento | Configuración |
|---|---|
| Pista | Rectángulo de 60 × 36 m en su línea central, con cuatro rectas y cuatro curvas de radio 8 m. |
| Carriles | Dos carriles de 2 m, uno por sentido. |
| Vehículo | Traxxas F1TENTH Ackermann, con las mallas 3D de `rbsf-racing`. |
| Velocidad máxima | 100 km/h, equivalentes a 27,78 m/s en `/r0/cmd_vel`. |
| Cajas | Dos por defecto, de 0,90 × 0,55 × 0,45 m, a 0,35 m/s por el carril derecho; inicialmente 5 y 17 m por delante. |

Las cajas siguen su recorrido sin frenar ni desviarse para evitar al coche.
Los choques se resuelven mediante la física del simulador y se notifican por ROS.

## Editar el escenario

Modificar `docker/config/scenario.yaml` y reiniciar con Ctrl+C y `make run`.
Los cambios de configuración no requieren `make build`: el mapa, los bordillos
y los recorridos se generan al arrancar.

| Parámetro | Uso |
|---|---|
| `traffic.count` | Número de cajas: entero de **0 a 6**. Cero desactiva el tráfico. |
| `traffic.start_distance` | Distancia inicial de la primera caja por delante del coche, en metros. |
| `traffic.spacing` | Separación inicial entre cajas consecutivas, en metros. |
| `traffic.box_length`, `traffic.box_width`, `traffic.box_height` | Largo, ancho y alto de las cajas, en metros. |
| `track.length`, `track.height` | Dimensiones del rectángulo de la línea central, en metros. |
| `track.corner_radius` | Radio de las cuatro curvas de la línea central, en metros. |
| `track.width` | Anchura total de la calzada, repartida entre los dos carriles. |
| `robot.max_speed` | Límite de velocidad en **m/s**; 100 km/h = 100 / 3,6 m/s. |

El radio debe ser menor que la mitad de la dimensión más pequeña del
rectángulo y mayor que la mitad de la anchura de la calzada más el espesor
del límite. Las cajas deben caber en su carril y sus posiciones iniciales
deben quedar dentro de una vuelta, sin solapamientos.

Con la configuración predeterminada, el coche comienza en **(x, y) = (0, −19) m**,
orientado hacia `+x`. Cambiar las dimensiones del circuito cambia esta posición.
