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

El coche y las dos cajas empiezan parados. El primer mensaje válido en
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
| Pista | Rectángulo de 40 × 24 m en su línea central, con cuatro rectas y cuatro curvas de radio 5 m. |
| Carriles | Dos carriles de 2 m, uno por sentido. |
| Vehículo | Traxxas F1TENTH Ackermann, con las mallas 3D de `rbsf-racing`. |
| Cajas | Dos, a 0,35 m/s por el carril derecho; inicialmente 5 y 17 m por delante. |

Las cajas siguen su recorrido sin frenar ni desviarse para evitar al coche.
Los choques se resuelven mediante la física del simulador y se notifican por ROS.
