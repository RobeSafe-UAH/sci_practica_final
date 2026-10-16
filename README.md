# Práctica final de SCI · Simulador

Simulador MVSim 3D con el modelo Traxxas de `rbsf-racing`, un Hokuyo y ocho
sonares derivados del LiDAR. La pista tiene cuatro rectas y cuatro curvas,
dos carriles, asfalto, césped y bordillos. La cámara sigue al coche.

Las dos cajas esperan al primer mensaje en `/r0/cmd_vel` para empezar a
circular. Las colisiones se notifican en `/r0/collision`.

| Documentación | Contenido |
|---|---|
| [Instalación](docs/index.md) | Preparar el equipo y construir la imagen. |
| [Ejecución](docs/ejecucion.md) | Lanzar, abrir el navegador y gestionar el contenedor. |
| [Topics ROS](docs/topics.md) | Control, sensores y colisiones. |
| [Repositorio](docs/repositorio.md) | Arquitectura, archivos y configuración del entorno. |

[Licencia Apache 2.0](LICENSE) · [Procedencia de los recursos](NOTICE)
