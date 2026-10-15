# Práctica final de SCI · Simulador

Entorno MVSim con un coche Ackermann, dos vehículos lentos, LiDAR y ocho
sonares. El control del coche se recibe por ROS 2 en `/r0/cmd_vel`.

| Documentación | Contenido |
|---|---|
| [Instalación](docs/index.md) | Preparar cada sistema operativo y construir con `make build`. |
| [Ejecución](docs/ejecucion.md) | Lanzar con `make run`, abrir el navegador y detener el simulador. |
| [Topics ROS](docs/topics.md) | Entrada de control, sensores, odometría y transformaciones. |

## Documentación en GitHub Pages

El sitio usa Material for MkDocs con tres pestañas principales y pestañas
por sistema operativo dentro de Instalación.

Para publicarlo, seleccionar **Settings → Pages → Build and deployment →
Source → GitHub Actions**. El workflow [Documentación](.github/workflows/docs.yml)
comprueba el sitio en las pull requests y lo publica al subir cambios a `main`.

La dirección del sitio será
[robesafe-uah.github.io/sci_practica_final](https://robesafe-uah.github.io/sci_practica_final/).

Para revisar el sitio localmente se necesita Python 3.10 o posterior:

```bash
python3 -m venv .venv-docs
.venv-docs/bin/python -m pip install -r requirements-docs.txt
make docs
```

Abrir <http://127.0.0.1:8000>. Estos comandos se ejecutan en Linux, macOS o
Ubuntu de WSL2. `make docs-build` comprueba el sitio y genera `site/`.
Python y MkDocs solo son necesarios para editar la documentación.

La procedencia de los componentes se recoge en [NOTICE](NOTICE).
[Licencia Apache 2.0](LICENSE).
