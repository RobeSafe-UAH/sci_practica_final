# Ejecución

Ejecutar los comandos desde la carpeta `sci_practica_final`, con Docker
abierto y la imagen construida mediante `make build`.

## Lanzar la simulación

```bash
make run
```

El comando arranca el contenedor y espera a que el escritorio esté
disponible. Utiliza la imagen local existente; para construirla o
actualizarla, ejecutar antes `make build`.

## Abrir el navegador

Abrir **[http://localhost:6080/vnc.html?autoconnect=true&resize=scale](http://localhost:6080/vnc.html?autoconnect=true&resize=scale)**.
Si aparece la bienvenida de noVNC, pulsar **Connect**.

La ventana de MVSim muestra el circuito, el coche del alumno y los dos
vehículos lentos. El coche del alumno empieza parado y se controla mediante
[`/r0/cmd_vel`](topics.md#entrada-de-control). Los vehículos lentos circulan
por el carril derecho, en su mismo sentido, dejando libre el carril
contrario para los adelantamientos.

## Circuito y vehículos

La elipse se distribuye como los circuitos de `rbsf-racing`: mapa ROS
con imagen y YAML, más una línea central CSV con las anchuras de la pista.
MVSim carga el contorno del mapa para las colisiones y los sensores.

| Elemento | Configuración inicial |
|---|---|
| Circuito | `sci_ellipse`: semiejes centrales de 7 y 4 m. |
| Carretera | Dos carriles de 1 m, uno por sentido. |
| Coche del alumno | F1TENTH Ackermann, batalla de 0,3302 m; modelo visual RoboRacer de `rbsf-racing`. |
| Circulación | Antihoraria, por el carril derecho. |
| Tráfico | Dos coches a 0,35 m/s, inicialmente 5 y 17 m por delante. |

## Comandos del Makefile

| Comando | Acción |
|---|---|
| `make build` | Construir o actualizar la imagen. |
| `make run` | Arrancar la simulación y esperar al escritorio. |
| `make status` | Consultar el estado del contenedor. |
| `make logs` | Seguir los registros; `Ctrl+C` deja de seguirlos. |
| `make restart` | Reiniciar la simulación y recuperar las posiciones iniciales. |
| `make recreate` | Recrear el contenedor para aplicar cambios en `.env`. |
| `make down` | Detener y retirar el contenedor. Conserva la imagen. |
| `make help` | Mostrar los comandos disponibles. |

## Configuración

El escenario se define en `config/scenario.yaml`. Se pueden ajustar el
sentido de circulación, los límites del coche, la velocidad y posición
inicial del tráfico, y los sensores. Después de editarlo, aplicar los
cambios con `make restart`.

Los archivos del circuito están en
`src/sci_mvsim/tracks/sci_ellipse/`. Sus dimensiones y resolución deben
coincidir con los metadatos de `track` en la configuración.

Los dos vehículos de tráfico siguen recorridos prefijados, participan en
las colisiones y no frenan para evitar al coche del alumno.

### Puerto web y dominio ROS

Para modificar las variables de ejecución, crear `.env`:

```bash
cp .env.example .env
```

| Variable | Valor inicial | Uso |
|---|---|---|
| `SCI_VNC_PORT` | `6080` | Puerto del escritorio web. |
| `ROS_DOMAIN_ID` | `42` | Dominio compartido con el control externo. |
| `SCI_SCREEN_WIDTH` | `1280` | Anchura de la pantalla virtual. |
| `SCI_SCREEN_HEIGHT` | `800` | Altura de la pantalla virtual. |
| `SCI_HEADLESS` | `false` | Mostrar la ventana de MVSim. Usar `true` para ocultarla. |

Después de editar `.env`, ejecutar `make recreate`. Si cambia el puerto,
cambiarlo también en la dirección del navegador.

## Resolver un fallo de arranque

1. Comprobar el motor con `docker info`.
2. Consultar `make status` y `make logs`.
3. Si falta la imagen, ejecutar `make build`.
4. En Docker Desktop, comprobar que la red host está activada.
5. Si el puerto está ocupado, cambiar `SCI_VNC_PORT` en `.env` y ejecutar
   `make recreate`.
6. Si se ve el escritorio sin MVSim, comprobar `SCI_HEADLESS=false` y
   consultar los errores de configuración en los registros.
