# Arquitectura del repositorio

## Alcance

El proyecto proporciona un entorno de simulación para un vehículo Ackermann:
física de movimiento y colisiones, de cero a seis obstáculos móviles, sensores y una
interfaz ROS 2. El control del vehículo se ejecuta fuera del contenedor.

El entorno utiliza ROS 2 Jazzy y MVSim. La presentación 3D incluye el modelo
Traxxas/F1TENTH de `rbsf-racing`, ruedas articuladas, asfalto, césped,
bordillos, señalización vial e iluminación con sombras. La cámara sigue al
vehículo. La representación gráfica y el mapa de colisiones son capas distintas.

## Organización

```text
sci_practica_final/
├── docker/
│   ├── config/scenario.yaml   Parámetros del circuito, vehículo, tráfico y sensores
│   ├── compose.yaml           Servicio, red y configuración del contenedor
│   ├── Dockerfile             Construcción de la imagen
│   ├── Dockerfile.dockerignore Archivos excluidos del contexto de construcción
│   ├── .env.example           Variables opcionales del contenedor
│   └── ...                    Arranque, escritorio virtual y configuración DDS
├── docs/
│   ├── pages/                 Páginas de la documentación
│   ├── mkdocs.yml             Navegación y presentación del sitio
│   └── requirements.txt       Dependencias para construir la documentación
├── src/
│   ├── sci_mvsim/             Generación del mundo e interfaz ROS 2
│   │   ├── definitions/       Vehículo, Hokuyo y cajas
│   │   ├── launch/            Lanzamiento de los procesos ROS
│   │   ├── models/f1tenth/    Mallas 3D del vehículo y su licencia
│   │   ├── resource/          Registro del paquete en ament
│   │   ├── sci_mvsim/         Módulos Python
│   │   ├── tracks/            Referencia del circuito predeterminado
│   │   ├── package.xml        Dependencias ROS
│   │   └── setup.py           Instalación de código y recursos
│   └── sci_start_gate/        Extensión nativa de MVSim para el inicio de sesión
├── Makefile                   Construcción y ejecución
├── README.md                  Acceso a la documentación publicada
└── LICENSE                    Licencia del proyecto
```

`README.md` enlaza las páginas publicadas en GitHub Pages.
`.github/workflows/docs.yml` construye el sitio con las dependencias de
`docs/requirements.txt` y lo publica. Las licencias de los recursos de terceros
se conservan junto a los modelos y al circuito.

## Componentes de simulación

| Componente | Responsabilidad |
|---|---|
| `geometry.py` | Geometría del circuito, interpolación por distancia y orientación de los recorridos. |
| `world.py` | Lectura de la configuración, preparación del mapa físico y generación del mundo MVSim. |
| `scene.py` | Superficie de la pista, marcas viales, bordillos 3D e iluminación. |
| `bridge.py` | Órdenes de control, vigilancia de su caducidad, sensores derivados, colisiones y transformaciones. |
| `simulator.launch.py` | Creación del mundo temporal y arranque de MVSim y del puente ROS. |
| `sci_start_gate` | Espera de la primera orden y sincronización del inicio del tráfico con el tiempo de simulación. |

`resource/sci_mvsim` es el marcador que permite localizar el paquete mediante
ament. `setup.cfg` establece la ubicación de sus ejecutables ROS. Los archivos
XML de `definitions/` describen los cuerpos físicos, el controlador Ackermann
y el Hokuyo; las mallas de `models/` aportan la representación visual del coche.

En `sci_start_gate`, `CMakeLists.txt` construye la biblioteca nativa y
`src/start_gate.cpp` registra el elemento que sincroniza el inicio dentro
de MVSim. `include/sci_start_gate/initial_lock.hpp` conserva la masa y la
inercia originales al habilitar el coche. El lanzamiento carga esta biblioteca
en el proceso de MVSim.

La imagen se construye por etapas: las herramientas de compilación se usan
en la etapa de construcción y la instalación resultante pasa a la imagen
de ejecución.

| Archivo de ejecución | Función |
|---|---|
| `docker/start.sh` | Iniciar y cerrar conjuntamente el escritorio virtual, noVNC y la simulación ROS. |
| `docker/healthcheck.py` | Supervisar la disponibilidad de los procesos y del escritorio web. |
| `docker/fastdds.xml` | Configurar el transporte DDS utilizado por ROS 2. |

## Secuencia de arranque

1. `make build` construye la imagen e instala los dos paquetes ROS.
2. `make run` crea el contenedor `sci-practica-final` y conecta la terminal a
   sus registros.
3. `docker/start.sh` prepara la pantalla virtual, el escritorio, noVNC y ROS.
4. El lanzamiento genera un mundo autocontenido en un directorio temporal
   del contenedor. Allí se preparan el mapa, las texturas y las animaciones.
5. MVSim publica las observaciones. El puente ofrece la interfaz pública
   descrita en [Topics ROS](topics.md).
6. El primer `/r0/cmd_vel` válido habilita el vehículo e inicia las cajas configuradas.
   Una orden cero también inicia la sesión.

Las cajas utilizan animaciones periódicas de MVSim. Su tiempo permanece en
cero antes de la primera orden y después avanza con el reloj de simulación.
No dependen de que el estudiante siga enviando consignas al coche.

Tras el inicio, el vehículo conserva su dinámica y sus colisiones físicas.
Si dejan de recibirse órdenes durante un segundo, el puente envía una
consigna cero. La detección de una colisión no modifica el control.

## Configuración del escenario

`docker/config/scenario.yaml` se monta en el contenedor en modo de solo lectura.
Las distancias se expresan en metros y los tiempos en segundos. Los ángulos
usan radianes salvo los parámetros que indican explícitamente grados.
La velocidad se expresa en m/s: **100 km/h = 27,77777777777778 m/s**.

| Sección | Parámetros principales |
|---|---|
| `track` | Dimensiones de la línea central, radio de las curvas, anchura total, sentido, espesor del límite y resolución del mapa. |
| `robot` | Nombre ROS, posición inicial por distancia recorrida, batalla y límites de velocidad y dirección. |
| `traffic` | Número de cajas (0–6), velocidad, dimensiones, distancia inicial y separación. |
| `lidar` | Posición, campo de visión, número de rayos, periodo y alcance. |
| `sonar` | Ocho sectores derivados del barrido del LiDAR. |

El circuito predeterminado tiene una línea central de **60 × 36 m**, cuatro curvas de
**radio 8 m** y una anchura total de **4 m**. Sus dos carriles tienen sentidos
opuestos. El vehículo y las cajas comienzan en el carril derecho, con
circulación antihoraria. El origen del vehículo coincide con el eje trasero:
`x` apunta hacia delante, `y` hacia la izquierda y `z` hacia arriba.

Las cajas miden **0,90 × 0,55 × 0,45 m**. `traffic.count` determina cuántas
se crean; las posiciones se calculan con `start_distance + índice × spacing`
desde el punto de salida del coche, siguiendo el carril derecho. No se
mantiene una lista de posiciones por caja. Con cero cajas, el circuito
conserva el vehículo, los sensores y la detección de colisiones con los límites.

Al reiniciar, el generador utiliza las dimensiones actuales de `track` para
crear el mapa físico, la línea central, las superficies, los bordillos y
las trayectorias del tráfico. Los cambios de `scenario.yaml` se aplican
con un nuevo `make run`, sin reconstruir la imagen.

La batalla de **0,3302 m** y la dirección máxima de **24°** corresponden al
modelo F1TENTH. Reducir los límites de control no modifica esa geometría.

## Archivos del circuito

Durante el arranque, `world.py` genera estos archivos en el directorio temporal
de la sesión. `src/sci_mvsim/tracks/circuito_sci/` conserva una referencia de
la configuración predeterminada:

| Archivo | Contenido |
|---|---|
| `circuito_sci.png` | Mapa de ocupación: blanco transitable y negro ocupado. |
| `circuito_sci.yaml` | Imagen, resolución, origen, umbrales y metadatos geométricos. |
| `circuito_sci_centerline.csv` | Coordenadas de la línea central y anchura disponible a derecha e izquierda. |

El formato sigue la integración de circuitos de `rbsf-racing`. El trazado
SCI es propio y se genera con cuatro rectas enlazadas por arcos circulares.
`world.py` conserva solo una banda del contorno ocupado para las colisiones
y el LiDAR 2D. El césped, las flechas y las líneas pintadas no son obstáculos.
Los bordillos visibles se alinean con esos límites; su malla no añade una
segunda geometría de colisión.

El mundo utiliza los archivos generados para la sesión. El PNG de referencia
del repositorio no limita las dimensiones editadas en `scenario.yaml`.

## Interfaz y estado de sesión

La entrada pública es `/r0/cmd_vel`, de tipo `geometry_msgs/msg/Twist`.
El puente publica el LiDAR, las ocho medias por sectores, la odometría,
las transformaciones y `/r0/collision`. Los topics de `/_sim/` conectan los
procesos internos; el control externo utiliza los nombres públicos.

`/r0/collision` se publica como `std_msgs/msg/Bool`. Su valor inicial es
`false`; pasa a `true` al detectar un choque y permanece así durante la
sesión. Un nuevo `make run` restablece las posiciones, la espera de la
primera orden y el estado de colisión.

## Ejecución y recursos

El contenedor incluye ROS, MVSim, las mallas y los servicios gráficos.
El navegador accede al escritorio a través de noVNC en el puerto **6080**.
Los puertos y el dominio ROS se pueden configurar mediante `docker/.env`, tomando
`docker/.env.example` como referencia. El Makefile selecciona
`docker/compose.yaml`, que conserva el nombre del proyecto y del contenedor.

La red utiliza el modo host para DDS. El renderizado se realiza por software.
El proceso de simulación se ejecuta con un usuario sin privilegios y la
configuración del escenario se mantiene separada de la imagen.

`make run` permanece en primer plano. Ctrl+C termina los procesos y elimina
el contenedor; la imagen construida se conserva para la siguiente sesión.
Los recursos generados durante la ejecución se almacenan en `/tmp` dentro
del contenedor y desaparecen al finalizar.
