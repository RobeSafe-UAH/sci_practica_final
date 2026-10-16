# Arquitectura del repositorio

## Alcance

El proyecto proporciona un entorno de simulación para un vehículo Ackermann:
física de movimiento y colisiones, dos obstáculos móviles, sensores y una
interfaz ROS 2. El control del vehículo se ejecuta fuera del contenedor.

El entorno utiliza ROS 2 Jazzy y MVSim. La presentación 3D incluye el modelo
Traxxas/F1TENTH de `rbsf-racing`, ruedas articuladas, asfalto, césped,
bordillos, señalización vial e iluminación con sombras. La cámara sigue al
vehículo. La representación gráfica y el mapa de colisiones son capas distintas.

## Organización

```text
sci_practica_final/
├── config/scenario.yaml       Parámetros del circuito, vehículo, tráfico y sensores
├── docker/                    Arranque, escritorio virtual y configuración DDS
├── docs/                      Documentación
├── src/
│   ├── sci_mvsim/             Generación del mundo e interfaz ROS 2
│   │   ├── definitions/       Vehículo, Hokuyo y cajas
│   │   ├── launch/            Lanzamiento de los procesos ROS
│   │   ├── models/f1tenth/    Mallas 3D del vehículo y su licencia
│   │   ├── resource/          Registro del paquete en ament
│   │   ├── sci_mvsim/         Módulos Python
│   │   ├── tracks/            Mapa y línea central del circuito
│   │   ├── package.xml        Dependencias ROS
│   │   └── setup.py           Instalación de código y recursos
│   └── sci_start_gate/        Extensión nativa de MVSim para el inicio de sesión
├── compose.yaml               Servicio, red y configuración del contenedor
├── Dockerfile                 Construcción de la imagen
├── Makefile                   Construcción y ejecución
└── mkdocs.yml                 Navegación de la documentación
```

`README.md` enlaza las páginas de uso. `requirements-docs.txt` contiene las
dependencias del sitio y `.github/workflows/docs.yml` lo publica en GitHub
Pages. `LICENSE` y `NOTICE` recogen las licencias y la procedencia de los recursos.

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
6. El primer `/r0/cmd_vel` válido habilita el vehículo e inicia las dos cajas.
   Una orden cero también inicia la sesión.

Las cajas utilizan animaciones periódicas de MVSim. Su tiempo permanece en
cero antes de la primera orden y después avanza con el reloj de simulación.
No dependen de que el estudiante siga enviando consignas al coche.

Tras el inicio, el vehículo conserva su dinámica y sus colisiones físicas.
Si dejan de recibirse órdenes durante un segundo, el puente envía una
consigna cero. La detección de una colisión no modifica el control.

## Configuración del escenario

`config/scenario.yaml` se monta en el contenedor en modo de solo lectura.
Las distancias se expresan en metros y los tiempos en segundos. Los ángulos
usan radianes salvo los parámetros cuyo nombre termina en `_deg`.

| Sección | Parámetros principales |
|---|---|
| `track` | Dimensiones de la línea central, radio de las curvas, anchura total, sentido, espesor del límite y resolución del mapa. |
| `robot` | Nombre ROS, posición inicial por distancia recorrida, batalla y límites de velocidad y dirección. |
| `traffic` | Número de cajas, velocidad y distancias iniciales por delante del vehículo. |
| `lidar` | Posición, campo de visión, número de rayos, periodo y alcance. |
| `sonar` | Ocho sectores derivados del barrido del LiDAR. |

El circuito tiene una línea central de **40 × 24 m**, cuatro curvas de
**radio 5 m** y una anchura total de **4 m**. Sus dos carriles tienen sentidos
opuestos. El vehículo y las cajas comienzan en el carril derecho, con
circulación antihoraria. El origen del vehículo coincide con el eje trasero:
`x` apunta hacia delante, `y` hacia la izquierda y `z` hacia arriba.

La batalla de **0,3302 m** y la dirección máxima de **24°** corresponden al
modelo F1TENTH. Reducir los límites de control no modifica esa geometría.

## Archivos del circuito

`src/sci_mvsim/tracks/circuito_sci/` contiene:

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

Las dimensiones y la resolución de `scenario.yaml` deben coincidir con el
mapa instalado. Si se modifica la geometría, el generador de `world.py` debe
regenerar el PNG, el YAML y el CSV antes de reconstruir la imagen.

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
Los puertos y el dominio ROS se pueden configurar mediante `.env`, tomando
`.env.example` como referencia.

La red utiliza el modo host para DDS. El renderizado se realiza por software.
El proceso de simulación se ejecuta con un usuario sin privilegios y la
configuración del escenario se mantiene separada de la imagen.

`make run` permanece en primer plano. Ctrl+C termina los procesos y elimina
el contenedor; la imagen construida se conserva para la siguiente sesión.
Los recursos generados durante la ejecución se almacenan en `/tmp` dentro
del contenedor y desaparecen al finalizar.
