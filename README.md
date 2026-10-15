# Práctica final de SCI: simulador MVSim

Simulación independiente de una carretera elíptica con dos carriles, un coche
Ackermann y dos coches lentos que circulan por el carril derecho. Incluye
LiDAR, ocho sonares y una interfaz ROS 2 Jazzy.

Se ejecuta en un contenedor y se ve en el navegador. Para lanzarla solo se
necesitan Docker, Compose y un navegador; ROS y Python se instalan dentro de
la imagen.

## 1. Instalar Docker

La imagen base y MVSim tienen binarios nativos para AMD64 y ARM64. La
ejecución, el escritorio, los sensores y el tráfico se han comprobado en
Linux AMD64, con 24 pruebas unitarias aprobadas. Windows, macOS Intel,
macOS Apple Silicon y Linux ARM64 quedan pendientes de prueba.

### Linux: Ubuntu

En una instalación nueva de Ubuntu 24.04, instalar Docker Engine y el
complemento Compose desde el repositorio oficial:

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources >/dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

Cerrar la sesión y volver a entrar para aplicar el grupo de Docker. Si ya
había otra instalación de Docker, consultar primero la sección de paquetes
incompatibles de la [guía oficial para Ubuntu](https://docs.docker.com/engine/install/ubuntu/).
En otras distribuciones, utilizar su instalador oficial equivalente.

### Windows: Docker Desktop con WSL2

1. En PowerShell como administrador, instalar Ubuntu en WSL2:

```powershell
wsl --install -d Ubuntu
wsl --update
```

2. Reiniciar si lo solicita Windows y abrir Ubuntu para completar la creación
   del usuario.
3. Instalar y abrir [Docker Desktop para Windows](https://docs.docker.com/desktop/setup/install/windows-install/).
   Seleccionar el motor WSL2 y contenedores Linux.
4. En Docker Desktop, abrir **Settings → Resources → WSL Integration** y
   habilitar la distribución Ubuntu.
5. Activar también la red host como se explica más abajo.

Ejecutar los comandos del proyecto desde la terminal Ubuntu de WSL.
Si la carpeta está en Windows, se puede acceder desde WSL, por ejemplo en
`/mnt/c/Users/TU_USUARIO/Downloads/sci_practica_final`.
Docker Desktop proporciona el motor; esta opción no necesita una segunda
instalación de Docker Engine dentro de Ubuntu.

Referencias: [instalación de WSL](https://learn.microsoft.com/en-us/windows/wsl/install)
e [integración de Docker con WSL2](https://docs.docker.com/desktop/features/wsl/).

### macOS: Intel y Apple Silicon

Se puede utilizar uno de estos motores:

- **OrbStack:** descargarlo desde su [guía oficial](https://docs.orbstack.dev/quick-start),
  abrir la aplicación y completar su configuración inicial. Incluye Docker
  y Compose; la [red host funciona directamente](https://docs.orbstack.dev/docker/host-networking).
- **Docker Desktop:** descargar el instalador correspondiente a **Intel**
  o **Apple Silicon** desde la [página oficial para Mac](https://docs.docker.com/desktop/setup/install/mac-install/),
  instalarlo en Aplicaciones y abrirlo. Activar la red host.

Docker selecciona la arquitectura de la imagen según el motor utilizado.
El simulador utiliza renderizado por software y una pantalla virtual dentro
del contenedor; no necesita GPU dedicada, CUDA, XQuartz ni WSLg.

### Activar la red host en Docker Desktop

El archivo Compose utiliza `network_mode: host`. En Docker Desktop se
necesita la versión **4.34 o posterior** y activar esta función:

1. Abrir Docker Desktop e iniciar sesión si lo solicita.
2. Ir a **Settings → Resources → Network**.
3. Marcar **Enable host networking**.
4. Pulsar **Apply and restart**.

Docker Engine en Linux y OrbStack proporcionan la red host directamente.
La configuración y las limitaciones de Desktop están en la
[documentación oficial de red host](https://docs.docker.com/engine/network/drivers/host/).

### Comprobar la instalación

Con el motor de Docker abierto, ejecutar en la terminal que se utilizará
para el proyecto:

```bash
docker info
docker compose version
```

Ambos comandos deben terminar correctamente. Utilizar una versión actual
del complemento Compose, invocado como `docker compose`.

## 2. Construir y lanzar la simulación

Copiar o descomprimir la carpeta completa del proyecto y abrir una terminal
en su carpeta padre. Después:

```bash
cd sci_practica_final
docker compose build
docker compose up -d --wait
```

La carpeta debe contener `compose.yaml`, `Dockerfile`, `config/` y
`src/`. No hace falta tener `rbsf-racing` junto a ella.
La primera construcción descarga las dependencias y puede tardar varios minutos.

Abrir [el escritorio del simulador](http://localhost:6080/vnc.html?autoconnect=true&resize=scale).
Si aparece la pantalla de bienvenida de noVNC, pulsar **Connect**.

Se debe ver la carretera completa, el coche azul inicialmente parado y los
dos coches naranjas circulando despacio. El coche azul permanece parado
hasta recibir órdenes por ROS 2; la simulación no incluye un controlador
de navegación o adelantamiento para él.

Para construir y arrancar en una sola orden también se puede utilizar:

```bash
docker compose up -d --build --wait
```

El estado saludable confirma los procesos y el escritorio web. La
comprobación de sensores se explica en el apartado 5.

## 3. Consultar, reiniciar y detener

Todos estos comandos se ejecutan desde `sci_practica_final`:

| Acción | Comando |
|---|---|
| Consultar el estado | `docker compose ps` |
| Ver los registros | `docker compose logs -f simulator` |
| Reiniciar y recuperar las posiciones iniciales | `docker compose restart simulator` |
| Detener y retirar el contenedor | `docker compose down` |
| Volver a arrancar | `docker compose up -d --wait` |

En los registros, `Ctrl+C` deja de seguir la salida y mantiene la simulación
en ejecución. `docker compose down` conserva los archivos de configuración y
la imagen construida.

## 4. Escenario y configuración

La configuración inicial es:

| Elemento | Valor |
|---|---|
| Carretera | Elipse de semiejes centrales 7 y 4 m; aproximadamente 16 × 10 m exteriores. |
| Carriles | Anchura total de 2 m: un carril de 1 m por sentido. |
| Coche azul | Ackermann F1TENTH, batalla de 0,3302 m y dirección máxima de 24°. |
| Posición inicial | Carril derecho, sentido antihorario; coche azul parado. |
| Tráfico | Dos coches a 0,35 m/s, inicialmente a 5 y 17 m por delante, en el mismo carril y sentido. |
| LiDAR | 270°, 1081 rayos, 20 Hz y alcance máximo de 12 m. |
| Sonares | Ocho sensores independientes, índices 0–7, cono de 15°, 20 Hz y alcance de 0,04 a 5 m. |

Editar [config/scenario.yaml](config/scenario.yaml) permite cambiar:

- `track.semi_major`, `track.semi_minor` y `track.width`: dimensiones de la carretera.
- `track.direction`: `ccw` para antihorario o `cw` para horario.
- `traffic.count`, `traffic.speed` y `traffic.start_distances`: cantidad,
  velocidad y distancia inicial de cada coche de tráfico.
- `robot`: nombre, batalla, posición inicial y límites del coche azul.
- `lidar` y `sonar`: posiciones, orientaciones, frecuencia y alcance de los sensores.

Debe haber una distancia inicial por cada coche de tráfico. Las distancias
se miden sobre el centro del carril derecho, desde el origen del recorrido.
El sentido horario sitúa ese carril en el lado interior de la elipse.
El generador comprueba que la geometría deje espacio para los coches y su giro.

La configuración se monta en modo lectura dentro del contenedor. Después
de editarla, aplicar los cambios con:

```bash
docker compose restart simulator
```

Los coches de tráfico siguen trayectorias prefijadas: son detectables,
participan en las colisiones y no frenan para evitar al coche azul.
Los sonares aproximan haces geométricos independientes con ruido de alcance;
no modelan propagación acústica. La odometría proporciona la pose exacta
del simulador.

### Puerto web y dominio ROS

Para personalizar las variables del contenedor, copiar `.env.example`
a `.env`:

```bash
cp .env.example .env
```

Las variables principales son:

| Variable | Valor inicial | Uso |
|---|---|---|
| `SCI_VNC_PORT` | `6080` | Puerto del escritorio web en localhost. |
| `ROS_DOMAIN_ID` | `42` | Dominio de la simulación ROS 2. |
| `SCI_SCREEN_WIDTH` | `1280` | Anchura de la pantalla virtual. |
| `SCI_SCREEN_HEIGHT` | `800` | Altura de la pantalla virtual. |
| `SCI_HEADLESS` | `false` | `true` oculta la ventana de MVSim; mantiene el escritorio de diagnóstico. |

Después de editar `.env`, recrear el contenedor:

```bash
docker compose up -d --force-recreate --wait
```

Si se cambia el puerto, utilizarlo también en la dirección del navegador.
Las simulaciones independientes que compartan red deben utilizar dominios
ROS diferentes. Para ejecutar varias en el mismo equipo también necesitan
puertos web y nombres de proyecto Compose distintos.

## 5. Comprobar los sensores

Esta prueba se ejecuta dentro del contenedor y no necesita ROS ni Python
instalados en el equipo. Con la simulación arrancada:

```bash
docker compose cp tests/smoke_ros.py simulator:/home/sci/smoke_ros.py
docker compose exec simulator bash -ic "python3 /home/sci/smoke_ros.py"
```

La prueba predeterminada observa LiDAR, los ocho sonares, odometría y
transformaciones TF. Comprueba los frames y que avancen los timestamps;
no envía órdenes al coche azul. El tráfico continúa su recorrido normal.

Para comprobar además el desplazamiento de los obstáculos, manteniendo
la prueba sin órdenes al coche azul:

```bash
docker compose exec simulator bash -ic "python3 /home/sci/smoke_ros.py --traffic"
```

Las pruebas unitarias se pueden ejecutar opcionalmente durante una
construcción separada, sin instalar herramientas en el equipo:

```bash
docker build --target test .
```

La imagen utilizada por Compose contiene la aplicación instalada y sus
dependencias de ejecución. Los directorios de compilación y las pruebas
permanecen en etapas separadas de la construcción.

## 6. Interfaz ROS 2

Los nombres siguientes corresponden a la configuración inicial `robot.name: r0`:

| Topic | Tipo ROS 2 | Contenido |
|---|---|---|
| `/r0/cmd_vel` | `geometry_msgs/msg/Twist` | Entrada: `linear.x` en m/s y `angular.z` en rad/s. |
| `/r0/scan` | `sensor_msgs/msg/LaserScan` | LiDAR; frame `r0/laser`. |
| `/r0/sonar0` … `/r0/sonar7` | `sensor_msgs/msg/Range` | Lecturas individuales; frames `r0/sonar0` … `r0/sonar7`. |
| `/r0/odom` | `nav_msgs/msg/Odometry` | Pose y velocidad; frame `r0/odom`. |
| `/r0/ground_truth/odom` | `nav_msgs/msg/Odometry` | Pose exacta en `map`. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Transformaciones del coche y sensores. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Reloj de simulación. |

Los topics bajo `/_sim/` son internos. Los sensores públicos se publican
con QoS reliable y profundidad 10. La entrada de velocidad se limita a
`robot.max_speed` —1 m/s inicialmente— y a la dirección máxima configurada.
El modelo Ackermann no permite girar sobre el sitio. Si pasa un segundo
sin una orden nueva, el puente publica una parada.

En los sonares, el eje `x` apunta hacia delante y `y` hacia la izquierda:

| Sonar | Ángulo respecto al coche | Dirección |
|---|---:|---|
| 0 | 0° | Delante |
| 1 | 45° | Delante e izquierda |
| 2 | 90° | Izquierda |
| 3 | 135° | Detrás e izquierda |
| 4 | 180° | Detrás |
| 5 | −135° | Detrás y derecha |
| 6 | −90° | Derecha |
| 7 | −45° | Delante y derecha |

Cada `Range` conserva el instante del barrido y representa el mínimo
retorno válido de su cono. `+Inf` indica ausencia de retorno dentro del
alcance; `-Inf`, un objeto demasiado cercano para cuantificarlo.
La ausencia de mensajes significa que aún no hay una medida disponible.

## Si no arranca o no aparece la ventana

1. Comprobar que el motor responde con `docker info`.
2. Revisar `docker compose ps` y `docker compose logs --tail=100 simulator`.
3. En Docker Desktop, comprobar que la red host está habilitada.
4. Si el puerto está ocupado, cambiar `SCI_VNC_PORT` en `.env` y recrear el contenedor.
5. Si aparece el escritorio pero no MVSim, comprobar que `SCI_HEADLESS=false`
   y revisar los errores de configuración en los registros.

## Procedencia

La geometría Ackermann, el sensor LiDAR y el patrón de integración de MVSim
proceden de `rbsf-racing`; las modificaciones se describen en
[NOTICE](NOTICE). El entorno se distribuye con [licencia Apache 2.0](LICENSE).
