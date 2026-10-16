# Instalación

El simulador se ejecuta en Docker. En el equipo solo se necesitan **Docker
con Compose, Git, Make y un navegador**. ROS 2 Jazzy, MVSim y la pantalla
virtual se instalan al construir la imagen.

## Preparar el sistema

=== "Linux"

    En una instalación nueva de **Ubuntu 24.04**, instalar Git, Make,
    Docker Engine y el complemento Compose:

    ```bash
    sudo apt update
    sudo apt install -y git make ca-certificates curl
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
    ```

    Si ya había otra instalación de Docker, consultar primero la sección
    de paquetes incompatibles de la
    [guía oficial de Ubuntu](https://docs.docker.com/engine/install/ubuntu/).
    Para otra distribución, utilizar su
    [guía de instalación](https://docs.docker.com/engine/install/).

    Para ejecutar Docker con el usuario habitual:

    ```bash
    sudo systemctl enable --now docker
    sudo usermod -aG docker "$USER"
    ```

    Cerrar la sesión y volver a entrar para aplicar el grupo. Docker Engine
    en Linux proporciona directamente la red host que utiliza el proyecto.

=== "Windows"

    1. Abrir PowerShell como administrador y ejecutar:

        ```powershell
        wsl --install -d Ubuntu
        wsl --update
        ```

    2. Reiniciar si lo solicita Windows y abrir Ubuntu para completar su
       configuración.
    3. Instalar y abrir
       [Docker Desktop para Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
       con el motor WSL2 y contenedores Linux.
    4. En **Settings → Resources → WSL Integration**, activar la distribución
       Ubuntu.
    5. En la terminal de Ubuntu, instalar Git y Make:

        ```bash
        sudo apt update
        sudo apt install -y git make
        ```

    6. Activar la red host como se indica debajo de estas pestañas.

    Ejecutar **todos los comandos del proyecto desde Ubuntu de WSL2**,
    incluida la clonación. Docker Desktop proporciona el motor: no instalar
    otro Docker Engine dentro de Ubuntu.

=== "macOS Intel"

    1. Instalar las herramientas de línea de comandos de Apple, que incluyen
       Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Descargar **Docker Desktop para Mac con chip Intel** desde la
       [página oficial](https://docs.docker.com/desktop/setup/install/mac-install/),
       instalarlo y abrir la aplicación.
    3. Activar la red host como se indica debajo de estas pestañas.

    Ejecutar los comandos del proyecto desde Terminal. La construcción
    selecciona la imagen AMD64.

=== "macOS Apple Silicon"

    1. Instalar las herramientas de línea de comandos de Apple, que incluyen
       Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Descargar **Docker Desktop para Mac con chip Apple Silicon** desde la
       [página oficial](https://docs.docker.com/desktop/setup/install/mac-install/),
       instalarlo y abrir la aplicación.
    3. Activar la red host como se indica debajo de estas pestañas.

    Ejecutar los comandos del proyecto desde Terminal. La construcción
    selecciona la imagen ARM64; no forzar `linux/amd64`.

### Red host en Docker Desktop

El proyecto utiliza `network_mode: host` para la comunicación ROS 2.
En **Docker Desktop 4.34 o posterior**:

1. Abrir **Settings → Resources → Network**.
2. Activar **Enable host networking**.
3. Pulsar **Apply and restart**.

La función debe estar habilitada antes de lanzar el simulador. Consultar la
[documentación de red host](https://docs.docker.com/engine/network/drivers/host/)
si la opción no aparece. Con OrbStack en macOS, la
[red host está disponible directamente](https://docs.orbstack.dev/docker/host-networking).

## Obtener y construir el proyecto

Con el motor de Docker abierto, comprobar las herramientas:

```bash
docker info
docker compose version
git --version
make --version
```

Descargar el repositorio y construir la imagen:

```bash
git clone https://github.com/RobeSafe-UAH/sci_practica_final.git
cd sci_practica_final
make build
```

Si ya se dispone de la carpeta del proyecto, abrir una terminal en ella y
ejecutar `make build`. La primera construcción descarga las dependencias y
puede tardar varios minutos. Repetirla después de modificar el código, las
definiciones del vehículo o el Dockerfile.

La construcción **no arranca** la simulación. Continuar en
[Ejecución](ejecucion.md).
