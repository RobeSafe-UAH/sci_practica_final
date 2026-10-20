---
hide:
  - toc
---
# Instalación

## Preparar el sistema

=== "Linux"

    1. Instalar Git y Make:

        ```bash
        sudo apt update
        sudo apt install -y git make
        ```

    2. Instalar Docker Engine y Compose siguiendo la [guía oficial de Ubuntu](https://docs.docker.com/engine/install/ubuntu/) o la [guía de la distribución utilizada](https://docs.docker.com/engine/install/).

    3. Habilitar Docker para el usuario:

        ```bash
        sudo systemctl enable --now docker
        sudo usermod -aG docker "$USER"
        ```

    4. Cerrar la sesión y volver a entrar.

=== "Windows"

    1. En **Windows 11 22H2 o posterior**, abrir PowerShell como administrador:

        ```powershell
        wsl --install -d Ubuntu
        wsl --update
        ```

    2. Reiniciar y abrir Ubuntu para completar su configuración.
    3. Crear o editar `%UserProfile%\.wslconfig` en Windows para activar la [red reflejada de WSL2](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking):

        ```ini
        [wsl2]
        networkingMode=mirrored
        ```

    4. Aplicar los cambios desde PowerShell y volver a abrir Ubuntu:

        ```powershell
        wsl --shutdown
        ```

    5. En Ubuntu de WSL2, instalar Git y Make:

        ```bash
        sudo apt update
        sudo apt install -y git make
        ```

    6. Instalar **Docker Engine y el complemento Compose dentro de Ubuntu**, siguiendo la [guía oficial](https://docs.docker.com/engine/install/ubuntu/).
    7. Habilitar Docker para el usuario:

        ```bash
        sudo systemctl enable --now docker
        sudo usermod -aG docker "$USER"
        ```

    8. Salir de Ubuntu y volver a entrar. Ejecutar los comandos del proyecto desde esa terminal.

    9. Permitir la comunicación ROS en el [firewall de Windows y WSL](https://learn.microsoft.com/en-us/windows/wsl/networking#wsl-and-firewall) si el sistema solicita autorización para la red utilizada.

=== "macOS Intel"

    1. Instalar Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Instalar y abrir [OrbStack](https://docs.orbstack.dev/quick-start). Incluye Docker y Compose.
    3. En **Settings → Docker**, mantener activado **Expose ports to LAN**. Comprobar en **Networks** que existe la red **`bridge`**. Si se utiliza una máquina Linux de OrbStack, activar también **Settings → Machines → Expose ports to LAN**.
    4. El contenedor utiliza el [modo `host` de OrbStack](https://docs.orbstack.dev/docker/host-networking), disponible sin configuración adicional.
    5. Como alternativa, instalar [Docker Desktop para Intel](https://docs.docker.com/desktop/setup/install/mac-install/), versión **4.34 o posterior**, y activar **Settings → Resources → Network → Enable host networking → Apply and restart**.

=== "macOS Apple Silicon"

    1. Instalar Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Instalar y abrir [OrbStack](https://docs.orbstack.dev/quick-start). Incluye Docker y Compose y ejecuta la imagen ARM64 de forma nativa.
    3. En **Settings → Docker**, mantener activado **Expose ports to LAN**. Comprobar en **Networks** que existe la red **`bridge`**. Si se utiliza una máquina Linux de OrbStack, activar también **Settings → Machines → Expose ports to LAN**.
    4. El contenedor utiliza el [modo `host` de OrbStack](https://docs.orbstack.dev/docker/host-networking), disponible sin configuración adicional.
    5. Como alternativa, instalar [Docker Desktop para Apple Silicon](https://docs.docker.com/desktop/setup/install/mac-install/), versión **4.34 o posterior**, y activar **Settings → Resources → Network → Enable host networking → Apply and restart**.

1. Si Docker se ejecuta en VirtualBox, VMware u otra máquina virtual, configurar su adaptador en modo **puente (`bridge`)**. El simulador y el equipo de control deben estar en la misma red.
2. Comprobar que Docker y Compose están disponibles:

    ```bash
    docker info
    docker compose version
    ```

3. El entorno gráfico se abre en el navegador; no requiere XQuartz, WSLg ni GPU dedicada.

## Obtener y construir el proyecto

1. Iniciar Docker y descargar el proyecto:

    ```bash
    git clone https://github.com/RobeSafe-UAH/sci_practica_final.git
    cd sci_practica_final
    ```

2. Editar **`ROS_DOMAIN_ID`** en **`docker/.env`** con el dominio elegido. Usar el mismo valor en el control.
3. Construir la imagen:

    ```bash
    make build
    ```

4. Continuar en [Ejecución](ejecucion.md).
