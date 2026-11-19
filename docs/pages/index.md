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

    1. En **Windows 11 22H2 o posterior**, abrir **Windows PowerShell como administrador** y ejecutar:

        ```powershell
        wsl --install -d Ubuntu
        wsl --update
        ```

    2. **Reiniciar** el equipo. Abrir Ubuntu desde PowerShell y configurar el usuario y la contraseña que se soliciten:

        ```powershell
        wsl -d Ubuntu
        ```

        Salir de Ubuntu:

        ```bash
        exit
        ```

    3. Activar la [red reflejada de WSL2](https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking) desde PowerShell:

        ```powershell
        @'
        [wsl2]
        networkingMode=mirrored
        '@ | Set-Content "$env:USERPROFILE\.wslconfig" -Encoding ascii
        ```

        !!! warning ""
            Este comando sobrescribe `.wslconfig` si ya existe.

    4. Aplicar los cambios:

        ```powershell
        wsl --shutdown
        ```

    5. Instalar **Git y Make** en Ubuntu desde PowerShell:

        ```powershell
        wsl -d Ubuntu -- sudo apt update
        wsl -d Ubuntu -- sudo apt install -y git make
        ```

    6. Entrar en Ubuntu e instalar **Docker Engine y Docker Compose**:

        ```powershell
        wsl -d Ubuntu
        ```

        ```powershell
        # Add Docker's official GPG key:
        sudo apt update
        sudo apt install ca-certificates curl
        sudo install -m 0755 -d /etc/apt/keyrings
        sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
        sudo chmod a+r /etc/apt/keyrings/docker.asc
        
        # Add the repository to Apt sources:
        sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
        Types: deb
        URIs: https://download.docker.com/linux/ubuntu
        Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
        Components: stable
        Architectures: $(dpkg --print-architecture)
        Signed-By: /etc/apt/keyrings/docker.asc
        EOF
        
        sudo apt update
        ```

        ```powershell
        sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
        ```


        Al terminar, salir de Ubuntu:

        ```bash
        exit
        ```

    7. Habilitar Docker y el acceso para el usuario desde PowerShell:

        ```powershell
        wsl -d Ubuntu -- sudo systemctl enable --now docker
        wsl -d Ubuntu -- bash -lc 'sudo usermod -aG docker "$USER"'
        wsl --shutdown
        ```

    8. Comprobar Docker y abrir Ubuntu para trabajar:

        ```powershell
        wsl -d Ubuntu -- docker compose version
        wsl -d Ubuntu -- docker run --rm hello-world
        wsl -d Ubuntu
        ```

    9. Permitir el tráfico de ROS en el [firewall de Windows y WSL](https://learn.microsoft.com/en-us/windows/wsl/networking#wsl-and-firewall) si es necesario para la red utilizada.

=== "macOS Intel"

    1. Instalar Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Instalar y abrir [OrbStack](https://docs.orbstack.dev/quick-start). Incluye Docker y Compose.
    3. En **Settings → Docker**, mantener activado **Expose ports to LAN**. Mantener la red predeterminada **`bridge`** de Docker.
    4. Este proyecto selecciona el [modo `host` de OrbStack](https://docs.orbstack.dev/docker/host-networking), disponible sin configuración adicional; no cambiarlo a `bridge` en `compose.yaml`.
    5. Como alternativa, instalar [Docker Desktop para Intel](https://docs.docker.com/desktop/setup/install/mac-install/), versión **4.34 o posterior**, y activar **Settings → Resources → Network → Enable host networking → Apply and restart**.

=== "macOS Apple Silicon"

    1. Instalar Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Instalar y abrir [OrbStack](https://docs.orbstack.dev/quick-start). Incluye Docker y Compose y ejecuta la imagen ARM64 de forma nativa.
    3. En **Settings → Docker**, mantener activado **Expose ports to LAN**. Mantener la red predeterminada **`bridge`** de Docker.
    4. Este proyecto selecciona el [modo `host` de OrbStack](https://docs.orbstack.dev/docker/host-networking), disponible sin configuración adicional; no cambiarlo a `bridge` en `compose.yaml`.
    5. Como alternativa, instalar [Docker Desktop para Apple Silicon](https://docs.docker.com/desktop/setup/install/mac-install/), versión **4.34 o posterior**, y activar **Settings → Resources → Network → Enable host networking → Apply and restart**.

## Obtener y construir el proyecto

1. Iniciar Docker y descargar el proyecto desde la terminal del sistema. En Windows, utilizar **Ubuntu de WSL2** y una carpeta de Windows accesible desde Ubuntu (`/mnt/c/...`) para abrirla también con MATLAB:

    ```bash
    git clone https://github.com/RobeSafe-UAH/sci_practica_final.git
    cd sci_practica_final
    ```

2. Elegir el dominio según el grupo y el puesto de laboratorio:

    | Grupo | Dominios |
    |---|---|
    | A1 | 1–20 |
    | A2 | 21–40 |
    | A3 | 41–60 |
    | A4 | 61–80 |
    | B1 | 81–100 |

    Sumar el número de puesto al inicio del bloque menos uno. Por ejemplo, **A2, puesto 02 → 22**. Abrir **`sci_practica_final/docker/.env`** y guardar ese número:

    ```dotenv
    ROS_DOMAIN_ID=22
    ```

    `make run` lee este archivo. Usar el mismo valor en MATLAB y en la
    configuración ROS 2 del modelo Simulink, como indica [MATLAB](matlab.md).
    Si se modifica con la simulación en marcha, detenerla con **Ctrl+C** y
    volver a ejecutar `make run` para crear el contenedor con el nuevo dominio.
3. Construir la imagen:

    ```bash
    make build
    ```

4. Continuar en [Ejecución](ejecucion.md).
