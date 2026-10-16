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

    1. Abrir PowerShell como administrador:

        ```powershell
        wsl --install -d Ubuntu
        wsl --update
        ```

    2. Reiniciar y abrir Ubuntu para completar su configuración.
    3. Instalar [Docker Desktop](https://docs.docker.com/desktop/setup/install/windows-install/) con el motor WSL2 y contenedores Linux.
    4. En Docker Desktop, activar Ubuntu en **Settings → Resources → WSL Integration**.
    5. Activar **Settings → Resources → Network → Enable host networking** y aplicar los cambios.
    6. En la terminal de Ubuntu, instalar Git y Make:

        ```bash
        sudo apt update
        sudo apt install -y git make
        ```

    7. Ejecutar los comandos del proyecto desde Ubuntu de WSL2.

=== "macOS Intel"

    1. Instalar Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Instalar y abrir [Docker Desktop para Mac con chip Intel](https://docs.docker.com/desktop/setup/install/mac-install/).
    3. Activar **Settings → Resources → Network → Enable host networking** y aplicar los cambios.

=== "macOS Apple Silicon"

    1. Instalar Git y Make:

        ```bash
        xcode-select --install
        ```

    2. Instalar y abrir [Docker Desktop para Mac con chip Apple Silicon](https://docs.docker.com/desktop/setup/install/mac-install/).
    3. Activar **Settings → Resources → Network → Enable host networking** y aplicar los cambios.

## Obtener y construir el proyecto

1. Iniciar Docker y descargar el proyecto:

    ```bash
    git clone https://github.com/RobeSafe-UAH/sci_practica_final.git
    cd sci_practica_final
    ```

2. Construir la imagen:

    ```bash
    make build
    ```

3. Continuar en [Ejecución](ejecucion.md).
