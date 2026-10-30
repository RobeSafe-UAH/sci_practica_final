---
hide:
  - toc
---
# Ejecución

## Construir y lanzar

1. Iniciar Docker y abrir una terminal en la carpeta `sci_practica_final`, donde se encuentra el `Makefile`. En Windows, utilizar **Ubuntu de WSL2**; en Linux y macOS, la terminal del sistema.
2. Construir la imagen la primera vez o tras actualizar el código:

    ```bash
    make build
    ```

3. Lanzar la simulación desde esa misma terminal:

    ```bash
    make run
    ```

4. Con el contenedor en ejecución, abrir [el simulador MVSim en el navegador](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). Pulsar **Connect** si aparece la bienvenida. La ventana de MVSim se muestra dentro del escritorio del navegador.
5. Mantener la terminal abierta durante la práctica. Al terminar, pulsar **Ctrl+C** en la terminal de `make run`: se detiene y elimina el contenedor. Para reiniciar la práctica desde la salida, ejecutar de nuevo `make run`.

## Abrir una terminal del contenedor

1. Con la simulación en ejecución, abrir otra terminal en la carpeta `sci_practica_final`. En Windows, abrir otra terminal de **Ubuntu de WSL2**.
2. Entrar en el contenedor:

    ```bash
    make attach
    ```

3. Comprobar el dominio desde esta terminal del contenedor:

    ```bash
    echo "$ROS_DOMAIN_ID"
    ```

    Debe coincidir con el valor guardado en **`docker/.env`** y con el que
    muestra la terminal de `make run`. Ejecutar aquí los comandos de la
    [guía ROS2](topics.md). El controlador de [MATLAB](matlab.md) se ejecuta
    en el equipo con ese mismo dominio.
4. Salir de esta terminal sin detener el simulador:

    ```bash
    exit
    ```

## Cambiar el dominio ROS2

1. Editar **`docker/.env`**, dentro de `sci_practica_final`, y guardar
   `ROS_DOMAIN_ID` con el valor elegido. La ruta completa es
   **`sci_practica_final/docker/.env`**.
2. Si la simulación está abierta, pulsar **Ctrl+C** en su terminal de
   `make run` para detener y retirar el contenedor anterior.
3. Desde el equipo, volver a lanzarla:

    ```bash
    make run
    ```

    El arranque muestra `ROS_DOMAIN_ID=...` en la terminal. Este valor procede
    de `docker/.env`; un dominio exportado previamente en el terminal del equipo
    no lo sustituye.

4. Abrir de nuevo `make attach` y comprobar el valor con
   `echo "$ROS_DOMAIN_ID"`. Una terminal de un contenedor anterior debe cerrarse
   y abrirse otra vez tras el cambio.
5. Configurar ese mismo dominio en [MATLAB y Simulink](matlab.md#configurar-el-dominio-ros2).

## Gestionar los contenedores

1. Ejecutar estos comandos desde una terminal del equipo, fuera de `make attach`. Ver los contenedores en ejecución:

    ```bash
    docker ps
    ```

2. Ver también los contenedores detenidos:

    ```bash
    docker ps -a
    ```

3. Detener la simulación:

    ```bash
    docker stop sci-practica-final
    ```

4. Eliminar un contenedor detenido:

    ```bash
    docker rm NOMBRE_O_ID
    ```
