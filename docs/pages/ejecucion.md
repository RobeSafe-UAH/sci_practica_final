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

4. Esperar a que aparezca la ventana de MVSim en el [simulador en el navegador](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). Pulsar **Connect** si aparece la bienvenida.
5. Mantener la terminal abierta durante la práctica. Al terminar, pulsar **Ctrl+C** en la terminal de `make run`: se detiene y elimina el contenedor. Para reiniciar la práctica desde la salida, ejecutar de nuevo `make run`.

## Abrir una terminal del contenedor

1. Con la simulación en ejecución, abrir otra terminal en la carpeta `sci_practica_final`. En Windows, abrir otra terminal de **Ubuntu de WSL2**.
2. Entrar en el contenedor:

    ```bash
    make attach
    ```

3. Ejecutar allí los comandos `ros2` de la [guía ROS2](topics.md).
4. Salir de esta terminal sin detener el simulador:

    ```bash
    exit
    ```

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
