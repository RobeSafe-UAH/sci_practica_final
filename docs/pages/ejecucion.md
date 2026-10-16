---
hide:
  - toc
---
# Ejecución

## Construir y lanzar

1. Desde la carpeta del proyecto, con Docker iniciado:

    ```bash
    make build
    make run
    ```

2. Abrir el [simulador en el navegador](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). Pulsar **Connect** si aparece la bienvenida.
3. Mantener la terminal abierta. **Ctrl+C** detiene y elimina el contenedor.

## Abrir una terminal del contenedor

1. Desde otra terminal en la carpeta del proyecto:

    ```bash
    make attach
    ```

## Gestionar los contenedores

1. Ver los contenedores en ejecución:

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
