---
hide:
  - toc
---
# MATLAB

MATLAB y Simulink se ejecutan en el equipo del alumno y se comunican por ROS2 con el simulador de Docker. Completar [Instalación](index.md) y [Ejecución](ejecucion.md) antes de empezar.

## 1. Preparar MATLAB y ROS2

1. Instalar **MATLAB**, **Simulink**, **ROS Toolbox** y **Fuzzy Logic Toolbox**. Para el controlador neuronal se necesita también **Deep Learning Toolbox**. Comprobar los productos instalados:

    ```matlab
    ver
    ```

2. Seleccionar **`matlab/` dentro del repositorio** como carpeta actual de MATLAB:

    | Archivo | Uso |
    |---|---|
    | `ackerman_ROS2_controller.slx` | Modelo principal, para R2025a o posterior. |
    | `ackerman_ROS2_controller2023.slx` | Alternativa para R2023b y R2024, con la misma conversión de dirección, límites y detección de meta que el modelo principal. |
    | `sensores.m` | Conducción manual o autónoma y visualización de sensores y velocidades. |
    | `controller.m` | Lanzamiento del controlador de Simulink y evaluación. |
    | `evaluate_controller.m` | Evaluación del registro de una prueba. |

    Para utilizar la alternativa de R2023b, cambiar la variable `modelo` de `controller.m` a `'ackerman_ROS2_controller2023.slx'`.

3. Obtener el dominio de [grupo y puesto de laboratorio](index.md#obtener-y-construir-el-proyecto) y editar `ROS_DOMAIN_ID` en **`docker/.env`**. Si se cambia con el simulador en marcha, detener `make run` con **Ctrl+C** y volver a ejecutarlo.
4. Editar **`dominioROS2` en `controller.m`** con el mismo número. Por ejemplo:

    ```matlab
    dominioROS2 = 22; % Ejemplo: A2, puesto 02.
    ```

    `sensores.m` lee el dominio directamente de `docker/.env`.

5. Abrir el modelo de Simulink.

    Desde MATLAB, acceder a la carpeta matlab/ y abrir el archivo ackerman_ROS2_controller.slx.

6. Configurar el dominio ROS 2 en Simulink.

    Dentro del modelo, hacer doble clic sobre uno de los bloques ROS 2 Subscribe. En la ventana de configuración del bloque, seleccionar Configure ROS 2 Domain ID and ROS Middleware (RMW).

    En la ventana que aparece, introducir el identificador asignado a la pareja de laboratorio en el campo Domain ID (ROS 2) y aceptar los cambios.

    Este identificador debe coincidir con el configurado anteriormente en Docker y MATLAB.

7. Con el simulador en marcha, comprobar los topics desde MATLAB. Sustituir `22` por el dominio elegido:

    ```matlab
    setenv('ROS_DOMAIN_ID', '22');
    ros2 topic list
    ```

    Deben aparecer `/r0/odom`, `/r0/scan`, `/r0/sonar0` … `/r0/sonar7` y `/r0/collision`. El dominio debe coincidir en **Docker, `controller.m` y la configuración de red de Simulink**.

## 2. Explorar sensores y velocidades

1. Mantener **`make run` activo** y detener cualquier otro programa que publique en `/r0/cmd_vel`. Desde MATLAB, ejecutar:

    ```matlab
    sensores
    ```

    Esta parte no necesita un FIS ni ejecutar el modelo de Simulink.

2. Elegir el modo en la ventana y pulsar **Activar / Pausar**. Para usar las flechas, seleccionar una zona libre de la figura. Cambiar de modo deja el movimiento en pausa:

    | Modo | Funcionamiento |
    |---|---|
    | **Manual** | Mantener **↑** para avanzar y **←/→** para girar. Soltar **↑** o pulsar **↓** frena; **espacio** pausa. Velocidad de **0–50 km/h**, ajustable con el deslizador; inicialmente **4 km/h**. |
    | **Autónomo** | Sigue el carril derecho a baja velocidad y se detiene después de una vuelta. Lee la geometría de `docker/config/scenario.yaml`. |

3. Observar los cuatro paneles:

    | Panel | Información |
    |---|---|
    | **LiDAR** | Los 90 valores de `/r0/scan`, representados alrededor del vehículo. |
    | **Sonares** | Distancias de `sonar_0` a `sonar_7`, en metros. |
    | **Velocidad** | `V` en km/h, con las curvas **Consigna** y **Real**. |
    | **Dirección** | `W` en grados, positiva hacia la izquierda, con las curvas **Consigna** y **Real**. |

    La dirección real se estima a partir de la odometría y se muestra a partir de 0,05 m/s; en reposo no se puede estimar.

4. Pausar con **Activar / Pausar** o **espacio**. En manual, cambiar a otra ventana también pausa el movimiento; volver a la figura y reactivarlo para continuar. **Esc** o cerrar la ventana finaliza la exploración y envía una orden de parada. Si se interrumpe con **Ctrl+C**, el puente detiene el vehículo como máximo un segundo después de la última publicación.
5. Ante una colisión, reiniciar el simulador con **Ctrl+C** y **`make run`** antes de repetir. La pérdida de datos de sensores también detiene el movimiento.

## 3. Diseñar el controlador

1. Con el escenario sin tráfico, abrir **Fuzzy Logic Designer** y crear un sistema **Mamdani**:

    ```matlab
    fuzzyLogicDesigner
    ```

2. Elegir los sonares necesarios; **no es obligatorio utilizar los ocho**. La plantilla conecta `sonar_0`, `sonar_1`, …, `sonar_7`, desde el lado derecho hasta el izquierdo, en metros. Para utilizar menos entradas, adaptar el Mux del modelo y conservar el mismo orden en el FIS. Los ángulos se consultan en [ROS2](topics.md#sonares).

3. Definir **dos salidas en este orden**:

    | Posición | Salida | Significado | Rango |
    |---|---|---|---|
    | 1 | `V` | Velocidad solicitada. | **0–50 km/h** |
    | 2 | `W` | Ángulo de dirección equivalente del eje delantero; positivo hacia la izquierda. | **−45° a +45°** |

4. Guardar el controlador como **`fuzzy_controller.fis`** en `matlab/`. El bloque **Controlador → Fuzzy Logic Controller** carga ese archivo. Si el sistema está en la variable `fis`, utilizar:

    ```matlab
    writeFIS(fis, 'fuzzy_controller.fis');
    ```

## 4. Ejecutar y evaluar

1. Lanzar el simulador desde la salida con **`make run`**.
2. Con `matlab/` como carpeta actual, ejecutar:

    ```matlab
    controller
    ```

    El script espera hasta **3 segundos** para recibir `/r0/odom` con QoS **Best Effort**. Si recibe odometría, inicia Simulink; si no, muestra un error.

3. El modelo principal termina al completar una vuelta, recibir una colisión o alcanzar **200 s de simulación**. También se puede pulsar **Stop** en Simulink. El contenedor permanece en ejecución; reiniciarlo antes de cada nuevo ensayo.
4. Consultar la figura de evaluación: duración, porcentaje de vuelta, invasión del carril contrario, trayectoria y consignas de velocidad y dirección. Para abrir de nuevo el último registro:

    ```matlab
    evaluate_controller
    ```

5. El bloque **Evaluacion_maniobra → To File** guarda `data.mat`, con una variable numérica `data` en formato **Array**. Cada columna contiene una muestra:

    | Fila | Contenido | Unidad |
    |---|---|---|
    | 1 | Tiempo de simulación. | s |
    | 2, 3 | Posición `x`, `y`. | m |
    | 4, 5, 6 | Orientación de Euler ZYX: guiñada, cabeceo y alabeo. | rad |
    | 7 | Consigna de velocidad. | km/h |
    | 8 | Consigna de dirección. | grados |

6. Cada prueba sobrescribe `data.mat`. Copiar el registro y el controlador a otra carpeta para conservar un resultado. La evaluación utiliza el circuito de **40 × 26 m**, radio **8 m** y anchura **1,5 m** de la práctica.
7. Tras ajustar el controlador sin tráfico, cambiar **`count: 2` dentro de `traffic:`** en `docker/config/scenario.yaml`. Reiniciar el simulador con **Ctrl+C** y **`make run`**, y repetir los ensayos y la evaluación con los dos vehículos.

## 5. Registrar datos y utilizar la red neuronal

1. Crear un script propio de adquisición y entrenamiento. Registrar las **90 distancias de `/r0/scan`** y las consignas **`V` y `W` antes de la conversión a `Twist`**, con sus marcas de tiempo e identificador de vuelta. `data.mat` contiene la evaluación, pero no los barridos LiDAR.
2. Preparar las matrices **`inputs` de 90 × N** y **`outputs` de 2 × N**, en km/h y grados. Diseñar y entrenar una red neuronal usando esos datos.
3. Generar el bloque con la red entrenada:

    ```matlab
    gensim(net, 0.05)
    ```

4. Guardar una copia del modelo y sustituir el controlador borroso por el bloque generado. Conectar la salida **Lidar** de **ROS Robot**, con **Data Type Conversion** a `double` y el preprocesamiento elegido. Mantener las dos salidas `[V; W]`, sus límites, la conversión a `Twist` y los bloques de parada y evaluación.
5. En `controller.m`, cambiar `modelo` al nombre de la copia neuronal.
