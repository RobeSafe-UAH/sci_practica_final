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

5. Abrir el modelo principal:

    ```matlab
    open_system('ackerman_ROS2_controller.slx');
    ```

6. En Simulink, abrir **Simulation → Prepare → ROS Toolbox → ROS Network**. En **Domain ID (ROS 2)**, introducir el mismo número y aplicar los cambios. Según la versión, también se accede desde los bloques ROS2 mediante **Configure ROS 2 Domain ID and ROS Middleware**. Este ajuste es independiente de `setenv` en MATLAB: si permanece en `0`, los bloques seguirán utilizando ese dominio. La [guía de MathWorks](https://www.mathworks.com/help/ros/ug/connect-to-a-ros-enabled-robot-from-simulink-over-ros2.html) muestra el diálogo.
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
    | **Manual** | Mantener **↑** para avanzar y **←/→** para girar. Soltar **↑** o pulsar **↓** frena; **espacio** pausa. Velocidad de **0–6 km/h**, ajustable con el deslizador; inicialmente **4 km/h**. |
    | **Autónomo** | Sigue el carril derecho a baja velocidad y se detiene después de una vuelta. Lee la geometría de `docker/config/scenario.yaml`. |

    El modo autónomo sirve para observar los sensores; no realiza adelantamientos. Su velocidad se limita a **3,4 km/h** y, si hay tráfico, al **95 % de su velocidad**.

3. Observar los cuatro paneles:

    | Panel | Información |
    |---|---|
    | **LiDAR** | Los 90 valores de `/r0/scan`, representados alrededor del vehículo. |
    | **Sonares** | Distancias de `sonar_0` a `sonar_7`, en metros. |
    | **Velocidad lineal** | Velocidad ordenada y medida por la odometría, en km/h. |
    | **Velocidad angular** | Velocidad de giro ordenada y medida, en rad/s. |

    Los sonares se calculan con los 1081 haces internos del Hokuyo; el LiDAR publicado se reduce a 90 valores. Los topics y ángulos se describen en [ROS2](topics.md).

4. Pausar con **Activar / Pausar** o **espacio**. En manual, cambiar a otra ventana también pausa el movimiento; volver a la figura y reactivarlo para continuar. **Esc** o cerrar la ventana finaliza la exploración y envía una orden de parada. Si se interrumpe con **Ctrl+C**, el puente detiene el vehículo como máximo un segundo después de la última publicación.
5. Ante una colisión, reiniciar el simulador con **Ctrl+C** y **`make run`** antes de repetir. La pérdida de datos de sensores también detiene el movimiento.

## 3. Diseñar el controlador

1. Con el escenario sin tráfico, abrir **Fuzzy Logic Designer** y crear un sistema **Mamdani**:

    ```matlab
    fuzzyLogicDesigner
    ```

2. Elegir los sonares necesarios; **no es obligatorio utilizar los ocho**. La plantilla conecta `sonar_0`, `sonar_1`, …, `sonar_7`, desde el lado derecho hasta el izquierdo, en metros. Para utilizar menos entradas, adaptar el Mux del modelo y conservar el mismo orden en el FIS. Los ángulos se consultan en [ROS2](topics.md#sonares).

    El modelo principal limita las entradas a **0–30 m**: sustituye `+Inf` por 30 m; los valores negativos, `NaN` y otros valores no finitos, por 0 m.

3. Definir **dos salidas en este orden**:

    | Posición | Salida | Significado | Rango |
    |---|---|---|---|
    | 1 | `V` | Velocidad solicitada. | **0–50 km/h** |
    | 2 | `W` | Ángulo de dirección equivalente del eje delantero; positivo hacia la izquierda. | **−45° a +45°** |

    `ROS Robot/Conversion_diff_ackerman` convierte estas consignas a `/r0/cmd_vel` mediante `v = V/3.6` y `omega = v*tan(deg2rad(W))/0.3302`. La batalla del vehículo es **0,3302 m**; una curva de radio 8 m requiere aproximadamente **2,36°** de dirección. El mensaje contiene `linear.x = v` en m/s y `angular.z = omega` en rad/s; MVSim aplica la geometría Ackermann a las ruedas delanteras.

4. Guardar el controlador como **`fuzzy_controller.fis`** en `matlab/`. El bloque **Controlador → Fuzzy Logic Controller** carga ese archivo. Si el sistema está en la variable `fis`, utilizar:

    ```matlab
    writeFIS(fis, 'fuzzy_controller.fis');
    ```

5. Comprobar las dimensiones:

    ```matlab
    fis = readfis('fuzzy_controller.fis');
    numel(fis.Inputs)     % 8
    numel(fis.Outputs)    % 2
    ```

    Los valores esperados son 8 y 2 en la plantilla; si se ha adaptado el Mux, el número de entradas debe coincidir con el del FIS. Los controladores diseñados y los archivos generados permanecen locales; no se incluyen al clonar el repositorio.

## 4. Ejecutar y evaluar

1. Cerrar `sensores`, guardar el controlador y el modelo, y reiniciar el simulador desde la salida con **Ctrl+C** y **`make run`**. Mantener **`traffic.count: 0`** para la primera fase. El paso fijo del modelo principal es **0,05 s**, correspondiente a **20 Hz**.
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
2. Incorporar al modelo un registro que asocie cada barrido nuevo con sus consignas. Conservar `header.stamp` de `/r0/scan`, comprobar que se reciben 90 medidas y excluir las muestras iniciales sin datos y las paradas por final de ensayo. La frecuencia común de 20 Hz no garantiza por sí sola esa correspondencia.
3. Preparar las matrices **`inputs` de 90 × N** y **`outputs` de 2 × N**, en km/h y grados. Tratar las distancias inválidas y aplicar el mismo preprocesamiento durante el entrenamiento y en Simulink. Separar los conjuntos por vueltas completas y entrenar la red según el enunciado.
4. Generar el bloque con la red entrenada:

    ```matlab
    gensim(net, 0.05)
    ```

5. Guardar una copia del modelo y sustituir el controlador borroso por el bloque generado. Conectar la salida **Lidar** de **ROS Robot**, con **Data Type Conversion** a `double` y el preprocesamiento elegido. Mantener las dos salidas `[V; W]`, sus límites, la conversión a `Twist` y los bloques de parada y evaluación. Si se modifica la recepción de LiDAR, mantener en **90** el tamaño máximo de `ranges` de `sensor_msgs/LaserScan` en [Variable Size Message](https://www.mathworks.com/help/ros/ug/manage-array-sizes-in-simulink-ros.html).
6. En `controller.m`, cambiar `modelo` al nombre de la copia neuronal. Probarla con **`traffic.count: 0`** y **`traffic.count: 2`**, reiniciando el simulador tras cada cambio, y comparar sus resultados con los del borroso mediante `evaluate_controller`.

## Resolver problemas

| Síntoma | Comprobación |
|---|---|
| MATLAB no encuentra un script, el FIS o el modelo. | Seleccionar `matlab/` como carpeta actual y comprobar los nombres. |
| El `.slx` requiere una versión más reciente. | Utilizar el modelo correspondiente a la versión de MATLAB y cambiar `modelo` en `controller.m`. |
| No aparecen topics o no llega odometría. | Comprobar `make run`, los dominios de Docker, MATLAB y Simulink, y la red indicada en [Instalación](index.md). |
| MATLAB ve topics, pero Simulink no recibe sensores. | Revisar **ROS Network → Domain ID (ROS 2)** y el QoS de los bloques Subscribe. |
| El explorador está parado. | Activar el movimiento, comprobar el modo y el estado de la ventana. En manual, mantener la flecha de avance. |
| Se ha detectado una colisión. | Reiniciar el simulador antes de volver a conducir. |
| La evaluación no corresponde a la última prueba. | Comprobar fecha y formato de `data.mat`; puede quedar un registro anterior si el modelo no llegó a ejecutarse. |
