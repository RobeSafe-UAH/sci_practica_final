---
hide:
  - toc
---
# MATLAB

MATLAB y Simulink se ejecutan **en el equipo del alumno** y se comunican por ROS2 con el simulador de Docker. Antes de empezar, completar [Instalación](index.md) y dejar la simulación en marcha siguiendo [Ejecución](ejecucion.md).

## Requisitos

1. Instalar **MATLAB**, **Simulink**, **ROS Toolbox** y **Fuzzy Logic Toolbox**. Comprobar los productos instalados desde MATLAB:

    ```matlab
    ver
    ```

2. Utilizar el modelo correspondiente a la versión de MATLAB:

    | Modelo en `matlab/` | Versión del archivo | Uso |
    |---|---|---|
    | `ackerman_ROS2_controller.slx` | R2025a | Modelo principal utilizado por `controller_ros2.m` y por esta guía. Abrir con R2025a o posterior. |
    | `ackerman_ROS2_controller2023.slx` | R2023b | Alternativa para versiones anteriores a R2025a, desde R2023b. |

    La alternativa de R2023b conserva una lógica anterior de filtrado de sensores y detección de meta. Para utilizarla, cambiar la variable `modelo` de `controller_ros2.m` a `'ackerman_ROS2_controller2023.slx'` y revisar esos bloques antes de comparar resultados con el modelo principal.

3. Abrir MATLAB y seleccionar **`matlab/` dentro del repositorio** como carpeta actual. En ella deben estar los dos scripts `.m`, el modelo elegido y el controlador propio `fuzzy_controller.fis`. MATLAB se utiliza fuera del contenedor; `make attach` se reserva para los comandos de terminal de la [guía ROS2](topics.md).

## Preparar el controlador difuso

1. Diseñar el controlador propio con **Fuzzy Logic Designer**:

    ```matlab
    fuzzyLogicDesigner
    ```

2. Definir **ocho entradas**, en el orden `sonar_0`, `sonar_1`, …, `sonar_7`. Son las distancias de los [sectores del LiDAR](topics.md#sonares), en **metros**, desde el lado derecho del vehículo hasta el izquierdo.

    En el modelo principal, las entradas al controlador quedan entre **0 y 30 m**: `+Inf` se sustituye por 30 m; las medidas negativas, `NaN` y otros valores no finitos se sustituyen por 0 m. Las distancias finitas superiores a 30 m se limitan a 30 m.

3. Definir **dos salidas en este orden**:

    | Posición | Salida | Significado | Rango |
    |---|---|---|---|
    | 1 | `V` | Velocidad solicitada. | **0 a 50 km/h** |
    | 2 | `W` | Mando de dirección; positivo hacia la izquierda. | **−45° a +45°** |

    **`W` es un ángulo en grados a la salida del FIS.** El subsistema `ROS Robot/Conversion_diff_ackerman` convierte `V` a m/s y calcula la velocidad angular en rad/s que publica en `angular.z` de `/r0/cmd_vel`. Al publicar directamente desde una terminal ROS2, utilizar las [unidades del mensaje `Twist`](topics.md#control).

4. Guardar el sistema como **`fuzzy_controller.fis`** en la carpeta `matlab/`. El bloque **Controlador → Fuzzy Logic Controller** carga ese archivo por su nombre. Guardarlo desde Fuzzy Logic Designer o, si el sistema está en la variable `fis`, utilizar [`writeFIS`](https://www.mathworks.com/help/fuzzy/mamfis.writefis.html):

    ```matlab
    writeFIS(fis, 'fuzzy_controller.fis');
    ```

5. Comprobar que MATLAB encuentra y puede leer el archivo:

    ```matlab
    fis = readfis('fuzzy_controller.fis');
    numel(fis.Inputs)     % Debe ser 8
    numel(fis.Outputs)    % Debe ser 2
    ```

    El FIS forma parte del trabajo del alumno y permanece local; no se incluye al clonar el repositorio.

## Configurar el dominio ROS2

1. Consultar **`ROS_DOMAIN_ID` en `docker/.env`**, dentro del proyecto. Ese es el archivo de configuración del contenedor. Si se modifica con el simulador en marcha, detener `make run` con **Ctrl+C** y volver a ejecutarlo.
2. Editar **`dominioROS2` en `matlab/controller_ros2.m`** con el mismo número. Por ejemplo, si en `docker/.env` aparece `ROS_DOMAIN_ID=42`:

    ```matlab
    dominioROS2 = 42;
    ```

    El script aplica el valor mediante `setenv('ROS_DOMAIN_ID', num2str(dominioROS2))` antes de crear su nodo de comprobación.

3. Abrir el modelo:

    ```matlab
    open_system('ackerman_ROS2_controller.slx');
    ```

4. En Simulink, abrir **ROS Network / Configure ROS Network** desde la pestaña **Simulation → Prepare → ROS Toolbox**; según la versión, también se accede desde los bloques ROS2 mediante **Configure ROS 2 Domain ID and ROS Middleware**. En **Domain ID (ROS 2)**, introducir el mismo número y aplicar los cambios. La [guía de MathWorks](https://www.mathworks.com/help/ros/ug/connect-to-a-ros-enabled-robot-from-simulink-over-ros2.html) muestra este ajuste.
5. Comprobar desde la ventana de comandos de MATLAB que aparecen los topics del simulador. Sustituir `42` por el dominio elegido:

    ```matlab
    setenv('ROS_DOMAIN_ID', '42');
    getenv('ROS_DOMAIN_ID')
    ros2 topic list
    ```

    Deben aparecer `/r0/odom`, `/r0/scan`, `/r0/sonar0` … `/r0/sonar7` y `/r0/collision`. Comprobar el valor en **los tres lugares: `docker/.env`, el script y la configuración de red de Simulink**. Cambiar solo la variable de MATLAB no corrige un dominio fijado de forma explícita en Simulink.

## Ejecutar una prueba

1. Mantener **`make run` activo** en la terminal del equipo y abrir el [simulador en el navegador](http://localhost:6080/vnc.html?autoconnect=true&resize=scale). Esperar a que termine de arrancar.
2. Guardar el FIS, los ajustes de dominio y el modelo. El modelo principal utiliza un paso fijo de **0,05 s**, correspondiente a los **20 Hz** de la configuración predeterminada del simulador. Si se cambia `simulation.rate_hz`, ajustar **Ctrl+E → Solver → Fixed-step size** a `1 / simulation.rate_hz`.
3. Detener cualquier publicación manual en `/r0/cmd_vel` antes de ejecutar el controlador, para que sus órdenes no se mezclen.
4. Desde la ventana de comandos de MATLAB, con `matlab/` como carpeta actual, ejecutar:

    ```matlab
    controller_ros2
    ```

5. El script muestra el dominio y espera **hasta 3 segundos** para recibir `/r0/odom`, de tipo `nav_msgs/Odometry`, mediante una suscripción **Best Effort**. Si recibe odometría, inicia Simulink. Si vence la espera, muestra un error y no ejecuta el modelo.
6. El modelo principal se detiene al cruzar la meta después de recorrer la vuelta, al recibir una colisión o al alcanzar su límite de **200 s de simulación**. Al terminar, el script busca `data.mat` y abre la evaluación si el registro tiene el formato esperado. También se puede pulsar **Stop** en Simulink para finalizar una prueba antes.
7. Para empezar otra prueba desde la salida, detener el simulador con **Ctrl+C** en la terminal de `make run` y volver a ejecutar `make run`. Esperar al arranque antes de lanzar de nuevo `controller_ros2`.

## Evaluar y comparar resultados

1. El bloque **Evaluacion_maniobra → To File** guarda **`data.mat`**, con una variable numérica llamada **`data`**. El formato esperado es **Array**: cada columna contiene una muestra y la primera fila contiene el tiempo, como describe la [documentación de To File](https://www.mathworks.com/help/simulink/slref/tofile.html).

    | Fila de `data` | Contenido | Unidad |
    |---|---|---|
    | 1 | Tiempo de simulación. | s |
    | 2 | Posición `x`. | m |
    | 3 | Posición `y`. | m |
    | 4, 5 y 6 | Orientación de Euler en orden ZYX: guiñada, cabeceo y alabeo. | rad |
    | 7 | Consigna de velocidad del controlador. | km/h |
    | 8 | Consigna de dirección del controlador. | grados |

2. La evaluación se lanza automáticamente al terminar `controller_ros2`. Para volver a evaluar el último registro desde la carpeta `matlab/`:

    ```matlab
    evaluate_controller
    ```

3. Consultar la figura y la estructura `estadisticas` que se muestra en la ventana de comandos. Incluyen duración del registro, porcentaje de vuelta recorrido, tiempo y porcentaje de invasión del carril contrario, trayectoria y valores mínimo, medio y máximo de velocidad y dirección. **Las gráficas de velocidad y dirección representan consignas del controlador**.
4. La evaluación utiliza la geometría fija del circuito de **40 × 26 m**, radio de curva de **8 m** y anchura total de **1,5 m**. Si se modifica el circuito, actualizar esa geometría en `evaluate_controller.m` y revisar la detección de meta del modelo antes de interpretar los resultados.
5. Cada ejecución sobrescribe `data.mat`. Para conservar una prueba, copiar el registro y el FIS utilizado a otra carpeta antes de volver a ejecutar el modelo.

## Resolver problemas

| Síntoma | Comprobación |
|---|---|
| MATLAB no encuentra `fuzzy_controller.fis` o el modelo. | Seleccionar `matlab/` como carpeta actual y comprobar el nombre y la extensión del archivo. |
| El archivo `.slx` requiere una versión más reciente. | Revisar la tabla de versiones y seleccionar el modelo adecuado en la variable `modelo` del script. |
| No aparece ningún topic o vence la espera de odometría. | Comprobar que `make run` sigue activo y que coinciden los tres ajustes de dominio. Revisar también la configuración de red del sistema indicada en [Instalación](index.md). |
| MATLAB ve topics, pero Simulink no recibe sensores. | Revisar el **Domain ID** de Simulink y los nombres, tipos y ajustes QoS de sus bloques Subscribe. Ver una lista de topics no confirma que se estén recibiendo mensajes. |
| El controlador no avanza. | Comprobar que el FIS devuelve una velocidad positiva en su primera salida y que la segunda contiene la dirección. Reiniciar el simulador si se ha detectado una colisión. |
| Aparece un aviso sobre `data.mat` o sus ocho filas. | Revisar el bloque **To File**: archivo `data.mat`, variable `data`, formato **Array** y señales conectadas al Mux de evaluación. |
| La evaluación no corresponde a la última prueba. | Comprobar la fecha de `data.mat`. Si el modelo no llegó a ejecutarse o falló antes de escribir el registro, puede quedar el archivo de una prueba anterior. |
