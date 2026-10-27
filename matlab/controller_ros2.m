%==========================================================================
% CONTROLLER_ROS2.M - CONTROL Y EVALUACION DESDE SIMULINK (ROS 2)
%==========================================================================
clearvars; clc; close all;

%% 1. CONFIGURACION DE ROS 2
% Debe coincidir con ROS_DOMAIN_ID del simulador y del modelo Simulink.
dominioROS2 = 42;
setenv('ROS_DOMAIN_ID', num2str(dominioROS2));

topicOdom = '/r0/odom';
modelo = 'ackerman_ROS2_controller.slx';
tiempoEspera = 3;

%% 2. ESPERAR A QUE EL SIMULADOR PUBLIQUE ODOMETRIA
nodo = ros2node('/matlab_controller_comprobacion');
odom = ros2subscriber(nodo, topicOdom, 'nav_msgs/Odometry', ...
                      'Reliability', 'besteffort');

fprintf('Esperando odometria ROS 2 en %s (dominio %d)...\n', ...
        topicOdom, dominioROS2);
[~, recibido, estado] = receive(odom, tiempoEspera);

clear odom nodo;

if ~recibido
    error(['No se ha recibido odometria (%s). Comprueba el nombre del ' ...
           'topico, su tipo, el dominio ROS 2 y la configuracion QoS.'], ...
           estado);
end

%% 3. EJECUTAR EL CONTROLADOR DE SIMULINK
fprintf('Odometria recibida. Iniciando %s...\n', modelo);
sim(modelo);

%% 4. EVALUAR EL RESULTADO
% El subsistema de evaluacion del .slx debe escribir data.mat.
if isfile('data.mat')
    archivo = load('data.mat', 'data');
    if isfield(archivo, 'data') && isnumeric(archivo.data) && ...
            size(archivo.data, 1) >= 8
        evaluate_controller;
    else
        warning(['Se encontro data.mat, pero su variable data no tiene ' ...
                 'las 8 filas que espera evaluate_controller.m. ' ...
                 'Revisa el Mux/To File del modelo Simulink.']);
    end
else
    warning(['No se encontro data.mat. Revisa el bloque To File ' ...
             'del subsistema de evaluacion en Simulink.']);
end

%% 5. FIN
fprintf('Simulacion finalizada.\n');
