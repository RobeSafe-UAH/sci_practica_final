function controller_ros2
clc;
cd(fileparts(mfilename('fullpath')));
dominioROS2 = 42;
tiempoEspera = 10;
modelo = 'ackerman_ROS2_controller.slx';
setenv('ROS_DOMAIN_ID',num2str(dominioROS2));

if ~isfile('fuzzy_controller.fis')
    error('Falta fuzzy_controller.fis en la carpeta matlab.');
end
fis = readfis('fuzzy_controller.fis');
if numel(fis.Inputs) ~= 8 || numel(fis.Outputs) ~= 2
    error('El FIS debe tener ocho entradas (sonar_0 a sonar_7) y dos salidas (V, W).');
end

nodo = ros2node('/matlab_controller_comprobacion',dominioROS2);
odom = ros2subscriber(nodo,'/r0/odom','nav_msgs/Odometry', ...
    'Reliability','besteffort');
fprintf('Esperando odometría ROS 2 (dominio %d)...\n',dominioROS2);
[~,recibido,estado] = receive(odom,tiempoEspera);
if ~recibido
    error('No se ha recibido odometría (%s). Comprueba el simulador y ROS_DOMAIN_ID.',estado);
end
clear odom;

mando = ros2publisher(nodo,'/r0/cmd_vel','geometry_msgs/Twist');
parada = ros2message(mando);
paradaFinal = onCleanup(@() send(mando,parada));
fprintf('Iniciando %s...\n',modelo);
sim(modelo);
clear paradaFinal mando nodo;

if isfile('data.mat')
    archivo = load('data.mat','data');
    if isfield(archivo,'data') && isnumeric(archivo.data) && size(archivo.data,1) >= 8
        evaluate_controller(true);
    else
        warning('data.mat debe contener una matriz numérica data de al menos ocho filas.');
    end
else
    warning('No se ha generado data.mat. Revisa el bloque To File del modelo.');
end
fprintf('Simulación finalizada.\n');
end
