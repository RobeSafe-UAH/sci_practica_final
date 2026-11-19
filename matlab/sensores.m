% SENSORES - Sensores y control manual o autonomo mediante ROS 2.
clearvars; clc;

carpetaMATLAB = fileparts(mfilename('fullpath'));
carpetaProyecto = fileparts(carpetaMATLAB);
configuracion = fileread(fullfile(carpetaProyecto, 'docker', 'config', 'scenario.yaml'));
entorno = fileread(fullfile(carpetaProyecto, 'docker', '.env'));
dominio = regexp(entorno, '(?m)^\s*ROS_DOMAIN_ID\s*=\s*(\d+)\s*(?:#.*)?$', 'tokens', 'once');
assert(~isempty(dominio), 'Falta ROS_DOMAIN_ID en docker/.env.');
dominioROS2 = str2double(dominio{1});
assert(dominioROS2 >= 0 && dominioROS2 <= 232, 'ROS_DOMAIN_ID debe estar entre 0 y 232.');

% Solo se leen los campos utilizados por la demostracion.
secciones = {'simulation', 'track', 'robot', 'traffic', 'lidar'};
campos = {{'rate_hz'}, {'length', 'height', 'corner_radius', 'width'}, ...
          {'wheelbase', 'max_steer_deg', 'max_speed'}, {'count', 'speed'}, {'x', 'y', 'max_range'}};
valores = {20, [40 26 8 1.5], [0.3302 45 50/3.6], [2 1], [0.27 0 30]};
for i = 1:numel(secciones)
    bloque = regexp(configuracion, ['(?ms)^' secciones{i} ':\s*\n(.*?)(?=^\S|\z)'], 'tokens', 'once');
    assert(~isempty(bloque), 'Falta la seccion %s del escenario.', secciones{i});
    for j = 1:numel(campos{i})
        numero = regexp(bloque{1}, ['(?m)^\s+' campos{i}{j} ':\s*([-+\d.eE]+)'], 'tokens', 'once');
        if ~isempty(numero)
            valores{i}(j) = str2double(numero{1});
        end
    end
end
assert(all(isfinite([valores{:}])), 'El escenario contiene parametros invalidos.');
periodo = 1 / min(20, valores{1}(1));
longitud = valores{2}(1); altura = valores{2}(2);
radio = valores{2}(3); ancho = valores{2}(4);
batalla = valores{3}(1); giroMaximo = deg2rad(min(30, valores{3}(2)));
velocidadMaxima = min(50/3.6, valores{3}(3));
maximoManual = min(6, 3.6*velocidadMaxima);
trafico = valores{4}(1); velocidadTrafico = valores{4}(2);
lidarX = valores{5}(1); lidarY = valores{5}(2); alcance = valores{5}(3);
assert(periodo > 0 && isfinite(periodo) && batalla > 0 && giroMaximo > 0 && ...
    velocidadMaxima > 0 && alcance > 0, 'Limites del escenario invalidos.');
assert(ancho > 0 && radio > ancho/2 && 2*radio < min(longitud, altura), 'Geometria del circuito invalida.');
assert(trafico >= 0 && trafico <= 6 && trafico == fix(trafico) && velocidadTrafico > 0, 'Trafico invalido.');
bloquePista = regexp(configuracion, '(?ms)^track:\s*\n(.*?)(?=^\S|\z)', 'tokens', 'once');
sentido = regexp(bloquePista{1}, '(?m)^\s+direction:\s*(ccw|cw)(?=\s|#|$)', 'tokens', 'once');
assert(~isempty(sentido), 'El sentido debe ser ccw o cw.');
signo = 1;
if strcmp(sentido{1}, 'cw'), signo = -1; end
desplazamiento = signo * ancho/4;
centros = [longitud/2-radio, -altura/2+radio; longitud/2-radio, altura/2-radio; ...
           -longitud/2+radio, altura/2-radio; -longitud/2+radio, -altura/2+radio];
ruta = [];
for i = 1:4
    angulo = linspace(-pi/2 + (i-1)*pi/2, (i-1)*pi/2, 101)';
    ruta = [ruta; centros(i,:) + (radio+desplazamiento)*[cos(angulo), sin(angulo)]]; %#ok<AGROW>
end
if signo < 0, ruta = flipud(ruta); end
segmentos = circshift(ruta, -1, 1) - ruta;
largos = vecnorm(segmentos, 2, 2);
acumulada = [0; cumsum(largos)];
longitudVuelta = acumulada(end);
velocidadAuto = min(3.4/3.6, velocidadMaxima);
if trafico > 0, velocidadAuto = min(velocidadAuto, 0.95*velocidadTrafico); end

figuraAnterior = findall(groot, 'Type', 'figure', 'Tag', 'SCI_explorar_sensores');
if ~isempty(figuraAnterior), delete(figuraAnterior); end
nodo = ros2node('/matlab_explorar_sensores', dominioROS2);
publicador = ros2publisher(nodo, '/r0/cmd_vel', 'geometry_msgs/Twist');
cero = ros2message(publicador);
scanSub = ros2subscriber(nodo, '/r0/scan', 'sensor_msgs/LaserScan', 'Reliability', 'besteffort');
odomSub = ros2subscriber(nodo, '/r0/odom', 'nav_msgs/Odometry', 'Reliability', 'besteffort');
choqueSub = ros2subscriber(nodo, '/r0/collision', 'std_msgs/Bool', 'Reliability', 'besteffort');
sonarSub = cell(1,8);
for i = 1:8
    sonarSub{i} = ros2subscriber(nodo, ['/r0/sonar' num2str(i-1)], 'sensor_msgs/Range', ...
        'Reliability', 'besteffort');
end

figura = figure('Name', 'SCI | Exploracion de sensores', 'NumberTitle', 'off', ...
    'Tag', 'SCI_explorar_sensores', 'Color', [0.97 0.97 0.97], ...
    'DefaultAxesColor', [1 1 1], 'DefaultAxesXColor', [.2 .2 .2], ...
    'DefaultAxesYColor', [.2 .2 .2], 'DefaultTextColor', [.15 .15 .15], ...
    'DefaultUicontrolForegroundColor', [.15 .15 .15], ...
    'DefaultUicontrolBackgroundColor', [1 1 1], ...
    'Position', [100 100 1180 760], 'MenuBar', 'none', 'ToolBar', 'none');
setappdata(figura, 'activo', false);
teclas = {'uparrow','downarrow','leftarrow','rightarrow','space','escape'};
for i = 1:numel(teclas), setappdata(figura, teclas{i}, false); end
figura.WindowKeyPressFcn = @(f,e)setappdata(f,e.Key,true);
figura.WindowKeyReleaseFcn = @(f,e)setappdata(f,e.Key,false);
setappdata(figura, 'focoPerdido', false);
setappdata(figura, 'escuchaFoco', addlistener(figura, 'FigureDeactivated', ...
    @(f,~)setappdata(f,'focoPerdido',true)));
uicontrol(figura, 'Style', 'text', 'String', 'Modo', 'Units', 'normalized', ...
    'Position', [.025 .945 .06 .035], 'BackgroundColor', figura.Color, 'HorizontalAlignment', 'left');
selector = uicontrol(figura, 'Style', 'popupmenu', 'String', {'Manual','Autonomo'}, ...
    'Units', 'normalized', 'Position', [.085 .94 .14 .045], ...
    'Callback', @(~,~)setappdata(figura,'activo',false));
boton = uicontrol(figura, 'Style', 'pushbutton', 'String', 'Activar / Pausar', ...
    'Units', 'normalized', 'Position', [.245 .94 .15 .045], ...
    'Callback', @(~,~)setappdata(figura,'activo',~getappdata(figura,'activo')));
uicontrol(figura, 'Style', 'text', 'String', 'Velocidad manual (km/h)', ...
    'Units', 'normalized', 'Position', [.425 .945 .19 .035], ...
    'BackgroundColor', figura.Color, 'HorizontalAlignment', 'left');
deslizador = uicontrol(figura, 'Style', 'slider', 'Min', 0, 'Max', maximoManual, 'Value', min(4,maximoManual), ...
    'SliderStep', [1/12 1/6], 'Units', 'normalized', 'Position', [.62 .95 .22 .025]);
textoVelocidad = uicontrol(figura, 'Style', 'text', 'String', sprintf('%.1f',deslizador.Value), 'Units', 'normalized', ...
    'Position', [.85 .945 .06 .035], 'BackgroundColor', figura.Color);
estado = uicontrol(figura, 'Style', 'text', 'String', 'Esperando sensores | ROS 2: dominio ' + string(dominioROS2), ...
    'Units', 'normalized', 'Position', [.025 .885 .95 .04], 'BackgroundColor', figura.Color, ...
    'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
uicontrol(figura, 'Style', 'text', ...
    'String', 'Manual: mantener ↑ para avanzar; ↓ frena; ← / → giran. Espacio: pausa. Esc: salir. Seleccionar la figura para usar el teclado.', ...
    'Units', 'normalized', 'Position', [.025 .005 .95 .035], ...
    'BackgroundColor', figura.Color, 'HorizontalAlignment', 'left');
panel = uipanel(figura, 'Units', 'normalized', 'Position', [.015 .055 .97 .825], ...
    'BorderType', 'none', 'BackgroundColor', figura.Color);
diseno = tiledlayout(panel, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
azul = [0.05 .35 .65]; naranja = [.85 .35 .08];
ejeLidar = nexttile(diseno); hold(ejeLidar,'on');
rayos = plot(ejeLidar, nan, nan, 'Color', [.65 .8 .9]);
puntos = plot(ejeLidar, nan, nan, '.', 'Color', azul, 'MarkerSize', 10);
plot(ejeLidar, [-.13 .45 .45 -.13 -.13], [-.171 -.171 .171 .171 -.171], ...
    'Color', [.15 .15 .15], 'LineWidth', 2);
quiver(ejeLidar, 0, 0, .7, 0, 0, 'Color', [.15 .15 .15], 'LineWidth', 1.5);
vistaCercana = max(2, min(4, 2*ancho));
axis(ejeLidar, 'equal'); xlim(ejeLidar, [-vistaCercana/2 1.5*vistaCercana]);
ylim(ejeLidar, [-vistaCercana/2 vistaCercana/2]); grid(ejeLidar,'on');
title(ejeLidar, 'LiDAR'); xlabel(ejeLidar,'x (m)'); ylabel(ejeLidar,'y (m)');
axtoolbar(ejeLidar, {'zoomin','zoomout','restoreview'});
ejeSonar = nexttile(diseno);
barras = bar(ejeSonar, 0:7, nan(1,8), 'FaceColor', azul);
xlim(ejeSonar,[-.7 7.7]); ylim(ejeSonar,[0 1]); xticks(ejeSonar,0:7); grid(ejeSonar,'on');
etiquetasSonar = text(ejeSonar, 0:7, zeros(1,8), repmat({''},1,8), ...
    'HorizontalAlignment','center', 'VerticalAlignment','bottom', ...
    'FontSize',10, 'FontWeight','bold', 'Color',[.15 .15 .15]);
title(ejeSonar,'Sonares'); xlabel(ejeSonar,'Numero del sonar'); ylabel(ejeSonar,'Distancia (m)');
ejeV = nexttile(diseno); hold(ejeV,'on'); grid(ejeV,'on');
lineaV = plot(ejeV,nan,nan,'Color',azul,'LineWidth',1.5);
lineaVReal = plot(ejeV,nan,nan,'Color',naranja,'LineWidth',1.2);
title(ejeV,'Velocidad longitudinal'); xlabel(ejeV,'Tiempo (s)'); ylabel(ejeV,'v (km/h)');
ylim(ejeV,[0 1]); legend(ejeV,{'Orden','Odometria'},'Location','northwest', ...
    'Color',[1 1 1], 'TextColor',[.15 .15 .15]);
ejeW = nexttile(diseno); hold(ejeW,'on'); grid(ejeW,'on');
lineaW = plot(ejeW,nan,nan,'Color',azul,'LineWidth',1.5);
lineaWReal = plot(ejeW,nan,nan,'Color',naranja,'LineWidth',1.2);
title(ejeW,'Velocidad angular'); xlabel(ejeW,'Tiempo (s)'); ylabel(ejeW,'omega (rad/s)');
ylim(ejeW,[-.15 .15]); legend(ejeW,{'Orden','Odometria'},'Location','northwest', ...
    'Color',[1 1 1], 'TextColor',[.15 .15 .15]);

reloj = tic;
sellos = nan(10,2); llegadas = -inf(10,1);
historial = zeros(0,5); colision = false; activoAnterior = false; modoAnterior = 1;
avance = 0; sAnterior = nan; ultimoDibujo = -inf; ultimoEscalado = -inf;
iniciado = false; vueltaCompletada = false;
fprintf('Exploracion ROS 2 en el dominio %d. Cierre la figura para terminar.\n', dominioROS2);
% Ctrl+C interrumpe las publicaciones; el simulador frena por su watchdog (1 s).
try
    while isgraphics(figura)
        inicioPaso = tic;
        drawnow limitrate;
        if ~isgraphics(figura), break; end
        ahora = toc(reloj);
        pulsadas = false(1,numel(teclas));
        for i = 1:numel(teclas)
            pulsadas(i) = getappdata(figura,teclas{i});
        end
        if pulsadas(6), break; end
        if pulsadas(5), setappdata(figura,'activo',false); end
        modo = selector.Value;
        if getappdata(figura,'focoPerdido')
            if modo == 1, setappdata(figura,'activo',false); end
            pulsadas(:) = false;
            for i = 1:numel(teclas), setappdata(figura,teclas{i},false); end
            setappdata(figura,'focoPerdido',false);
        end
        if modo ~= modoAnterior
            setappdata(figura,'activo',false); vueltaCompletada = false;
        end
        activo = getappdata(figura,'activo');
        if activo && ~activoAnterior, vueltaCompletada = false; end
        scan = scanSub.LatestMessage; odom = odomSub.LatestMessage;
        sonares = nan(1,8); mensajes = [{scan,odom}, cell(1,8)];
        for i = 1:8
            mensajes{i+2} = sonarSub{i}.LatestMessage;
        end
        for i = 1:10
            mensaje = mensajes{i};
            if ~isempty(mensaje) && isfield(mensaje,'header')
                sello = double([mensaje.header.stamp.sec, mensaje.header.stamp.nanosec]);
                if all(isfinite(sello)) && ~isequal(sello,sellos(i,:))
                    sellos(i,:) = sello; llegadas(i) = ahora;
                end
            end
        end
        choque = choqueSub.LatestMessage;
        if ~isempty(choque), colision = colision || logical(choque.data); end
        valido = all(ahora-llegadas <= 0.5) && ~isempty(choque);
        vReal = nan; wReal = nan; x = nan; y = nan; rumbo = nan;
        if ~isempty(odom)
            p = odom.pose.pose.position; q = odom.pose.pose.orientation;
            x = double(p.x); y = double(p.y);
            rumbo = atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
            vReal = double(odom.twist.twist.linear.x);
            wReal = double(odom.twist.twist.angular.z);
            valido = valido && all(isfinite([x y rumbo vReal wReal]));
        end
        distancias = [];
        if ~isempty(scan)
            distancias = double(scan.ranges(:));
            valido = valido && numel(distancias) == 90 && isfinite(scan.angle_min) && isfinite(scan.angle_increment);
            distancias(distancias == inf) = alcance;
            distancias(~isfinite(distancias) | distancias < 0) = nan;
            distancias = min(distancias,alcance);
            valido = valido && all(isfinite(distancias));
        end
        for i = 1:8
            if ~isempty(mensajes{i+2})
                sonares(i) = double(mensajes{i+2}.range);
                if sonares(i) == inf, sonares(i) = alcance; end
            end
        end
        sonares(~isfinite(sonares) | sonares < 0) = nan;
        sonares = min(sonares,alcance);
        valido = valido && all(isfinite(sonares));
        v = 0; w = 0; obstaculo = false; vuelta = false;
        if colision || ~valido
            setappdata(figura,'activo',false); activo = false;
        end
        if ~activo
            pulsadas(:) = false;
            for i = 1:numel(teclas), setappdata(figura,teclas{i},false); end
        end
        if activo && valido && ~colision
            if ~iniciado
                iniciado = true;
                figura.DeleteFcn = @(~,~)send(publicador,cero);
            end
            if modo == 1
                if pulsadas(1) && ~pulsadas(2)
                    v = deslizador.Value/3.6;
                end
                giro = giroMaximo * (double(pulsadas(3))-double(pulsadas(4)));
                w = v*tan(giro)/batalla;
            else
                proyeccion = sum(([x y]-ruta).*segmentos,2)./largos.^2;
                proyeccion = max(0,min(1,proyeccion));
                cercanos = ruta + proyeccion.*segmentos;
                [~,indice] = min(sum((cercanos-[x y]).^2,2));
                s = acumulada(indice)+proyeccion(indice)*largos(indice);
                if ~activoAnterior || modoAnterior ~= modo
                    avance = 0;
                elseif isfinite(sAnterior)
                    salto = mod(s-sAnterior+longitudVuelta/2,longitudVuelta)-longitudVuelta/2;
                    if abs(salto) < 2, avance = avance+salto; end
                end
                sAnterior = s;
                vuelta = avance >= longitudVuelta-0.1;
                objetivoS = mod(s+0.8,longitudVuelta);
                objetivoI = find(acumulada(1:end-1) <= objetivoS,1,'last');
                objetivo = ruta(objetivoI,:) + (objetivoS-acumulada(objetivoI))/largos(objetivoI)*segmentos(objetivoI,:);
                diferencia = objetivo-[x y];
                lateral = -sin(rumbo)*diferencia(1)+cos(rumbo)*diferencia(2);
                v = velocidadAuto;
                w = 2*v*lateral/max(sum(diferencia.^2),0.01);
                angulos = double(scan.angle_min)+(0:numel(distancias)-1)'*double(scan.angle_increment);
                frontal = distancias(abs(angulos) < deg2rad(12));
                obstaculo = isempty(frontal) || min(frontal) < 0.85;
                if obstaculo, v = 0; w = 0; end
                if vuelta
                    v = 0; w = 0; activo = false; vueltaCompletada = true;
                    setappdata(figura,'activo',false);
                end
            end
        end
        w = max(-v*tan(giroMaximo)/batalla,min(v*tan(giroMaximo)/batalla,w));
        orden = cero; orden.linear.x = v; orden.angular.z = w;
        if iniciado, send(publicador,orden); end
        historial(end+1,:) = [ahora,3.6*v,3.6*vReal,w,wReal]; %#ok<SAGROW>
        historial = historial(historial(:,1) >= ahora-30,:);
        if ahora-ultimoDibujo >= 0.1
            if ~isempty(distancias)
                angulos = double(scan.angle_min)+(0:numel(distancias)-1)'*double(scan.angle_increment);
                px = lidarX+distancias.*cos(angulos); py = lidarY+distancias.*sin(angulos);
                rayos.XData = reshape([repmat(lidarX,1,numel(px));px';nan(1,numel(px))],[],1);
                rayos.YData = reshape([repmat(lidarY,1,numel(py));py';nan(1,numel(py))],[],1);
                puntos.XData = px; puntos.YData = py;
            end
            barras.YData = sonares;
            lineaV.XData = historial(:,1); lineaV.YData = historial(:,2);
            lineaVReal.XData = historial(:,1); lineaVReal.YData = historial(:,3);
            lineaW.XData = historial(:,1); lineaW.YData = historial(:,4);
            lineaWReal.XData = historial(:,1); lineaWReal.YData = historial(:,5);
            xlim(ejeV,[max(0,ahora-30) max(5,ahora)]); xlim(ejeW,ejeV.XLim);
            limites = [max(1,ceil(1.15*max([0 sonares(isfinite(sonares))])*2)/2), ...
                max(1,ceil(1.2*max(historial(:,2:3),[],'all','omitnan')*2)/2), ...
                max(.15,ceil(1.2*max(abs(historial(:,4:5)),[],'all','omitnan')*20)/20)];
            ejes = [ejeSonar ejeV ejeW];
            for i = 1:3
                actual = ejes(i).YLim(2);
                if limites(i) > actual || (ahora-ultimoEscalado >= 1 && limites(i) < .85*actual)
                    inferior = 0;
                    if i == 3, inferior = -limites(i); end
                    ylim(ejes(i),[inferior limites(i)]);
                end
            end
            if ahora-ultimoEscalado >= 1, ultimoEscalado = ahora; end
            for i = 1:8
                if isfinite(sonares(i))
                    etiquetasSonar(i).String = sprintf('%.2f',sonares(i));
                    etiquetasSonar(i).Position = [i-1 sonares(i)+.02*ejeSonar.YLim(2) 0];
                else
                    etiquetasSonar(i).String = '';
                end
            end
            textoVelocidad.String = sprintf('%.1f',deslizador.Value);
            estado.ForegroundColor = [.2 .2 .2];
            if colision
                estado.String = 'Colision: movimiento bloqueado. Reinicie la simulacion.';
                estado.ForegroundColor = [.75 .05 .05]; boton.Enable = 'off';
            elseif ~valido
                estado.String = 'Esperando medidas validas y recientes. Movimiento en pausa.';
                estado.ForegroundColor = [.45 .45 .45];
            elseif vueltaCompletada
                estado.String = 'Vuelta completada. Puede activar otra vuelta.';
            elseif ~activo
                estado.String = 'En pausa. Seleccione el modo y pulse Activar / Pausar.';
            elseif obstaculo
                estado.String = 'Autonomo: detenido ante un obstaculo. No realiza adelantamientos.';
            elseif modo == 1
                estado.String = sprintf('Manual | v = %.1f km/h | omega = %.2f rad/s',3.6*v,w);
            else
                estado.String = sprintf('Autonomo | vuelta = %.0f %% | v = %.1f km/h',100*avance/longitudVuelta,3.6*v);
            end
            ultimoDibujo = ahora;
        end
        activoAnterior = activo; modoAnterior = modo;
        pause(max(0.001,periodo-toc(inicioPaso)));
    end
catch errorExploracion
    if iniciado
        try
            send(publicador,cero);
        catch
            % El watchdog detiene el coche si la comunicacion ya no esta disponible.
        end
    end
    if isgraphics(figura)
        figura.DeleteFcn = []; figura.WindowKeyPressFcn = []; figura.WindowKeyReleaseFcn = [];
        boton.Enable = 'off'; selector.Enable = 'off';
        estado.String = 'Exploracion detenida por un error. Consulte la ventana de comandos.';
        estado.ForegroundColor = [.75 .05 .05];
    end
    clear sonarSub scanSub odomSub choqueSub publicador nodo;
    rethrow(errorExploracion);
end
if iniciado, send(publicador,cero); pause(0.05); end
if isgraphics(figura)
    figura.DeleteFcn = []; delete(figura);
end
clear sonarSub scanSub odomSub choqueSub publicador nodo;
fprintf('Exploracion finalizada.\n');
