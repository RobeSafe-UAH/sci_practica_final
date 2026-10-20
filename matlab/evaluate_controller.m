function estadisticas = evaluate_controller(graficar)
if nargin < 1, graficar = true; end
r = load('data.mat', 'data');
data = r.data;
data = data(:, all(isfinite(data([1 2 3 7 8],:)), 1));
if isempty(data), error('No hay muestras validas para evaluar.'); end
[t, i] = unique(data(1,:));
data = data(:,i);
t = t-t(1);
x = data(2,:); y = data(3,:);
v = data(7,:); volante = data(8,:);
dt = [diff(t) 0];
T = t(end);
pesos = dt/max(T, eps);
if T == 0, pesos(:) = 1/numel(t); end

% Circuito fijo: 40 x 26 m, radio 8 m y dos carriles de 0,75 m.
largo = 40; alto = 26; radio = 8; ancho = 1.5;
c = [largo/2-radio, alto/2-radio];
q = abs([x; y])-c';
distancia = hypot(max(q(1,:),0), max(q(2,:),0)) + min(max(q,[],1),0)-radio;
fuera = abs(distancia) > ancho/2;
invade = distancia < 0 & ~fuera;

a = reshape(linspace(-pi/2,0,65)' + (0:3)*pi/2, [], 1);
centros = repelem([c(1) -c(2); c; -c(1) c(2); -c], 65, 1);
u = [cos(a) sin(a)];
pista = centros+radio*u;
d = circshift(pista,-1)-pista;
l = hypot(d(:,1),d(:,2));
s0 = [0; cumsum(l(1:end-1))];
L = sum(l);
muestras = find(~fuera);
avance = zeros(size(muestras));
for k = 1:numel(muestras)
    posicion = [x(muestras(k)) y(muestras(k))];
    f = max(0,min(1,sum((posicion-pista).*d,2)./l.^2));
    [~,j] = min(sum((pista+f.*d-posicion).^2,2));
    avance(k) = s0(j)+f(j)*l(j);
end
vuelta = 100*max(0,min(1,sum(mod(diff(avance)+L/2,L)-L/2)/L));

estadisticas = struct('tiempoTotal_s', T, 'vuelta_pct', vuelta, ...
    'tiempoInvasion_s', sum(dt.*invade), ...
    'invasion_pct', 100*sum(dt.*invade)/max(T,eps), ...
    'tiempoFuera_s', sum(dt.*fuera), ...
    'velMin_kmh', min(v), 'velMedia_kmh', sum(pesos.*v), 'velMax_kmh', max(v), ...
    'volanteMin_deg', min(volante), 'volanteMedia_deg', sum(pesos.*volante), ...
    'volanteMax_deg', max(volante));
disp(estadisticas);
if ~graficar, return; end

fondo = [0.95 0.96 0.98]; texto = [0.10 0.16 0.25]; gris = [0.43 0.49 0.57];
azul = [0.12 0.43 0.86]; rojo = [0.85 0.23 0.29]; verde = [0.04 0.56 0.43];
pantalla = get(groot,'ScreenSize');
fig = figure('Name','SCI | Evaluacion del controlador','NumberTitle','off', ...
    'Color',fondo,'Position',[30 40 min(1280,pantalla(3)-60) min(800,pantalla(4)-110)]);
annotation(fig,'rectangle',[0 0.905 1 0.095],'FaceColor',texto,'EdgeColor','none');
annotation(fig,'textbox',[0.04 0.95 0.92 0.04],'String','Evaluación del controlador', ...
    'Color','w','FontName','Arial','FontSize',20,'FontWeight','bold','EdgeColor','none','Margin',0);
annotation(fig,'textbox',[0.04 0.917 0.92 0.025], ...
    'String','SCI  /  Traxxas Ackermann  /  Circuito 40 × 26 m  /  Sentido antihorario', ...
    'Color',[0.70 0.78 0.88],'FontName','Arial','FontSize',10,'EdgeColor','none','Margin',0);

etiquetas = {'TIEMPO TOTAL','VUELTA RECORRIDA','CARRIL CONTRARIO','FUERA DE PISTA','VELOCIDAD MEDIA'};
valores = {sprintf('%.2f s',T), sprintf('%.1f %%',vuelta), ...
    sprintf('%.1f %%',estadisticas.invasion_pct), sprintf('%.2f s',estadisticas.tiempoFuera_s), ...
    sprintf('%.1f km/h',estadisticas.velMedia_kmh)};
detalles = {'Duración del registro','Avance neto sobre el trazado', ...
    sprintf('%.2f s de invasión',estadisticas.tiempoInvasion_s), ...
    sprintf('%.1f %% del tiempo',100*estadisticas.tiempoFuera_s/max(T,eps)), 'Consigna del controlador'};
colores = [azul; verde; 0.83 0.49 0.12; rojo; azul];
for k = 1:5
    izquierda = 0.04+(k-1)*0.1872;
    annotation(fig,'rectangle',[izquierda 0.755 0.1712 0.12], ...
        'FaceColor','w','EdgeColor',[0.86 0.89 0.93],'LineWidth',0.7);
    annotation(fig,'rectangle',[izquierda 0.872 0.1712 0.003], ...
        'FaceColor',colores(k,:),'EdgeColor','none');
    annotation(fig,'textbox',[izquierda+0.012 0.839 0.148 0.021],'String',etiquetas{k}, ...
        'Color',gris,'FontName','Arial','FontSize',9,'FontWeight','bold','EdgeColor','none','Margin',0);
    annotation(fig,'textbox',[izquierda+0.012 0.784 0.148 0.047],'String',valores{k}, ...
        'Color',colores(k,:),'FontName','Arial','FontSize',23,'FontWeight','bold','EdgeColor','none','Margin',0);
    annotation(fig,'textbox',[izquierda+0.012 0.762 0.148 0.022],'String',detalles{k}, ...
        'Color',gris,'FontName','Arial','FontSize',8.5,'EdgeColor','none','Margin',0);
end

panel = uipanel(fig,'Position',[0.04 0.075 0.53 0.62],'BackgroundColor','w','BorderType','none');
trayectoria = axes('Parent',panel,'Position',[0.10 0.17 0.84 0.70]);
hold(trayectoria,'on');
for r = [radio+ancho/2, radio-ancho/2]
    p = centros+r*u;
    color = [0.88 0.91 0.94];
    if r < radio, color = [1 1 1]; end
    patch(trayectoria,p(:,1),p(:,2),color,'EdgeColor',[0.58 0.65 0.74],'LineWidth',1);
end
plot(trayectoria,pista([1:end 1],1),pista([1:end 1],2),'--','Color',[0.67 0.73 0.81],'LineWidth',1);
recorrido = plot(trayectoria,x,y,'Color',azul,'LineWidth',2.3);
contrario = scatter(trayectoria,x(invade),y(invade),18,rojo,'filled');
inicio = scatter(trayectoria,x(1),y(1),65,verde,'filled','MarkerEdgeColor','w','LineWidth',1.2);
final = scatter(trayectoria,x(end),y(end),65,texto,'s','filled','MarkerEdgeColor','w','LineWidth',1.2);
axis(trayectoria,'equal'); axis(trayectoria,'tight');
xlim(trayectoria,xlim(trayectoria)+[-1 1]); ylim(trayectoria,ylim(trayectoria)+[-1 1]);
xlabel(trayectoria,'x [m]'); ylabel(trayectoria,'y [m]');
title(trayectoria,'Trayectoria y ocupación de carril','FontWeight','bold','FontSize',12);
legend(trayectoria,[recorrido contrario inicio final], ...
    {'Recorrido','Carril contrario','Inicio','Final'},'Location','southoutside', ...
    'Orientation','horizontal','Box','off','FontSize',9);

panel = uipanel(fig,'Position',[0.59 0.395 0.37 0.30],'BackgroundColor','w','BorderType','none');
velocidad = axes('Parent',panel,'Position',[0.15 0.22 0.80 0.51]);
area(velocidad,t,v,'FaceColor',[0.89 0.94 1],'EdgeColor','none'); hold(velocidad,'on');
plot(velocidad,t,v,'Color',azul,'LineWidth',1.8);
ylim(velocidad,[0 50]); yticks(velocidad,0:10:50); xlim(velocidad,[0 max(T,1)]);
xlabel(velocidad,'Tiempo [s]'); ylabel(velocidad,'Velocidad [km/h]');
title(velocidad,{'Velocidad solicitada',sprintf('Mín. %.1f  /  Media %.1f  /  Máx. %.1f km/h', ...
    estadisticas.velMin_kmh,estadisticas.velMedia_kmh,estadisticas.velMax_kmh)}, ...
    'FontSize',10,'FontWeight','normal');

panel = uipanel(fig,'Position',[0.59 0.075 0.37 0.30],'BackgroundColor','w','BorderType','none');
direccion = axes('Parent',panel,'Position',[0.15 0.22 0.80 0.51]);
plot(direccion,t,volante,'Color',[0.46 0.32 0.78],'LineWidth',1.8); hold(direccion,'on');
yline(direccion,0,'--','Color',[0.70 0.75 0.82]);
ylim(direccion,[-45 45]); yticks(direccion,-45:15:45); xlim(direccion,[0 max(T,1)]);
xlabel(direccion,'Tiempo [s]'); ylabel(direccion,'Volante [grados]');
title(direccion,{'Mando de volante',sprintf('Mín. %.1f  /  Media %.1f  /  Máx. %.1f grados', ...
    estadisticas.volanteMin_deg,estadisticas.volanteMedia_deg,estadisticas.volanteMax_deg)}, ...
    'FontSize',10,'FontWeight','normal');

for ax = [trayectoria velocidad direccion]
    set(ax,'FontName','Arial','FontSize',9,'XColor',texto,'YColor',texto, ...
        'Color','w','GridColor',[0.75 0.80 0.86],'GridAlpha',0.3,'Box','off','TickDir','out');
    grid(ax,'on');
end
annotation(fig,'textbox',[0.04 0.02 0.92 0.025], ...
    'String','Velocidad y volante corresponden a las consignas del controlador.', ...
    'Color',gris,'FontName','Arial','FontSize',9,'EdgeColor','none','Margin',0);
escala = min([1 fig.Position(3)/1280 fig.Position(4)/800]);
textos = findall(fig,'-property','FontSize');
for k = 1:numel(textos)
    textos(k).FontSize = textos(k).FontSize*escala;
end
end
