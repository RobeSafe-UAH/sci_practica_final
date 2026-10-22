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
    'velMin_kmh', min(v), 'velMedia_kmh', sum(pesos.*v), 'velMax_kmh', max(v), ...
    'volanteMin_deg', min(volante), 'volanteMedia_deg', sum(pesos.*volante), ...
    'volanteMax_deg', max(volante));
disp(estadisticas);
if ~graficar, return; end

azul = [0.13 0.35 0.58]; rojo = [0.70 0.20 0.19];
texto = [0.12 0.18 0.25]; gris = [0.43 0.47 0.51];
pantalla = get(groot,'ScreenSize');
tamano = min([1240 800],max([900 650],pantalla(3:4)-[60 110]));
fig = figure('Name','Evaluación del controlador — SCI','NumberTitle','off', ...
    'Color','w','Position',[30 40 tamano]);
annotation(fig,'textbox',[0.045 0.925 0.91 0.045], ...
    'String','Evaluación del controlador','FontName','Arial','FontSize',19, ...
    'FontWeight','bold','Color',texto,'EdgeColor','none','Margin',0);
annotation(fig,'textbox',[0.045 0.895 0.91 0.025], ...
    'String','SCI  ·  Circuito 40 × 26 m  ·  Sentido antihorario', ...
    'FontName','Arial','FontSize',10,'Color',gris,'EdgeColor','none','Margin',0);
annotation(fig,'line',[0.045 0.955],[0.88 0.88],'Color',[0.80 0.83 0.86]);
etiquetas = {'Tiempo total','Vuelta recorrida','Carril contrario'};
valores = {sprintf('%.2f s',T),sprintf('%.1f %%',vuelta), ...
    sprintf('%.1f %%',estadisticas.invasion_pct)};
detalles = {'Duración del registro','Avance sobre el circuito', ...
    sprintf('%.2f s de invasión',estadisticas.tiempoInvasion_s)};
for k = 1:3
    izquierda = 0.045+(k-1)*0.315;
    color = texto;
    if k == 3, color = rojo; end
    annotation(fig,'textbox',[izquierda 0.831 0.28 0.025],'String',etiquetas{k}, ...
        'FontName','Arial','FontSize',11,'FontWeight','bold', ...
        'Color',color,'EdgeColor','none','Margin',0);
    annotation(fig,'textbox',[izquierda 0.776 0.28 0.048],'String',valores{k}, ...
        'FontName','Arial','FontSize',24,'Color',color,'EdgeColor','none','Margin',0);
    annotation(fig,'textbox',[izquierda 0.749 0.28 0.023],'String',detalles{k}, ...
        'FontName','Arial','FontSize',10,'Color',gris,'EdgeColor','none','Margin',0);
end
disposicion = tiledlayout(fig,2,2,'TileSpacing','compact','Padding','compact');
disposicion.OuterPosition = [0.025 0.065 0.95 0.645];

trayectoria = nexttile(disposicion,1,[2 1]);
hold(trayectoria,'on');
for r = [radio+ancho/2, radio-ancho/2]
    p = centros+r*u;
    color = [0.90 0.93 0.95];
    if r < radio, color = [1 1 1]; end
    patch(trayectoria,p(:,1),p(:,2),color,'EdgeColor',gris,'LineWidth',0.8);
end
plot(trayectoria,pista([1:end 1],1),pista([1:end 1],2),'--','Color',gris,'LineWidth',0.8);
recorrido = plot(trayectoria,x,y,'Color',azul,'LineWidth',1.8);
contrario = scatter(trayectoria,x(invade),y(invade),12,rojo,'filled');
inicio = plot(trayectoria,x(1),y(1),'o','Color',azul, ...
    'MarkerFaceColor','w','MarkerSize',7,'LineWidth',1.2);
final = plot(trayectoria,x(end),y(end),'s','Color','k','MarkerFaceColor','k','MarkerSize',6);
axis(trayectoria,'equal'); axis(trayectoria,'tight');
xlim(trayectoria,xlim(trayectoria)+[-1 1]); ylim(trayectoria,ylim(trayectoria)+[-1 1]);
xlabel(trayectoria,'x [m]'); ylabel(trayectoria,'y [m]');
title(trayectoria,'Trayectoria y ocupación de carril','FontWeight','bold','FontSize',11);
legend(trayectoria,[recorrido contrario inicio final], ...
    {'Recorrido','Carril contrario','Inicio','Final'},'Location','southoutside', ...
    'Orientation','horizontal','NumColumns',2,'Box','off','FontSize',9);

velocidad = nexttile(disposicion,2);
area(velocidad,t,v,'FaceColor',[0.92 0.95 0.98],'EdgeColor','none');
hold(velocidad,'on');
plot(velocidad,t,v,'Color',azul,'LineWidth',1.5);
ylim(velocidad,[0 50]); yticks(velocidad,0:10:50); xlim(velocidad,[0 max(T,1)]);
xlabel(velocidad,'Tiempo [s]'); ylabel(velocidad,'Velocidad [km/h]');
title(velocidad,'Velocidad solicitada','FontSize',11,'FontWeight','bold');
subtitle(velocidad,sprintf('Mínima: %.1f   Media: %.1f   Máxima: %.1f km/h', ...
    estadisticas.velMin_kmh,estadisticas.velMedia_kmh,estadisticas.velMax_kmh), ...
    'FontSize',9,'Color',gris);

direccion = nexttile(disposicion,4);
plot(direccion,t,volante,'Color',azul,'LineWidth',1.5); hold(direccion,'on');
yline(direccion,0,':','Color',gris);
ylim(direccion,[-45 45]); yticks(direccion,-45:15:45); xlim(direccion,[0 max(T,1)]);
xlabel(direccion,'Tiempo [s]'); ylabel(direccion,'Volante [grados]');
title(direccion,'Mando de dirección','FontSize',11,'FontWeight','bold');
subtitle(direccion,sprintf('Mínimo: %.1f   Media: %.1f   Máximo: %.1f grados', ...
    estadisticas.volanteMin_deg,estadisticas.volanteMedia_deg,estadisticas.volanteMax_deg), ...
    'FontSize',9,'Color',gris);

for ax = [trayectoria velocidad direccion]
    set(ax,'FontName','Arial','FontSize',10,'XColor',texto,'YColor',texto, ...
        'Color','w','GridColor',gris,'GridAlpha',0.15,'Box','off','TickDir','out');
    grid(ax,'on');
end
annotation(fig,'textbox',[0.045 0.018 0.91 0.025], ...
    'String','Velocidad y dirección corresponden a las consignas del controlador.', ...
    'FontName','Arial','FontSize',9,'Color',gris,'EdgeColor','none','Margin',0);
end
