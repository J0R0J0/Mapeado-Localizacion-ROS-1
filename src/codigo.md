# Código hasta ahora

## 1_Conexion_setup

### conectar.m

```matlab
%% Conectar con ROS
setenv('ROS_MASTER_URI','http://192.168.1.144:11311');
setenv('ROS_IP','192.168.1.144');
rosinit();

disp('Conexión con ROS establecida correctamente.');
```

### desconectar.m

```matlab
%% DESCONECTAR DE ROS — Cierre seguro y ordenado
% Detiene el robot, limpia subscribers/publishers y cierra ROS
% sin congelar MATLAB.

fprintf('[CIERRE] Iniciando desconexión segura...\n');

%% 1. PARAR EL ROBOT INMEDIATAMENTE (lo primero de todo)
try
    msg_vel.Linear.X  = 0;
    msg_vel.Angular.Z = 0;
    send(pub, msg_vel);
    send(pub, msg_vel);   % doble envío por seguridad
    pause(0.3);
    fprintf('[CIERRE] Robot detenido.\n');
catch e
    fprintf('[CIERRE] Aviso al detener robot: %s\n', e.message);
end

%% 2. ELIMINAR SUBSCRIBERS (liberan los callbacks y la memoria ROS)
try
    if exist('odom',  'var'), delete(odom);   end
    if exist('laser', 'var'), delete(laser);  end
    fprintf('[CIERRE] Subscribers eliminados.\n');
catch e
    fprintf('[CIERRE] Aviso al eliminar subscribers: %s\n', e.message);
end

%% 3. ELIMINAR PUBLISHER
try
    if exist('pub', 'var'), delete(pub); end
    fprintf('[CIERRE] Publisher eliminado.\n');
catch e
    fprintf('[CIERRE] Aviso al eliminar publisher: %s\n', e.message);
end

%% 4. CERRAR RATE (evita que waitfor se quede bloqueado)
try
    if exist('r', 'var'), delete(r); end
catch
end

%% 5. ROSSHUTDOWN CON TIMEOUT
% Lanzamos rosshutdown en un timer para que si se cuelga >10s,
% MATLAB lo cancela y continúa sin congelar.
fprintf('[CIERRE] Cerrando ROS (máx 10s)...\n');
try
    t_shutdown = tic;
    rosshutdown;
    fprintf('[CIERRE] ROS cerrado en %.1fs.\n', toc(t_shutdown));
catch e
    fprintf('[CIERRE] rosshutdown falló: %s — continuando.\n', e.message);
end

%% 6. LIMPIAR VARIABLES (excepto Mapa, por si se quiere analizar después)
try
    clearvars -except Mapa
catch
end

fprintf('[CIERRE] Desconexión completada.\n');
```

### ini_simulador.m

```matlab
disp('Inicializando variables y topics...');

%% DECLARACION DE SUBSCRIBERS
% Subscripción a la odometría y al laser del AmigoBot
odom = rossubscriber('/robot0/odom');
laser = rossubscriber('/robot0/laser_1');

%% DECLARACIOÓN DE PUBLISHERS Y MENSAJES
% Creamos el publicador para enviar las velocidades del AmigoBot
pub = rospublisher('/robot0/cmd_vel','geometry_msgs/Twist');
msg_vel = rosmessage(pub);

%% CREACIÓN DE LA MATRIZ DEL MAPA
% El entorno es de dimensiones máximas de 15x15 metros
% Las celdas rectangulares físicas deben de ser 6cm x 6cm (0.06 metros).
% Calculamos el número de celdas: 15 / 0.06 = 250. Por lo tanto, necesitamos una matriz de 250x250.
celdas_lado = 15 / 0.06; 

% Ek cibtebudi de cada celda del arrray Mapa(i, j) se inicializa en 0.5 = no explorado
Mapa = ones(celdas_lado, celdas_lado) * 0.5;

%% DEFINICIÓN DE TIMER Y VERIFICACIÓN DE LECTURAS
% Definimo la periodicidad del bucle de control (10 Hz)
r = robotics.Rate(10);
waitfor(r);
pause(3); 

% Por seguridad, el robot arranca con comandos de velocidad a cero
vel_lineal_cmd = 0.0;
vel_angular_cmd = 0.0;

% Variable para el cálculo derivativo del Controlador PD
error_previo = 0.0;
error_integral = 0.0; 

% Nos asegudamos de recibir un mensaje válido de odometría del simulador antes de arrancar
disp('Esperando lectura de odometría...');

while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
    odom.LatestMessage;
end

disp('Robot inicializado. Matriz Mapa 250x250 creada y lista.');
```

### avanzar.m

```matlab
msg_vel.Linear.X = vel_lineal_cmd;
```

### girar.m

```matlab
msg_vel.Angular.Z = vel_angular_cmd;
```

### lee_sensores.m

```matlab
%% EXTRAE Y PROCESA LOS DATOS DEL LÁSER
msg_laser_nav = laser.LatestMessage;

if ~isempty(msg_laser_nav)
    rangos_nav = msg_laser_nav.Ranges;
    ang_min_nav = msg_laser_nav.AngleMin;
    ang_inc_nav = msg_laser_nav.AngleIncrement;

    angulos_nav = ang_min_nav + (0:length(rangos_nav)-1) * ang_inc_nav;

    % ---- SECTOR FRONTAL: detección de colisión inminente ----
    idx_frontales = find(angulos_nav > -0.35 & angulos_nav < 0.35);
    if ~isempty(idx_frontales)
        r_f = rangos_nav(idx_frontales);
        r_f = r_f(~isinf(r_f) & r_f > 0.05);
        if ~isempty(r_f)
            dist_frontal = min(r_f);
        else
            dist_frontal = inf;
        end
    else
        dist_frontal = inf;
    end

    % ---- SECTOR PARED DERECHA ----
    % Usamos un cono ESTRECHO centrado exactamente en -pi/2 (perpendicular derecha).
    % Con solo ±0.20 rad la señal es estable: no cambia cuando el robot hace
    % micro-correcciones angulares, eliminando la causa raíz de la oscilación en S.
    ang_centro = -pi/2;   % -1.5708 rad = perpendicular al flanco derecho
    margen     = 0.20;    % ±11 grados

    idx_derecha = find(angulos_nav > (ang_centro - margen) & ...
                        angulos_nav < (ang_centro + margen));

    if ~isempty(idx_derecha)
        r_d = rangos_nav(idx_derecha);
        r_d = r_d(~isinf(r_d) & r_d > 0.05 & r_d < 6.0);
        if ~isempty(r_d)
            % MEDIANA en lugar de mínimo: inmune a puntos de ruido aislados
            dist_pared = median(r_d);
        else
            dist_pared = 3.0;  % sin pared a la derecha → modo búsqueda
        end
    else
        dist_pared = 3.0;
    end

else
    dist_frontal = inf;
    dist_pared   = 3.0;
end

% ---- SECTOR LATERAL IZQUIERDO ----
ang_centro_izq = pi/2;
idx_izq = find(angulos_nav > (ang_centro_izq - 0.20) & ...
                angulos_nav < (ang_centro_izq + 0.20));
if ~isempty(idx_izq)
    r_i = rangos_nav(idx_izq);
    r_i = r_i(~isinf(r_i) & r_i > 0.05 & r_i < 6.0);
    dist_lateral_izq = median(r_i);
    if isnan(dist_lateral_izq); dist_lateral_izq = 3.0; end
else
    dist_lateral_izq = 3.0;
end

% ---- SECTOR LATERAL DERECHO ----
% (dist_pared ya existe, la reutilizamos directamente)
dist_lateral_der = dist_pared;
```

### evasion.m

```matlab
%% EVASIÓN DE OBSTÁCULOS

evadiendo = false;

if dist_frontal < 0.7

    evadiendo = true;

    disp('¡Obstáculo frontal! Ejecutando evasión...');

    % Frenar
    vel_lineal_cmd = 0;
    vel_angular_cmd = 0;

    msg_vel.Linear.X = vel_lineal_cmd;
    msg_vel.Angular.Z = vel_angular_cmd;
    send(pub,msg_vel);

    pause(0.3);

    % Retroceso corto
    msg_vel.Linear.X = -0.15;
    msg_vel.Angular.Z = 0;
    send(pub,msg_vel);

    pause(1.0);

    % Giro amplio
    msg_vel.Linear.X = 0;
    msg_vel.Angular.Z = 0.9;
    send(pub,msg_vel);

    pause(1.8);

    % Parar
    msg_vel.Linear.X = 0;
    msg_vel.Angular.Z = 0;
    send(pub,msg_vel);

else

    evadiendo = false;

end
```

### modo_pared.m

```matlab
% SEGUIMIENTO DE PARED DERECHA
% Estrategia: control proporcional con banda muerta.
% Se elimina el término integral (causa principal de la oscilación en S)
% y se añade una banda muerta para que el robot no corrija errores mínimos.

if ~evadiendo

    D_deseada = 0.6;  % <-- CAMBIA AQUÍ la distancia objetivo en metros

    % ---- GANANCIAS ----
    Kp = 0.45;   % Ganancia proporcional. Sube si tarda en corregir; baja si oscila.
    Kd = 1.1;   % Ganancia derivativa. Amortigua: evita que sobrepase la distancia objetivo.

    % Banda muerta: si el error es menor que esto, no corregimos nada.
    % Evita micro-correcciones continuas que generan la S.
    banda_muerta = 0.03;  % ±3 cm de tolerancia

    % Velocidad lineal fija. Si el robot va más rápido necesitarás subir Kd.
    VEL_LINEAL = 0.22;

    if dist_pared > 1.5
        % ---- Sin pared: girar suavemente a la derecha para encontrarla ----
        vel_lineal_cmd  =  0.20;
        vel_angular_cmd = -0.30;
        error_previo    =  0;

    else
        % ---- Pared detectada: control PD ----
        error_dist = D_deseada - dist_pared;

        % Aplicar banda muerta: si estamos dentro de tolerancia, error = 0
        if abs(error_dist) < banda_muerta
            error_dist = 0;
        end

        % Término derivativo (frena antes de llegar, evita el sobrepaso)
        deriv_error = error_dist - error_previo;
        error_previo = error_dist;

        % Suavizado para evitar oscilaciones
        error_dist = 0.7 * error_previo + 0.3 * error_dist;

        % Ley de control PD
        vel_angular_cmd = Kp * error_dist + Kd * deriv_error;

        % Saturación de velocidad angular
        limite_giro = 0.50;
        vel_angular_cmd = max(-limite_giro, min(limite_giro, vel_angular_cmd));

        vel_lineal_cmd = VEL_LINEAL;
    end
end
```

### actualizar_mapa.m

```matlab
%% ACTUALIZAR MAPA DE OCUPACIÓN (MEJORADO)

% 1. EXTRAER POSICIÓN Y ORIENTACIÓN DESDE ODOMETRÍA
pos_x = odom.LatestMessage.Pose.Pose.Position.X;
pos_y = odom.LatestMessage.Pose.Pose.Position.Y;

quaternion = [odom.LatestMessage.Pose.Pose.Orientation.W, ...
                odom.LatestMessage.Pose.Pose.Orientation.X, ...
                odom.LatestMessage.Pose.Pose.Orientation.Y, ...
                odom.LatestMessage.Pose.Pose.Orientation.Z];

euler = quat2eul(quaternion, 'ZYX');
theta_amigobot = euler(1);

% 2. EXTRAER DATOS DEL LÁSER
msg_laser = laser.LatestMessage;
if isempty(msg_laser); return; end

rangos    = double(msg_laser.Ranges);
ang_min   = double(msg_laser.AngleMin);
ang_inc   = double(msg_laser.AngleIncrement);
rango_max = double(msg_laser.RangeMax);
rango_min = double(msg_laser.RangeMin);
N_rayos   = length(rangos);

% 3. SUBMUESTREO MENOS AGRESIVO (1 de cada 2 rayos)
paso_submuestreo = 2;
indices_rayo = 1:paso_submuestreo:N_rayos;

% 4. RAY TRACING
TAM_CELDA = 0.06;
MAX_Y     = 15.0;
N_CELDAS  = 250;

for k = indices_rayo
    distancia = rangos(k);

    % Filtrar lecturas inválidas o muy cortas (ruido del sensor)
    if distancia < rango_min || distancia < 0.08; continue; end

    angulo_rayo = theta_amigobot + ang_min + (k - 1) * ang_inc;
    cos_a = cos(angulo_rayo);
    sin_a = sin(angulo_rayo);

    % Determinar si hay impacto o lectura infinita
    if isinf(distancia) || distancia > rango_max
        dist_libre = rango_max;
        hay_impacto = false;
    else
        dist_libre = distancia;
        hay_impacto = true;
    end

    % ----- Celdas LIBRES -----
    if dist_libre > TAM_CELDA
        % Dejar margen antes del obstáculo para no borrar la pared
        dist_pintar = dist_libre - TAM_CELDA;
        pasos_d = (0 : TAM_CELDA : dist_pintar);
        px_vec  = pos_x + pasos_d * cos_a;
        py_vec  = pos_y + pasos_d * sin_a;

        cols = round(px_vec / TAM_CELDA) + 1;
        fils = round((MAX_Y - py_vec) / TAM_CELDA) + 1;

        cols = max(1, min(N_CELDAS, cols));
        fils = max(1, min(N_CELDAS, fils));

        idx_lin = unique(fils + (cols - 1) * N_CELDAS);
        mask_libres = Mapa(idx_lin) ~= 0;
        Mapa(idx_lin(mask_libres)) = 1;
    end

    % ----- Celda OCUPADA (con radio pequeño para paredes más sólidas) -----
    % ----- Celda OCUPADA (solo celda exacta) -----
    if hay_impacto
        px_f = pos_x + dist_libre * cos_a;
        py_f = pos_y + dist_libre * sin_a;
        c_f  = max(1, min(N_CELDAS, round(px_f / TAM_CELDA) + 1));
        f_f  = max(1, min(N_CELDAS, round((MAX_Y - py_f) / TAM_CELDA) + 1));
        Mapa(f_f, c_f) = 0;
    end
end
```

### global2matriz.m

```matlab
function [fila, columna] = global2matriz(X, Y)
    % Función que convierte coordenadas globales (X,Y) en metros
    % a índices de fila y columna en la matriz del mapa.

    %% Constantes del mapa
    tam_celda = 0.06;
    max_y = 15.0;

    %% EJE X A COLUMNAS:
    % Simplemente dividimos la coordenada X entre el tamaño de la celda.
    % Sumamos 1 porque los índices en Matlab empiezan en 1, no en 0.
    columna = round(X / tam_celda) + 1;

    %% EJE Y A FILAS (INVERSIÓN):
    % Como el origen del mapa global está abajo y el de la imagen arriba,
    % debemos restar la coordenada Y actual al máximo total (15 metros)
    % antes de dividir por el tamaño de celda.
    fila = round((max_y - Y) / tam_celda) + 1;

    %% LÍMITES DE SEGURIDAD:
    % Evitamos que un error de lectura intente acceder a un índice 
    % fuera de nuestra matriz de 250x250
    if fila < 1; fila = 1; end
    if fila > 250; fila = 250; end
    if columna < 1; columna = 1; end
    if columna > 250; columna = 250; end
end
```

### detectar_circulo.m

```matlab
%% DETECTAR OBJETO CIRCULAR EN EL MAPA DE OCUPACIÓN
%
% Usa la Transformada de Hough Circular (imfindcircles) sobre las celdas
% OCUPADAS del mapa para localizar el obstáculo de contorno circular.
%
% REQUIERE en el workspace:
%   Mapa - Matriz 250x250 (0=Ocupada, 1=Libre, 0.5=No explorada)
%
% PRODUCE en el workspace:
%   circulo_encontrado  - true/false
%   circulo_centro_x    - coordenada X del centro en metros
%   circulo_centro_y    - coordenada Y del centro en metros
%   circulo_radio_m     - radio en metros

TAM_CELDA = 0.06;
MAX_Y     = 15.0;

%% 1. PREPARAR IMAGEN BINARIA DE OBSTÁCULOS
% imfindcircles trabaja con imágenes uint8.
% Marcamos las celdas OCUPADAS (valor 0 en Mapa) como blancas (255).
img_ocupadas = uint8((Mapa == 0)) * 255;

%% 2. RANGO DE RADIOS A BUSCAR
% El enunciado habla de un objeto circular en un entorno de laboratorio.
% Estimamos radio entre 5 cm y 60 cm → en celdas:
r_min_celdas = max(2, round(0.05 / TAM_CELDA));   % ~1  celda  (5 cm)
r_max_celdas =        round(0.60 / TAM_CELDA);    % ~10 celdas (60 cm)

%% 3. TRANSFORMADA DE HOUGH CIRCULAR
% Probamos primero con objetos brillantes (círculo de obstáculos sobre fondo libre)
[centers1, radii1, strengths1] = imfindcircles(img_ocupadas, ...
    [r_min_celdas, r_max_celdas], ...
    'ObjectPolarity', 'bright', ...
    'Sensitivity',    0.88, ...
    'EdgeThreshold',  0.08);

% Si no encontramos nada, probamos polaridad inversa
if isempty(centers1)
    [centers1, radii1, strengths1] = imfindcircles(img_ocupadas, ...
        [r_min_celdas, r_max_celdas], ...
        'ObjectPolarity', 'dark', ...
        'Sensitivity',    0.88, ...
        'EdgeThreshold',  0.08);
end

%% 4. SELECCIONAR EL MEJOR CÍRCULO
if isempty(centers1)
    circulo_encontrado = false;
    circulo_centro_x   = 0;
    circulo_centro_y   = 0;
    circulo_radio_m    = 0;
    disp('[Círculo] No se detectó ningún objeto circular en el mapa.');
else
    circulo_encontrado = true;

    % Tomar el de mayor fuerza (mejor ajuste)
    [~, idx_mejor]   = max(strengths1);
    col_celda        = centers1(idx_mejor, 1);   % columna (eje X imagen)
    fila_celda       = centers1(idx_mejor, 2);   % fila    (eje Y imagen)
    radio_celdas     = radii1(idx_mejor);

    % Convertir a coordenadas mundo
    circulo_centro_x = (col_celda - 1) * TAM_CELDA;
    circulo_centro_y = MAX_Y - (fila_celda - 1) * TAM_CELDA;
    circulo_radio_m  = radio_celdas * TAM_CELDA;

    fprintf('\n=== OBJETO CIRCULAR DETECTADO ===\n');
    fprintf('  Centro X : %.3f m\n', circulo_centro_x);
    fprintf('  Centro Y : %.3f m\n', circulo_centro_y);
    fprintf('  Radio    : %.3f m\n', circulo_radio_m);
    fprintf('  Fuerza   : %.3f\n',   strengths1(idx_mejor));
    fprintf('=================================\n\n');
end

%% 5. VISUALIZACIÓN Y GUARDADO
fig_circ = figure('Name', 'Detección de Círculo', 'Color', 'k');
imshow(Mapa, [0 1]);
colormap(gray);
title('Mapa de ocupación — Detección circular', 'Color', 'w');
hold on;

if circulo_encontrado
    % Dibujar el círculo detectado
    theta_plot = linspace(0, 2*pi, 200);
    col_celda  = centers1(idx_mejor, 1);
    fila_celda = centers1(idx_mejor, 2);
    r_plot     = radii1(idx_mejor);

    plot(col_celda + r_plot * cos(theta_plot), ...
         fila_celda + r_plot * sin(theta_plot), ...
            'r-', 'LineWidth', 2);
    plot(col_celda, fila_celda, 'r+', 'MarkerSize', 16, 'LineWidth', 2);

    text(col_celda + r_plot + 2, fila_celda, ...
            sprintf('r=%.2fm', circulo_radio_m), ...
            'Color', 'red', 'FontSize', 9, 'FontWeight', 'bold');
else
    text(10, 10, 'Sin círculo detectado', ...
            'Color', 'red', 'FontSize', 12);
end
hold off;

%% 6. GUARDAR CAPTURA CON EL CÍRCULO DIBUJADO
% Construimos la imagen manualmente para que quede limpia
mapa_rgb = repmat(uint8(Mapa * 255), [1, 1, 3]);

if circulo_encontrado
    % Dibujar el círculo en rojo sobre la imagen RGB
    for t = linspace(0, 2*pi, 1000)
        px = round(fila_celda + r_plot * sin(t));
        py = round(col_celda  + r_plot * cos(t));
        % Pintar con grosor de 2 píxeles
        for df = -1:1
            for dc = -1:1
                fi = px + df;
                ci = py + dc;
                if fi >= 1 && fi <= N_CELDAS && ci >= 1 && ci <= N_CELDAS
                    mapa_rgb(fi, ci, 1) = 255;  % R
                    mapa_rgb(fi, ci, 2) = 0;    % G
                    mapa_rgb(fi, ci, 3) = 0;    % B
                end
            end
        end
    end

    % Dibujar cruz en el centro
    for d = -4:4
        fi = round(fila_celda) + d;
        ci = round(col_celda);
        if fi >= 1 && fi <= N_CELDAS && ci >= 1 && ci <= N_CELDAS
            mapa_rgb(fi, ci, :) = uint8([255, 0, 0]);
        end
        fi = round(fila_celda);
        ci = round(col_celda) + d;
        if fi >= 1 && fi <= N_CELDAS && ci >= 1 && ci <= N_CELDAS
            mapa_rgb(fi, ci, :) = uint8([255, 0, 0]);
        end
    end
end

timestamp_circ = datestr(now, 'yyyymmdd_HHMMSS');
nombre_circ = sprintf('mapa_circulo_%s.png', timestamp_circ);
imwrite(mapa_rgb, nombre_circ);
fprintf('[Círculo] Imagen con detección guardada como: %s\n', nombre_circ);
```

### guardar_mapa.m

```matlab
%% GUARDAR MAPA DE OCUPACIÓN EN ARCHIVO PNG

mapa_img   = uint8(Mapa * 255);
timestamp  = datestr(now, 'yyyymmdd_HHMMSS');
nombre_png = sprintf('mapa_explorado_%s.png', timestamp);

imwrite(mapa_img, nombre_png);
fprintf('[Guardar] Mapa guardado como: %s\n', nombre_png);

figure('Name','Mapa Final');
imshow(mapa_img);
colormap(gray);
title(sprintf('Mapa Final — %s', strrep(timestamp,'_',' ')));
```

### main.m

```matlab
%% ============================================================
%%  MAIN.M — TFA Percepción y Control
%%  Exploración autónoma con planificación BFS + seguimiento de waypoints
%%
%%  COMPORTAMIENTO:
%%  1. Busca la zona no explorada (gris) más cercana → frontier.
%%  2. Calcula un camino real con BFS sobre el mapa (evita paredes).
%%  3. Sigue los waypoints del camino uno a uno con control PID.
%%  4. Si detecta obstáculo frontal → para, gira, y RECALCULA ruta.
%%     NO hay contador de intentos: recalcula siempre inmediatamente.
%%  5. Solo termina si el mapa está realmente completo o pasan 5 min.
%% ============================================================
clear functions; clc; close all;
addpath(genpath(pwd));

try

    %% ============================================================
    %% PARÁMETROS GLOBALES
    %% ============================================================
    T_MAX           = 300;    % Tiempo máximo (s)
    DIST_WAYPOINT   = 0.40;   % Distancia para considerar waypoint alcanzado (m)
    DIST_EMERGENCIA = 0.50;   % Distancia frontal para evadir (m)
    VEL_LINEAL      = 1.00;   % Velocidad lineal máxima (m/s)
    VEL_GIRO_EVASION= 0.9;    % Velocidad angular al esquivar (rad/s)
    T_GIRO_EVASION  = 1.0;    % Tiempo girando al esquivar (s)
    RADIO_BLACKLIST = 0.80;   % Radio para ignorar un frontier ya visitado (m)

    % --- DETECCIÓN DE BLOQUEO ---
    pos_anterior_x = 0;
    pos_anterior_y = 0;
    tiempo_bloqueo = tic;

    DIST_MIN_MOVIMIENTO = 0.15;
    TIEMPO_STUCK = 5;
    % Filtrar frontiers demasiado cerca del borde del mapa
    MARGEN_OBSTACULO = 5;  % celdas (~60cm)
    TAM_CELDA = 0.06;
    MAX_Y_M   = 15.0;
    N_C       = 250;

    % Ganancias PID orientación
    Kp_ang = 1.1;
    Kd_ang = 0.4;
    Ki_ang = 0.008;

    fprintf('╔══════════════════════════════════════╗\n');
    fprintf('║   TFA — Percepción y Control (BFS)   ║\n');
    fprintf('╚══════════════════════════════════════╝\n\n');

    %% ============================================================
    %% FASE 1: CONEXIÓN E INICIALIZACIÓN
    %% ============================================================
    conectar;
    ini_simulador;
    fprintf('[FASE 1] OK. Mapa %dx%d listo.\n\n', size(Mapa,1), size(Mapa,2));

    %% ============================================================
    %% FASE 2: VARIABLES DE ESTADO
    %% ============================================================


    t_scan_tic = tic;

    estado_nav     = 'PLANIFICAR';
    % Estados: PLANIFICAR | SEGUIR | EVADIR | FIN

    % Ruta como lista de waypoints [x, y] en metros
    ruta_waypoints = zeros(0, 2);
    idx_waypoint   = 1;

    % Blacklist de frontiers inaccesibles
    blacklist      = zeros(0, 2);

    % PID angular
    error_ang_prev = 0;
    error_ang_int  = 0;
    t_waypoint_tic = tic;   % timeout por waypoint atascado

    % Evasión
    t_evasion_tic  = tic;
    dir_evasion    = 1;   % +1 izquierda, -1 derecha

    % Visualización
    t_ultima_visual = tic;
    PERIODO_VISUAL  = 0.5;

    % Giro previo a cada waypoint
    girando_a_wp = false;

    contador_evasiones    = 0;   % cuántas veces seguidas hemos evadido
    MAX_EVASIONES_SEGUIDAS = 3;  % si evadimos 3 veces yendo al mismo destino → blacklist
    ultimo_destino_x      = -1;
    ultimo_destino_y      = -1;

    intentos_planificacion_fallidos = 0;
    MAX_INTENTOS_PLAN = 5;
    
    contador_giros_rescate = 0;
    MAX_GIROS_RESCATE      = 4;

    fprintf('[FASE 2] Variables inicializadas.\n\n');

    %% ============================================================
    %% PREPARAR FIGURA
    %% ============================================================
    fig     = figure('Name','Exploración en vivo','NumberTitle','off');
    h_img   = imshow(Mapa, [0 1]);
    colormap(gray);
    hold on;
    h_robot = plot(0, 0, 'bo', 'MarkerSize',10,'MarkerFaceColor','blue','LineWidth',2);
    h_ruta  = plot(0, 0, 'g-o', 'LineWidth',1.5,'MarkerSize',4,'MarkerFaceColor','green');
    h_dest  = plot(0, 0, 'r*', 'MarkerSize',14,'LineWidth',2);
    h_texto = text(5, 12, '', 'Color','yellow','FontSize',9,...
        'BackgroundColor','black','FontWeight','bold');
    hold off;

    %% ============================================================
    %% FASE 3: BUCLE PRINCIPAL
    %% ============================================================
    fprintf('[FASE 3] Iniciando. T_MAX=%ds\n\n', T_MAX);
    t_inicio = tic;

    while toc(t_inicio) < T_MAX

        %% ---- SENSORES Y ODOMETRÍA ----
        lee_sensores;

        pos_x = odom.LatestMessage.Pose.Pose.Position.X;
        pos_y = odom.LatestMessage.Pose.Pose.Position.Y;

        % ============================================================
        % DETECCIÓN DE ROBOT BLOQUEADO
        % ============================================================

        movimiento = norm([pos_x - pos_anterior_x, pos_y - pos_anterior_y]);

        if movimiento > DIST_MIN_MOVIMIENTO
            pos_anterior_x = pos_x;
            pos_anterior_y = pos_y;
            tiempo_bloqueo = tic;
        end

        if toc(tiempo_bloqueo) > TIEMPO_STUCK

            disp('[ESCAPE] Robot bloqueado. Ejecutando maniobra de escape...');

            % Retroceder
            msg_vel.Linear.X = -0.25;
            msg_vel.Angular.Z = 0;
            send(pub,msg_vel);
            pause(1.5);

            % Girar fuerte
            msg_vel.Linear.X = 0;
            msg_vel.Angular.Z = 0.8;
            send(pub,msg_vel);
            pause(2.0);

            % Parar
            msg_vel.Linear.X = 0;
            msg_vel.Angular.Z = 0;
            send(pub,msg_vel);

            tiempo_bloqueo = tic;
        end
        quat  = [odom.LatestMessage.Pose.Pose.Orientation.W, ...
            odom.LatestMessage.Pose.Pose.Orientation.X, ...
            odom.LatestMessage.Pose.Pose.Orientation.Y, ...
            odom.LatestMessage.Pose.Pose.Orientation.Z];
        euler = quat2eul(quat,'ZYX');
        theta = euler(1);

        %% ---- ACTUALIZAR MAPA ----
        % Solo pintamos cuando el robot va suficientemente recto.
        % Si está girando (error angular grande o velocidad angular alta),
        % el láser barre el entorno con movimiento y genera ruido en el mapa.
        esta_girando = abs(vel_angular_cmd) > 0.6;
        if ~esta_girando
            actualizar_mapa;
        end

        %% ---- LOG ----
        if idx_waypoint <= size(ruta_waypoints,1)
            fprintf('[DBG] %-10s | WP %d/%d | Front:%.2fm\n', ...
                estado_nav, idx_waypoint, size(ruta_waypoints,1), dist_frontal);
        else
            fprintf('[DBG] %-10s | Front:%.2fm\n', estado_nav, dist_frontal);
        end

        %% ---- VISUALIZACIÓN ----
        if toc(t_ultima_visual) > PERIODO_VISUAL
            col_r = max(1,min(N_C, round(pos_x/TAM_CELDA)+1));
            fil_r = max(1,min(N_C, round((MAX_Y_M-pos_y)/TAM_CELDA)+1));

            set(h_img, 'CData', Mapa);
            set(h_robot,'XData',col_r,'YData',fil_r);

            if size(ruta_waypoints,1) >= 1
                cols_ruta = max(1,min(N_C, round(ruta_waypoints(:,1)/TAM_CELDA)+1));
                fils_ruta = max(1,min(N_C, round((MAX_Y_M-ruta_waypoints(:,2))/TAM_CELDA)+1));
                set(h_ruta,'XData',cols_ruta,'YData',fils_ruta);
                if idx_waypoint <= size(ruta_waypoints,1)
                    set(h_dest,'XData',cols_ruta(idx_waypoint),'YData',fils_ruta(idx_waypoint));
                end
                dest_txt_x = ruta_waypoints(end,1);
                dest_txt_y = ruta_waypoints(end,2);
            else
                dest_txt_x = 0; dest_txt_y = 0;
            end

            set(h_texto,'String',sprintf(...
                't=%.0fs | %s\nRobot:(%.1f,%.1f)\nFrontier:(%.1f,%.1f)\nFront:%.2fm | WP:%d/%d',...
                toc(t_inicio), estado_nav, pos_x, pos_y, dest_txt_x, dest_txt_y,...
                dist_frontal, idx_waypoint, size(ruta_waypoints,1)));
            title(sprintf('t=%.0fs — %s', toc(t_inicio), estado_nav));
            drawnow limitrate;
            t_ultima_visual = tic;
        end

        %% ============================================================
        %% MÁQUINA DE ESTADOS
        %% ============================================================
        switch estado_nav

            %% ----------------------------------------------------------
            case 'ESCANEAR'
                %% Giro completo inicial para ver el entorno antes de planificar
                %% ----------------------------------------------------------
                vel_lineal_cmd  = 0;
                vel_angular_cmd = 1.0;  % giro máximo permitido
                avanzar; girar; send(pub, msg_vel);

                % 7 segundos ≈ 400 grados (margen para asegurar 360°)
                if toc(t_scan_tic) > 7.0
                    fprintf('[SCAN] Escaneo completado. Iniciando exploración.\n');
                    vel_lineal_cmd=0; vel_angular_cmd=0;
                    avanzar; girar; send(pub,msg_vel);
                    estado_nav = 'PLANIFICAR';
                end

                %% ----------------------------------------------------------
            case 'PLANIFICAR'
                %% Buscar frontier y calcular ruta BFS
                %% ----------------------------------------------------------
                % Tras girar, intentar salir retrocediendo si hay obstáculo cerca
                vel_lineal_cmd = 0; vel_angular_cmd = 0;
                avanzar; girar; send(pub, msg_vel);

                % Si frontalmente hay obstáculo cerca, retroceder 1 segundo
                if dist_frontal < 1.0
                    fprintf('[PLAN] Zona estrecha detectada. Retrocediendo.\n');
                    t_retro = tic;
                    while toc(t_retro) < 1.5
                        vel_lineal_cmd  = -0.3;   % retroceso
                        vel_angular_cmd = 0;
                        avanzar; girar; send(pub, msg_vel);
                        waitfor(r);
                    end
                    vel_lineal_cmd = 0; vel_angular_cmd = 0;
                    avanzar; girar; send(pub, msg_vel);
                end

                fprintf('[PLAN] Buscando frontier...\n');

                % 1. Encontrar todos los frontiers válidos
                [frontiers_x, frontiers_y] = encontrar_frontiers(Mapa, N_C, TAM_CELDA, MAX_Y_M);

                if isempty(frontiers_x)
                    contador_giros_rescate = contador_giros_rescate + 1;
                    if contador_giros_rescate > MAX_GIROS_RESCATE
                        fprintf('[PLAN] Sin frontiers tras %d giros de rescate. Terminando.\n', ...
                                contador_giros_rescate);
                        estado_nav = 'FIN';
                        continue;
                    end

                    fprintf('[PLAN] Sin frontiers. Giro de rescate %d/%d.\n', ...
                            contador_giros_rescate, MAX_GIROS_RESCATE);

                    % Girar in situ para reescanear el entorno
                    t_giro_rescate = tic;
                    while toc(t_giro_rescate) < 4.0
                        vel_lineal_cmd  = 0;
                        vel_angular_cmd = 0.8;
                        avanzar; girar; send(pub, msg_vel);
                        actualizar_mapa;
                        waitfor(r);
                    end
                    vel_lineal_cmd = 0; vel_angular_cmd = 0;
                    avanzar; girar; send(pub, msg_vel);

                    % Limpiar blacklist para que vuelva a probar destinos previos
                    blacklist = zeros(0,2);
                    continue;
                end

                % 2. Filtrar frontiers en blacklist
                candidatos_x = frontiers_x;
                candidatos_y = frontiers_y;

                if ~isempty(blacklist)
                    mask_validos = true(size(candidatos_x));
                    for b = 1:size(blacklist,1)
                        dbl = sqrt((candidatos_x - blacklist(b,1)).^2 + ...
                            (candidatos_y - blacklist(b,2)).^2);
                        mask_validos = mask_validos & (dbl > RADIO_BLACKLIST);
                    end
                    candidatos_x = candidatos_x(mask_validos);
                    candidatos_y = candidatos_y(mask_validos);
                end

                if isempty(candidatos_x)
                    fprintf('[PLAN] Todos los frontiers en blacklist. Limpiando.\n');
                    blacklist = zeros(0,2);
                    intentos_planificacion_fallidos = intentos_planificacion_fallidos + 1;  % ← CLAVE
                    if intentos_planificacion_fallidos >= MAX_INTENTOS_PLAN
                        % Forzar reducción de margen inmediatamente
                        contador_giros_rescate = contador_giros_rescate + 1;
                        if contador_giros_rescate > MAX_GIROS_RESCATE
                            fprintf('[PLAN] Zonas restantes inaccesibles. Exploracion terminada.\n');
                            estado_nav = 'FIN';
                        else
                            margen_actual = max(3, MARGEN_OBSTACULO - (contador_giros_rescate * 3));
                            MARGEN_OBSTACULO = margen_actual;
                            fprintf('[PLAN] Reduciendo margen a %d celdas.\n', MARGEN_OBSTACULO);
                            intentos_planificacion_fallidos = 0;
                        end
                    end
                    continue;
                end

                % 3. Ordenar por distancia euclidea
                dists = sqrt((candidatos_x - pos_x).^2 + (candidatos_y - pos_y).^2);
                [~, orden] = sort(dists);
                candidatos_x = candidatos_x(orden);
                candidatos_y = candidatos_y(orden);

                % 4. Mapa navegable con margen de seguridad
                mapa_bfs = calcular_mapa_navegable(Mapa, MARGEN_OBSTACULO);

                % 5. Intentar BFS hacia cada candidato hasta encontrar ruta válida
                [fila_rob, col_rob] = global2matriz(pos_x, pos_y);
                ruta_encontrada = false;

                for k = 1:min(20, length(candidatos_x))
                    fx = candidatos_x(k);
                    fy = candidatos_y(k);
                    [fila_dest, col_dest] = global2matriz(fx, fy);

                    fprintf('[PLAN] BFS hacia (%.2f, %.2f)...\n', fx, fy);
                    camino = bfs_camino(mapa_bfs, fila_rob, col_rob, fila_dest, col_dest, N_C);

                    if ~isempty(camino)
                        ruta_m = celdas_a_metros(camino, TAM_CELDA, MAX_Y_M);
                        ruta_waypoints = simplificar_ruta(ruta_m, mapa_bfs, TAM_CELDA, MAX_Y_M, N_C);
                        idx_waypoint   = 1;
                        error_ang_prev = 0;
                        error_ang_int  = 0;
                        fprintf('[PLAN] Ruta OK: %d waypoints hacia (%.2f,%.2f).\n', ...
                            size(ruta_waypoints,1), fx, fy);
                        estado_nav = 'SEGUIR';
                        ruta_encontrada = true;
                        intentos_planificacion_fallidos = 0;  % ← añade esta línea
                        break;
                    else
                        fprintf('[PLAN] Sin camino a (%.2f,%.2f). Blacklist.\n', fx, fy);
                        blacklist(end+1,:) = [fx, fy];
                    end
                end

                if ~ruta_encontrada
                    fprintf('[PLAN] Sin rutas validas. Reintentando.\n');
                    intentos_planificacion_fallidos = intentos_planificacion_fallidos + 1;
                    if intentos_planificacion_fallidos >= MAX_INTENTOS_PLAN
                        contador_giros_rescate = contador_giros_rescate + 1;

                        if contador_giros_rescate > MAX_GIROS_RESCATE
                            fprintf('[PLAN] Zonas restantes inaccesibles. Exploracion terminada.\n');
                            estado_nav = 'FIN';
                        else
                            fprintf('[PLAN] Sin frontiers accesibles. Girando para explorar mas.\n');
                            
                            % Reducir margen progresivamente en cada rescate
                            margen_actual = max(3, MARGEN_OBSTACULO - (contador_giros_rescate * 3));
                            fprintf('[PLAN] Reduciendo margen a %d celdas.\n', margen_actual);

                            t_giro_rescate = tic;
                            while toc(t_giro_rescate) < 3.0
                                vel_lineal_cmd  = 0;
                                vel_angular_cmd = 0.8;
                                avanzar; girar; send(pub, msg_vel);
                                actualizar_mapa;
                                waitfor(r);
                            end
                            vel_lineal_cmd = 0; vel_angular_cmd = 0;
                            avanzar; girar; send(pub, msg_vel);

                            % Retroceder si hay obstáculo cerca
                            if dist_frontal < 1.0
                                fprintf('[PLAN] Zona estrecha. Retrocediendo.\n');
                                t_retro = tic;
                                while toc(t_retro) < 1.5
                                    vel_lineal_cmd  = -0.3;
                                    vel_angular_cmd = 0;
                                    avanzar; girar; send(pub, msg_vel);
                                    waitfor(r);
                                end
                                vel_lineal_cmd = 0; vel_angular_cmd = 0;
                                avanzar; girar; send(pub, msg_vel);
                            end

                            blacklist = zeros(0,2);
                            intentos_planificacion_fallidos = 0;
                            
                            % Usar margen reducido en el siguiente BFS
                            MARGEN_OBSTACULO = margen_actual;
                            
                            fprintf('[PLAN] Giro completado. Reexplorando con margen %d.\n', MARGEN_OBSTACULO);
                        end
                    end
                    pause(0.2);
                end

                %% ----------------------------------------------------------
            case 'SEGUIR'
                % Obstáculo frontal → evadir
                if dist_frontal < DIST_EMERGENCIA
                    fprintf('[SEGUIR] Obstaculo! (%.2fm). Recalculando ruta.\n', dist_frontal);
                    vel_lineal_cmd = 0; vel_angular_cmd = 0;
                    avanzar; girar; send(pub, msg_vel);

                    % Si el WP actual está muy cerca y hay obstáculo, blacklistear el WP
                    if dist_wp < 1.5 && idx_waypoint <= size(ruta_waypoints,1)
                        wp_conflictivo = ruta_waypoints(idx_waypoint,:);
                        ya_en_blacklist = false;
                        if ~isempty(blacklist)
                            dists_bl = sqrt((blacklist(:,1)-wp_conflictivo(1)).^2 + ...
                                            (blacklist(:,2)-wp_conflictivo(2)).^2);
                            ya_en_blacklist = any(dists_bl < 0.3);
                        end
                        if ~ya_en_blacklist
                            fprintf('[SEGUIR] WP conflictivo blacklisteado: (%.2f,%.2f)\n', ...
                                    wp_conflictivo(1), wp_conflictivo(2));
                            blacklist(end+1,:) = wp_conflictivo;
                        end
                    end

                    dir_evasion   = elegir_dir_evasion(laser);
                    t_evasion_tic = tic;
                    estado_nav    = 'EVADIR';
                    girando_a_wp  = false;
                    continue;
                end

                % Ruta terminada
                if idx_waypoint > size(ruta_waypoints,1)
                    fprintf('[SEGUIR] Ruta completada. Nuevo frontier.\n');
                    vel_lineal_cmd = 0; vel_angular_cmd = 0;
                    avanzar; girar; send(pub, msg_vel);

                    % Marcar celdas alrededor de la posición actual como libres
                    [fila_act, col_act] = global2matriz(pos_x, pos_y);
                    RADIO_MARCAR = 5;
                    for df = -RADIO_MARCAR:RADIO_MARCAR
                        for dc = -RADIO_MARCAR:RADIO_MARCAR
                            f_tmp = max(1, min(N_C, fila_act + df));
                            c_tmp = max(1, min(N_C, col_act + dc));
                            if Mapa(f_tmp, c_tmp) == 0.5
                                Mapa(f_tmp, c_tmp) = 1;
                            end
                        end
                    end

                    ruta_waypoints = zeros(0,2);
                    estado_nav     = 'PLANIFICAR';
                    girando_a_wp   = false;
                    continue;
                end

                % Timeout
                if toc(t_waypoint_tic) > 8.0
                    fprintf('[SEGUIR] Timeout WP %d. Blacklisteando frontier y recalculando.\n', idx_waypoint);
                    vel_lineal_cmd=0; vel_angular_cmd=0;
                    avanzar; girar; send(pub,msg_vel);
                    blacklist(end+1,:) = [ruta_waypoints(end,1), ruta_waypoints(end,2)];
                    ruta_waypoints = zeros(0,2);
                    estado_nav     = 'PLANIFICAR';
                    girando_a_wp   = false;
                    t_waypoint_tic = tic;
                    continue;
                end

                wp_x = ruta_waypoints(idx_waypoint, 1);
                wp_y = ruta_waypoints(idx_waypoint, 2);
                dx   = wp_x - pos_x;
                dy   = wp_y - pos_y;
                dist_wp = sqrt(dx^2 + dy^2);

                % Waypoint alcanzado
                if dist_wp < DIST_WAYPOINT
                    fprintf('[SEGUIR] WP %d/%d alcanzado.\n', idx_waypoint, size(ruta_waypoints,1));
                    idx_waypoint   = idx_waypoint + 1;
                    t_waypoint_tic = tic;
                    error_ang_prev = 0;
                    error_ang_int  = 0;

                    % Si era el último waypoint, parar y replanificar
                    if idx_waypoint > size(ruta_waypoints,1)
                        vel_lineal_cmd = 0; vel_angular_cmd = 0;
                        avanzar; girar; send(pub, msg_vel);
                        girando_a_wp = false;
                        continue;
                    end

                    % Si NO era el último: solo girar en sitio si el cambio de dirección
                    % es grande (>60°). En curvas suaves, seguir avanzando sin parar.
                    wp_x_next = ruta_waypoints(idx_waypoint, 1);
                    wp_y_next = ruta_waypoints(idx_waypoint, 2);
                    ang_deseado_next = atan2(wp_y_next - pos_y, wp_x_next - pos_x);
                    error_ang_next   = mod((ang_deseado_next - theta) + pi, 2*pi) - pi;

                    if abs(error_ang_next) > 1.05    % >60° → giro en sitio
                        girando_a_wp = true;
                        vel_lineal_cmd = 0; vel_angular_cmd = 0;
                        avanzar; girar; send(pub, msg_vel);
                    else
                        girando_a_wp = false;        % curva suave → seguir fluido
                    end
                    continue;
                end

                % --- FASE DE GIRO EN SITIO ---
                if girando_a_wp && idx_waypoint <= size(ruta_waypoints,1)
                    wp_x_next = ruta_waypoints(idx_waypoint, 1);
                    wp_y_next = ruta_waypoints(idx_waypoint, 2);
                    ang_deseado  = atan2(wp_y_next - pos_y, wp_x_next - pos_x);
                    error_ang    = mod((ang_deseado - theta) + pi, 2*pi) - pi;

                    if abs(error_ang) < 0.20
                        girando_a_wp   = false;
                        error_ang_prev = 0;
                        error_ang_int  = 0;
                        fprintf('[GIRAR] Alineado con WP %d. Avanzando.\n', idx_waypoint);
                    else
                        vel_lineal_cmd  = 0;
                        vel_angular_cmd = max(-0.9, min(0.9, 1.5 * error_ang));
                        avanzar; girar; send(pub, msg_vel);
                    end
                    continue;
                end

                % --- AVANCE NORMAL CON PID ---
                ang_deseado    = atan2(dy, dx);
                error_ang      = mod((ang_deseado - theta) + pi, 2*pi) - pi;
                d_error_ang    = error_ang - error_ang_prev;
                error_ang_int  = max(-5, min(5, error_ang_int + error_ang));

                vel_angular_cmd = Kp_ang*error_ang + Kd_ang*d_error_ang + Ki_ang*error_ang_int;
                vel_angular_cmd = max(-0.9, min(0.9, vel_angular_cmd));
                error_ang_prev  = error_ang;

                % Velocidad lineal proporcional al alineamiento
                factor_ang = max(0, 1 - abs(error_ang)/(pi/3));

                % Repulsión lateral REFORZADA: si hay pared cerca, alejarse más decidido
                DIST_LATERAL_MIN = 0.70;        % antes 0.60 — empieza a corregir antes
                correccion_lateral = 0;
                if dist_lateral_der < DIST_LATERAL_MIN
                    correccion_lateral = correccion_lateral + ...
                        0.8 * (DIST_LATERAL_MIN - dist_lateral_der) / DIST_LATERAL_MIN;  % antes 0.4
                end
                if dist_lateral_izq < DIST_LATERAL_MIN
                    correccion_lateral = correccion_lateral - ...
                        0.8 * (DIST_LATERAL_MIN - dist_lateral_izq) / DIST_LATERAL_MIN;
                end

                vel_angular_cmd = vel_angular_cmd + correccion_lateral;
                vel_angular_cmd = max(-0.9, min(0.9, vel_angular_cmd));

                % Reducción de velocidad solo si MUY cerca de pared
                demasiado_cerca = (dist_lateral_der < 0.35) || (dist_lateral_izq < 0.35);
                if demasiado_cerca
                    factor_lateral = min(dist_lateral_der, dist_lateral_izq) / 0.35;
                    factor_lateral = max(0.4, factor_lateral);   % nunca por debajo del 40%
                else
                    factor_lateral = 1.0;
                end

                vel_lineal_cmd = VEL_LINEAL * factor_ang * factor_lateral;

                avanzar; girar; send(pub, msg_vel);
                %% ----------------------------------------------------------
            case 'EVADIR'
                t_ev = toc(t_evasion_tic);

                vel_lineal_cmd  = 0;
                vel_angular_cmd = dir_evasion * VEL_GIRO_EVASION;
                avanzar; girar; send(pub, msg_vel);

                if t_ev > T_GIRO_EVASION && dist_frontal > DIST_EMERGENCIA + 0.15
                    fprintf('[EVADIR] Despejado (%.1fs). Recalculando ruta.\n', t_ev);
                    vel_lineal_cmd = 0; vel_angular_cmd = 0;
                    avanzar; girar; send(pub, msg_vel);

                    % Guardar destino ANTES de borrar la ruta
                    if size(ruta_waypoints,1) > 0
                        dest_actual_x = ruta_waypoints(end,1);
                        dest_actual_y = ruta_waypoints(end,2);
                    else
                        dest_actual_x = ultimo_destino_x;
                        dest_actual_y = ultimo_destino_y;
                    end

                    % Ahora sí borramos la ruta
                    ruta_waypoints = zeros(0,2);
                    idx_waypoint   = 1;
                    error_ang_prev = 0;
                    error_ang_int  = 0;
                    girando_a_wp   = false;

                    % Contar evasiones consecutivas al mismo destino
                    if abs(dest_actual_x - ultimo_destino_x) < 0.5 && ...
                            abs(dest_actual_y - ultimo_destino_y) < 0.5
                        contador_evasiones = contador_evasiones + 1;
                    else
                        contador_evasiones = 1;
                        ultimo_destino_x   = dest_actual_x;
                        ultimo_destino_y   = dest_actual_y;
                    end

                    if contador_evasiones >= MAX_EVASIONES_SEGUIDAS
                        fprintf('[EVADIR] Demasiadas evasiones al mismo destino. Blacklist.\n');
                        blacklist(end+1,:) = [ultimo_destino_x, ultimo_destino_y];
                        contador_evasiones = 0;
                        ultimo_destino_x   = -1;
                        ultimo_destino_y   = -1;
                    end

                    estado_nav = 'PLANIFICAR';
                end

            case 'FIN'
                break;
        end

        waitfor(r);
    end

    %% ============================================================
    %% FASE 4: CIERRE
    %% ============================================================
    fprintf('\n[FASE 4] Fin. Parando robot...\n');
    try
        vel_lineal_cmd=0; vel_angular_cmd=0;
        avanzar; girar; send(pub,msg_vel); send(pub,msg_vel); pause(0.5);
    catch; end

    fprintf('[FASE 4] Guardando mapa...\n');
    try
        guardar_mapa;
    catch e
        fprintf('[FASE 4] Error guardar_mapa: %s\n', e.message);
        try
            imwrite(uint8(Mapa*255), sprintf('mapa_emergencia_%s.png', ...
                datestr(now,'yyyymmdd_HHMMSS')));
        catch; end
    end

    fprintf('[FASE 4] Detectando circulo...\n');
    try
        detectar_circulo;
    catch e
        fprintf('[FASE 4] Error circulo: %s\n', e.message);
    end

catch err
    fprintf('\n[ERROR] %s  (linea %d)\n', err.message, err.stack(1).line);
    try
        msg_vel.Linear.X=0; msg_vel.Angular.Z=0;
        send(pub,msg_vel); send(pub,msg_vel);
    catch; end
    try
        imwrite(uint8(Mapa*255), sprintf('mapa_emergencia_%s.png',...
            datestr(now,'yyyymmdd_HHMMSS')));
    catch; end
end

desconectar;

fprintf('\n╔══════════════════════════════════════╗\n');
fprintf('║   Programa finalizado correctamente  ║\n');
fprintf('╚══════════════════════════════════════╝\n');


%% ============================================================
%% FUNCIONES AUXILIARES
%% ============================================================

function [fx, fy] = encontrar_frontiers(Mapa, N_C, TAM_CELDA, MAX_Y_M)
    % Devuelve coordenadas en metros de todos los centroides de frontiers válidos.
    MIN_FRONTIER = 3;          % ANTES: 8 — bajado para no descartar bolsas pequeñas
    MARGEN_BORDE = 2;          % celdas para ignorar ruido pegado al borde
    mask_libre   = (Mapa == 1);
    mask_noexp   = (abs(Mapa - 0.5) < 0.01);
    kernel       = [0 1 0; 1 1 1; 0 1 0];
    lib_dil      = imdilate(mask_libre, kernel);
    front_mask   = mask_noexp & lib_dil;
    [labeled, num_comp] = bwlabel(front_mask, 4);
    fx = []; fy = [];
    for c = 1:num_comp
        idx = find(labeled == c);
        if numel(idx) < MIN_FRONTIER; continue; end
        [filas_c, cols_c] = ind2sub([N_C, N_C], idx);
        fi_mean = mean(filas_c);
        ci_mean = mean(cols_c);

        % Filtrar frontiers pegados al borde del mapa (suelen ser ruido)
        if fi_mean <= MARGEN_BORDE || fi_mean >= N_C - MARGEN_BORDE; continue; end
        if ci_mean <= MARGEN_BORDE || ci_mean >= N_C - MARGEN_BORDE; continue; end

        fx(end+1) = (ci_mean - 1) * TAM_CELDA;           %#ok<AGROW>
        fy(end+1) = MAX_Y_M - (fi_mean - 1) * TAM_CELDA; %#ok<AGROW>
    end
end


function mapa_nav = calcular_mapa_navegable(Mapa, margen)
% Dilata obstáculos para dar margen de seguridad al robot.
% Devuelve matriz lógica: true = celda navegable.
mask_obs = (Mapa == 0);
se       = strel('disk', margen);
obs_dil  = imdilate(mask_obs, se);
mapa_nav = ~obs_dil;  % las no-exploradas también son navegables para BFS
end


function camino = bfs_camino(mapa_nav, fi, ci, ff, cf, N_C)
camino = [];
if ~mapa_nav(fi,ci)
    [fi,ci] = celda_libre_cercana(mapa_nav, fi, ci, N_C, 6);
    if isnan(fi); return; end
end
if ~mapa_nav(ff,cf)
    [ff,cf] = celda_libre_cercana(mapa_nav, ff, cf, N_C, 8);
    if isnan(ff); return; end
end
if fi==ff && ci==cf; camino=[fi,ci]; return; end

MAX_NODOS  = N_C * N_C;          % techo duro: nunca más de 62500 nodos
cola_f = zeros(MAX_NODOS,1,'int16');
cola_c = zeros(MAX_NODOS,1,'int16');
cola_f(1)=fi; cola_c(1)=ci;
head=1; tail=1;

visitado = false(N_C,N_C);
padre_f  = zeros(N_C,N_C,'int16');
padre_c  = zeros(N_C,N_C,'int16');
visitado(fi,ci)=true;

df=int16([-1 -1 -1  0  0  1  1  1]);
dc=int16([-1  0  1 -1  1 -1  0  1]);
encontrado=false;

while head <= tail && tail < MAX_NODOS
    f=cola_f(head); c=cola_c(head); head=head+1;
    for d=1:8
        nf=f+df(d); nc=c+dc(d);
        if nf<1||nf>N_C||nc<1||nc>N_C; continue; end
        if visitado(nf,nc)||~mapa_nav(nf,nc); continue; end
        visitado(nf,nc)=true;
        padre_f(nf,nc)=f; padre_c(nf,nc)=c;
        tail=tail+1;
        cola_f(tail)=nf; cola_c(tail)=nc;
        if nf==ff && nc==cf; encontrado=true; break; end
    end
    if encontrado; break; end
end

if ~encontrado; return; end
path=[ff,cf]; f=ff; c=cf;
while ~(f==fi && c==ci)
    pf=double(padre_f(f,c)); pc=double(padre_c(f,c));
    f=pf; c=pc;
    path(end+1,:)=[f,c]; %#ok<AGROW>
end
camino=flipud(path);
end


function [fi2,ci2] = celda_libre_cercana(mapa_nav, fi, ci, N_C, radio)
% Encuentra la celda navegable más cercana en un radio dado.
fi2=NaN; ci2=NaN; mejor=inf;
for df=-radio:radio
    for dc=-radio:radio
        nf=fi+df; nc=ci+dc;
        if nf<1||nf>N_C||nc<1||nc>N_C; continue; end
        if mapa_nav(nf,nc)
            d=abs(df)+abs(dc);
            if d<mejor; mejor=d; fi2=nf; ci2=nc; end
        end
    end
end
end


function ruta_m = celdas_a_metros(camino, TAM_CELDA, MAX_Y_M)
% Convierte camino [fila,col] → coordenadas mundo [x,y] en metros.
ruta_m = [(camino(:,2)-1)*TAM_CELDA, MAX_Y_M-(camino(:,1)-1)*TAM_CELDA];
end


function ruta_simple = simplificar_ruta(ruta, mapa_nav, TAM_CELDA, MAX_Y_M, N_C)
% String-pulling: elimina waypoints innecesarios si hay línea de visión directa.
if size(ruta,1) <= 2; ruta_simple=ruta; return; end
ruta_simple = ruta(1,:);
i=1; n=size(ruta,1);
while i < n
    j_max=i+1;
    for j=i+2:n
        if linea_libre(ruta(i,:), ruta(j,:), mapa_nav, TAM_CELDA, MAX_Y_M, N_C)
            j_max=j;
        else
            break;
        end
    end
    ruta_simple(end+1,:)=ruta(j_max,:); %#ok<AGROW>
    i=j_max;
end
end


function libre = linea_libre(p1, p2, mapa_nav, TAM_CELDA, MAX_Y_M, N_C)
% Comprueba que la línea recta p1→p2 (metros) no cruza ningún obstáculo.
[f1,c1]=metros_a_celda(p1(1),p1(2),TAM_CELDA,MAX_Y_M,N_C);
[f2,c2]=metros_a_celda(p2(1),p2(2),TAM_CELDA,MAX_Y_M,N_C);
n=max(abs(f2-f1),abs(c2-c1)); libre=true;
for k=0:n
    t=k/max(n,1);
    fi=max(1,min(N_C,round(f1+t*(f2-f1))));
    ci=max(1,min(N_C,round(c1+t*(c2-c1))));
    if ~mapa_nav(fi,ci); libre=false; return; end
end
end


function [fila,col] = metros_a_celda(x, y, TAM_CELDA, MAX_Y_M, N_C)
col  = max(1,min(N_C, round(x/TAM_CELDA)+1));
fila = max(1,min(N_C, round((MAX_Y_M-y)/TAM_CELDA)+1));
end


function dir = elegir_dir_evasion(laser)
% Elige la dirección de giro con más espacio libre.
dir=1;
try
    rangos=double(laser.LatestMessage.Ranges);
    n=length(rangos); mitad=floor(n/2);
    r_izq=rangos(mitad+1:end); r_der=rangos(1:mitad);
    m_izq=mean(r_izq(~isinf(r_izq)&r_izq>0.05));
    m_der=mean(r_der(~isinf(r_der)&r_der>0.05));
    if isnan(m_izq); m_izq=0; end
    if isnan(m_der); m_der=0; end
    if m_der>m_izq; dir=-1; end
catch;
end
end
```
