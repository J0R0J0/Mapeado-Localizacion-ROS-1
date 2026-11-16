%% ============================================================
%% ACTUALIZAR_MAPA.M
%% Actualización del mapa de ocupación mediante ray tracing
%% ============================================================

%% ============================================================
%% 1. EXTRAER POSICIÓN Y ORIENTACIÓN DEL ROBOT
%% ============================================================

% Posición actual del robot en coordenadas globales.
pos_x = odom.LatestMessage.Pose.Pose.Position.X;
pos_y = odom.LatestMessage.Pose.Pose.Position.Y;

% Extracción del cuaternión de orientación.
quaternion = [ ...
    odom.LatestMessage.Pose.Pose.Orientation.W, ...
    odom.LatestMessage.Pose.Pose.Orientation.X, ...
    odom.LatestMessage.Pose.Pose.Orientation.Y, ...
    odom.LatestMessage.Pose.Pose.Orientation.Z];

% Conversión del cuaternión a ángulos de Euler.
euler = quat2eul(quaternion, 'ZYX');

% Orientación del robot en el plano XY.
theta_amigobot = euler(1);

%% ============================================================
%% 2. EXTRAER DATOS DEL SENSOR LÁSER
%% ============================================================

% Último mensaje del LiDAR.
msg_laser = laser.LatestMessage;

% Si no existe mensaje válido, salir.
if isempty(msg_laser)
    return;
end

% Distancias medidas por el láser.
rangos = double(msg_laser.Ranges);

% Ángulo mínimo del escaneo.
ang_min = double(msg_laser.AngleMin);

% Incremento angular entre rayos.
ang_inc = double(msg_laser.AngleIncrement);

% Rango máximo y mínimo del sensor.
rango_max = double(msg_laser.RangeMax);
rango_min = double(msg_laser.RangeMin);

% Número total de rayos.
N_rayos = length(rangos);

%% ============================================================
%% 3. SUBMUESTREO DEL LÁSER
%% ============================================================

% Se usa un rayo de cada dos para reducir carga computacional.
paso_submuestreo = 2;

% Índices de los rayos seleccionados.
indices_rayo = 1:paso_submuestreo:N_rayos;

%% ============================================================
%% 4. PARÁMETROS DEL MAPA
%% ============================================================

% Tamaño físico de cada celda.
TAM_CELDA = 0.06;

% Dimensión máxima del mapa en Y.
MAX_Y = 15.0;

% Número total de celdas por lado.
N_CELDAS = 250;

%% ============================================================
%% 5. RAY TRACING
%% ============================================================

% Para cada rayo del láser:
for k = indices_rayo
    % Distancia detectada por el rayo actual.
    distancia = rangos(k);

    %% ========================================================
    %% FILTRADO DE MEDIDAS INVÁLIDAS
    %% ========================================================

    % Ignorar:
    % - lecturas menores que el rango mínimo
    % - ruido extremadamente cercano
    if distancia < rango_min || distancia < 0.08
        continue;
    end

    %% ========================================================
    %% ÁNGULO GLOBAL DEL RAYO
    %% ========================================================

    % Ángulo absoluto del rayo en el entorno.
    angulo_rayo = theta_amigobot + ...
                    ang_min + ...
                    (k - 1) * ang_inc;

    % Precálculo trigonométrico.
    cos_a = cos(angulo_rayo);
    sin_a = sin(angulo_rayo);

    %% ========================================================
    %% DETERMINAR SI EXISTE IMPACTO
    %% ========================================================

    % Si la lectura es infinita o supera el rango máximo,
    % se considera espacio libre.
    if isinf(distancia) || distancia > rango_max
        dist_libre = rango_max;
        hay_impacto = false;
    else
        dist_libre = distancia;
        hay_impacto = true;
    end

    %% ========================================================
    %% PINTAR CELDAS LIBRES
    %% ========================================================

    if dist_libre > TAM_CELDA
        % Se deja un pequeño margen antes del obstáculo
        % para no borrar paredes reales.
        dist_pintar = dist_libre - TAM_CELDA;

        % Distancias intermedias a lo largo del rayo.
        pasos_d = (0 : TAM_CELDA : dist_pintar);

        % Coordenadas físicas del recorrido del rayo.
        px_vec = pos_x + pasos_d * cos_a;
        py_vec = pos_y + pasos_d * sin_a;

        % Conversión a coordenadas de matriz.
        cols = round(px_vec / TAM_CELDA) + 1;
        fils = round((MAX_Y - py_vec) / TAM_CELDA) + 1;

        % Saturación de índices.
        cols = max(1, min(N_CELDAS, cols));
        fils = max(1, min(N_CELDAS, fils));

        % Conversión a índices lineales.
        idx_lin = unique(fils + (cols - 1) * N_CELDAS);

        % Solo se actualizan las celdas que no sean obstáculos.
        mask_libres = Mapa(idx_lin) ~= 0;

        % Marcado de espacio libre.
        Mapa(idx_lin(mask_libres)) = 1;
    end

    %% ========================================================
    %% PINTAR CELDA OCUPADA
    %% ========================================================

    if hay_impacto
        % Coordenada física del impacto.
        px_f = pos_x + dist_libre * cos_a;
        py_f = pos_y + dist_libre * sin_a;

        % Conversión a coordenadas de matriz.
        c_f = max(1, min(N_CELDAS, ...
                round(px_f / TAM_CELDA) + 1));

        f_f = max(1, min(N_CELDAS, ...
                round((MAX_Y - py_f) / TAM_CELDA) + 1));

        % Marcado del obstáculo.
        Mapa(f_f, c_f) = 0;
    end
end