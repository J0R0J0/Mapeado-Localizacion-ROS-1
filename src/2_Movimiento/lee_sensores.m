%% ============================================================
%% LEE_SENSORES.M
%% Lectura y procesamiento de datos del sensor láser
%% ============================================================

%% ============================================================
%% OBTENER MENSAJE DEL LÁSER
%% ============================================================

% Se obtiene el último mensaje publicado por el LiDAR del robot.
msg_laser_nav = laser.LatestMessage;

%% ============================================================
%% COMPROBAR QUE EXISTEN DATOS VÁLIDOS
%% ============================================================

if ~isempty(msg_laser_nav)

    %% ========================================================
    %% EXTRAER DATOS DEL LÁSER
    %% ========================================================

    % Distancias medidas por el sensor láser.
    rangos_nav = msg_laser_nav.Ranges;

    % Ángulo mínimo del barrido láser.
    ang_min_nav = msg_laser_nav.AngleMin;

    % Incremento angular entre medidas consecutivas.
    ang_inc_nav = msg_laser_nav.AngleIncrement;

    % Vector completo de ángulos asociados a cada medida.
    angulos_nav = ang_min_nav + ...
        (0:length(rangos_nav)-1) * ang_inc_nav;

    %% ========================================================
    %% SECTOR FRONTAL
    %% Detección de obstáculos delante del robot
    %% ========================================================

    % Se seleccionan únicamente las medidas comprendidas
    % en un cono frontal de ±0.35 radianes.
    idx_frontales = find(angulos_nav > -0.35 & ...
                            angulos_nav < 0.35);

    if ~isempty(idx_frontales)
        % Extraemos las distancias frontales.
        r_f = rangos_nav(idx_frontales);

        % Eliminamos:
        % - infinitos
        % - medidas inválidas muy pequeñas
        r_f = r_f(~isinf(r_f) & r_f > 0.05);

        if ~isempty(r_f)
            % Distancia frontal mínima detectada.
            dist_frontal = min(r_f);
        else
            % Si no hay medidas válidas:
            dist_frontal = inf;
        end
    else
        dist_frontal = inf;
    end

    %% ========================================================
    %% SECTOR PARED DERECHA
    %% Seguimiento de pared lateral
    %% ========================================================

    % Centro angular perpendicular a la derecha.
    ang_centro = -pi/2;

    % Margen angular del sector.
    margen = 0.20;

    % Selección de medidas del lateral derecho.
    idx_derecha = find(angulos_nav > (ang_centro - margen) & ...
                        angulos_nav < (ang_centro + margen));

    if ~isempty(idx_derecha)
        % Extraemos distancias del lateral derecho.
        r_d = rangos_nav(idx_derecha);

        % Eliminamos:
        % - infinitos
        % - distancias inválidas
        % - valores excesivamente grandes
        r_d = r_d(~isinf(r_d) & ...
                    r_d > 0.05 & ...
                    r_d < 6.0);

        if ~isempty(r_d)
            % Se usa la mediana para reducir ruido y oscilaciones.
            dist_pared = median(r_d);
        else
            % Si no hay pared detectada.
            dist_pared = 3.0;
        end
    else
        dist_pared = 3.0;
    end
else
    %% ========================================================
    %% SI NO HAY MENSAJE DEL LÁSER
    %% ========================================================

    dist_frontal = inf;
    dist_pared   = 3.0;
end

%% ============================================================
%% SECTOR LATERAL IZQUIERDO
%% ============================================================

% Centro angular perpendicular a la izquierda.
ang_centro_izq = pi/2;

% Selección de medidas laterales izquierdas.
idx_izq = find(angulos_nav > (ang_centro_izq - 0.20) & ...
                angulos_nav < (ang_centro_izq + 0.20));

if ~isempty(idx_izq)
    % Extraemos distancias izquierdas.
    r_i = rangos_nav(idx_izq);

    % Filtrado de medidas inválidas.
    r_i = r_i(~isinf(r_i) & ...
                r_i > 0.05 & ...
                r_i < 6.0);

    % Distancia lateral izquierda.
    dist_lateral_izq = median(r_i);

    % Control de posibles NaN.
    if isnan(dist_lateral_izq)
        dist_lateral_izq = 3.0;
    end
else
    dist_lateral_izq = 3.0;
end

%% ============================================================
%% SECTOR LATERAL DERECHO
%% ============================================================

% La distancia lateral derecha reutiliza la distancia calculada
% previamente para el seguimiento de pared.
dist_lateral_der = dist_pared;
