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
    RADIO_BLACKLIST = 0.80;   % Radio para ignorar frontiers cerca de uno fallido (m)
    RADIO_DUPLICADO = 0.30;   % Radio para detectar waypoints duplicados en blacklist (m)

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
            vel_lineal_cmd = -0.25;
            vel_angular_cmd = 0;
            avanzar; girar; send(pub,msg_vel);
            pause(1.5);

            % Girar fuerte
            vel_lineal_cmd = 0;
            vel_angular_cmd = 0.8;
            avanzar; girar; send(pub,msg_vel);
            pause(2.0);

            % Parar
            vel_lineal_cmd = 0;
            vel_angular_cmd = 0;
            avanzar; girar; send(pub,msg_vel);

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
                            ya_en_blacklist = any(dists_bl < RADIO_DUPLICADO);
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
    catch
    end

    fprintf('[FASE 4] Guardando mapa...\n');
    try
        guardar_mapa;
    catch e
        fprintf('[FASE 4] Error guardar_mapa: %s\n', e.message);
        try
            imwrite(uint8(Mapa*255), sprintf('mapa_emergencia_%s.png', ...
                datestr(now,'yyyymmdd_HHMMSS')));
        catch
        end
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
    catch; 
    end
    try
        imwrite(uint8(Mapa*255), sprintf('mapa_emergencia_%s.png',...
            datestr(now,'yyyymmdd_HHMMSS')));
    catch
    end
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
    if size(ruta,1) <= 2
        ruta_simple=ruta; 
        return; 
    end
    
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
        if ~mapa_nav(fi,ci) 
            libre=false; 
            return; 
        end
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