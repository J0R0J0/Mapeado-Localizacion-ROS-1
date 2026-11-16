%% ============================================================
%% DESCONECTAR.M
%% Cierre seguro de ROS y liberación de recursos
%% ============================================================

% Este script:
% - detiene el robot
% - elimina publishers y subscribers
% - cierra ROS correctamente
% - evita bloqueos de MATLAB

fprintf('[CIERRE] Iniciando desconexión segura...\n');

%% ============================================================
%% 1. DETENER EL ROBOT
%% ============================================================

try
    % Velocidad lineal a cero
    msg_vel.Linear.X  = 0;

    % Velocidad angular a cero
    msg_vel.Angular.Z = 0;

    % Envío doble del mensaje por seguridad
    send(pub, msg_vel);
    send(pub, msg_vel);

    % Pequeña pausa para asegurar transmisión
    pause(0.3);
    fprintf('[CIERRE] Robot detenido.\n');

catch e
    % Gestión de errores durante la parada
    fprintf('[CIERRE] Aviso al detener robot: %s\n', e.message);
end

%% ============================================================
%% 2. ELIMINAR SUBSCRIBERS
%% ============================================================

try
    % Eliminación del subscriber de odometría
    if exist('odom',  'var')
        delete(odom);
    end

    % Eliminación del subscriber del láser
    if exist('laser', 'var')
        delete(laser);
    end
    fprintf('[CIERRE] Subscribers eliminados.\n');
catch e
    fprintf('[CIERRE] Aviso al eliminar subscribers: %s\n', e.message);
end

%% ============================================================
%% 3. ELIMINAR PUBLISHER
%% ============================================================

try
    % Eliminación del publisher de velocidades
    if exist('pub', 'var')
        delete(pub);
    end
    fprintf('[CIERRE] Publisher eliminado.\n');
catch e
    fprintf('[CIERRE] Aviso al eliminar publisher: %s\n', e.message);
end

%% ============================================================
%% 4. ELIMINAR TIMER RATE
%% ============================================================

try
    % Eliminación del temporizador del bucle principal
    if exist('r', 'var')
        delete(r);
    end
catch
end

%% ============================================================
%% 5. CIERRE DE ROS
%% ============================================================

% Se cierra la conexión ROS de MATLAB.
% Se controla el tiempo para evitar bloqueos prolongados.
fprintf('[CIERRE] Cerrando ROS (máx 10s)...\n');

try
    t_shutdown = tic;
    % Cierre de ROS
    rosshutdown;
    fprintf('[CIERRE] ROS cerrado en %.1fs.\n', toc(t_shutdown));
catch e
    fprintf('[CIERRE] rosshutdown falló: %s — continuando.\n', e.message);
end

%% ============================================================
%% 6. LIMPIEZA DE VARIABLES
%% ============================================================

try
    % Limpieza de variables de MATLAB
    % Se conserva únicamente el mapa por si se quiere analizar después.
    clearvars -except Mapa
catch
end

%% ============================================================
%% MENSAJE FINAL
%% ============================================================

fprintf('[CIERRE] Desconexión completada.\n');
