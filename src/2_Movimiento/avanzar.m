%% ============================================================
%% AVANZAR.M
%% Asignación de velocidad lineal al robot
%% ============================================================

% Este archivo actualiza la componente lineal del mensaje ROS
% que posteriormente será enviado al robot mediante el publisher.

% Velocidad lineal en el eje X del robot:
% - valores positivos → avance
% - valores negativos → retroceso
% - valor cero        → robot detenido
msg_vel.Linear.X = vel_lineal_cmd;
