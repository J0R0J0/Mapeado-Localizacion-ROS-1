%% ============================================================
%% GIRAR.M
%% Asignación de velocidad angular al robot
%% ============================================================

% Este archivo actualiza la componente angular del mensaje ROS
% utilizado para controlar el giro del robot.

% Velocidad angular sobre el eje Z:
% - valores positivos → giro antihorario (izquierda)
% - valores negativos → giro horario (derecha)
% - valor cero        → sin rotación
msg_vel.Angular.Z = vel_angular_cmd;