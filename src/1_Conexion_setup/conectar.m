%% ============================================================
%% CONECTAR.M
%% Conexión entre MATLAB y el sistema ROS del simulador
%% ============================================================

%% Configuración de la dirección del ROS MASTER
% Se indica la IP y el puerto del sistema ROS que está ejecutándose
% en la máquina virtual Ubuntu. El puerto estándar de ROS es el 11311.
setenv('ROS_MASTER_URI','http://192.168.1.144:11311');

%% Configuración de la IP local de MATLAB
% Se establece la IP del dispositivo que ejecuta MATLAB para que
% ROS pueda intercambiar mensajes correctamente.
setenv('ROS_IP','192.168.1.144');

%% Inicialización de la conexión ROS
% MATLAB se conecta al ROS MASTER especificado anteriormente.
rosinit();

%% Mensaje informativo
% Indica por consola que la conexión se ha realizado correctamente.
disp('Conexión con ROS establecida correctamente.');
