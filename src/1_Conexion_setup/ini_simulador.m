%% ============================================================
%% DECLARACIÓN DE PUBLISHERS Y MENSAJES
%% ============================================================

% Publisher para enviar velocidades al robot
% El topic /cmd_vel controla velocidades lineales y angulares.
pub = rospublisher('/robot0/cmd_vel','geometry_msgs/Twist');

% Creación del mensaje ROS asociado al publisher
msg_vel = rosmessage(pub);

%% ============================================================
%% CREACIÓN DE LA MATRIZ DEL MAPA
%% ============================================================

% El entorno del simulador tiene dimensiones máximas de 15x15 metros.

% Tamaño físico de cada celda del mapa:
% 6 cm = 0.06 metros.
% Número de celdas por lado:
% 15 / 0.06 = 250 celdas.
celdas_lado = 15 / 0.06;

% Inicialización del mapa:
% 0     → obstáculo
% 0.5   → zona no explorada
% 1     → zona libre
%
% Inicialmente todo el mapa es desconocido (0.5).
Mapa = ones(celdas_lado, celdas_lado) * 0.5;

%% ============================================================
%% DEFINICIÓN DE TIMER Y FRECUENCIA DEL BUCLE
%% ============================================================

% Se establece una frecuencia de control de 10 Hz.
% Esto significa que el bucle principal se ejecutará cada 0.1 segundos.
r = robotics.Rate(10);

% Espera para sincronizar correctamente el timer.
waitfor(r);

% Pequeña pausa adicional para asegurar la recepción de mensajes ROS.
pause(3);

%% ============================================================
%% VARIABLES INICIALES DE VELOCIDAD
%% ============================================================

% Velocidad lineal inicial del robot.
vel_lineal_cmd = 0.0;

% Velocidad angular inicial del robot.
vel_angular_cmd = 0.0;

%% ============================================================
%% VERIFICACIÓN DE LECTURA DE ODOMETRÍA
%% ============================================================

% Antes de arrancar el programa se comprueba que el simulador
% está enviando datos válidos de odometría.

disp('Esperando lectura de odometría...');

% El programa queda esperando hasta recibir correctamente
% el identificador del robot desde ROS.
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
    % Actualización continua del mensaje de odometría.
    odom.LatestMessage;
end

%% ============================================================
%% MENSAJE FINAL DE INICIALIZACIÓN
%% ============================================================

disp('Robot inicializado. Matriz Mapa 250x250 creada y lista.');
