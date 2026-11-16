%% ============================================================
%% GUARDAR_MAPA.M
%% Guardado del mapa final de ocupación
%% ============================================================

%% ============================================================
%% CONVERSIÓN DE LA MATRIZ A IMAGEN
%% ============================================================

% Conversión del mapa de ocupación a formato uint8.
% Valores:
% 0     → negro (obstáculo)
% 0.5   → gris (desconocido)
% 1     → blanco (libre)
mapa_img = uint8(Mapa * 255);

%% ============================================================
%% GENERAR NOMBRE ÚNICO
%% ============================================================

% Timestamp para evitar sobrescribir archivos.
timestamp = datestr(now, 'yyyymmdd_HHMMSS');

% Nombre del archivo PNG.
nombre_png = sprintf('mapa_explorado_%s.png', timestamp);

%% ============================================================
%% GUARDAR IMAGEN
%% ============================================================

% Escritura del mapa en disco.
imwrite(mapa_img, nombre_png);

fprintf('[Guardar] Mapa guardado como: %s\n', nombre_png);

%% ============================================================
%% MOSTRAR MAPA FINAL
%% ============================================================

figure('Name','Mapa Final');

imshow(mapa_img);

colormap(gray);

title(sprintf('Mapa Final — %s', ...
    strrep(timestamp,'_',' ')));
