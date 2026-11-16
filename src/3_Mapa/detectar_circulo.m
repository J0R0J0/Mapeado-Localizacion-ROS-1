%% ============================================================
%% DETECTAR_CIRCULO.M
%% Detección de objetos circulares mediante Transformada de Hough
%% ============================================================

% Este script detecta obstáculos circulares presentes en el mapa
% de ocupación generado durante la exploración.

%% ============================================================
%% PARÁMETROS DEL MAPA
%% ============================================================

TAM_CELDA = 0.06;
MAX_Y = 15.0;

%% ============================================================
%% 1. CREAR IMAGEN BINARIA DE OBSTÁCULOS
%% ============================================================

% Las celdas ocupadas (valor 0) se convierten en píxeles blancos.
img_ocupadas = uint8((Mapa == 0)) * 255;

%% ============================================================
%% 2. DEFINIR RANGO DE RADIOS
%% ============================================================

% Radio mínimo en celdas.
r_min_celdas = max(2, round(0.05 / TAM_CELDA));

% Radio máximo en celdas.
r_max_celdas = round(0.60 / TAM_CELDA);

%% ============================================================
%% 3. TRANSFORMADA DE HOUGH CIRCULAR
%% ============================================================

% Búsqueda de círculos brillantes.
[centers1, radii1, strengths1] = imfindcircles( ...
    img_ocupadas, ...
    [r_min_celdas, r_max_celdas], ...
    'ObjectPolarity', 'bright', ...
    'Sensitivity', 0.88, ...
    'EdgeThreshold', 0.08);

% Si no se detectan círculos brillantes,
% probar polaridad inversa.
if isempty(centers1)
    [centers1, radii1, strengths1] = imfindcircles( ...
        img_ocupadas, ...
        [r_min_celdas, r_max_celdas], ...
        'ObjectPolarity', 'dark', ...
        'Sensitivity', 0.88, ...
        'EdgeThreshold', 0.08);
end

%% ============================================================
%% 4. SELECCIONAR EL MEJOR CÍRCULO
%% ============================================================

if isempty(centers1)
    % No se ha encontrado ningún círculo.
    circulo_encontrado = false;

    circulo_centro_x = 0;
    circulo_centro_y = 0;
    circulo_radio_m = 0;

    disp('[Círculo] No se detectó ningún objeto circular.');
else
    circulo_encontrado = true;

    % Selección del círculo con mayor fuerza.
    [~, idx_mejor] = max(strengths1);

    % Coordenadas del centro.
    col_celda = centers1(idx_mejor,1);
    fila_celda = centers1(idx_mejor,2);

    % Radio en celdas.
    radio_celdas = radii1(idx_mejor);

    %% ========================================================
    %% CONVERSIÓN A COORDENADAS REALES
    %% ========================================================

    circulo_centro_x = (col_celda - 1) * TAM_CELDA;

    circulo_centro_y = MAX_Y - ...
                       (fila_celda - 1) * TAM_CELDA;

    circulo_radio_m = radio_celdas * TAM_CELDA;

    %% ========================================================
    %% MOSTRAR RESULTADOS
    %% ========================================================

    fprintf('\n=== OBJETO CIRCULAR DETECTADO ===\n');

    fprintf('  Centro X : %.3f m\n', circulo_centro_x);
    fprintf('  Centro Y : %.3f m\n', circulo_centro_y);
    fprintf('  Radio    : %.3f m\n', circulo_radio_m);
    fprintf('  Fuerza   : %.3f\n', strengths1(idx_mejor));

    fprintf('=================================\n\n');
end