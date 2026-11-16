function [fila, columna] = global2matriz(X, Y)
    % Función que convierte coordenadas globales (X,Y) en metros
    % a índices de fila y columna en la matriz del mapa.

    %% Constantes del mapa
    tam_celda = 0.06;
    max_y = 15.0;

    %% EJE X A COLUMNAS:
    % Simplemente dividimos la coordenada X entre el tamaño de la celda.
    % Sumamos 1 porque los índices en Matlab empiezan en 1, no en 0.
    columna = round(X / tam_celda) + 1;

    %% EJE Y A FILAS (INVERSIÓN):
    % Como el origen del mapa global está abajo y el de la imagen arriba,
    % debemos restar la coordenada Y actual al máximo total (15 metros)
    % antes de dividir por el tamaño de celda.
    fila = round((max_y - Y) / tam_celda) + 1;

    %% LÍMITES DE SEGURIDAD:
    % Evitamos que un error de lectura intente acceder a un índice 
    % fuera de nuestra matriz de 250x250
    if fila < 1; fila = 1; end
    if fila > 250; fila = 250; end
    if columna < 1; columna = 1; end
    if columna > 250; columna = 250; end
end
