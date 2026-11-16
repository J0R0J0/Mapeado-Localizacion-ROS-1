Percepción y Control
Grado en Ingeniería de Computadores

TRABAJO FINAL DE LA ASIGNATURA

El   objetivo   de   este   trabajo   final   de   asignatura   es   aplicar   los   conocimientos   adquiridos   sobre
percepción   y   control   a   lo   largo   de   la   asignatura,   usando   para   ello   la   plataforma   de   desarrollo
Matlab/ROS.

Introducción

Se   plantea   realizar   la   exploración   y   mapeado   de   un   entorno   utilizando   la   plataforma   robótica
Amigobot utilizada en el desarrollo de prácticas de laboratorio. El entorno en el que se encuentra el
robot  Amigobot   es   de   dimensiones   máximas   conocidas   (15x15   metros),   pero   se   desconoce   la
posición de los obstáculos y paredes que lo forman. El robot Amigobot dispone de sensores de
distancia   ultrasónicos,   sensores   de   distancia   de   tipo   LIDAR,   y   un   sistema   de   posicionamiento
mediante odometría.

Figura 1: Exploración del entorno y generación de mapa de ocupación.

Objetivos

El objetivo principal de este trabajo es la obtención del mapa de ocupación del entorno en el que se
encuentra   el   robot   a   partir   de   los   sensores   sonar   y/o   láser   y   el   sistema   de   posicionamiento
disponibles en la plataforma Amigobot (Ver Figura 1). Como objetivo secundario se pide identificar
automáticamente en el mapa un objeto de contorno circular y proporcionar las coordenadas del
centro y su radio.

Para ello se propone el desarrollo de una aplicación que permita realizar las siguientes tareas:

• Navegación del entorno. Se deberá definir una estrategia para que el robot navegue por el
entorno a partir de la información de sus sensores y sin colisionar con paredes ni obstáculos
existentes.

• Creación de un mapa de ocupación del entorno explorado. El entorno en el que se mueve
el robot se define como un mapa de ocupación formado por celdas rectangulares de 6cm x
6cm. Dichas celdas tendrán tres posibles estados: Ocupada (0), Libre (1) y No explorada
(0.5).   El estado de cada celda del entorno se asignará en función de las lecturas de los
sensores a bordo del robot y del movimiento del mismo.

• Exploración   completa   del   entorno.  Se   debe   garantizar   que   el   entorno   es   explorado
completamente (salvo zonas no accesibles), guiando al robot hacia zonas no exploradas
mediante el análisis del mapa de ocupación obtenido hasta ese momento.

• Evasión de los obstáculos durante la navegación: como se ha mencionado anteriormente,
el robot no debe chocar con los obstáculos o paredes. Si el robot se envía a una posición
para la exploración del mapa, este debe esquivar los obstáculos que encuentre en su camino
utilizando para ello las lecturas de los sensores y el mapa de ocupación obtenido hasta ese
momento.

•

Identificación   de   objetos   circulares   en   el   mapa:  se   debe   encontrar   un   objeto   circular
mediante el análisis del mapa de ocupación. Se deberá proporcionar su radio y su centro.

Desarrollo

Consideración y condiciones en el desarrollo del trabajo:

• El trabajo se realizará en el entorno de simulación STDR y el modelo de robot Amigobot
utilizado en anteriores prácticas. Para el desarrollo de esta práctica se encuentran disponibles
2 mapas de prueba tfa_mapa1.png y tfa_mapa2.png, así como sus respectivos archivos yaml.

•

Se dispone de la información de odometría (robot0/odom), de los sensores de ultrasonidos
(robot0/sonar_X) y el láser (robot0/laser_0) para realizar la navegación y exploración
del entorno.

• La posición inicial del robot dentro del entorno es conocida y garantizará que el robot está
situado   en   una   zona   libre   de   obstáculos,   dejando   al   menos   0.5   metros   de   distancia   al
obstáculo más cercano.  Las estrategias propuestas deben evaluarse en diferentes posiciones
iniciales dentro del mapa para comprobar su eficacia.

• La velocidad máxima con la que se puede operar el robot debe ser: Velocidad lineal: 1 m/s.

Velocidad angular: 1 rad/s.

• El mapa explorado se guarda en un array  Mapa  de NxN posiciones, donde cada posición
corresponde   a   una   celda   del   entorno.   El   contenido   de   cada   celda   del   array  Mapa(i,j)
corresponderá al código correspondiente (0-Ocupada, 1-Libre, 0.5-No Explorada).

Figura 2: Modificación del valor de cada celda del array
Mapa para reflejar la medida láser.

• El   tiempo   máximo   de   exploración   es   de   5   minutos.     Una   vez   transcurridos,   el   mapa
explorado hasta ese momento en el array Mapa(i,j) se guardará en un archivo .png al final
de la ejecución. (Ver función imwrite en Matlab o similares).

•

•

•

Se valorará el uso de estrategias de control (PID) para la navegación y exploración del
entorno.

Se debe tener en cuenta que al moverse hacia cierta zona no explorada, el robot puede
encontrar obstáculos que impidan moverse hasta dicho destino. Se valorará el uso de alguna
estrategia de evasión que permita bordear el obstáculo sin que choque el robot y que llegue a
su destino.

Se debe tener en cuenta que pueden existir zonas del entorno que no son accesibles por el
robot   después   de   realizar   la   exploración,   ya   que   corresponden   a   espacios   cerrados   por
paredes y obstáculos.

• Un haz del láser o los sensores de ultrasonidos permiten conocer el estado de aquellas celdas
que se encuentran entre el robot y el punto donde se detecta obstáculo o pared (ver Figura
2). Para ello, debe marcar o actualizar en el mapa de ocupación, todo el camino que recorre
el   láser   basado   en   su   distancia   y   orientación.   Esto   requiere   realizar   funciones   para   la
transformación   de   coordenadas   entre   el   sistema   interno   del   robot,   las   posiciones   y
orientaciones de los sensores y el sistema de coordenadas global del mapa. Existen tres
sistemas de coordenadas que deben tenerse en cuenta en la práctica:

◦ Sistema de coordenadas local del robot. En este sistema de coordenadas se define la
posición y orientación de los sensores de ultrasonidos y las medidas del sensor láser. Se
puede consultar el archivo  Amigobot.xml  en la carpeta “robots” del simulador para
ver la posición de los sensores o consultar la información suministrada por los topics
robot0/sonar_X y robot0/laser_0.

◦ Sistema de coordenadas global del entorno. Este sistema de coordenadas es fijado por
el   simulador   STDR   y   utilizado   como   referencia   para   las   medidas   de   posición

suministradas por el topic “odom”. El origen de coordenadas se encuentra situado en la
esquina inferior izquierda del mapa utilizado en el simulador.

◦ Sistema de coordenadas del array Mapa, utilizado para guardar el mapa explorado.
Las posiciones de este array son números enteros, correspondientes a las posiciones de
las celdas del mapa de tamaño 6 cm x 6 cm. Se debe tener en cuenta que el array Mapa,
cuando es visualizado como una imagen (ver función imshow, imwrite en Matlab) se
asume   la   esquina   superior   izquierda   como   origen   de   coordenadas   y   la   coordenada
vertical de la imagen es descendente para posiciones consecutivas y corresponde a la
primera coordenada del array  Mapa. Para obtener una visualización correcta se debe
realizar una conversión de coordenadas.

• El modo de movimiento del robot para la exploración del mapa es a libre elección siempre
que   garantice   que   el   robot   no   se   choca   con   el   entorno   y   que   se   realiza   la   exploración
completa del mapa. Algunos ejemplos ilustrativos de movimientos de exploración que se
pueden combinar para realizar la exploración incluyen:

a. Modo pared: realiza un seguimiento de una pared, tal como se ha implementado en
la práctica 2, tomando en consideración, que puede haber obstáculos y por cada
vuelta que complete el robot, tendrá que ir aumentando la distancia.

b. Modo Espiral: el robot realiza movimientos circulares ampliando el radio del

círculo de movimiento cada vuelta que da sobre la circunferencia.

c. Modo Aleatorio: el robot se mueve en línea recta a una velocidad constante hacia
una dirección, cuando choca, cambia la orientación y sigue moviéndose en línea
recta

•

Para mejorar la visualización de la exploración, es recomendable que, a medida que se vaya
explorando, se vaya mostrando el mapa resultante. (ver función imshow en Matlab)

Evaluación

Cada   grupo   de   laboratorio   deberá   entregar   una   memoria   y   grabar   un   vídeo   mostrando   el
funcionamiento de su propuesta y haciendo una breve exposición de los métodos empleados con
una   duración   aproximada   de   10   minutos.   En   ese   tiempo   se   debe   mostrar   la   ejecución   de   los
algoritmos y la explicación del enfoque realizado, indicando también los algoritmos probados y
descartados.   Esta   exposición,   parecida   a   una   presentación   oral   en   clase,   se   debe   apoyar   en
transparencias (por ejemplo en Powerpoint) con explicaciones. En aquellos grupos formados por
más de una persona, todos los miembros del grupo deben participar en la presentación.

Se valorarán los siguientes aspectos de las prácticas:

• Algoritmo utilizado de exploración. (Capacidad de explorar el mapa completo, guiando el

robot hacia zonas no exploradas en un mínimo de tiempo)

• Algoritmo de evasión de obstáculos.

• Estrategias de control utilizadas. (Uso de controladores PID o de otras estrategias vistas en

clase y las prácticas).

• Uso eficiente de los sensores disponibles en el robot para garantizar la obtención de un mapa

del entorno preciso y fiable.

• Memoria y presentación del trabajo.

• Unos días antes de la entrega se subirá un nuevo mapa a la plataforma virtual que permitirá
evaluar los algoritmos empleados. Además de la presentación, se deberá subir un video
mostrando   el   funcionamiento   del   robot   en   este   mapa   de   evaluación.   Se   considerará   el
resultado obtenido en este mapa para la valoración de la práctica.

•

Participación en la competición (opcional) el día de la presentación final de la práctica. La
competición   consistirá   en   el   mapeo   de   un   entorno   nuevo,   diferente   del   utilizado   en   la
entrega, y se considerará ganador el equipo que realice el mapeo completo y reconocimiento
del objeto circular en el menor tiempo. Los detalles de la competición se publicarán en Aula
Virtual en los próximos días.


