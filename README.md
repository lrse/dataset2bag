Este paquete permite convertir datasets de imagenes y otros sensores a formato ROSBAG

# Sensores

El programa de conversion soporta diversos sensores:

* Imagenes (camara mono o camara estereo) + calibracion
* Odometria

# Modo de Uso

El conversor espera que se le indique un directorio del cual obtener los imagenes. Por otro lado, para setear
los timestamps de las imagenes, se puede indicar un archivo conteniendo los timestamps o bien se puede asumir un framerate
constante.

Ejemplo de imagenes mono:

    dataset2bag --images="img/frame%4d.png" --calib=calibration.txt --timestamps=timestamps.txt -o out.bag

# Formato de archivos

*IMPORTANTE*: en los archivos de timestamps y odometria, tener cuidado de no dejar una linea en blanco al final

## Calibracion

La calibracion contiene los parametros en formato similar al de ROS: parametros intrinsecos (3x3), de distorsion (5x1) y los parametros extrinsecos de rotacion (3x3) y traslacion (3x1).
Estos ultimos solo tienen sentido en el caso de un par estereo. En el caso monocular, asi como la camara estereo izquierda, la rotacion deberia ser la identidad y la traslacion un vector nulo.

    K11 K12 K13
    K21 K22 K23
    K31 K32 K33

    D1 D2 D3 D4 D5

    R11 R12 R13
    R21 R22 R33
    R31 R32 R33

    Tx Ty Tz

## Timestamps

El archivo de timestamps deberia tener una linea por cada imagen, en el formato de:

    [segundos] [nanosegundos]

## Odometria

Formato:

    [segundos] [nanosegundos] [x] [y] [angulo]

Las unidades son metros y radianes.

## IMU

Formato:

    [segundos] [nanosegundos] [aX] [aY] [aZ] [wX] [wY] [wZ] [R1] ... [R9]

El archivo contiene: timestamp, aceleración (m/s^2), velocidad angular (rad/s), orientacion como matriz
de 3x3

## Ground-truth

Este archivo contiene informacion de poses de ground-truth en 2D. El formato es el siguiente:

    [segundos] [nanosegundos] [x] [y] [theta]

Opcionalmente, se puede utilizar un archivo que contenga la informacion de covarianza de la pose:

    [segundos] [nanosegundos] [x] [y] [theta] [Cxx] [Cxy] [Cxt] [Cyx] [Cyy] [Cyt] [Ctx] [Cty] [Ctt]

## Laser

En este archivo se tiene primero unos valores del sensor:

    [angle_increment] [scan_count] [min_range] [max_range] [min_angle] [max_angle]

donde los angulos son en grados y las distancias son en metros.

Luego, se tiene una linea por scaneo con el siguiente formato:

    [segundos] [nanosegundos] [range_1] ... [range_N]


