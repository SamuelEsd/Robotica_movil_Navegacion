//World.h

/*
  
  A   Eje Y (VERDE) (HEIGHT)
  |
  |
  ------------------PIZARRÓN---------------------
  |    |                                |        | 
  |    |                                |        | 
  |    |        TABLE                   |        | 
  |    |        -----------------       | "L"    | 
  |    |        |       |       |       | TABLE  |
  |    |        |       |       |       |        |
  |    |        |       |       |       |------- |
  |    |        |       |       |       |        |
  |    |        |       |       |       |        |
  |____|        |       |       |       |        |
  |             |       |       |       |        |
  |             |       |       |       |        |
  |             |       |       |       |        |
  |             |       |       |       |        |
  |             |       |       |       |        |
  |             |       |       |       |        |
  |             |       |       |       |        |
  |____         |_______|_______|       ---------.
  |    |                                         .Entrada 
  |    |                                         . 
  |    |         ______________          ________.
  |    |        |              |        |        |
  |    |        |              |        |        |
  |    |        |              |        |        |
  ---------------------------------------------------->  eje X (ROJO) (WIDTH)
*/

#ifndef __WORLD_HPP_INCLUDED__   
#define __WORLD_HPP_INCLUDED__   

#include "nav_msgs/Odometry.h"

// -----------------------INICIALIZACION DE CONSTANTES-----------------------------------------
/// El eje X es rojo.
/// El eje Y es verde.
/// El eje Z apunta hacia arriba y el marcador es azul.

const int WIDTH = 30+2;      /// A lo largo del eje rojo x
const int HEIGHT = 36+2;     /// A lo largo del eje verde (+2 por las paredes)

// Medidas dependientes del ancho y alto del cuarto
// de las cuales se calculará las medidas de las mesas
const int WIDTH_unit = (WIDTH-2)/6;      /// A lo largo del eje rojo x
const int HEIGHT_unit = (HEIGHT-2)/6;     /// A lo largo del eje verde

// Medidas de las mesas
const int WIDTH_table = HEIGHT_unit+WIDTH_unit-1; // -1 porque las posiciones empiean de 0
const int HEIGHT_table = WIDTH_unit-1;

const double HALF_table_width = (WIDTH_table+1)/2.0;
const double HALF_table_height = (HEIGHT_table+1)/2.0;

// Resolución de los cuadros del grid
const double RESOLUTION = 0.2;


static nav_msgs::OccupancyGrid createMap();
static bool filledRectangle(char* data, int i, int j);
static void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value);
double rayTracing(char* data, double x, double y, double angle);

#endif 