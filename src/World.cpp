#include "World.hpp"
#include <ros/ros.h>


// -----------------------DEFINITION OF FUNCTIONS-----------------------------------------

/** Creates a OccupancyGrid with defaul. */
// http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
static nav_msgs::OccupancyGrid createMap(){
  // new grid
  nav_msgs::OccupancyGrid map;

  map.header.frame_id = "/odom";
  map.header.stamp = ros::Time::now();
  
  map.info.resolution = RESOLUTION;      
  map.info.width = WIDTH;                
  map.info.height = HEIGHT;              
  map.info.origin.orientation.w = 1.0;
  
  // Array that represents the grid of the world
  int size = WIDTH * HEIGHT;
  char* data = new char[size];
  for(int i = 0; i < size; i++) {
    data[i] = 0;
  }
  //                  y    x    y     x
  // Walls      col1 fil1 col2 fill2
  fillRectangle(data, 0,   3,   0,   WIDTH-1, 100); 
  fillRectangle(data, 3, 0, HEIGHT-1, 0, 100);      

  // First and second table (the closest to row 0)
  fillRectangle(data, 1, 1, 1+(WIDTH_table), 1+(HEIGHT_table), 100);
  fillRectangle(data, (HEIGHT-2)-(WIDTH_table), 1, (HEIGHT-2), 1+(HEIGHT_table), 100); 

  // Third table
  fillRectangle(data, 1, (((WIDTH-2)/4)+1), 1+(HEIGHT_table), (((WIDTH-2)/4)+1)+(WIDTH_table), 50);
  
  // The four tables together in the center of the room
  //                 inicio de mesa más abajo   inicio mesa más arriba    inicio mesa más arriba
  int mid_width = (( ((WIDTH-2)-(WIDTH_table)) - (1+(WIDTH_table)))/2  ) + (1+(WIDTH_table))+1;
  //                   inicio pared pizarrón     inicio de mesa más izquierda       inicio mesa más izquierda
  int mid_height = ( ( (HEIGHT-2)        -       (1+(HEIGHT_table)) )/2  )     +     (1+(HEIGHT_table))+1;
  fillRectangle(data, (mid_height-1)-(WIDTH_table), (mid_width-1)-(HEIGHT_table), mid_height+(WIDTH_table), mid_width+(HEIGHT_table), 50);    

  // Penultima y ultima mesa (doble) (las más cercanas a la fila última)
  fillRectangle(data, 1, (WIDTH-2)-(WIDTH_table), 1+(HEIGHT_table), (WIDTH-2), 50);
  fillRectangle(data, (HEIGHT-2)-(WIDTH_table*2), (WIDTH-2)-(HEIGHT_table), (HEIGHT-2), (WIDTH-2), 100);    
    

  // Dos paredes
  fillRectangle(data, HEIGHT-1, 1, HEIGHT-1, WIDTH-1, 100);   // Renglón ultimo. Las columnas van de 0 a WIDTH-1.  Los renglones corren sobre el eje Y+height-1.
  fillRectangle(data, 1, WIDTH-1, HEIGHT-1, WIDTH-1, 100);  // Columna ultima. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X+wheight-1.

  
  map.data = std::vector<int8_t>(data, data + size);

  return map;
}

/** Verifies if the cell [i,j] is occupied. */
static bool filledRectangle(char* data, int i, int j) // i = y j = x
{
  return data[i*WIDTH+j] != 0;
}

/** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied. */
static void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
{
  for(int i = i1; i <= i2; i++) {
    for(int j = j1; j <= j2; j++){
	    data[i*WIDTH+j] = value;
  	}
  }
}

double euclideanDistance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

double rayTracing(char* data, double x, double y, double angle){
  // ROS_INFO("entra a raytracing");
  // ROS_INFO("X: %lf Y: %lf Angle: %lf",x,y,angle);
  // Truncate the angle between Pi and -Pi if is greater than Pi
  if(angle > M_PI){
    angle -= 2*M_PI;
  }
  double m = tan(angle);
  double b = y - m*x;

  // nx ny values of the next intersection of the ray with the 
  // x axis
  double nx, ny;
  // values of x and y in every iteration
  double actual_x = x;
  double actual_y = y;

  // First 2 quadrants
  if (angle >= 0){
    // ROS_INFO("Primeros 2 cuadrantes");
    // ROS_INFO("X: %lf Y: %lf Angle: %lf",x,y,angle);
    // First quadrant
    if (angle < M_PI/2){
      // while the ray still on the map and has not 
      // found an obstacle
      while(int(actual_x) < WIDTH && int(actual_y) < HEIGHT){
        // ROS_INFO("Primer cuadrante");
        // ROS_INFO("X: %lf Y: %lf Angle: %lf",x,y,angle);
        // Get our new nx and ny values
        nx = floor(actual_x+1);
        ny = m*(nx) + b;

        // Case 1
        // If the ray has not yet crossed the y-axis
        if (floor(ny) == floor(actual_y)){
          actual_x = nx;
          actual_y = ny;
        }

        // Case 2
        // If the ray intersects both y and x axes,
        // we must verify the collisions with the 3 squares
        // shown below:
        //        Left->|_|_|<-Middle
        //                |_|<-Down
        //
        else if (floor(actual_y+1) == ny){
          actual_y = ny;
          actual_x = nx;
          if(filledRectangle(data, floor(actual_y), floor(actual_x-1)) || //Left
            filledRectangle(data, floor(actual_y), floor(actual_x)) ||    //Middle
            filledRectangle(data, floor(actual_y-1), floor(actual_x))){   //Down
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }

        // Case 3
        // If the ray has already crossed the y-axis 
        else if (floor(ny) > floor(actual_y)){
          actual_y = floor(actual_y+1);
          actual_x = (actual_y - b)/m;
        }
        // If cases 1 or 2 occur, verify if square in 
        // next position is filled
        if(filledRectangle(data, floor(actual_y), floor(actual_x))){
          return euclideanDistance(x,y,actual_x,actual_y);
        }
      }
    }

    // Second quadrant
    else if (angle <= M_PI) {
      // while the ray still on the map and has not 
      // found an obstacle
      while(int(actual_x) > 0 && int(actual_y) < HEIGHT){

        // ROS_INFO("Segundo cuadrante");
        // ROS_INFO("X: %lf Y: %lf Angle: %lf",x,y,angle);
        // Get our new nx and ny values
        nx = ceil(actual_x-1);
        ny = m*(nx) + b;

        // Case 1
        // If the ray has not yet crossed the y-axis
        if (floor(ny) == floor(actual_y)){
          actual_x = nx;
          actual_y = ny;

          // verify the collition  
          if(filledRectangle(data, floor(actual_y), floor(actual_x-1))){
            return euclideanDistance(x,y,actual_x,actual_y);
          }
	      }

        // Case 2
        // If the ray intersects both y and x axes,
        // we must verify the collisions with the 3 squares
        // shown below:
        //        Middle->|_|_|<-Right
        //          Down->|_|
        //
        else if (floor(actual_y+1) == ny){
          actual_y = ny;
          actual_x = nx;
          if(filledRectangle(data, floor(actual_y-1), floor(actual_x-1)) || //Down
            filledRectangle(data, floor(actual_y), floor(actual_x-1)) ||    //Middle
            filledRectangle(data, floor(actual_y), floor(actual_x))){       //Right
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }

        // Case 3
        // If the ray has already crossed the y-axis 
        else if (floor(ny) > floor(actual_y)){
          actual_y = floor(actual_y+1);
          actual_x = (actual_y - b)/m;

          // verify the collition
          if(filledRectangle(data, floor(actual_y), floor(actual_x))){
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }
      }
    }
  }

  else {
    // ROS_INFO("Ultimos 2 cuadrantes");
    // Fourth quadrant
    if (angle > -M_PI/2) {
      // while the ray still on the map and has not 
      // found an obstacle
      while(int(actual_x) < WIDTH && int(actual_y) > 0){
        // Get our new nx and ny values
        nx = floor(actual_x+1);
        ny = m*(nx) + b;

        // Case 1
        // If the ray has not yet crossed the y-axis
        if (ceil(ny) == ceil(actual_y)){
          actual_x = nx;
          actual_y = ny;
          
          // verify the collition 
          if(filledRectangle(data, floor(actual_y), floor(actual_x))){
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }

        // Case 2
        // If the ray intersects both y and x axes,
        // we must verify the collisions with the 3 squares
        // shown below:    _
        //               _|_|<-UP
        //        Left->|_|_|<-Middle
        //
        else if (ceil(actual_y-1) == ny){
          actual_y = ny;
          actual_x = nx;
          if(filledRectangle(data, floor(actual_y-1), floor(actual_x-1)) ||   //Left
            filledRectangle(data, floor(actual_y-1), floor(actual_x)) ||    //Middle
            filledRectangle(data, floor(actual_y), floor(actual_x))){       //UP
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }

        // Case 3
        // If the ray has already crossed the y-axis 
        else if (floor(ny) < floor(actual_y)){
          actual_y = ceil(actual_y-1);
          actual_x = (actual_y - b)/m;
          if(filledRectangle(data, floor(actual_y-1), floor(actual_x))){
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }
      }
    }

    // Third quadrant
    else if( angle > -M_PI){
      // while the ray still on the map and has not 
      // found an obstacle
      while(int(actual_x) > 0 && int(actual_y) > 0){
        // Get our new nx and ny values
        nx = ceil(actual_x-1);
        ny = m*(nx) + b;

        // Case 1
        // If the ray has not yet crossed the y-axis
        if (ceil(ny) == ceil(actual_y)){
          actual_x = nx;
          actual_y = ny;
          
          // verify the collition 
          if(filledRectangle(data, floor(actual_y), floor(actual_x-1))){
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }

        // Case 2
        // If the ray intersects both y and x axes,
        // we must verify the collisions with the 3 squares
        // shown below:    _
        //            UP->|_|_
        //        Middle->|_|_|<-Right
        //
        else if (ceil(actual_y-1) == ny){
          actual_y = ny;
          actual_x = nx;
          if(filledRectangle(data, floor(actual_y), floor(actual_x-1)) ||   //Up
            filledRectangle(data, floor(actual_y-1), floor(actual_x-1)) ||  //Middle
            filledRectangle(data, floor(actual_y)-1, floor(actual_x))){     //Right
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }

        // Case 3
        // If the ray has already crossed the y-axis 
        else if (floor(ny) < floor(actual_y)){
          actual_y = ceil(actual_y-1);
          actual_x = (actual_y - b)/m;
          if(filledRectangle(data, floor(actual_y-1), floor(actual_x))){
            return euclideanDistance(x,y,actual_x,actual_y);
          }
        }
      }
    }
  }
  return 0.3;
}