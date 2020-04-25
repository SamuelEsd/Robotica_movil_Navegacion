// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <math.h>
// %EndTag(INCLUDES)%



/*
  ------------------PIZARRÓN------------------
  |    |                              |       | 
  |    |                              |       | 
  |    |        TABLE                 |       | 
  |    |        -----------------     | "L"   | 
  |    |        |       |       |     | TABLE |
  |    |        |       |       |     |       |
  |    |        |       |       |     |-------|
  |    |        |       |       |     |       |
  |    |        |       |       |     |       |
  |____|        |       |       |     |       |
  |             |       |       |     |       |
  |             |       |       |     |       |
  |             |       |       |     |       |
  |             |       |       |     |       |
  |             |       |       |     |       |
  |             |       |       |     |       |
  |             |       |       |     |       |
  |____         |_______|_______|     --------.
  |    |                                      .Entrada 
  |    |                                      . 
  |    |         ______________       ________.
  |    |        |              |     |        |
  |    |        |              |     |        |
  |    |        |              |     |        |
  ---------------------------------------------
*/


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

const double RESOLUTION = 0.2;

double linear_pose_x;
double linear_pose_y;
double yaw_angle;

geometry_msgs::Pose kobuki_pose;

// -----------------------FUCNIONES AUXILIARES ------------------------------------------------
/** Sets the cells between [i1,j1] and [i2,j2] inclusive as occupied with probability value. */
void fillRectangle(char* data, int i1, int j1, int i2, int j2, int value)
{
  for(int i = i1; i <= i2; i++)
    {
      for(int j = j1; j <= j2; j++)
	{
	  data[i*WIDTH+j] = value;
	}
    }
}

bool filledRectangle(char* data, int i, int j)
{
  return data[i*WIDTH+j] == 100;
}

double euclideanDistance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}


double rayTracing(char* data, double x, double y, double angle){
  if(angle > M_PI){
    angle -= 2*M_PI;
  }
  double m = tan(angle);
  double b = y - m*x;
  ROS_INFO("Angulo: %lf", angle);
  ROS_INFO("m: %lf", m);

  // nx ny valores nuevos resultantes de evaluar el proximo punto
  // en la dirección del rayo que pase por un valor entero sobre el eje x
  double nx, ny;
  double actual_x = x;
  double actual_y = y;

  
  if (angle >= 0){

    // Primer cuadrante
    if (angle < M_PI/2){

      // Si No hay pendiente
      // Caso a parte pues no se puede despejar x con m = 0
      if (m == 0){
        nx = floor(x + 1);
        while(int(nx) < WIDTH && !filledRectangle(data, y, nx)){
          nx += 1;
        }
        nx = floor(nx)*.3;
        return nx;
      }
      
      // Si m != 0
      // Mientras no nos salgamos del mapa buscamos hasta encontrar
      // un obstaculo
      while(int(actual_x) < WIDTH && int(actual_y) < HEIGHT){
	// Despejamos la posición del punto y
	// cuando el rayo intersecta con el próximo
	// valor entero de x
	nx = floor(actual_x+1);
        ny = m*(nx) + b;
	// Si todavía no se pasa por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje x
	if (floor(ny) == floor(actual_y)){
	  actual_x = nx;
	  actual_y = ny;
	}
	// Si y intersecta al siguiente valor
	// entero sobre el eje de las y al igual que
	// el valor entero de x sobre su eje entonces
	// es una esquina checamos si alguno de los cuadrados
	// adyacentes están ocupados
	else if ( abs(ny - floor(actual_y+1) < 0.005) ){
	  if(filledRectangle(data, floor(actual_y+1), floor(nx)) ||
	     filledRectangle(data, floor(actual_y), floor(nx)) ||
	     filledRectangle(data, floor(actual_y+1), floor(nx-1))){
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	  else {
	    actual_y = ny;
	    actual_x = nx;
	  }
	}
	// Si ya pasó por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje y
	// despejando x 
	else if (floor(ny) > floor(actual_y)){
	  actual_y = floor(actual_y+1);
	  actual_x = (actual_y - b)/ m;
	}
	if(filledRectangle(data, floor(actual_y), floor(actual_x))){
	  return euclideanDistance(x,y,actual_x,actual_y);
	}
      }
    }

    else if (angle <= M_PI) {
      // Si No hay pendiente
      // Caso a parte pues no se puede despejar x con m = 0
      if (m == 0){
        nx = ceil(x - 1);
        while(int(nx) > 0 && !filledRectangle(data, y, nx)){
          nx -= 1;
        }
        nx = floor(nx)*.3;
        return nx;
      }
      std::cout << "Segundo cuadrante angulo: " << angle << std::endl;
      // Si m != 0
      // Mientras no nos salgamos del mapa buscamos hasta encontrar
      // un obstaculo
      double actual_x = x;
      double actual_y = y;
      while(int(actual_x) > 0 && int(actual_y) < HEIGHT){
	// Despejamos la posición del punto y
	// cuando el rayo intersecta con el próximo
	// valor entero de x
	nx = ceil(actual_x-1);
        ny = m*(nx) + b;
	// Si todavía no se pasa por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje x
	if (floor(ny) == floor(actual_y)){
	  actual_x = nx;
	  actual_y = ny;
	  
	  if(filledRectangle(data, floor(actual_y), floor(actual_x-1))){
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	}
	// Si y intersecta al siguiente valor
	// entero sobre el eje de las y al igual que
	// el valor entero de x sobre su eje entonces
	// es una esquina checamos si alguno de los cuadrados
	// adyacentes están ocupados
	else if ( abs(ny - floor(actual_y+1) < 0.005) ){
	  std::cout << "Me estoy ciclando aquí angulo: " << angle << std::endl;
	  std::cout << "Primero cuadro " << floor(actual_y) << " " << nx-1 << std::endl;
	  std::cout << "Segundo cuadro " << floor(actual_y+1) << " " << nx-1 << std::endl;
	  std::cout << "Tercero cuadro " << floor(actual_y+1) << " " << nx << std::endl;
	  if(filledRectangle(data, floor(actual_y+1), nx) ||
	     filledRectangle(data, floor(actual_y+1), nx-1) ||
	     filledRectangle(data, floor(actual_y), nx-1)){
	    std::cout << "Pero no entro aquí angulo: " << angle << std::endl;
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	  else {
	    actual_y = ny;
	    actual_x = nx;
	  }
	}
	// Si ya pasó por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje y
	// despejando x 
	else if (floor(ny) > floor(actual_y)){
	  actual_y = floor(actual_y+1);
	  actual_x = (actual_y - b)/ m;
	  if(filledRectangle(data, floor(actual_y), floor(actual_x))){
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	}
      }
    }
  }
  else {
    
    // cuarto cuadrante
    if (angle > -M_PI/2) {
      
      std::cout << "Cuarto cuadrante angulo: " << angle << std::endl;
      // Si m != 0
      // Mientras no nos salgamos del mapa buscamos hasta encontrar
      // un obstaculo
      double actual_x = x;
      double actual_y = y;
      while(int(actual_x) < WIDTH && int(actual_y) > 0){
	// Despejamos la posición del punto y
	// cuando el rayo intersecta con el próximo
	// valor entero de x
	nx = floor(actual_x+1);
        ny = m*(nx) + b;
	// Si todavía no se pasa por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje x
	if (ceil(ny) == ceil(actual_y)){
	  actual_x = nx;
	  actual_y = ny;
	  
	  if(filledRectangle(data, floor(actual_y), floor(actual_x))){
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	}
	// Si y intersecta al siguiente valor
	// entero sobre el eje de las y al igual que
	// el valor entero de x sobre su eje entonces
	// es una esquina checamos si alguno de los cuadrados
	// adyacentes están ocupados
	else if ( abs(ny - ceil(actual_y-1) < 0.005) ){
	  std::cout << "Me estoy ciclando aquí angulo: " << angle << std::endl;
	  std::cout << "Primero cuadro " << ceil(actual_y-1) << " " << floor(nx) << std::endl;
	  std::cout << "Segundo cuadro " << ceil(actual_y-2) << " " << floor(nx) << std::endl;
	  std::cout << "Tercero cuadro " << ceil(actual_y-2) << " " << floor(nx-1) << std::endl;
	  if(filledRectangle(data, ceil(actual_y-1), nx) ||
	     filledRectangle(data, ceil(actual_y-2), nx) ||
	     filledRectangle(data, ceil(actual_y-2), nx-1)){
	    std::cout << "Pero no entro aquí angulo: " << angle << std::endl;
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	  else {
	    actual_y = ny;
	    actual_x = nx;
	  }
	}
	// Si ya pasó por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje y
	// despejando x 
	else if (floor(ny) < floor(actual_y)){
	  actual_y = ceil(actual_y-1);
	  actual_x = (actual_y - b)/ m;
	  
	  if(filledRectangle(data, floor(actual_y-1), floor(actual_x))){
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	}
      }
    }

    else if( angle > -M_PI){

      
      std::cout << "Tercer cuadrante angulo: " << angle << std::endl;
      // Si m != 0
      // Mientras no nos salgamos del mapa buscamos hasta encontrar
      // un obstaculo
      double actual_x = x;
      double actual_y = y;
      while(int(actual_x) > 0 && int(actual_y) > 0){
	// Despejamos la posición del punto y
	// cuando el rayo intersecta con el próximo
	// valor entero de x
	nx = ceil(actual_x-1);
        ny = m*(nx) + b;
	// Si todavía no se pasa por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje x
	if (ceil(ny) == ceil(actual_y)){
	  if(filledRectangle(data, floor(ny), floor(nx-1))){
	    std::cout << "Choqué por x con : " << nx << " " << ny << std::endl;
	    return euclideanDistance(x,y,actual_x,actual_y);
	  }
	  actual_x = nx;
	  actual_y = ny;
	}
	// Si y intersecta al siguiente valor
	// entero sobre el eje de las y al igual que
	// el valor entero de x sobre su eje entonces
	// es una esquina checamos si alguno de los cuadrados
	// adyacentes están ocupados
	else if ( abs(ny - ceil(actual_y-1) < 0.005) ){
	  // std::cout << "Me estoy ciclando aquí angulo: " << angle << std::endl;
	  // std::cout << "Primero cuadro " << ceil(actual_y-1) << " " << floor(nx-1) << std::endl;
	  // std::cout << "Segundo cuadro " << ceil(actual_y-2) << " " << floor(nx-1) << std::endl;
	  // std::cout << "Tercero cuadro " << ceil(actual_y-2) << " " << floor(nx) << std::endl;
	  if(filledRectangle(data, ceil(ny), nx-1) ||
	     filledRectangle(data, ceil(ny)-1, nx-1) ||
	     filledRectangle(data, ceil(ny)-1, nx)){
	    std::cout << "Choqué por en medio con : " << nx << " " << actual_y << std::endl;
	    return euclideanDistance(x,y,nx,ny);
	  }
	  else{
	    actual_y = ny;
	    actual_x = nx;
	  }
	}
	// Si ya pasó por el próximo
	// valor entero sobre el eje y nos movemos
	// al punto sobre el valor entero sobre el eje y
	// despejando x 

	else if (floor(ny) < floor(actual_y)){
	  if(filledRectangle(data, floor(ny), floor(nx))){
	    return euclideanDistance(x,y,nx,ny);
	  }
	  actual_y = ceil(ny);
	  actual_x = (actual_y - b)/ m;
	}     
      
	// else if (ceil(ny) < ceil(actual_y)){
	//   actual_y = ceil(actual_y-1);
	//   actual_x = (actual_y - b)/ m;
	  
	//   if(filledRectangle(data, ceil(actual_y-1), floor(actual_x))){
	//     std::cout << "Choqué por y con : " << actual_x << " " << actual_y << std::endl;
	//     return euclideanDistance(x,y,actual_x,actual_y);
	//   }
	// }
      }
    }
  }
  std::cout << "No entró a nada angulo: " << angle << std::endl;
  std::cout << "No entró a nada m: " << m << std::endl;
  return 0;
}


/** Receives the message of the navigation goal from rviz. */
void receiveNavGoal(const geometry_msgs::PoseStamped& poseStamped)
{
  ROS_INFO("\nFrame: %s\nMove to: [%f, %f, %f] - [%f, %f, %f, %f]",
           poseStamped.header.frame_id.c_str(),
           poseStamped.pose.position.x,
           poseStamped.pose.position.y,
           poseStamped.pose.position.z,
           poseStamped.pose.orientation.x,
           poseStamped.pose.orientation.y,
           poseStamped.pose.orientation.z,
           poseStamped.pose.orientation.w);
}



/**
 * Get values of position from odom
 */
void positionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  linear_pose_x = msg->pose.pose.position.x;
  linear_pose_y = msg->pose.pose.position.y;

  kobuki_pose = msg->pose.pose;
  
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  yaw_angle = tf::getYaw(pose.getRotation());
}



// -----------------------METODO PRINCIPAL------------------------------------------------------
// %Tag(INIT)%
int main( int argc, char** argv )
{
  // ---------------------INICIALIZACION DE NODOS NECESARIOS -----------------------------------
  ros::init(argc, argv, "basic_map");
  ros::NodeHandle n;
  ros::Rate r(20);
  // Para publicar los markadores correspondientes a las mesas
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Para publicar el mapa con colores correspondientes a si el cuadro está ocupado o no
  ros::Publisher occupancy_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_marker", 1);
  // Para suscribirnos al Objetivo al cual deseamos se mueva la kobuki
  ros::Subscriber obj_sub = n.subscribe("/move_base_simple/goal", 5, receiveNavGoal); // Máximo 5 mensajes en la cola.
  // Para suscribirnos a la posición actual de la kobuki
  ros::Subscriber odom_sub = n.subscribe("/odom", 1, positionCallback);
  // %EndTag(INIT)%


  // --------------------INICIALIZACION DEL MAPA ------------------------------------------------
  // %Tag(MAP_INIT)%
  nav_msgs::OccupancyGrid map;

  // http://docs.ros.org/api/nav_msgs/html/msg/OccupancyGrid.html
  map.header.frame_id = "/odom";
  map.header.stamp = ros::Time::now();   // No caduca
  
  map.info.resolution = RESOLUTION;             // [m/cell]
  map.info.width = WIDTH;                // [cells]
  map.info.height = HEIGHT;              // [cells]
  map.info.origin.orientation.w = 1.0;
  
  // Arreglo con las casillas del mapa para colorear
  int size = WIDTH * HEIGHT;
  char* data = new char[size];
  for(int i = 0; i < size; i++) {
    data[i] = 0;
  }
  //                  y    x    y     x
  // Dos paredes      col1 fil1 col2 fill2
  fillRectangle(data, 0,   3,   0,   WIDTH-1, 100); // Columna 0. Los renglones va de 0 a HEIGHT-1.  Las columnas corren sobre el eje X. Rojo
  fillRectangle(data, 3, 0, HEIGHT-1, 0, 100);  // Renglón 0. Las columnas van de 1 a WIDTH-1.  Los renglones corren sobre el eje Y. Verde 

  // Primer y Segunda mesa (las más cercanas al renglón 0)
  fillRectangle(data, 1, 1, 1+(WIDTH_table), 1+(HEIGHT_table), 100);
  fillRectangle(data, (HEIGHT-2)-(WIDTH_table), 1, (HEIGHT-2), 1+(HEIGHT_table), 100); 

  // tercer mesa
  fillRectangle(data, 1, (((WIDTH-2)/4)+1), 1+(HEIGHT_table), (((WIDTH-2)/4)+1)+(WIDTH_table), 50);
  
  // cuatro mesas juntas
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
  
  // %EndTag(MAP_INIT)%




  //----------------- INICIO CREACION DE MARCADORES-------------------------------------------------
  double centroDMesaW = (WIDTH_table+1)/2.0;
  double centroDMesaH = (HEIGHT_table+1)/2.0;
  // %Tag(MARKER_INIT)%
  //-----------------------------MESA 1---------------------------------------
  visualization_msgs::Marker marker0;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker0.header.frame_id = "/odom";
  marker0.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker0.ns = "table_marker_0";
  marker0.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker0.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker0.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker0.pose.position.x = (centroDMesaH+1)*RESOLUTION;
  marker0.pose.position.y = (centroDMesaW+1)*RESOLUTION;
  marker0.pose.position.z = 0.6;
  marker0.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker0.scale.x = (HEIGHT_table+1)*RESOLUTION;
  marker0.scale.y = (WIDTH_table+1)*RESOLUTION;
  marker0.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker0.color.r = 0.0f;
  marker0.color.g = 1.0f;
  marker0.color.b = 0.0f;
  marker0.color.a = 0.5f;

  marker0.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%


  // %Tag(MARKER_INIT)%
  //-------------------------MESA 2---------------------------------------------------------
  visualization_msgs::Marker marker1;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker1.header.frame_id = "/odom";
  marker1.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker1.ns = "table_marker_1";
  marker1.id = 1;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker1.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker1.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker1.pose.position.x = (centroDMesaH+1)*RESOLUTION;
  marker1.pose.position.y = ((HEIGHT-1)-(centroDMesaW))*RESOLUTION;
  marker1.pose.position.z = 0.6;
  marker1.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker1.scale.x = (HEIGHT_table+1)*RESOLUTION;
  marker1.scale.y = (WIDTH_table+1)*RESOLUTION;
  marker1.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker1.color.r = 0.0f;
  marker1.color.g = 1.0f;
  marker1.color.b = 0.0f;
  marker1.color.a = 0.5f;

  marker1.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%




  // %Tag(MARKER_INIT)%
  //-------------------------MESA 3-----------------------------------------------
  visualization_msgs::Marker marker2;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker2.header.frame_id = "/odom";
  marker2.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker2.ns = "table_marker_2";
  marker2.id = 2;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker2.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker2.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker2.pose.position.x = ((((WIDTH-2)/4)+1)+centroDMesaW)*RESOLUTION;
  marker2.pose.position.y = (1+centroDMesaH)*RESOLUTION;
  marker2.pose.position.z = 0.6;
  marker2.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker2.scale.x = (WIDTH_table+1)*RESOLUTION;
  marker2.scale.y = (HEIGHT_table+1)*RESOLUTION;
  marker2.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker2.color.r = 0.0f;
  marker2.color.g = 1.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 0.5;

  marker2.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%




  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker3;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker3.header.frame_id = "/odom";
  marker3.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker3.ns = "table_marker_3";
  marker3.id = 3;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker3.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker3.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker3.pose.position.x = mid_width*RESOLUTION;
  marker3.pose.position.y = mid_height*RESOLUTION;
  marker3.pose.position.z = 0.6;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker3.scale.x = (HEIGHT_table+1)*RESOLUTION*2;
  marker3.scale.y = (WIDTH_table+1)*RESOLUTION*2;
  marker3.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker3.color.r = 0.0f;
  marker3.color.g = 1.0f;
  marker3.color.b = 0.0f;
  marker3.color.a = 0.5f;

  marker3.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%


  
  // %Tag(MARKER_INIT)%
  //-------------------------MESA 4-----------------------------------------------
  visualization_msgs::Marker marker4;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker4.header.frame_id = "/odom";
  marker4.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker4.ns = "table_marker_4";
  marker4.id = 4;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker4.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker4.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker4.pose.position.x = ((WIDTH-1)-centroDMesaW)*RESOLUTION;
  marker4.pose.position.y = (1+centroDMesaH)*RESOLUTION;
  marker4.pose.position.z = 0.6;
  marker4.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker4.scale.x = (WIDTH_table+1)*RESOLUTION;
  marker4.scale.y = (HEIGHT_table+1)*RESOLUTION;
  marker4.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker4.color.r = 0.0f;
  marker4.color.g = 1.0f;
  marker4.color.b = 0.0f;
  marker4.color.a = 0.5f;

  marker4.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%

  

  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker marker5;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker5.header.frame_id = "/odom";
  marker5.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker5.ns = "table_marker_5";
  marker5.id = 5;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker5.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker5.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker5.pose.position.x = ((WIDTH-1)-centroDMesaH)*RESOLUTION;
  marker5.pose.position.y = ((HEIGHT-1)-((centroDMesaW*2)-0.5))*RESOLUTION;
  marker5.pose.position.z = 0.6;
  marker5.pose.orientation.x = 0.0;
  marker5.pose.orientation.y = 0.0;
  marker5.pose.orientation.z = 0.0;
  marker5.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker5.scale.x = (HEIGHT_table+1)*RESOLUTION;
  marker5.scale.y = ((WIDTH_table*2)+1)*RESOLUTION;
  marker5.scale.z = 1.0;

  // Set the color -- be sure to set alpha to something non-zero!
  marker5.color.r = 0.0f;
  marker5.color.g = 1.0f;
  marker5.color.b = 0.0f;
  marker5.color.a = 0.5;

  marker5.lifetime = ros::Duration();


  // %EndTag(MARKER_INIT)%


  // %Tag(MARKER_INIT)%
  int num_sensores = 6;
  std::vector<visualization_msgs::Marker> sensores;
  for (int i = 0; i < num_sensores; i++){
    double sensor_angle = (yaw_angle+(((2*M_PI)/num_sensores)*i));
    visualization_msgs::Marker line_marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    line_marker.header.frame_id = "/base_link";
    line_marker.header.stamp = ros::Time::now();

    line_marker.ns = "line_marker_" + std::to_string(i);
    line_marker.id = 10+i;

    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;

    line_marker.pose.position.x = kobuki_pose.position.x;
    line_marker.pose.position.y = kobuki_pose.position.y;
    line_marker.pose.orientation.w = 1.0;
    // line_marker_0.pose = kobuki_pose;

  
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    line_marker.scale.x = 0.05f;
    line_marker.scale.y = 0.00f;
    line_marker.scale.z = 0.00f;

    // Set the color -- be sure to set alpha to something non-zero!
    line_marker.color.r = 0.0f;
    line_marker.color.g = 0.0f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 1.5f;

    // marker line points
    geometry_msgs::Point p;
    // first point
    p.x = 0;
    p.y = 0;
    p.z = 0.0;

    line_marker.points.push_back(p);

    double distace_to_colision = rayTracing(data, linear_pose_x*pow(RESOLUTION,-1), linear_pose_y*pow(RESOLUTION,-1), sensor_angle);
    p.x += distace_to_colision*cos(sensor_angle);
    p.y += distace_to_colision*sin(sensor_angle);
    line_marker.points.push_back(p);

    line_marker.lifetime = ros::Duration();

    sensores.push_back(line_marker);
  }
  
  
  while (ros::ok())
    {
      occupancy_pub.publish(map);
      marker_pub.publish(marker0);
      marker_pub.publish(marker1);
      marker_pub.publish(marker2);
      marker_pub.publish(marker3);
      marker_pub.publish(marker4);
      marker_pub.publish(marker5);
      //      line_marker_0.pose = kobuki_pose;
      for(int i = 0; i < num_sensores; i++){
	double sensor_angle = (yaw_angle+(((2*M_PI)/num_sensores)*i));
        sensores[i].pose.position.x = kobuki_pose.position.x;
	sensores[i].pose.position.y = kobuki_pose.position.y;
	double distace_to_colision = RESOLUTION*rayTracing(data, linear_pose_x*pow(RESOLUTION,-1), linear_pose_y*pow(RESOLUTION,-1), sensor_angle);
	ROS_INFO("distance to colision: %lf", distace_to_colision);
        sensores[i].points[1].x = distace_to_colision*cos(sensor_angle);
        sensores[i].points[1].y = distace_to_colision*sin(sensor_angle);
	marker_pub.publish(sensores[i]);
	
      }
      
      // %Tag(SLEEP_END)%
      ros::spinOnce();
      r.sleep();
    }
  // %EndTag(SLEEP_END)%
}
// %EndTag(FULLTEXT)%
