# sim_basics

Ejemplo elemental sobre cómo utilizar una malla de ocupación en ROS (OccupancyGrid). Abajo se muestra una imagen del programa corriendo, con etiquetas añadidas que explican los sistemas de coordenadas.

![Grid axes](./images/Mundo.png)

Se puede clonar dentro del src de cualquier espacio de trabajo de catkin.

Se añade código a la versión inicial para simular el aula de clases.

Para la ejecución es necesario tener una terminal corriendo la kobuki(es necesario hacer los source correspondientes):
  -roslaunch kobuki_softnode full.launch
  
  
Y después ejecutar en una nueva terminal(es necesario hacer los source correspondientes):
  -rosrun sim_basics basic_map
  
Al final inicializar rviz(es necesario hacer los source correspondientes):
   -rosrun rviz rviz 

Y detro de rviz poner los siguientes valores:
  ->Global Options
   |-->Fixed Frame: odom
   
Agregar dos nuevos dysplay de tipo map y marker respetcivamente 

  ->Map
  |-->Topic: /occupancy_marker
  ->Marker
  |-->visualization_marker
