#include "Vector2d.cpp"
#include "iostream"

int main(){
  Vector2d inicio(-1.5,0);
  inicio.print();
  Vector2d temp(0,0);
  temp.print();
  inicio.sum(temp);
  inicio.print();
  
  Vector2d temp2(-2,M_PI/2 );
  temp2.print();
  inicio.sum(temp2);
  inicio.print();

  Vector2d temp3(2,0 );
  temp3.print();
  inicio.sum(temp3);
  inicio.print();

  Vector2d temp4(2, M_PI/2);
  temp4.print();
  inicio.sum(temp4);
  inicio.print();
  return 0;
}
