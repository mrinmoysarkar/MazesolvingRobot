void setup()
{
  size(600,600);
  background(255);
  smooth();
}
int x=400, y=400,z=20;
int flag=0,ang=0;
int dx=0;
void draw()
{
  background(255);
  pushMatrix();
  translate(250,250);
  translate(dx,32);
  stroke(100);
  rotate(radians(ang));
  fill(200,0,0);
  rect(-12,0,12,37);
  ellipse(0,0,100,100);
  fill(0,45,87);
  line(-50,0,50,0);
  line(0,50,0,-50);
  popMatrix();
  fill(0,37,200);
 // ellipse(x,y,5,5);
  if(flag == 0)
  {
    ang++;
    dx++;
    if(ang == 360 || dx == height-250)
    {
      ang=0;
      flag=1;
    }
  }
  else if(flag==1)
  {
    ang--;
    dx--;
    if(ang == 0||dx==0)
    {
      flag=0;
    }
  }
  text("mouseX "+mouseX+"    mouseY"+mouseY, 100, 200);
}

