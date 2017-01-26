void setup()
{
  size(700,600);
  background(255,255,255);
  smooth();
  
 // PImage img;
  //img = loadImage("C:\\Users\\Public\\Pictures\\Sample Pictures\\Koala.jpg");
  //image(img,0,0);
  //println(img);
}
int x,y,x1,y1,hight,widt;
int clickno = 1;
int flag = 0;
void draw()
{
  fill(0,255,200);
  rect(0,0,20,20);
  fill(255,0,0);
  textSize(20);
  text("C",3,18);
  
  if(flag == 1)
  {
    flag = 0;
    fill(0,0,0);
    rect(x,y,widt,hight);
  }
}
void mousePressed()
{
  if(mouseX < 20 && mouseY < 20)
  {
    clear();
    background(255,255,255);
  }
  else if(clickno == 1)
  {
    x=mouseX;
    y=mouseY;
    clickno = 2;
  }
  else if(clickno == 2)
  {
    x1 = mouseX;
    y1=mouseY;
    if(abs(x-x1) <= abs(y-y1))
    {
      hight = abs(y-y1);
      widt = 10;
    }
    else
    {
      widt = abs(x-x1);
      hight = 10;
    }
    clickno=1;
    flag = 1;
  }
}
