void setup()
{
  size(240,500);
  String i="LULRRLLULLSULLULLLLRLULULLLULULULLLLULULSULLULLLULULSULULLLLUSLL";
  solve_maze(i);
}
void draw()
{
}
String solve_maze(String total_path)
{
  int len = total_path.length();
  println(len);
  for(int i=0;i<len-2;i++)
  {
    String sub = total_path.substring(i,i+3);
    if(sub.equals("LUL"))
    {
     total_path = total_path.substring(0,i)+"S"+total_path.substring(i+3,len);
     len = total_path.length();
     i=-1;
    }
    else if(sub.equals("SUL") || sub.equals("LUS"))
    { 
     total_path = total_path.substring(0,i)+"R"+total_path.substring(i+3,len);
     len = total_path.length();
     i=-1;
    }
    else if(sub.equals("RUL"))
    { 
     total_path = total_path.substring(0,i)+"U"+total_path.substring(i+3,len);
     len = total_path.length();
     i=-1;
    }
  }
  println(total_path);
  return total_path;
}
