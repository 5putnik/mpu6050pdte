import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class Processing extends PApplet {



float accel_x, accel_y, accel_z, tmp,gyro_x, gyro_y, gyro_z = 0;
Serial myPort;
final int vSize = 200;
float dadox[] = new float[vSize], dadoy[] = new float[vSize];
int c = 0;

PFont f;

public void setup()
{
  
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil('\n');
  noStroke();
  colorMode(RGB, 1);
  f = createFont("Arial", 16, true);
}

public void draw()
{
  rectMode(CORNERS);
  int i;
  c++;
  if(c == vSize)
    c = 0;
  dadox[c] = 595 - (accel_y * 200) / 32768;
  dadoy[c] = 210 - (accel_z * 200) / 32768;
  
  background(255, 255, 255);
  textFont(f, 16);
  
  
  fill(0);
  rect(400,10,790,410);
  fill(255, 255, 255);
  rect(401,11,789,409);
  fill(0);
  rect(589,10,590,410);
  rect(400,210,790,211);
  
  fill(0, 255, 0);
  for(i=0;i<vSize;i++)
    if(dadox[i] != 0 && dadoy[i] != 0)
      rect(dadox[i], dadoy[i], 2 + dadox[i], 2 + dadoy[i]);
  
  
  

  fill(0);
  /* Imprime na tela os valores lidos pelo sensor */
  if(accel_x != -1.0f)
    text("Leitura acelerometro X: " + accel_x, 10, 20);
  else
    text("Leitura acelerometro X: Erro na comunicacao!", 10, 20);
  
  if(accel_y != -1.0f)
    text("Leitura acelerometro Y: " + accel_y + "(" + dadoy[c] + ")", 10, 40);
  else
    text("Leitura acelerometro Y: Erro na comunicacao!", 10, 40);
  
  if(accel_z != -1.0f)
    text("Leitura acelerometro Z: " + accel_z + "(" + dadox[c] + ")", 10, 60);
  else
    text("Leitura acelerometro Z: Erro na comunicacao!", 10, 60);
  
  if(gyro_x != -1.0f)
    text("Leitura giroscopio X:" + gyro_x, 10, 100);
  else
    text("Leitura giroscopio X: Erro na comunicacao!", 10, 100);
  
  if(gyro_y != -1.0f)
    text("Leitura giroscopio Y: " + gyro_y, 10, 120);
  else
    text("Leitura giroscopio Y: ", 10, 120);
  if(gyro_z != -1.0f)
    text("Leitura giroscopio Z: " + gyro_z, 10, 140);
  else
    text("Leitura giroscopio Z: Erro na comunicacao!", 10, 140);
  if(tmp != 36.53f)
    text("Temperatura: " + tmp + "\u00baC", 10, 180);
  else
    text("Temperatura: Erro na comunicacao!", 10, 180);
  /* Fim impressao */
  
  
}

public void serialEvent(Serial myPort)
{
  String xString = myPort.readStringUntil('\n');
  float rate = 0.01f;
  if(xString != null)
  {
    String  temp[]  =  split(xString,":");
    if(xString.charAt(0)  ==  '#'  &&  temp.length==8)
    {
      accel_x = PApplet.parseFloat(temp[1]);
      accel_y = PApplet.parseFloat(temp[2]);
      accel_z = PApplet.parseFloat(temp[3]);
      tmp = PApplet.parseFloat(temp[4]);
      gyro_x = PApplet.parseFloat(temp[5]);
      gyro_y = PApplet.parseFloat(temp[6]);
      gyro_z = PApplet.parseFloat(temp[7]);
    }
  }
}
  public void settings() {  size(800, 600, P2D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "Processing" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
