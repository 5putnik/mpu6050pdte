import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import controlP5.*; 
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




float accel_x, accel_y, accel_z, tmp,gyro_x, gyro_y, gyro_z = 0; // Valores medidos pelo sensor
String acx, acy, acz, gyx, gyy, gyz; // Valores convertidos para mostrar na tela 

Serial myPort; // Porta serial do Arduino a ser lida
final int vSize = 400; // Quantidade maxima de dados a ser salva
float dadox[] = new float[vSize], dadoy[] = new float[vSize]; // Dados a serem mostrados no grafico
int c = 0; // Contador

PFont f; // Fonte do texto

ControlP5 cp5; // Controle para utilizar objetos
DropdownList axismode, // lista para escolher o modo eixo (XY ou YT)
             fillmode, // lista para escolher o modo preenchimento (linha ou ponto)
             valorx, // lista de valores a serem plotados (Eixo X ou valor 1)
             valory, // lista de valores a serem plotados (Eixo Y ou valor 2)
             ac_unit, // lista de unidade do acelerometro (m/s\u00b2 ou g)
             gy_unit; // lista de unidade do giroscopio (\u00ba/s ou rad/s)

public void setup() // Inicializacao do programa
{
   // Gerando uma tela 800x600 com renderizacao 2D melhorada
  myPort = new Serial(this, Serial.list()[0], 9600); // Associando MyPort as portas seriais do computador
  myPort.bufferUntil('\n'); // Busca por \n
  colorMode(RGB, 1);
  f = createFont("Arial", 16, true); // Escolhendo fonte do texto como Arial 16
  
  cp5 = new ControlP5(this);
  axismode = cp5.addDropdownList("Representacao",10, 200, 100, 84);
  fillmode = cp5.addDropdownList("Preenchimento",120, 200, 100, 84);
  valorx = cp5.addDropdownList("Eixo X", 10, 300, 100, 84);
  valory = cp5.addDropdownList("Eixo Y", 120, 300, 100, 84);
  ac_unit = cp5.addDropdownList("Unidade ac", 270, 20, 100, 84);
  gy_unit = cp5.addDropdownList("Unidade gy", 270, 100, 100, 84);
  axis_create(axismode); // Cria a lista axismode
  fill_create(fillmode); // Cria a lista fillmode
  x_create(valorx); // Cria a lista valorx
  y_create(valory); // Cria a lista valory
  ac_create(ac_unit); // Cria a lista ac_unit
  gy_create(gy_unit); // Cria a lista gy_unit
  
}

public void draw() // Rotina em repeticao permanente
{
  background(255, 255, 255); // Tela de fundo branca
  textFont(f, 16); // Fonte tamanho 16
  
  rectMode(CORNERS); // Modo de desenho dos retangulos como CORNERS
  int mode_xy = 1, // 1: Modo XY, 0: Modo YT
      mode_line = 1, // 1: Modo linha, 0: Modo ponto
      i; // Variavel geral de laco
  final int XMAX = 800,
            gap = 10,
            sqrwidth = 400;
  c++; // Contando em qual execucao esta
  if(c == vSize) // Se o programa encontra-se no valor maximo de dados que se pode salvar
    c = 0; // Sobrescreve o dado mais antigo
    
  switch(PApplet.parseInt(valorx.getValue())) // Selecao de variavel eixo X
  {
    case 0:
      dadox[c] = accel_x;
      break;
    case 1:
      dadox[c] = accel_y;
      break;
    case 2:
      dadox[c] = accel_z;
      break;
    case 3:
      dadox[c] = gyro_x;
      break;
    case 4:
      dadox[c] = gyro_y;
      break;
    case 5:
      dadox[c] = gyro_z;
      break;
  }
  
  switch(PApplet.parseInt(valory.getValue())) // Selecao de variavel eixo Y
  {
    case 0:
      dadoy[c] = accel_x;
      break;
    case 1:
      dadoy[c] = accel_y;
      break;
    case 2:
      dadoy[c] = accel_z;
      break;
    case 3:
      dadoy[c] = gyro_x;
      break;
    case 4:
      dadoy[c] = gyro_y;
      break;
    case 5:
      dadoy[c] = gyro_z;
      break;
  }
  
  noFill(); // Desabilita preenchimento
  rect(XMAX - (gap + sqrwidth), gap, XMAX - gap, sqrwidth + gap); // Grade externa dos eixos
  fill(0); // Preenche proximos desenhos de preto
  line(XMAX - (gap + sqrwidth), gap + sqrwidth/2, XMAX - gap, gap + sqrwidth/2); // Eixo X do plano cartesiano
  mode_xy = PApplet.parseInt(axismode.getValue()); // Le lista axismode
  mode_line = PApplet.parseInt(fillmode.getValue()); // Le lista fillmode
  
  dadoy[c] = gap + sqrwidth/2 - (dadoy[c] * sqrwidth/2) / 32768; // Dados de plot do eixo y
  if(mode_xy != 0)
  {
    dadox[c] = XMAX - (gap + sqrwidth/2) + (dadox[c] * sqrwidth/2) / 32768; // Dados de plot do eixo x
    line(XMAX - (gap + sqrwidth/2), gap, XMAX - (gap + sqrwidth/2), sqrwidth + gap); // Eixo Y do plano cartesiano
    fill(0, 255, 0); // Preenche proximos desenhos de verde
    stroke(0, 255, 0); // Habilita linhas de contorno verdes
    for(i=1;i<vSize;i++) // Varre todos os dados
      if(dadox[i] != 0 && dadoy[i] != 0 && dadox[i-1] != 0 && dadoy[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(dadox[i-1], dadoy[i-1], dadox[i], dadoy[i]);
        else
          rect(dadox[i] - 1, dadoy[i] - 1, dadox[i] + 1, dadoy[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels
    stroke(0); // Habilita linhas de contorno pretas
  }
  else
  {
    dadox[c] = gap + sqrwidth/2 - (dadox[c] * sqrwidth/2) / 32768; // Dados de plot do eixo x
    
    for(i=1;i<vSize;i++)
    {
      stroke(0, 255, 0); // Habilita linhas de contorno verdes
      if(dadoy[i] != 0 && dadoy[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadoy[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadoy[i]);
        else
          rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadoy[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadoy[i] + 1);
          
      stroke(0, 0, 255); // Habilita linhas de contorno azuis
      if(dadox[i] != 0 && dadox[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadox[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadox[i]);
        else
          rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadox[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadox[i] + 1);
    }
     stroke(0); // Habilita linhas de contorno pretas
  }
  switch(PApplet.parseInt(ac_unit.getValue())) // Selecao de unidade do acelerometro
  {
    case 0:
      acx = nf((accel_x * 2 * 9.81f) / 32768, 1, 3) + " m/s\u00b2"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acy = nf((accel_y * 2 * 9.81f) / 32768, 1, 3) + " m/s\u00b2"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acz = nf((accel_z * 2 * 9.81f) / 32768, 1, 3) + " m/s\u00b2"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      break;
    case 1:
      acx = nf((accel_x * 2) / 32768, 1, 3) + " g"; // Conversao para valores fisicos (gravidades)
      acy = nf((accel_y * 2) / 32768, 1, 3) + " g"; // Conversao para valores fisicos (gravidades)
      acz = nf((accel_z * 2) / 32768, 1, 3) + " g"; // Conversao para valores fisicos (gravidades)
      break;
  }

  switch(PApplet.parseInt(gy_unit.getValue())) // Selecao de unidade do giroscopio
  {
    case 0:
      gyx = nf((gyro_x * 250) / 32768, 1, 3) + " \u00ba/seg"; // Conversao para valores fisicos (graus por segundo)
      gyy = nf((gyro_y * 250) / 32768, 1, 3) + " \u00ba/seg"; // Conversao para valores fisicos (graus por segundo)
      gyz = nf((gyro_z * 250) / 32768, 1, 3) + " \u00ba/seg"; // Conversao para valores fisicos (graus por segundo)
      break;
    case 1:
      gyx = nf((gyro_x * 250 * PI / 180) / 32768, 1, 3) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyy = nf((gyro_y * 250 * PI / 180) / 32768, 1, 3) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyz = nf((gyro_z * 250 * PI / 180) / 32768, 1, 3) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      break;
  }

  fill(0); // Preenche proximos desenhos de preto
  text("Valores a serem representados no grafico:", 10, 290); // Texto informativo
  if(accel_x == -1.0f && accel_y == -1.0f && accel_z == -1.0f && gyro_x == -1.0f && gyro_y == -1.0f && gyro_z == -1.0f && tmp == 36.53f) // Se todos forem iguais ao valor que geralmente representa erro no protocolo I2C de comunicacao
  {
    text("Leitura acelerometro X: Erro na comunicacao!", 10, 20); // Imprime mensagem de erro
    text("Leitura acelerometro Y: Erro na comunicacao!", 10, 40); // Imprime mensagem de erro
    text("Leitura acelerometro Z: Erro na comunicacao!", 10, 60); // Imprime mensagem de erro
    text("Leitura giroscopio X: Erro na comunicacao!", 10, 100); // Imprime mensagem de erro
    text("Leitura giroscopio Y: Erro na comunicacao!", 10, 120); // Imprime mensagem de erro
    text("Leitura giroscopio Z: Erro na comunicacao!", 10, 140); // Imprime mensagem de erro
    text("Temperatura: Erro na comunicacao!", 10, 180); // Imprime mensagem de erro
  }
  else
  {
    text("Leitura acelerometro X: " + acx, 10, 20); // Imprime valor lido
    text("Leitura acelerometro Y: " + acy, 10, 40); // Imprime valor lido
    text("Leitura acelerometro Z: " + acz, 10, 60); // Imprime valor lido
    text("Leitura giroscopio X: " + gyx, 10, 100); // Imprime valor lido
    text("Leitura giroscopio Y: " + gyy, 10, 120); // Imprime valor lido
    text("Leitura giroscopio Z: " + gyz, 10, 140); // Imprime valor lido
    text("Temperatura: " + tmp + "\u00baC", 10, 180); // Imprime valor lido
  }
}

public void serialEvent(Serial myPort) // Rotina de toda vez que algo for escrito na porta serial
{
  String xString = myPort.readStringUntil('\n'); // Ler o que foi escrito ate a quebra de linha
  if(xString != null) // Se algo foi lido
  {
    String  temp[]  =  split(xString,":"); // Separar os dados cada vez que dois-pontos for encontrado
    if(xString.charAt(0)  ==  '#'  &&  temp.length==8) // Se o primeiro caractere escrito for cerquilha e 8 elementos forem lidos
    { 
      /* 
       * Protocolo de comunicacao definido por mim:
       * Mensagem enviada pelo Arduino:
       * #X:32768:32768:32768:32768:32768:32768:32768
       * Sendo cada numero entre os dois-pontos uma das leituras, na ordem:
       * acelerometro x, y, z, temperatura, giroscopio x, y, z
       * */
      accel_x = PApplet.parseFloat(temp[1]); // Atualiza variavel global
      accel_y = PApplet.parseFloat(temp[2]); // Atualiza variavel global
      accel_z = PApplet.parseFloat(temp[3]); // Atualiza variavel global
      tmp = PApplet.parseFloat(temp[4]); // Atualiza variavel global
      gyro_x = PApplet.parseFloat(temp[5]); // Atualiza variavel global
      gyro_y = PApplet.parseFloat(temp[6]); // Atualiza variavel global
      gyro_z = PApplet.parseFloat(temp[7]); // Atualiza variavel global
    }
  }
}

public void axis_create(DropdownList ddl) // Customizar a lista axismode 
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("Modo YT", 0); // Adicionado item
  ddl.addItem("Modo XY", 1); // Adicionado item
}

public void fill_create(DropdownList ddl) // Customizar a lista fillmode
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("Pontos", 0); // Adicionado item
  ddl.addItem("Linhas", 0); // Adicionado item
}

public void x_create(DropdownList ddl) // Customizar a lista valorx
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("Acel X", 0); // Adicionado item
  ddl.addItem("Acel Y", 1); // Adicionado item
  ddl.addItem("Acel Z", 2); // Adicionado item
  ddl.addItem("Gyro X", 3); // Adicionado item
  ddl.addItem("Gyro Y", 4); // Adicionado item
  ddl.addItem("Gyro Z", 5); // Adicionado item
}

public void y_create(DropdownList ddl) // Customizar a lista valory
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("Acel X", 0); // Adicionado item
  ddl.addItem("Acel Y", 1); // Adicionado item
  ddl.addItem("Acel Z", 2); // Adicionado item
  ddl.addItem("Gyro X", 3); // Adicionado item
  ddl.addItem("Gyro Y", 4); // Adicionado item
  ddl.addItem("Gyro Z", 5); // Adicionado item
}

public void ac_create(DropdownList ddl) // Customizar a lista fillmode
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("m/s\u00b2", 0); // Adicionado item
  ddl.addItem("g", 0); // Adicionado item
}

public void gy_create(DropdownList ddl) // Customizar a lista fillmode
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("grau/s", 0); // Adicionado item
  ddl.addItem("rad/s", 0); // Adicionado item
}

public void ddl_standard(DropdownList ddl) // Customizacao padrao de toda lista
{
  ddl.setBackgroundColor(color(220)); // Cor de fundo da lista
  ddl.setItemHeight(20); // Tamanho de cada item mostrado
  ddl.setBarHeight(15); // Tamanho da barra
  ddl.setColorBackground(color(60)); // Cor do fundo para itens e barra
  ddl.setColorActive(color(255,128)); // Cor do item quando ativado por mouse
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
