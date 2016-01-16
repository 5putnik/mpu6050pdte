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

final float reg_ac = 2, // Faixa do acelerometro: +/- 2g
            reg_gy = 250; // Faixa do giroscopio: +/- 250\u00ba/s

Serial myPort; // Porta serial do Arduino a ser lida
int vSize = 400; // Quantidade de dados a ser salva
final int maxSize = 1000; // Quantidade maxima de dados a ser salva
float dadox[] = new float[maxSize], dadoy[] = new float[maxSize], dadoz[] = new float[maxSize]; // Dados a serem mostrados no grafico
float f_dadox[] = new float[maxSize], f_dadoy[] = new float[maxSize], f_dadoz[] = new float[maxSize]; // Dados a serem filtrados
int c = 0; // Contador

final float dt = 0.01f; // Passo entre as medicoes (10 ms)

PFont f; // Fonte do texto

ControlP5 cp5; // Controle para utilizar objetos
DropdownList axismode, // lista para escolher o modo eixo (XY ou YT)
             fillmode, // lista para escolher o modo preenchimento (linha ou ponto)
             valorx, // lista de valores a serem plotados (Eixo X ou valor 1)
             valory, // lista de valores a serem plotados (Eixo Y ou valor 2)
             valorz, // lista de valores a serem plotados (valor 3, apenas para YT)
             ac_unit, // lista de unidade do acelerometro (m/s\u00b2 ou g)
             gy_unit, // lista de unidade do giroscopio (\u00ba/s ou rad/s)
             filter, // lista de filtros
             sample; // selecao de dados amostrados
             
final int XMAX = 800, // Parametro: Tamanho X da tela
          gap = 10, // Parametro: espacamento geral
          sqrwidth = 400; // Parametro: tamanho do grafico
          
int mode_xy = 1, // 1: Modo XY, 0: Modo YT
    mode_line = 1, // 1: Modo linha, 0: Modo ponto
    mode_x = 1,
    mode_y = 1,
    mode_filter = 1,
    mode_sample = 1;
    
public void setup() // Inicializacao do programa
{
   // Gerando uma tela 800x600 com renderizacao 2D melhorada
  myPort = new Serial(this, Serial.list()[0], 9600); // Associando MyPort as portas seriais do computador
  myPort.bufferUntil('\n'); // Busca por \n
  colorMode(RGB, 1);
  f = createFont("Arial", 16, true); // Escolhendo fonte do texto como Arial 16
  
  cp5 = new ControlP5(this);
  axismode = cp5.addDropdownList("Modo YT", 10, 220, 100, 84);
  fillmode = cp5.addDropdownList("Pontos", 120, 220, 100, 84);
  valorx = cp5.addDropdownList("Acel X", 10, 270, 100, 84);
  valory = cp5.addDropdownList("Acel Y", 120, 270, 100, 84);
  valorz = cp5.addDropdownList("Acel Z", 230, 270, 100, 84);
  ac_unit = cp5.addDropdownList("m/s^2", 270, 20, 100, 84);
  gy_unit = cp5.addDropdownList("grau/s", 270, 100, 100, 84);
  filter = cp5.addDropdownList("Raw", XMAX/2 + 45, 2*gap + sqrwidth, 100, 84);
  sample = cp5.addDropdownList("400 (padrao)", XMAX/2 + 245, 2*gap + sqrwidth, 100, 84);
  
  axis_create(axismode); // Cria a lista axismode
  fill_create(fillmode); // Cria a lista fillmode
  x_create(valorx); // Cria a lista valorx
  y_create(valory); // Cria a lista valory
  z_create(valorz); // Cria a lista valorz
  ac_create(ac_unit); // Cria a lista ac_unit
  gy_create(gy_unit); // Cria a lista gy_unit
  filter_create(filter); // Cria a lista filter
  sample_create(sample); // Cria a lista sample
  
}

public void draw() // Rotina em repeticao permanente
{
  background(255, 255, 255); // Tela de fundo branca
  textFont(f, 16); // Fonte tamanho 16
  rectMode(CORNERS); // Modo de desenho dos retangulos como CORNERS
  int i; // Variavel geral de laco
  switch(PApplet.parseInt(sample.getValue())) // Selecao de quantidade de dados gravados por ciclo
  {
    case 0:
      vSize = 400;
      break;
    case 1:
      vSize = 50;
      break;
    case 2:
      vSize = 100;
      break;
    case 3:
      vSize = 200;
      break;
    case 4:
      vSize = 250;
      break;
    case 5:
      vSize = 500;
      break;
    case 6:
      vSize = 750;
      break;
    case 7:
      vSize = 1000;
      break;
  }

  c++; // Contando em qual execucao esta
  if(c == vSize) // Se o programa encontra-se no valor maximo de dados que se pode salvar
    c = 0; // Sobrescreve o dado mais antigo
  
  if(mode_x != PApplet.parseInt(valorx.getValue()) || mode_y != PApplet.parseInt(valory.getValue()) || mode_filter != PApplet.parseInt(filter.getValue()) || mode_xy != PApplet.parseInt(axismode.getValue()) || mode_sample != PApplet.parseInt(sample.getValue())) // Se o grafico for alterado
    for(i=0;i<vSize;i++)
    {
      dadox[i] = 0; // Reseta (limpa) eixo X do grafico
      dadoy[i] = 0; // Reseta (limpa) eixo Y do grafico
      dadoz[i] = 0; // Reseta (limpa) eixo Z do grafico
      c = 0; // Volta contador ao inicio
    }
  mode_sample = PApplet.parseInt(sample.getValue());
  mode_x = PApplet.parseInt(valorx.getValue());
  mode_y = PApplet.parseInt(valory.getValue());
  mode_filter = PApplet.parseInt(filter.getValue());
  
  switch(PApplet.parseInt(valorx.getValue())) // Selecao de variavel eixo X
  {
    case 0:
      f_dadox[c] = accel_x;
      break;
    case 1:
      f_dadox[c] = accel_y;
      break;
    case 2:
      f_dadox[c] = accel_z;
      break;
    case 3:
      f_dadox[c] = gyro_x;
      break;
    case 4:
      f_dadox[c] = gyro_y;
      break;
    case 5:
      f_dadox[c] = gyro_z;
      break;
  }
  switch(PApplet.parseInt(valory.getValue())) // Selecao de variavel eixo Y
  {
    case 0:
      f_dadoy[c] = accel_y;
      break;
    case 1:
      f_dadoy[c] = accel_z;
      break;
    case 2:
      f_dadoy[c] = accel_z;
      break;
    case 3:
      f_dadoy[c] = gyro_x;
      break;
    case 4:
      f_dadoy[c] = gyro_y;
      break;
    case 5:
      f_dadoy[c] = gyro_z;
      break;
  }
  
  switch(PApplet.parseInt(valorz.getValue())) // Selecao de variavel 3
  {
    case 0:
      f_dadoz[c] = accel_z;
      break;
    case 1:
      f_dadoz[c] = accel_y;
      break;
    case 2:
      f_dadoz[c] = accel_x;
      break;
    case 3:
      f_dadoz[c] = gyro_x;
      break;
    case 4:
      f_dadoz[c] = gyro_y;
      break;
    case 5:
      f_dadoz[c] = gyro_z;
      break;
  }
  
  if(c > 0) switch(PApplet.parseInt(filter.getValue())) // Selecao do filtro (somente a partir da segunda leitura)
  {
    case 0: // Valores crus: fazer nada
      break;
    case 1:  // Passa-baixas
      f_dadox[c] = lowpass(f_dadox[c], f_dadox[c-1], 0.96f);
      f_dadoy[c] = lowpass(f_dadoy[c], f_dadoy[c-1], 0.96f);
      f_dadoz[c] = lowpass(f_dadoz[c], f_dadoz[c-1], 0.96f);
      break;
    case 2:  // Kalman
      // UNDER CONSTRUCTION
      break;
    case 3:  // Complementar
      // UNDER CONSTRUCTION
      break;
  }
  
  noFill(); // Desabilita preenchimento
  rect(XMAX - (gap + sqrwidth), gap, XMAX - gap, sqrwidth + gap); // Grade externa dos eixos
  fill(0); // Preenche proximos desenhos de preto
  line(XMAX - (gap + sqrwidth), gap + sqrwidth/2, XMAX - gap, gap + sqrwidth/2); // Eixo X do plano cartesiano
  mode_xy = PApplet.parseInt(axismode.getValue()); // Le lista axismode
  mode_line = PApplet.parseInt(fillmode.getValue()); // Le lista fillmode
  
  dadoy[c] = gap + sqrwidth/2 - (f_dadoy[c] * sqrwidth/2) / 32768; // Dados de plot do eixo y
  if(mode_xy != 0)
  {
    dadox[c] = XMAX - (gap + sqrwidth/2) + (f_dadox[c] * sqrwidth/2) / 32768; // Dados de plot do eixo x
    valorz.hide();
    line(XMAX - (gap + sqrwidth/2), gap, XMAX - (gap + sqrwidth/2), sqrwidth + gap); // Eixo Y do plano cartesiano
    fill(0, 255, 0); // Preenche proximos desenhos de verde
    stroke(0, 255, 0); // Habilita linhas de contorno verdes
    for(i=1;i<vSize;i++) // Varre todos os dados
      if(dadox[i] != 0 && dadoy[i] != 0 && dadox[i-1] != 0 && dadoy[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(dadox[i-1], dadoy[i-1], dadox[i], dadoy[i]); // Desenha linhas
        else
          rect(dadox[i] - 1, dadoy[i] - 1, dadox[i] + 1, dadoy[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels
    stroke(0); // Habilita linhas de contorno pretas
  }
  else
  {
    valorz.show();
    dadox[c] = gap + sqrwidth/2 - (f_dadox[c] * sqrwidth/2) / 32768; // Dados de plot 1
    dadoz[c] = gap + sqrwidth/2 - (f_dadoz[c] * sqrwidth/2) / 32768; // Dados de plot 3
    for(i=1;i<vSize;i++)
    {
      stroke(255, 0, 0); // Habilita linhas de contorno vermelhos
      if(dadox[i] != 0 && dadox[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadox[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadox[i]); // Desenha linhas
        else
          rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadox[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadox[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels

      stroke(0, 255, 0); // Habilita linhas de contorno verdes
      if(dadoy[i] != 0 && dadoy[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadoy[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadoy[i]); // Desenha linhas
        else
          rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadoy[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadoy[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels

      stroke(0, 0, 255); // Habilita linhas de contorno azuis
      if(dadoz[i] != 0 && dadoz[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadoz[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadoz[i]); // Desenha linhas
        else
          rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadoz[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadoz[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels
    }
     stroke(0); // Habilita linhas de contorno pretas
  }
  switch(PApplet.parseInt(ac_unit.getValue())) // Selecao de unidade do acelerometro
  {
    case 0:
      acx = nf(conv_ac(accel_x, 0), 1, 3) + " m/s\u00b2"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acy = nf(conv_ac(accel_y, 0), 1, 3) + " m/s\u00b2"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acz = nf(conv_ac(accel_z, 0), 1, 3) + " m/s\u00b2"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      break;
    case 1:
      acx = nf(conv_ac(accel_x, 1), 1, 3) + " g"; // Conversao para valores fisicos (gravidades)
      acy = nf(conv_ac(accel_x, 1), 1, 3) + " g"; // Conversao para valores fisicos (gravidades)
      acz = nf(conv_ac(accel_x, 1), 1, 3) + " g"; // Conversao para valores fisicos (gravidades)
      break;
  }

  switch(PApplet.parseInt(gy_unit.getValue())) // Selecao de unidade do giroscopio
  {
    case 0:
      gyx = nf(conv_gy(gyro_x, 0), 1, 3) + " \u00ba/seg"; // Conversao para valores fisicos (graus por segundo)
      gyy = nf(conv_gy(gyro_y, 0), 1, 3) + " \u00ba/seg"; // Conversao para valores fisicos (graus por segundo)
      gyz = nf(conv_gy(gyro_z, 0), 1, 3) + " \u00ba/seg"; // Conversao para valores fisicos (graus por segundo)
      break;
    case 1:
      gyx = nf(conv_gy(gyro_x, 1), 1, 3) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyy = nf(conv_gy(gyro_x, 1), 1, 3) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyz = nf(conv_gy(gyro_x, 1), 1, 3) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      break;
  }

  fill(0); // Preenche proximos desenhos de preto
  text("Valores a serem representados no grafico:", 10, 260); // Texto informativo
  text("Propriedades de visualizacao:", 10, 210); // Texto informativo
  text("Unidade:", 270, 16); // Texto informativo
  text("Unidade:", 270, 96); // Texto informativo
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
  
  text("Filtro:", XMAX/2, 3*gap + sqrwidth); 
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
  ddl.addItem("Acel Y", 0); // Adicionado item
  ddl.addItem("Acel X", 1); // Adicionado item
  ddl.addItem("Acel Z", 2); // Adicionado item
  ddl.addItem("Gyro X", 3); // Adicionado item
  ddl.addItem("Gyro Y", 4); // Adicionado item
  ddl.addItem("Gyro Z", 5); // Adicionado item
}

public void z_create(DropdownList ddl) // Customizar a lista valorz
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("Acel Z", 0); // Adicionado item
  ddl.addItem("Acel Y", 1); // Adicionado item
  ddl.addItem("Acel X", 2); // Adicionado item
  ddl.addItem("Gyro X", 3); // Adicionado item
  ddl.addItem("Gyro Y", 4); // Adicionado item
  ddl.addItem("Gyro Z", 5); // Adicionado item
}

public void ac_create(DropdownList ddl) // Customizar a lista do acelerometro
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("m/s^2", 0); // Adicionado item
  ddl.addItem("g", 1); // Adicionado item
}

public void gy_create(DropdownList ddl) // Customizar a lista do giroscopio
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("grau/s", 0); // Adicionado item
  ddl.addItem("rad/s", 1); // Adicionado item
}

public void filter_create(DropdownList ddl) // Customizar a lista do filtro
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("Raw", 0); // Adicionado item
  ddl.addItem("Low-pass", 1); // Adicionado item
  ddl.addItem("Kalman", 2); // Adicionado item
  ddl.addItem("Complementary", 3); // Adicionado item
}

public void sample_create(DropdownList ddl) // Customizar a lista do filtro
{
  ddl_standard(ddl); // Parametros iniciais da lista
  ddl.addItem("400 (padrao)", 0); // Adicionado item
  ddl.addItem("50", 1); // Adicionado item
  ddl.addItem("100", 2); // Adicionado item
  ddl.addItem("200", 3); // Adicionado item
  ddl.addItem("250", 3); // Adicionado item
  ddl.addItem("500", 5); // Adicionado item
  ddl.addItem("750", 6); // Adicionado item
  ddl.addItem("1000", 7); // Adicionado item
}

public void ddl_standard(DropdownList ddl) // Customizacao padrao de toda lista
{
  ddl.setBackgroundColor(color(220)); // Cor de fundo da lista
  ddl.setItemHeight(20); // Tamanho de cada item mostrado
  ddl.setBarHeight(15); // Tamanho da barra
  ddl.setColorBackground(color(60)); // Cor do fundo para itens e barra
  ddl.setColorActive(color(255,128)); // Cor do item quando ativado por mouse
  ddl.close();
}

public float lowpass(float read, float old, float pct)
{
  return pct*read + (1-pct)*old;
}

public float conv_ac(float val, int c) // c = 0: m/s^2 , c = 1: g
{
  if(c == 0)
    return (val * reg_ac * 9.81f) / 32768;
  else
    return (val * reg_ac) / 32768;
}

public float conv_gy(float val, int c) // c = 0: \u00ba/s , c = 1: g
{
  if(c == 0)
    return (val * reg_gy) / 32768;
  else
    return (val * reg_gy * PI / 180) / 32768;
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
