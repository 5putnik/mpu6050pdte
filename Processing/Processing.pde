import controlP5.*;
import processing.serial.*;

class Obj_Kalman
{
  float Qt,
        Qtb,
        R,
        P00,
        P01,
        P10,
        P11,
        bias,
        ang;

  Obj_Kalman(float Qt,
        float Qtb,
        float R,
        float P00,
        float P01,
        float P10,
        float P11,
        float bias,
        float ang)
  {
    this.Qt = Qt;
    this.Qtb = Qtb;
    this.R = R;
    this.P00 = P00;
    this.P01 = P01;
    this.P10 = P10;
    this.P11 = P11;
    this.bias = bias;
    this.ang = ang;
  }

  float kalman_step(float read_vel, float read_ang)
  {
    float rate = read_vel - bias;
    ang += dt * rate;
    P00 += dt * (dt*P11 - P01 - P10 + Qt);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += dt * Qtb;
    float S = P00 + R;
    float K0 = P00/S;
    float K1 = P10/S;
    float temp_p00 = P00;
    float temp_p01 = P01;
    P00 -= K0*temp_p00;
    P01 -= K0*temp_p01;
    P10 -= K1*temp_p00;
    P11 -= K1*temp_p01;
    float y = read_ang - ang;
    ang += K0*y;
    bias += K1*y;
    
    return ang;
  }
}

float accel_x, accel_y, accel_z, tmp,gyro_x, gyro_y, gyro_z = 0; // Valores medidos pelo sensor
String acx, acy, acz, gyx, gyy, gyz; // Valores convertidos para mostrar na tela 

final float reg_ac = 2, // Faixa do acelerometro: +/- 2g
            reg_gy = 250; // Faixa do giroscopio: +/- 250º/s

Serial myPort; // Porta serial do Arduino a ser lida
int vSize = 400; // Quantidade de dados a ser salva
final int maxSize = 1000; // Quantidade maxima de dados a ser salva
float dadox[] = new float[maxSize], dadoy[] = new float[maxSize], dadoz[] = new float[maxSize]; // Dados a serem mostrados no grafico
float f_dadox[] = new float[maxSize], f_dadoy[] = new float[maxSize], f_dadoz[] = new float[maxSize]; // Dados a serem filtrados
int c = 0; // Contador

Obj_Kalman k_x, k_y, k_z; // Variaveis que serao submetidas ao filtro Kalman

float ang_x = 0, // Valor Angulo X em graus
      ang_y = 0, // Valor Angulo Y em graus
      ang_z = 0; // Valor Angulo Z em graus

float grav_x, grav_y, grav_z; // Componentes da gravidade
float gx = 0, // Leitura inicial da gravidade
      gy = 32768/reg_ac, // Leitura inicial da gravidade (admite-se gravidade atuando unicamente no eixo Y)
      gz = 0; // Leitura inicial da gravidade

final float dt = 0.02; // Passo entre as medicoes (20 ms)

PFont f; // Fonte do texto

ControlP5 cp5; // Controle para utilizar objetos
DropdownList axismode, // lista para escolher o modo eixo (XY ou YT)
             fillmode, // lista para escolher o modo preenchimento (linha ou ponto)
             valorx, // lista de valores a serem plotados (Eixo X ou valor 1)
             valory, // lista de valores a serem plotados (Eixo Y ou valor 2)
             valorz, // lista de valores a serem plotados (valor 3, apenas para YT)
             ac_unit, // lista de unidade do acelerometro (m/s² ou g)
             gy_unit, // lista de unidade do giroscopio (º/s ou rad/s)
             filter, // lista de filtros
             scale, // lista de escalas
             sample; // selecao de dados amostrados
             
final int XMAX = 800, // Parametro: Tamanho X da tela
          gap = 10, // Parametro: espacamento geral
          sqrwidth = 400; // Parametro: tamanho do grafico
          
int mode_xy = 1, // 1: Modo XY, 0: Modo YT
    mode_line = 1, // 1: Modo linha, 0: Modo ponto
    mode_x = 1, // Modo de selecao do eixo X (modo XY) ou variavel 1 (modo YT)
    mode_y = 1, // Modo de selecao do eixo Y (modo XY) ou variavel 2 (modo YT)
    mode_z = 1, // Modo de selecao variavel 3 (apenas modo YT)
    mode_filter = 1, // 1: Filtro passa-baixas selecionado, 0: Valores crus
    mode_scale = 1, // Modo de selecao de escala
    mode_sample = 1; // Modo de selecao da taxa de dados salvos por ciclo
    
void setup() // Inicializacao do programa
{
  size(800, 600, P2D); // Gerando uma tela 800x600 com renderizacao 2D melhorada
  myPort = new Serial(this, Serial.list()[0], 9600); // Associando MyPort as portas seriais do computador
  myPort.bufferUntil('\n'); // Busca por \n
  colorMode(RGB, 1);
  f = createFont("Arial", 16, true); // Escolhendo fonte do texto como Arial 16
  cp5 = new ControlP5(this); // Inicializacao dos controles P5
  axismode = cp5.addDropdownList("Modo YT", 10, 220, 100, 84); // Insercao da lista axismode
  fillmode = cp5.addDropdownList("Pontos", 120, 220, 100, 84); // Insercao da lista fillmode
  valorx = cp5.addDropdownList("Acel X", 10, 270, 100, 84); // Insercao da lista valorx
  valory = cp5.addDropdownList("Acel Y", 120, 270, 100, 84); // Insercao da lista valory
  valorz = cp5.addDropdownList("Acel Z", 230, 270, 100, 84); // Insercao da lista valorz
  ac_unit = cp5.addDropdownList("m/s^2", 270, 20, 100, 84); // Insercao da lista ac_unit
  gy_unit = cp5.addDropdownList("grau/s", 270, 100, 100, 84); // Insercao da lista gy_unit
  filter = cp5.addDropdownList("Raw", XMAX/2 + 45, 2*gap + sqrwidth, 100, 84); // Insercao da lista filter
  scale = cp5.addDropdownList("1x", XMAX/2 + 150, 2*gap + sqrwidth, 100, 84); // Insercao da lista scale
  sample = cp5.addDropdownList("400 (padrao)", XMAX/2 + 255, 2*gap + sqrwidth, 100, 84); // Insercao da lista sample
  
  ddl_standard(axismode, "Modo YT:Modo XY"); // Cria a lista axismode
  ddl_standard(fillmode, "Pontos:Linhas"); // Cria a lista fillmode
  ddl_standard(scale, "1x:2x:3x:4x:5x"); // Cria a lista scale
  ddl_standard(filter, "Raw:Low-pass"); // Cria a lista filter
  ddl_standard(sample, "400 (padrao):50:100:200:250:500:750:1000"); // Cria a lista filter
  ddl_standard(ac_unit, "m/s^2:g"); // Cria a lista ac_unit
  ddl_standard(gy_unit, "grau/s:rad/s"); // Cria a lista gy_unit
  ddl_standard(valorx, "Acel X:Acel Y:Acel Z:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z"); // Cria a lista valorx
  ddl_standard(valory, "Acel Y:Acel X:Acel Z:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z"); // Cria a lista valory
  ddl_standard(valorz, "Acel Z:Acel Y:Acel X:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z"); // Cria a lista valorz
  
  
  
  valorx.setBackgroundColor(color(255, 0, 0)); // Cor de fundo da lista
  valory.setBackgroundColor(color(0, 255, 0)); // Cor de fundo da lista
  valorz.setBackgroundColor(color(0, 0, 255)); // Cor de fundo da lista
  
  // Inicializando os parametros do filtro Kalman (Q, R, P etc.)
  k_x = new Obj_Kalman(0.002, // Qt
        0.003, // Qtb
        0.01, // R
        0, // P00
        0, // P01
        0, // P10
        0, // P11
        0, // bias
        0); // ang
   k_y = new Obj_Kalman(0.002, // Qt
        0.003, // Qtb
        0.01, // R
        0, // P00
        0, // P01
        0, // P10
        0, // P11
        1.0, // bias
        0); // ang
   k_z = new Obj_Kalman(0.002, // Qt
        0.003, // Qtb
        0.01, // R
        0, // P00
        0, // P01
        0, // P10
        0, // P11
        0, // bias
        0); // ang
}

void draw() // Rotina em repeticao permanente
{
  background(255, 255, 255); // Tela de fundo branca
  textFont(f, 16); // Fonte tamanho 16
  rectMode(CORNERS); // Modo de desenho dos retangulos como CORNERS
  int i; // Variavel geral de laco
  
  ang_x = k_x.kalman_step(conv_gy(gyro_x, 0), ang_x); // Filtro de Kalman para selecionar o angulo
  ang_y = k_y.kalman_step(conv_gy(gyro_y, 0), ang_y); // Filtro de Kalman para selecionar o angulo
  ang_z = k_z.kalman_step(conv_gy(gyro_z, 0), ang_z); // Filtro de Kalman para selecionar o angulo
  
  grav_x = rot(gx, gy, gz, ang_x*PI/180, ang_y*PI/180, ang_z*PI/180, 0); // Rotacionando o vetor (gx,gy,gz) com as leituras do giroscopio
  grav_y = rot(gx, gy, gz, ang_x*PI/180, ang_y*PI/180, ang_z*PI/180, 1); // Rotacionando o vetor (gx,gy,gz) com as leituras do giroscopio
  grav_z = rot(gx, gy, gz, ang_x*PI/180, ang_y*PI/180, ang_z*PI/180, 2); // Rotacionando o vetor (gx,gy,gz) com as leituras do giroscopio
  
  text("Gravidade x: " + nf(conv_ac(grav_x,0),1,0) + "m/s² (" + nf(conv_ac(grav_x,1),1,0) + " g)", 10, 480); // Imprimindo o valor da gravidade na tela
  text("Gravidade y: " + nf(conv_ac(grav_y,0),1,0) + "m/s² (" + nf(conv_ac(grav_y,1),1,0) + " g)", 10, 500); // Imprimindo o valor da gravidade na tela
  text("Gravidade z: " + nf(conv_ac(grav_z,0),1,0) + "m/s² (" + nf(conv_ac(grav_z,1),1,0) + " g)", 10, 520); // Imprimindo o valor da gravidade na tela
  
  text("Angulo x: " + nf(ang_x,1,0) + "º", 10, 540); // Imprimindo o valor do angulo na tela
  text("Angulo y: " + nf(ang_y,1,0) + "º", 10, 560); // Imprimindo o valor do angulo na tela
  text("Angulo z: " + nf(ang_z,1,0) + "º", 10, 580); // Imprimindo o valor do angulo na tela
  
  switch(int(sample.getValue())) // Selecao de quantidade de dados gravados por ciclo
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
  
  if(mode_x != int(valorx.getValue()) || 
     mode_y != int(valory.getValue()) || 
     mode_z != int(valorz.getValue()) || 
     mode_filter != int(filter.getValue()) || 
     mode_xy != int(axismode.getValue()) || 
     mode_sample != int(sample.getValue()) || 
     mode_scale != int(scale.getValue())) // Se o grafico for alterado
    for(i=0;i<vSize;i++)
    {
      dadox[i] = 0; // Reseta (limpa) eixo X (modo XY) ou o valor 1 (modo YT) do grafico
      dadoy[i] = 0; // Reseta (limpa) eixo Y (modo XY) ou o valor 2 (modo YT) do grafico
      dadoz[i] = 0; // Reseta (limpa) o valor 3 (apenas modo YT) do grafico
      c = 0; // Volta contador ao inicio
    }
    
  mode_sample = int(sample.getValue()); // Salvando alteracoes na lista
  mode_x = int(valorx.getValue()); // Salvando alteracoes na lista
  mode_y = int(valory.getValue()); // Salvando alteracoes na lista
  mode_z = int(valorz.getValue()); // Salvando alteracoes na lista
  mode_filter = int(filter.getValue()); // Salvando alteracoes na lista
  mode_scale = int(scale.getValue());  // Salvando alteracoes na lista
  
  switch(int(valorx.getValue())) // Selecao de variavel eixo X
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
    default:
      f_dadox[c] = getdata(int(valorx.getValue()));
  }
  switch(int(valory.getValue())) // Selecao de variavel eixo Y
  {
    case 0:
      f_dadoy[c] = accel_y;
      break;
    case 1:
      f_dadoy[c] = accel_z;
      break;
    case 2:
      f_dadoy[c] = accel_x;
      break;
    default:
      f_dadoy[c] = getdata(int(valory.getValue()));
  }
  
  switch(int(valorz.getValue())) // Selecao de variavel 3
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
    default:
      f_dadoz[c] = getdata(int(valorz.getValue()));
  }
  
  if(c > 0) switch(int(filter.getValue())) // Selecao do filtro (somente a partir da segunda leitura)
  {
    case 0: // Valores crus: fazer nada
      break;
    case 1:  // Passa-baixas
      f_dadox[c] = lowpass(f_dadox[c], f_dadox[c-1], 0.96);
      f_dadoy[c] = lowpass(f_dadoy[c], f_dadoy[c-1], 0.96);
      f_dadoz[c] = lowpass(f_dadoz[c], f_dadoz[c-1], 0.96);
      break;
  }
  
  noFill(); // Desabilita preenchimento
  rect(XMAX - (gap + sqrwidth), gap, XMAX - gap, sqrwidth + gap); // Grade externa dos eixos
  fill(0); // Preenche proximos desenhos de preto
  line(XMAX - (gap + sqrwidth), gap + sqrwidth/2, XMAX - gap, gap + sqrwidth/2); // Eixo X do plano cartesiano
  mode_xy = int(axismode.getValue()); // Le lista axismode
  mode_line = int(fillmode.getValue()); // Le lista fillmode
  
  dadoy[c] = gap + sqrwidth/2 - (1 + mode_scale)*(f_dadoy[c] * sqrwidth/2) / 32768; // Dados de plot do eixo y
  if(mode_xy != 0)
  {
    dadox[c] = XMAX - (gap + sqrwidth/2) + (1 + mode_scale)*(f_dadox[c] * sqrwidth/2) / 32768; // Dados de plot do eixo x
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
    dadox[c] = gap + sqrwidth/2 - (1 + mode_scale)*(f_dadox[c] * sqrwidth/2) / 32768; // Dados de plot 1
    dadoz[c] = gap + sqrwidth/2 - (1 + mode_scale)*(f_dadoz[c] * sqrwidth/2) / 32768; // Dados de plot 3
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
  switch(int(ac_unit.getValue())) // Selecao de unidade do acelerometro
  {
    case 0:
      acx = nf(conv_ac(accel_x, 0), 1, 2) + " m/s²"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acy = nf(conv_ac(accel_y, 0), 1, 2) + " m/s²"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acz = nf(conv_ac(accel_z, 0), 1, 2) + " m/s²"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      break;
    case 1:
      acx = nf(conv_ac(accel_x, 1), 1, 2) + " g"; // Conversao para valores fisicos (gravidades)
      acy = nf(conv_ac(accel_y, 1), 1, 2) + " g"; // Conversao para valores fisicos (gravidades)
      acz = nf(conv_ac(accel_z, 1), 1, 2) + " g"; // Conversao para valores fisicos (gravidades)
      break;
  }

  switch(int(gy_unit.getValue())) // Selecao de unidade do giroscopio
  {
    case 0:
      gyx = nf(conv_gy(gyro_x, 0), 1, 2) + " º/seg"; // Conversao para valores fisicos (graus por segundo)
      gyy = nf(conv_gy(gyro_y, 0), 1, 2) + " º/seg"; // Conversao para valores fisicos (graus por segundo)
      gyz = nf(conv_gy(gyro_z, 0), 1, 2) + " º/seg"; // Conversao para valores fisicos (graus por segundo)
      break;
    case 1:
      gyx = nf(conv_gy(gyro_x, 1), 1, 2) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyy = nf(conv_gy(gyro_y, 1), 1, 2) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyz = nf(conv_gy(gyro_z, 1), 1, 2) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      break;
  }

  fill(0); // Preenche proximos desenhos de preto
  text("Valores a serem representados no grafico:", 10, 260); // Texto informativo
  text("Propriedades de visualizacao:", 10, 210); // Texto informativo
  text("Filtro:", XMAX/2, 3*gap + sqrwidth); // Texto informativo 
  text("Unidade:", 270, 16); // Texto informativo
  text("Unidade:", 270, 96); // Texto informativo
  if(accel_x == -1.0 && accel_y == -1.0 && accel_z == -1.0 && gyro_x == -1.0 && gyro_y == -1.0 && gyro_z == -1.0 && tmp == 36.53) // Se todos forem iguais ao valor que geralmente representa erro no protocolo I2C de comunicacao
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
    text("Temperatura: " + tmp + "ºC", 10, 180); // Imprime valor lido
  }
}

void serialEvent(Serial myPort) // Rotina de toda vez que algo for escrito na porta serial
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
      accel_x = float(temp[1]); // Atualiza variavel global
      accel_y = float(temp[2]); // Atualiza variavel global
      accel_z = float(temp[3]); // Atualiza variavel global
      tmp = float(temp[4]); // Atualiza variavel global
      gyro_x = float(temp[5]); // Atualiza variavel global
      gyro_y = float(temp[6]); // Atualiza variavel global
      gyro_z = float(temp[7]); // Atualiza variavel global
    }
  }
}

void ddl_standard(DropdownList ddl, String s) // Customizacao padrao de toda lista
{
  String  temp[]  =  split(s,":"); // Separar os dados cada vez que dois-pontos for encontrado
  int iMax = temp.length;
  for(int i=0;i<iMax;i++)
    ddl.addItem(temp[i], i); // Adicionado item
  ddl.setBackgroundColor(color(220)); // Cor de fundo da lista
  ddl.setItemHeight(20); // Tamanho de cada item mostrado
  ddl.setBarHeight(15); // Tamanho da barra
  ddl.setColorBackground(color(60)); // Cor do fundo para itens e barra
  ddl.setColorActive(color(255,128)); // Cor do item quando ativado por mouse
  ddl.close();
}

float lowpass(float read, float old, float pct)
{
  return pct*read + (1-pct)*old;
}

float conv_ac(float val, int c) // c = 0: m/s^2 , c = 1: g
{
  if(c == 0)
    return (val * reg_ac * 9.81) / 32768;
  else
    return (val * reg_ac) / 32768;
}

float conv_gy(float val, int c) // c = 0: º/s , c = 1: g
{
  if(c == 0)
    return (val * reg_gy) / 32768;
  else
    return (val * reg_gy * PI / 180) / 32768;
}

float getdata(int val)
{
  switch(val) // Selecao de variavel
  {
    case 3:
      return gyro_x;
    case 4:
      return gyro_y;
    case 5:
      return gyro_z;
    case 6:
      return ang_x*32768/360;
    case 7:
      return ang_y*32768/360;
    case 8:
      return ang_z*32768/360;
    default:
      return -1.0;
  }
}

/* Funcao rot() retorna o valor rotacionado de uma variavel
 * A funcao usa as coordenadas originais (x, y, z) e rotaciona
 * de acordo com os angulos de euler (a, b, c) (em radianos). 
 * A funcao retorna uma das tres coordenadas, de acordo com o valor
 * da flag 'resp'
 * */
float rot(float x, float y, float z, float a, float b, float c, int resp) 
{
  switch(resp)
  {
    case 0: // Resposta: eixo x
      return (x*cos(b)*cos(c)+z*sin(b)-y*cos(b)*sin(c));
    case 1: // Resposta: eixo y
      return (-z*cos(b)*sin(a)+x*(cos(c)*sin(a)*sin(b)+cos(a)*sin(c))+y*(cos(a)*cos(c)-sin(a)*sin(b)*sin(c)));
    case 2: // Resposta: eixo z
      return (z*cos(a)*cos(b)+x*(sin(a)*sin(c)-cos(a)*cos(c)*sin(b))+y*(cos(c)*sin(a)+cos(a)*sin(b)*sin(c)));
    default:
      return 0; // Retorna 0 para todos os outros casos (teoricamente nao poderia existir)
  }
}