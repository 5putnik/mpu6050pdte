import controlP5.*;
import processing.serial.*;

final float version = 1.2;

// Parametros fixos
float dt = 0.02; // Passo entre as medicoes (20 ms) (pode variar)
int acu = 0; // Variavel auxiliar para o calculo variavel do dt

final float big_P = 50; // Parametro do filtro de Kalman: define a incerteza
final float small_Qt = 0.003; // Parametro do filtro de Kalman: define a velocidade de resposta
final float small_Qtb = 0.003; // Parametro do filtro de Kalman: define a velocidade de resposta
final float small_R = 0.01; // Parametro do filtro de Kalman: define a velocidade de resposta

final float reg_ac = 2, // Faixa do acelerometro: +/- 2g
            reg_gy = 250, // Faixa do giroscopio: +/- 250º/s
            reg_ang = 360, // Faixa do angulo: +/- 360º
            reg_pos = 100; // Faixa do deslocamento: +/- 10 cm

final int maxSize = 1000; // Quantidade maxima de dados a ser salva

final int XMAX = 800, // Parametro: Tamanho X da tela
          gap = 10, // Parametro: espacamento geral
          sqrwidth = 400; // Parametro: tamanho do grafico
          
String out = "../../output_"+year()+"_"+nf(month(),2)+"_"+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2)+".csv";
// Fim dos parametros fixos

class displacement
{
  float x,
        //v,
        //a,
        ang,
        r;
  displacement()
  {
    this.x = 0;
    //this.v = 0;
    //this.a = 0;
  }
  float step(float read)
  {
    float r = sqrt(d1*d1 + d2*d2 - 2*d1*d2*cos(((360 - d3)*PI/180))); // Lei dos cossenos. "360 - d3" pois o angulo indicado representa o suplementar. "*PI/180" para converter de graus para radianos
    //float v0 = this.v;
    //float x0 = this.x;
    this.x = -(read * PI / 180) * r; // Convertendo o valor de graus para radianos. Valor negativo pois a mandibula abre para baixo
    //this.v = (x - x0)/dt;
    //this.a = (v - v0)/dt;
    return this.x;
  }
}
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

  Obj_Kalman(float Qt, float Qtb, float R, float P00, float P01, float P10, float P11, float bias, float ang) // Definindo a estrutura de dados do filtro de Kalman
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

  float kalman_step(float read_vel, float read_ang) // Definindo a funcao do filtro com base no valor anterior e no valor lido
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

boolean start_prog = false;

float accel_x, accel_y, accel_z, tmp,gyro_x, gyro_y, gyro_z = 0; // Valores medidos pelo sensor
String acx, acy, acz, gyx, gyy, gyz; // Valores convertidos para mostrar na tela 

float d1, d2, d3; // Valores escritos pelo usuario na tela (medidas da mandibula)

Serial myPort; // Porta serial do Arduino a ser lida
int vSize = 400; // Quantidade de dados a ser salva

float dadox[] = new float[maxSize], dadoy[] = new float[maxSize], dadoz[] = new float[maxSize]; // Dados a serem mostrados no grafico
float f_dadox[] = new float[maxSize], f_dadoy[] = new float[maxSize], f_dadoz[] = new float[maxSize]; // Dados a serem filtrados
int c = 0; // Contador

Obj_Kalman k_x, k_y, k_z; // Variaveis que serao submetidas ao filtro Kalman (rotacao)

displacement d_x, d_y, d_z; // Variaveis que serao submetidas ao filtro Kalman (translacao)

float ang_x = 0, // Valor Angulo X em graus
      ang_y = 0, // Valor Angulo Y em graus
      ang_z = 0, // Valor Angulo Z em graus
      dis_x = 0, // Distancia no eixo X (abertura)
      dis_y = 0, // Distancia no eixo Y (desvio)
      dis_z = 0; // Distancia no eixo Z (protracao)

float grav_x, grav_y, grav_z; // Componentes da gravidade

float grav = 9.81; // Gravidade
int g_c = 0; // Contador auxiliar para calculo da gravidade

float gx = 0, // Leitura inicial da gravidade
      gy = 1, // Leitura inicial da gravidade (admite-se gravidade atuando unicamente no eixo Y)
      gz = 0; // Leitura inicial da gravidade



PFont f; // Fonte do texto

ControlP5 cp5; // Controle para utilizar objetos
CheckBox cb_save, // CheckBox para salvar arquivos
         cb_hide; // CheckBox que esconde informacoes
         
DropdownList axismode, // lista para escolher o modo eixo (XY ou YT)
             fillmode, // lista para escolher o modo preenchimento (linha ou ponto)
             qtd, // lista de quantidade de valores a serem plotados (apenas para YT)
             valorx, // lista de valores a serem plotados (Eixo X ou valor 1)
             valory, // lista de valores a serem plotados (Eixo Y ou valor 2)
             valorz, // lista de valores a serem plotados (valor 3, apenas para YT)
             ac_unit, // lista de unidade do acelerometro (m/s² ou g)
             gy_unit, // lista de unidade do giroscopio (º/s ou rad/s)
             filter, // lista de filtros
             scale, // lista de escalas
             sample, // selecao de dados amostrados
             savemode; // modo de armazenamento

int checksave = 0; // Variavel para detectar se o arquivo foi salvo uma vez
             
int mode_xy = 1, // 1: Modo XY, 0: Modo YT
    mode_line = 1, // 1: Modo linha, 0: Modo ponto
    mode_x = 1, // Modo de selecao do eixo X (modo XY) ou variavel 1 (modo YT)
    mode_y = 1, // Modo de selecao do eixo Y (modo XY) ou variavel 2 (modo YT)
    mode_z = 1, // Modo de selecao variavel 3 (apenas modo YT)
    mode_filter = 1, // 1: Filtro passa-baixas selecionado, 0: Valores crus
    mode_scale = 1, // Modo de selecao de escala
    mode_sample = 1; // Modo de selecao da taxa de dados salvos por ciclo
    
float scx, scy, scz; // Escala para plot (2g, 4g, 8g, 16g, 250º/s, 500º/s, 1000º/s etc)

void setup() // Inicializacao do programa
{
  if(getFolder().equals("Processing"))
    out = "../output_"+year()+"_"+nf(month(),2)+"_"+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2)+".csv";
  size(800, 600, P2D); // Gerando uma tela 800x600 com renderizacao 2D melhorada
  if(Serial.list().length == 0)
    return;
  myPort = new Serial(this, Serial.list()[0], 9600); // Associando MyPort as portas seriais do computador
  myPort.bufferUntil('\n'); // Busca por \n
  colorMode(RGB, 1);
  f = createFont("Arial", 16, true); // Escolhendo fonte do texto como Arial 16
  cp5 = new ControlP5(this); // Inicializacao dos controles P5
  axismode = cp5.addDropdownList("Modo YT", 10, 80, 100, 84); // Insercao da lista axismode
  fillmode = cp5.addDropdownList("Pontos", 120, 80, 100, 84); // Insercao da lista fillmode
  valorx = cp5.addDropdownList("Acel X", 10, 140, 100, 84); // Insercao da lista valorx
  valory = cp5.addDropdownList("Acel Y", 120, 140, 100, 84); // Insercao da lista valory
  valorz = cp5.addDropdownList("Acel Z", 230, 140, 100, 84); // Insercao da lista valorz
  ac_unit = cp5.addDropdownList("m/s^2", 270, 175, 100, 84); // Insercao da lista ac_unit
  gy_unit = cp5.addDropdownList("grau/s", 270, 255, 100, 84); // Insercao da lista gy_unit
  filter = cp5.addDropdownList("Raw", XMAX/2 + 45, 2*gap + sqrwidth, 100, 84); // Insercao da lista filter
  scale = cp5.addDropdownList("1x", XMAX/2 + 150, 2*gap + sqrwidth, 100, 84); // Insercao da lista scale
  sample = cp5.addDropdownList("400 (padrao)", XMAX/2 + 255, 2*gap + sqrwidth, 100, 84); // Insercao da lista sample
  savemode = cp5.addDropdownList("Salvar uma vez", 120, 15, 100, 84); // Insercao da lista savemode
  qtd = cp5.addDropdownList("3", 340, 140, 30, 84); // Insercao da lista qtd
  
  ddl_standard(axismode, "Modo YT:Modo XY"); // Cria a lista axismode
  ddl_standard(fillmode, "Pontos:Linhas"); // Cria a lista fillmode
  ddl_standard(scale, "1x:2x:3x:4x:5x"); // Cria a lista scale
  ddl_standard(filter, "Raw:Low-pass"); // Cria a lista filter
  ddl_standard(sample, "400 (padrao):50:100:200:250:500:750:1000"); // Cria a lista filter
  ddl_standard(ac_unit, "m/s^2:g"); // Cria a lista ac_unit
  ddl_standard(gy_unit, "grau/s:rad/s"); // Cria a lista gy_unit
  ddl_standard(valorx, "Acel X:Acel Y:Acel Z:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z:Abertura:Desvio"); // Cria a lista valorx
  ddl_standard(valory, "Acel Y:Acel Z:Acel X:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z:Abertura:Desvio"); // Cria a lista valory
  ddl_standard(valorz, "Acel Z:Acel Y:Acel X:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z:Abertura:Desvio"); // Cria a lista valorz
  ddl_standard(savemode, "Salvar uma vez:Sobrepor arquivo:Salvar continuamente"); // Cria a lista savemode
  ddl_standard(qtd, "3:2:1"); // Cria a lista qtd
  
  cb_save = cb_standard("Salvar arquivo?", 10, 10); // Cria o CheckBox cb_save
  cb_hide = cb_standard("Esconder textos desnecessarios", 10, 35); // Cria o CheckBox cb_hide
  
  txt_standard("txt1", XMAX/2 + 155, 2*gap + sqrwidth + 20);
  txt_standard("txt2", XMAX/2 + 155, 2*gap + sqrwidth + 45);
  txt_standard("txt3", XMAX/2 + 155, 2*gap + sqrwidth + 70);
  
  bang_standard("Salvar preferencias", XMAX/2 + 155, 2*gap + sqrwidth + 95);
  
  valorx.setBackgroundColor(color(255, 0, 0)); // Cor de fundo da lista
  valory.setBackgroundColor(color(0, 255, 0)); // Cor de fundo da lista
  valorz.setBackgroundColor(color(0, 0, 255)); // Cor de fundo da lista
  
  // Inicializando os parametros do filtro Kalman (Q, R, P etc.)
  k_x = new Obj_Kalman(small_Qt, // Qt
        small_Qtb, // Qtb
        small_R, // R
        big_P, // P00
        0, // P01
        0, // P10
        big_P, // P11
        0, // bias
        0); // ang
   k_y = new Obj_Kalman(small_Qt, // Qt
        small_Qtb, // Qtb
        small_R, // R
        big_P, // P00
        0, // P01
        0, // P10
        big_P, // P11
        0, // bias
        0); // ang
   k_z = new Obj_Kalman(small_Qt, // Qt
        small_Qtb, // Qtb
        small_R, // R
        big_P, // P00
        0, // P01
        0, // P10
        big_P, // P11
        0, // bias
        0); // ang
    d_x = new displacement();
    d_y = new displacement();
    d_z = new displacement();
}

void draw() // Rotina em repeticao permanente
{
  if(Serial.list().length == 0)
  {
    background(0); // Tela de fundo preta
    text("ERRO: Arduino nao conectado. Conecte o Arduino e reinicie o programa.",200,200);
  }
  if(!start_prog)
    return;
  background(255, 255, 255); // Tela de fundo branca
  textFont(f, 16); // Fonte tamanho 16
  rectMode(CORNERS); // Modo de desenho dos retangulos como CORNERS
  int i; // Variavel geral de laco
  if(g_c < 10)
  {
    g_c++;
    grav = 0.04*grav + 9.81 * 0.96 * sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z); 
  }
  
  if(g_c == 1) loadPreferences(); // Carrega as preferencias
  
  text("v" + nf(version, 1, 0), XMAX-50, 580); 
  
  ang_x = k_x.kalman_step(gyro_x, ang_x); // Filtro de Kalman para selecionar o angulo
  ang_y = k_y.kalman_step(gyro_y, ang_y); // Filtro de Kalman para selecionar o angulo
  ang_z = k_z.kalman_step(gyro_z, ang_z); // Filtro de Kalman para selecionar o angulo
  
  dis_x = d_x.step(ang_x);
  dis_y = d_y.step(ang_y);
  //dis_z = d_z.step(ang_z);
  
  grav_x = rot(gx, gy, gz, ang_x*PI/180, ang_y*PI/180, ang_z*PI/180, 0); // Rotacionando o vetor (gx,gy,gz) com as leituras do giroscopio
  grav_y = rot(gx, gy, gz, ang_x*PI/180, ang_y*PI/180, ang_z*PI/180, 1); // Rotacionando o vetor (gx,gy,gz) com as leituras do giroscopio
  grav_z = rot(gx, gy, gz, ang_x*PI/180, ang_y*PI/180, ang_z*PI/180, 2); // Rotacionando o vetor (gx,gy,gz) com as leituras do giroscopio
  
  if((int)cb_hide.getArrayValue()[0] == 0)
  {
    text("Abertura: " + int(dis_x) + " mm", 10, 360);
    text("Desvio: " + int(dis_y) + " mm", 10, 380);
    
    text("Drifting x: " + nf(k_x.bias, 1, 2) + "º/s (" + nf(grav_x, 1, 2) + " g)", 10, 420); // Imprimindo o valor da gravidade na tela
    text("Drifting y: " + nf(k_y.bias, 1, 2) + "º/s (" + nf(grav_y, 1, 2) + " g)", 10, 440); // Imprimindo o valor da gravidade na tela
    text("Drifting z: " + nf(k_z.bias, 1, 2) + "º/s (" + nf(grav_z, 1, 2) + " g)", 10, 460); // Imprimindo o valor da gravidade na tela
    
    text("Gravidade x: " + nf(grav_x * grav, 1, 2) + "m/s² (" + nf(grav_x, 1, 2) + " g)", 10, 480); // Imprimindo o valor da gravidade na tela
    text("Gravidade y: " + nf(grav_y * grav, 1, 2) + "m/s² (" + nf(grav_y, 1, 2) + " g)", 10, 500); // Imprimindo o valor da gravidade na tela
    text("Gravidade z: " + nf(grav_z * grav, 1, 2) + "m/s² (" + nf(grav_z, 1, 2) + " g)", 10, 520); // Imprimindo o valor da gravidade na tela
    
    text("Angulo x (pitch): " + nf(ang_x, 1, 2) + "º", 10, 540); // Imprimindo o valor do angulo na tela
    text("Angulo y (yaw)  :" + nf(ang_y, 1, 2) + "º", 10, 560); // Imprimindo o valor do angulo na tela
    text("Angulo z (roll) : " + nf(ang_z, 1, 2) + "º", 10, 580); // Imprimindo o valor do angulo na tela
  }
  
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
  {
    c = 0; // Sobrescreve o dado mais antigo
    // Salvando o Arquivo
    if((int)cb_save.getArrayValue()[0] == 1)
    {
      switch(int(savemode.getValue()))
      {
        case 0:
          cb_save.toggle(0);
        case 1:
          writecsv(0, f_dadox, f_dadoy, f_dadoz, out);
          break;
          case 2:
          String doc[] = loadStrings(out);
          if(doc.length == 1)
            writecsv(0, f_dadox, f_dadoy, f_dadoz, out);
          else
            writecsv(1, f_dadox, f_dadoy, f_dadoz, out);
          
      }
    }
  }
  
  if((int)cb_save.getArrayValue()[0] == 1) // Se cb_save estiver marcada
      if(!cb_save.isVisible()) savemode.show(); // Mostrar savemode se a CheckBox estiver marcada
    else
      if(cb_save.isVisible()) savemode.hide(); // Ocultar savemode se a CheckBox estiver desmarcada
  
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
      scx = reg_ac;
      break;
    case 1:
      f_dadox[c] = accel_y;
      scx = reg_ac;
      break;
    case 2:
      f_dadox[c] = accel_z;
      scx = reg_ac;
      break;
    case 3: case 4: case 5:
      scx = reg_gy;
      f_dadox[c] = getdata(int(valorx.getValue()));
      break;
    case 6: case 7: case 8:
      scx = reg_ang;
      f_dadox[c] = getdata(int(valorx.getValue()));
      break;
    case 9: case 10: case 11:
      scx = reg_pos;
      f_dadox[c] = getdata(int(valorx.getValue()));
  }
  switch(int(valory.getValue())) // Selecao de variavel eixo Y
  {
    case 0:
      f_dadoy[c] = accel_y;
      scy = reg_ac;
      break;
    case 1:
      f_dadoy[c] = accel_z;
      scy = reg_ac;
      break;
    case 2:
      f_dadoy[c] = accel_x;
      scy = reg_ac;
      break;
    case 3: case 4: case 5:
      scy = reg_gy;
      f_dadoy[c] = getdata(int(valory.getValue()));
      break;
    case 6: case 7: case 8:
      scy = reg_ang;
      f_dadoy[c] = getdata(int(valory.getValue()));
      break;
    case 9: case 10: case 11:
      scy = reg_pos;
      f_dadoy[c] = getdata(int(valory.getValue()));
  }
  
  switch(int(valorz.getValue())) // Selecao de variavel 3
  {
    case 0:
      f_dadoz[c] = accel_z;
      scz = reg_ac;
      break;
    case 1:
      f_dadoz[c] = accel_y;
      scz = reg_ac;
      break;
    case 2:
      f_dadoz[c] = accel_x;
      scz = reg_ac;
      break;
    case 3: case 4: case 5:
      scz = reg_gy;
      f_dadoz[c] = getdata(int(valorz.getValue()));
      break;
    case 6: case 7: case 8:
      scz = reg_ang;
      f_dadoz[c] = getdata(int(valorz.getValue()));
      break;
    case 9: case 10: case 11:
      scz = reg_pos;
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
  
  dadoy[c] = gap + sqrwidth/2 - (1 + mode_scale)*(f_dadoy[c] * sqrwidth/2) / scy; // Dados de plot do eixo y
  if(mode_xy != 0)
  {
    dadox[c] = XMAX - (gap + sqrwidth/2) + (1 + mode_scale)*(f_dadox[c] * sqrwidth/2) / scx; // Dados de plot do eixo x
    if(valorz.isVisible()) {valorz.hide();println("!");}
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
    dadox[c] = gap + sqrwidth/2 - (1 + mode_scale)*(f_dadox[c] * sqrwidth/2) / scx; // Dados de plot 1
    dadoz[c] = gap + sqrwidth/2 - (1 + mode_scale)*(f_dadoz[c] * sqrwidth/2) / scz; // Dados de plot 3
    for(i=1;i<vSize;i++)
    {
      stroke(255, 0, 0); // Habilita linhas de contorno vermelhos
      if(dadox[i] != 0 && dadox[i-1] != 0) // Se eles forem validos
        if(mode_line != 0)
          line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadox[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadox[i]); // Desenha linhas
        else
          rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadox[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadox[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels
      
      switch(int(qtd.getValue()))
    {
      case 0: // Desenha dado 3
        stroke(0, 0, 255); // Habilita linhas de contorno azuis
        if(dadoz[i] != 0 && dadoz[i-1] != 0) // Se eles forem validos
          if(mode_line != 0)
            line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadoz[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadoz[i]); // Desenha linhas
          else
            rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadoz[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadoz[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels  
          
      case 1: // Desenha dado 2
        stroke(0, 255, 0); // Habilita linhas de contorno verdes
        if(dadoy[i] != 0 && dadoy[i-1] != 0) // Se eles forem validos
          if(mode_line != 0)
            line(XMAX - (gap + sqrwidth) + sqrwidth*(i-1)/vSize, dadoy[i-1], XMAX - (gap + sqrwidth) + sqrwidth*i/vSize, dadoy[i]); // Desenha linhas
          else
            rect(XMAX - (gap + sqrwidth) + sqrwidth*i/vSize - 1, dadoy[i] - 1, XMAX - (gap + sqrwidth) + sqrwidth*i/vSize + 1, dadoy[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels
    }
      

      

      
    }
     stroke(0); // Habilita linhas de contorno pretas
  }
  switch(int(ac_unit.getValue())) // Selecao de unidade do acelerometro
  {
    case 0:
      acx = nf(accel_x * grav, 1, 2) + " m/s²"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acy = nf(accel_y * grav, 1, 2) + " m/s²"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      acz = nf(accel_z * grav, 1, 2) + " m/s²"; // Conversao para valores fisicos (metro por segundo ao quadrado)
      break;
    case 1:
      acx = nf(accel_x, 1, 2) + " g"; // Conversao para valores fisicos (gravidades)
      acy = nf(accel_y, 1, 2) + " g"; // Conversao para valores fisicos (gravidades)
      acz = nf(accel_z, 1, 2) + " g"; // Conversao para valores fisicos (gravidades)
      break;
  }

  switch(int(gy_unit.getValue())) // Selecao de unidade do giroscopio
  {
    case 0:
      gyx = nf(gyro_x, 1, 2) + " º/seg"; // Conversao para valores fisicos (graus por segundo)
      gyy = nf(gyro_y, 1, 2) + " º/seg"; // Conversao para valores fisicos (graus por segundo)
      gyz = nf(gyro_z, 1, 2) + " º/seg"; // Conversao para valores fisicos (graus por segundo)
      break;
    case 1:
      gyx = nf(gyro_x * PI/180, 1, 2) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyy = nf(gyro_y * PI/180, 1, 2) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      gyz = nf(gyro_z * PI/180, 1, 2) + " rad/seg"; // Conversao para valores fisicos (radianos por segundo)
      break;
  }
  fill(0); // Preenche proximos desenhos de preto
  text("Valores a serem representados no grafico:", 10, 135); // Texto informativo
  text("Propriedades de visualizacao:", 10, 75); // Texto informativo
  text("Filtro:", XMAX/2, 3*gap + sqrwidth); // Texto informativo
  text("Ramo da mandibula:", XMAX/2, 2*gap + sqrwidth + 35); // Texto informativo
  text("Corpo da mandibula:", XMAX/2, 2*gap + sqrwidth + 60); // Texto informativo
  text("Angulo da mandibula:", XMAX/2, 2*gap + sqrwidth + 85); // Texto informativo
  text("mm", XMAX/2 + 260, 2*gap + sqrwidth + 35); // Texto informativo
  text("mm", XMAX/2 + 260, 2*gap + sqrwidth + 60); // Texto informativo
  text("º", XMAX/2 + 260, 2*gap + sqrwidth + 85); // Texto informativo

  if((int)cb_hide.getArrayValue()[0] == 0)
  {
    if(!ac_unit.isVisible()) ac_unit.show();
    if(!gy_unit.isVisible()) gy_unit.show();
    text("Unidade:", 270, 170); // Texto informativo
    text("Unidade:", 270, 250); // Texto informativo
    if(accel_x == -1.0 && accel_y == -1.0 && accel_z == -1.0 && gyro_x == -1.0 && gyro_y == -1.0 && gyro_z == -1.0 && tmp == 36.53) // Se todos forem iguais ao valor que geralmente representa erro no protocolo I2C de comunicacao
    {
      text("Leitura acelerometro X: Erro na comunicacao!", 10, 180); // Imprime mensagem de erro
      text("Leitura acelerometro Y: Erro na comunicacao!", 10, 200); // Imprime mensagem de erro
      text("Leitura acelerometro Z: Erro na comunicacao!", 10, 220); // Imprime mensagem de erro
      text("Leitura giroscopio X: Erro na comunicacao!", 10, 260); // Imprime mensagem de erro
      text("Leitura giroscopio Y: Erro na comunicacao!", 10, 280); // Imprime mensagem de erro
      text("Leitura giroscopio Z: Erro na comunicacao!", 10, 300); // Imprime mensagem de erro
      text("Temperatura: Erro na comunicacao!", 10, 340); // Imprime mensagem de erro
    }
    else
    {
      text("Leitura acelerometro X: " + acx, 10, 180); // Imprime valor lido
      text("Leitura acelerometro Y: " + acy, 10, 200); // Imprime valor lido
      text("Leitura acelerometro Z: " + acz, 10, 220); // Imprime valor lido
      text("Leitura giroscopio X: " + gyx, 10, 260); // Imprime valor lido
      text("Leitura giroscopio Y: " + gyy, 10, 280); // Imprime valor lido
      text("Leitura giroscopio Z: " + gyz, 10, 300); // Imprime valor lido
      text("Temperatura: " + tmp + "ºC", 10, 320); // Imprime valor lido
    }
  }
  else
  {
    if(ac_unit.isVisible()) ac_unit.hide();
    if(gy_unit.isVisible()) gy_unit.hide();
  }
}
 
void keyReleased() // Evento que ocorre toda vez que uma tecla for pressionada
{
  d1 = gettext(1);
  d2 = gettext(2);
  d3 = gettext(3);
}

public void controlEvent(ControlEvent ev)
{
  if(ev.isFrom(cb_save))
  {
    out = "../../output_"+year()+"_"+nf(month(),2)+"_"+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2)+".csv";
    if(getFolder().equals("Processing"))
      out = "../output_"+year()+"_"+nf(month(),2)+"_"+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2)+".csv";
    String tbl[] = new String[1]; // Inicializando arquivo de output ao inicializar o programa
    tbl[0] = "x"; // Preenchendo a primeira linha com qualquer coisa para sobrescrever qualquer arquivo que possa existir
    saveStrings(out, tbl); // Salvando arquivo sob o nome de output_[data]_[hora].csv
  }
  if((ev.isFrom(cb_save)) || (ev.isFrom(cb_hide)))
    return;
  if(ev.getController().getName().equals("Salvar preferencias"))
    savePreferences();
}

void loadPreferences()
{
  String fname = "data.xyz";
  if(!(getFolder().equals("Processing")))
  {
    fname = "../" + fname;
  }
  String st[] = loadStrings(fname);
  if(st == null)
    return;
  cb_save.setArrayValue(decode(st[0]));
  savemode.setValue(decode(st[1])[0]);
  cb_hide.setArrayValue(decode(st[2]));
  axismode.setValue(decode(st[3])[0]);
  fillmode.setValue(decode(st[4])[0]);
  valorx.setValue(decode(st[5])[0]);
  valory.setValue(decode(st[6])[0]);
  valorz.setValue(decode(st[7])[0]);
  qtd.setValue(decode(st[8])[0]);
  ac_unit.setValue(decode(st[9])[0]);
  gy_unit.setValue(decode(st[10])[0]);
  scale.setValue(decode(st[11])[0]);
  sample.setValue(decode(st[12])[0]);
  
  cp5.get(Textfield.class,"txt1").setText(nf(decode(st[13])[0], 1, 4));
  cp5.get(Textfield.class,"txt2").setText(nf(decode(st[14])[0], 1, 4));
  cp5.get(Textfield.class,"txt3").setText(nf(decode(st[15])[0], 1, 4));
  d1 = gettext(1);
  d2 = gettext(2);
  d3 = gettext(3);
  
  rename(savemode,"Salvar uma vez:Sobrepor arquivo:Salvar continuamente",decode(st[1])[0]);
  rename(axismode,"Modo YT:Modo XY",decode(st[3])[0]);
  rename(fillmode,"Pontos:Linhas",decode(st[4])[0]);
  rename(valorx,"Acel X:Acel Y:Acel Z:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z:Abertura:Desvio",decode(st[5])[0]);
  rename(valory,"Acel Y:Acel Z:Acel X:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z:Abertura:Desvio",decode(st[6])[0]);
  rename(valorz,"Acel Z:Acel Y:Acel X:Gyro X:Gyro Y:Gyro Z:Ang X:Ang Y:Ang Z:Abertura:Desvio",decode(st[7])[0]);
  rename(qtd,"3:2:1",decode(st[8])[0]);
  rename(ac_unit,"m/s^2:g",decode(st[9])[0]);
  rename(gy_unit,"grau/s:rad/s",decode(st[10])[0]);
  rename(scale,"1x:2x:3x:4x:5x",decode(st[11])[0]);
  rename(sample,"400 (padrao):50:100:200:250:500:750:1000",decode(st[12])[0]);
}
void rename(DropdownList d, String s, float index)
{
  String v[] = split(s, ":");
  if(index > v.length)
    return;
  d.setLabel(v[(int)index]);
}
void savePreferences() // Salva as preferencias
{
  String fname = "data.xyz";
  if(!(getFolder().equals("Processing")))
  {
    fname = "../" + fname;
  }
  String st[] = new String[16];
  
  st[0] = "saveFile=" + cb_save.getArrayValue()[0];
  st[1] = "saveMode=" + savemode.getValue();
  st[2] = "hideText=" + cb_hide.getArrayValue()[0];
  st[3] = "axisMode=" + axismode.getValue();
  st[4] = "fillMode=" + fillmode.getValue();
  st[5] = "val1=" + valorx.getValue();
  st[6] = "val2=" + valory.getValue();
  st[7] = "val3=" + valorz.getValue();
  st[8] = "valNum=" + qtd.getValue();
  st[9] = "unitAcc=" + ac_unit.getValue();
  st[10] = "unitGyr=" + gy_unit.getValue();
  st[11] = "scaleMode=" + scale.getValue();
  st[12] = "sampleRate=" + sample.getValue();
  st[13] = "measure1=" + gettext(1);
  st[14] = "measure2=" + gettext(2);
  st[15] = "measure3=" + gettext(3);
  saveStrings(fname,st);
}

float[] decode(String val)
{
  float st[] = new float[1];
  String stemp[] = split(val, "=");
  st[0] = float(stemp[1]);
  return st;
}
void serialEvent(Serial myPort) // Rotina de toda vez que algo for escrito na porta serial
{
  String xString = myPort.readStringUntil('\n'); // Ler o que foi escrito ate a quebra de linha
  if(xString != null) // Se algo foi lido
  {
    String  temp[]  =  split(xString,":"); // Separar os dados cada vez que dois-pontos for encontrado
    if(xString.charAt(0)  ==  '#'  &&  temp.length==8) // Se o primeiro caractere escrito for cerquilha e 8 elementos forem lidos
    {
      dt = (millis() - acu)/1000.0;
      if(!start_prog)
        dt = 0;
      acu = millis();
      start_prog = true; // habilita o programa a inicializar
      /* 
       * Protocolo de comunicacao definido por mim:
       * Mensagem enviada pelo Arduino:
       * #X:32768:32768:32768:32768:32768:32768:32768
       * Sendo cada numero entre os dois-pontos uma das leituras, na ordem:
       * acelerometro x, y, z, temperatura, giroscopio x, y, z
       * */
      accel_x = (float(temp[1]) * reg_ac) / 32768; // Atualiza variavel global e converte de representacao em escala para valor fisico
      accel_y = (float(temp[2]) * reg_ac) / 32768; // Atualiza variavel global e converte de representacao em escala para valor fisico
      accel_z = (float(temp[3]) * reg_ac) / 32768; // Atualiza variavel global e converte de representacao em escala para valor fisico
      tmp = float(temp[4]); // Atualiza variavel global
      gyro_x = (float(temp[5]) * reg_gy) / 32768; // Atualiza variavel global e converte de representacao em escala para valor fisico
      gyro_y = (float(temp[6]) * reg_gy) / 32768; // Atualiza variavel global e converte de representacao em escala para valor fisico
      gyro_z = (float(temp[7]) * reg_gy) / 32768; // Atualiza variavel global e converte de representacao em escala para valor fisico
    }
  }
}

void stop() // Finalizando o programa
{
  
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

CheckBox cb_standard(String title, int xpos, int ypos)
{
  return cp5.addCheckBox("cb_" + title)
                .setPosition(xpos, ypos)
                .setColorForeground(color(0))
                .setColorBackground(color(0))
                .setColorActive(color(255, 0, 0))
                .setColorLabel(color(0))
                .setSize(20, 20)
                .addItem(title, 0);
}

void txt_standard(String title, int xpos, int ypos)
{
  cp5.addTextfield(title)
     .setPosition(xpos,ypos)
     .setSize(100, 20)
     .setColorValue(color(0))
     .setColorBackground(color(255,255,255))
     .setFocus(true)
     .setFont(createFont("arial",15))
     .setColorForeground(color(0))
     .setColorActive(color(0));
}

void bang_standard(String title, int xpos, int ypos)
{
  cp5.addBang(title)
       .setPosition(xpos, ypos)
       .setSize(40, 40)
       .setColorActive(color(0))
       .setColorBackground(color(0))
       .setColorCaptionLabel(color(0))
       .setColorForeground(color(0));
}
float lowpass(float read, float old, float pct)
{
  return pct*read + (1-pct)*old;
}

float getdata(int val)
{
  switch(val) // Selecao de variavel
  {
    case 0:
      return accel_x;
    case 1:
      return accel_y;
    case 2:
      return accel_z;
    case 3:
      return gyro_x;
    case 4:
      return gyro_y;
    case 5:
      return gyro_z;
    case 6:
      return ang_x;
    case 7:
      return ang_y;
    case 8:
      return ang_z;
    case 9:
      return dis_x;
    case 10:
      return dis_y;
    case 11:
      return dis_z;
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

void writecsv(int rFlag, float data1[], float data2[], float data3[], String fname)
{
  Table tbl = new Table();
  if(rFlag == 0)
  {
    tbl = new Table();
    tbl.setString(0,0,"sep=");
    tbl.setString(1,0,"Dado 1");
    tbl.setString(1,1,"Dado 2");
    if(mode_xy == 0)
    tbl.setString(1,2,"Dado 3");
  }
  else
    tbl = loadTable(fname);
  
  TableRow line;
  for(int i=0;i<vSize;i++)
  {
    line = tbl.addRow();
    line.setFloat(0, data1[i]);
    line.setFloat(1, data2[i]);
    if(mode_xy == 0)
    line.setFloat(2, data3[i]);
  }
  saveTable(tbl, fname);
  
  String lines[] = loadStrings(fname);
  lines[0] = "sep=,";
  saveStrings(fname, lines);
}

String getFolder()
{
  String full = sketchPath("");
  String  temp[]  =  split(full,"\\");
  return temp[temp.length-2];
}

float gettext(int val)
{
  String txtname = "txt"+val;
  int i, dt = 0;
  String num = cp5.get(Textfield.class, txtname).getText();
  if(num.length() == 0)
    return 0;
  for(i = 0;i < num.length();i++)
  {
    if(int(num.charAt(i)) == ',')
      num = num.replace(',', '.');
    if(int(num.charAt(i)) == '.')
      dt++;
    if(dt>1)
      return 0;
    if((int(num.charAt(i)) < '0' || int(num.charAt(i)) > '9') && int(num.charAt(i)) != '.')
      return 0;
  }
  
  return float(num);
}