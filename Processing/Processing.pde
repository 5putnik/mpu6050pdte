import processing.serial.*;

float accel_x, accel_y, accel_z, tmp,gyro_x, gyro_y, gyro_z = 0; // Valores medidos pelo sensor
Serial myPort; // Porta serial do Arduino a ser lida
final int vSize = 200; // Quantidade maxima de dados a ser salva
float dadox[] = new float[vSize], dadoy[] = new float[vSize]; // Dados a serem mostrados no grafico
int c = 0; // Contador

PFont f; // Fonte do texto

void setup() // Inicializacao do programa
{
  size(800, 600, P2D); // Gerando uma tela 800x600 com renderizacao 2D melhorada
  myPort = new Serial(this, Serial.list()[0], 9600); // Associando MyPort as portas seriais do computador
  myPort.bufferUntil('\n'); // Busca por \n
  colorMode(RGB, 1);
  f = createFont("Arial", 16, true); // Escolhendo fonte do texto como Arial 16
}

void draw() // Rotina em repeticao permanente
{
  rectMode(CORNERS); // Modo de desenho dos retangulos como CORNERS
  int i; // Variavel geral de laco
  c++; // Contando em qual execucao esta
  if(c == vSize) // Se o programa encontra-se no valor maximo de dados que se pode salvar
    c = 0; // Sobrescreve o dado mais antigo
  dadox[c] = 595 - (accel_y * 200) / 32768; // Linha temporaria: dados de plot do eixo x
  dadoy[c] = 210 - (accel_z * 200) / 32768; // Linha temporaria: dados de plot do eixo y
  
  background(255, 255, 255); // Tela de fundo branca
  textFont(f, 16); // Fonte tamanho 16
  
  
  noFill(); // Desabilita preenchimento
  rect(400, 10, 790, 410); // [Numeros magicos temporarios] Grade externa dos eixos
  fill(0); // Preenche proximos desenhos de preto
  line(590,10, 590, 410); // [Numeros magicos temporarios] Eixo Y do plano cartesiano
  line(400,210,790,210); // [Numeros magicos temporarios] Eixo X do plano cartesiano
  
  fill(0, 255, 0); // Preenche proximos desenhos de verde
  stroke(0, 255, 0); // Habilita linhas de contorno verdes
  for(i=0;i<vSize;i++) // Varre todos os dados
    if(dadox[i] != 0 && dadoy[i] != 0) // Se eles forem validos
      rect(dadox[i] - 1, dadoy[i] - 1, dadox[i] + 1, dadoy[i] + 1); // Desenha um "ponto" de tamanho 3x3 pixels
  stroke(0); // Habilita linhas de contorno pretas
  
  

  fill(0); // Preenche proximos desenhos de preto
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
    text("Leitura acelerometro X: " + accel_x, 10, 20); // Imprime valor lido
    text("Leitura acelerometro Y: " + accel_y, 10, 40); // Imprime valor lido
    text("Leitura acelerometro Z: " + accel_z, 10, 60); // Imprime valor lido
    text("Leitura giroscopio X: " + gyro_x, 10, 100); // Imprime valor lido
    text("Leitura giroscopio Y: " + gyro_y, 10, 120); // Imprime valor lido
    text("Leitura giroscopio Z: " + gyro_z, 10, 140); // Imprime valor lido
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