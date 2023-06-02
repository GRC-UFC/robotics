//Esta eh uma implementacao de controle PID para o carrinho seguidor de linha 'V2' do GRCFC
//Autor: Luis Ferlim - Engenharia de Computacao - UFC - GRC - 2023
//co-autor: Bruno Said - Engenharia de Computacao - UFC - GRC - 2023

//formula PID: Kp * P + Ki * I + Kd * D
// obs: PID continuo: Kp* e[k] + Ki *sum(e[n]) + Kd *(e[k] - e[k-1])


//----- SENSORES -----
//leitura analogica dos sensores com range previamente determinado por 0-930 931-1023
#define S0 A0 
#define S1 A1 
#define S2 A2
#define S3 A3
#define S4 A4
#define S5 A5
#define S6 A6
#define S7 A7

//----- MOTOR ESQUERDA -----
#define INPUT1 2
#define INPUT2 4
#define ENABLE1 3

//----- MOTOR DIREITA -----
#define INPUT3 7
#define INPUT4 8
#define ENABLE2 9

//----- CONSTANTES -----
String sensores = ""; //Leitura de todos os sensores
int veloA = 150; //atentar as velocidades base
int veloB = 150; //velocidade base dos motores PWM

int Ki = 0; //para nao acumularmos tanto erro
int Kp = 35; //valor padrao pra nos
int Kd = 35; //ir ajustando ateh ficar bom

int P =  0,I = 0, D = 0, PID = 0; //inicializando as variaveis que vao sofrer variacao conforme o erro
int velEsq = 0, velDir = 0; //estas sao as variaveis que irao fazer os motores PWM se moverem de fato
float erro = 0, erroAnterior = 0; //calculo do erro

void setup() {
   //Motor esquerda
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  pinMode(ENABLE1, OUTPUT);

  //Motor direita
  pinMode(INPUT3, OUTPUT);
  pinMode(INPUT4, OUTPUT);
  pinMode(ENABLE2, OUTPUT);

  //Sensores
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  pinMode(S6, INPUT);
  pinMode(S7, INPUT);
  }


void calculoPID()
{
  if (erro == 0) {I = 0;}
  P = erro;
  I = I + erro; // acumulo de erro (somatorio)
  if (I > 255) { I = 255;}
  else if (I < -255) { I = -255;}
  D = erro - erroAnterior; //PID discreto
  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroAnterior = erro;
}

/*
  Fucao para efetuar a leitura dos sensores
  Retorna uma string contentdo a leitura dos sensores
  1 -> preto
  0 -> branco
  
  Ideias: Utilizar bits para melhorar a representacao
*/

//inversor digital
bool inversorAtoD(int V_analogico){
  if (V_analogico > 930){
    return 1; //nao refletindo -> preto
  } else{
    return 0; //refletindo luz -> branco
  }
}

void lerSensores() {

  int sensor_0 = inversorAtoD(analogRead(S0));
  int sensor_1 = inversorAtoD(analogRead(S1));
  int sensor_2 = inversorAtoD(analogRead(S2));
  int sensor_3 = inversorAtoD(analogRead(S3));
  int sensor_4 = inversorAtoD(analogRead(S4));
  int sensor_5 = inversorAtoD(analogRead(S5));
  int sensor_6 = inversorAtoD(analogRead(S6));
  int sensor_7 = inversorAtoD(analogRead(S7));

  sensores = String(sensor_0) + String(sensor_1) + String(sensor_2) + String(sensor_3) + String(sensor_4) + String(sensor_5) + String(sensor_6) + String(sensor_7);
}

  //serve para calcular o erro, nosso e(t)
void calculaErro() {
  //positivo para a direita e negativo para a esquerda
  //casos de meiandro
  if (sensores == "00011000") erro = 0;
  else if (sensores == "00011100") erro = 0;
  else if (sensores == "00111000") erro = 0;
  //casos de pares
  else if (sensores == "11000000") erro = -3.75;
  else if (sensores == "01100000") erro = -2.50;
  else if (sensores == "00110000") erro = -1.25;
  else if (sensores == "00001100") erro = 1.25;
  else if (sensores == "00000110") erro = 2.50;
  else if (sensores == "00000011") erro = 3.75;
  //casos de borda - > maior erro 
  else if (sensores == "10000000") erro = -4;
  else if (sensores == "00000001") erro = 4;
  //casos de trio
  else if (sensores == "11100000") erro = -3.5;
  else if (sensores == "01110000") erro = -2.0;
  else if (sensores == "00111000") erro = -0.5;
  else if (sensores == "00011100") erro = 0.5;
  else if (sensores == "00001110") erro = 2.0;
  else if (sensores == "00000111") erro = 3.5;
  //GAP -> ir para frente
  else if (sensores == "00000000") erro = 0;
  //casos onde a qtd de sensores em preto eh maior que 3 -> cruzamento
  else {
    erro = 0;
  }
}

void controleMotor(){
  if (PID >= 0)
  {
    velEsq = veloB;
    velDir = veloA - PID;
  }
  else
  {
    velEsq = veloB + PID;
    velDir = veloA;
  }
  
  digitalWrite(INPUT1, HIGH);
  digitalWrite(INPUT2, LOW);
  digitalWrite(INPUT3, HIGH);
  digitalWrite(INPUT4, LOW); //verificar se o robo nao ta andando ao contrario
  analogWrite(ENABLE1, velEsq);
  analogWrite(ENABLE2, velDir);

}

void loop() {
 lerSensores();
 calculaErro();
 calculoPID();
 controleMotor();
}