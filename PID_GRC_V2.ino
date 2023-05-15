//Esta eh uma implementacao de controle PID para o carrinho seguidor de linha 'V2' do GRCFC
//Autor: Luis Ferlim - Engenharia de Computacao - UFC - GRC - 2023
//co-autor: Bruno Said - Engenharia de Computacao - UFC - GRC - 2023

//formula PID: Kp * P + Ki * I + Kd * D
// obs: PID continuo: Kp* e[k] + Ki *sum(e[n]) + Kd *(e[k] - e[k-1])


//----- SENSORES -----
#define SENSOR_0 12 //dir
#define SENSOR_1 11 //meio
#define SENSOR_2 10 //esq

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
int veloA = 255; //atentar as velocidades base
int veloB = 255; //velocidade base dos motores PWM

int Ki = 0; //para nao acumularmos tanto erro
int Kp = 35; //valor padrao pra nos
int Kd = 0; //ir ajustando ateh ficar bom

int P =  0,I = 0, D = 0, PID = 0; //inicializando as variaveis que vao sofrer variacao conforme o erro
int velEsq = 0, velDir = 0; //estas sao as variaveis que irao fazer os motores PWM se moverem de fato
int erro = 0, erroAnterior = 0; //calculo do erro

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
  pinMode(SENSOR_0, INPUT);
  pinMode(SENSOR_1, INPUT);
  pinMode(SENSOR_2, INPUT);
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
void lerSensores() {
  int sensor_direita = digitalRead(SENSOR_0);
  int sensor_centro = digitalRead(SENSOR_1);
  int sensor_esquerda = digitalRead(SENSOR_2);

  sensores = String(sensor_esquerda) + String(sensor_centro) + String(sensor_direita);
}

  //serve para calcular o erro, nosso e(t)
void calculaErro() {
  //positivo para a direita e negativo para a esquerda
  if (sensores == "010") erro = 0;
  else if (sensores == "110") erro = -1;
  else if (sensores == "100") erro = -2;
  else if (sensores == "011") erro = 1;
  else if (sensores == "001") erro = 2;
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
  
  digitalWrite(INPUT1, LOW);
  digitalWrite(INPUT2, HIGH);
  digitalWrite(INPUT3, LOW);
  digitalWrite(INPUT4, HIGH); //verificar se o robo nao ta andando ao contrario
  analogWrite(ENABLE1, velEsq);
  analogWrite(ENABLE2, velDir);

}

void loop() {
 lerSensores();
 calculaErro();
 calculoPID();
 controleMotor();
}
