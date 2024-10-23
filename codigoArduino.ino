//Código do projeto: Bracelete Assistivo para Mobilidade 01 (BAM-01)
//Desenvolvedor: Marcos Vinícius da Silva Santos
//E-mail: markvin787.com
//Data: 05/08/2024

//--------------------------------------------------------------------------------------------
//Bibliotecas

#include <Melopero_VL53L1X.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>

//--------------------------------------------------------------------------------------------
//Variáveis e definições

#define vibracall_1 2
#define vibracall_2 4
#define carregando 33
#define carga_bateria 14
#define sensorParaFrente 12
#define sensorParaTras 13
#define pushButton 32
#define buzzer 26

float distancia;
int modo = 1;
int modoAnterior;
String recebido;
int pressionado;
int timer1;
int timer2;
int vezes=1;
int espera_gravacao;
int aviso=0;

//--------------------------------------------------------------------------------------------

Melopero_VL53L1X sensor;

//--------------------------------------------------------------------------------------------

Adafruit_MPU6050 mpu;

//--------------------------------------------------------------------------------------------
//Pré configurações do bluetooth

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

//--------------------------------------------------------------------------------------------
//Criação de uma função para pulsar o vibracall 1

void pulsar1()
{
  analogWrite(vibracall_2,0);
  if(timer1>149)
    analogWrite(vibracall_1,0);
  else
    analogWrite(vibracall_1,255);
}

//--------------------------------------------------------------------------------------------
//Criação de uma função para pulsar o vibracall 2

void pulsar2()
{
  analogWrite(vibracall_1,0);
  if(timer1>149)
    analogWrite(vibracall_2,0);
  else
    analogWrite(vibracall_2,255);
}

//--------------------------------------------------------------------------------------------
//

void printStatus(String msg, VL53L1_Error status)
{
  Serial.print(msg);
  Serial.println(status);
}

//--------------------------------------------------------------------------------------------

void setup()
{ 
//--------------------------------------------------------------------------------------------
//Declarações

  pinMode(vibracall_1, OUTPUT); //motor vibracall 1
  pinMode(vibracall_2, OUTPUT); //motor vibracall 2
  pinMode(carregando, INPUT); //Push button
  pinMode(carga_bateria, INPUT); //Push button
  pinMode(sensorParaFrente, INPUT_PULLUP); //Push button
  pinMode(sensorParaTras, INPUT_PULLUP); //Push button
  pinMode(pushButton, INPUT_PULLUP); //Push button
  pinMode(buzzer, OUTPUT);

//--------------------------------------------------------------------------------------------
//Configurações do bluetooth

  Serial.begin(115200);
  SerialBT.begin(115200);
  SerialBT.begin("BlindGuard"); //Bluetooth device name
  while (! Serial)
  {
    delay(1);
  }

//--------------------------------------------------------------------------------------------
//Configurações do sensor de distancia

  VL53L1_Error status = 0;
  Wire.begin(); // use Wire1.begin() to use I2C-1 
  sensor.initI2C(0x29, Wire); // use sensor.initI2C(0x29, Wire1); to use I2C-1
  
  status = sensor.initSensor();
  printStatus("Device initialized : ", status);
  
  status = sensor.setDistanceMode(VL53L1_DISTANCEMODE_MEDIUM);
  printStatus("Set distance mode : ", status);

  /*  Timing budget is the time required by the sensor to perform one range 
   *  measurement. The minimum and maximum timing budgets are [20 ms, 1000 ms] */
  status = sensor.setMeasurementTimingBudgetMicroSeconds(66000);
  printStatus("Set timing budget: ", status);

  /*  Sets the inter-measurement period (the delay between two ranging operations) in milliseconds. The minimum 
   *  inter-measurement period must be longer than the timing budget + 4 ms.*/
  status = sensor.setInterMeasurementPeriodMilliSeconds(75);
  printStatus("Set inter measurement time: ", status);

  //If the above constraints are not respected the status is -4: VL53L1_ERROR_INVALID_PARAMS

  status = sensor.clearInterruptAndStartMeasurement();
  printStatus("Start measurement: ", status);
  
//--------------------------------------------------------------------------------------------
//Configurações do sensor giroscópio/aceleômetro

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  delay(100);

//--------------------------------------------------------------------------------------------
//Configurações da EEPROM

  EEPROM.begin(2); //Inicialização de 2 endereços da EEPROM

//--------------------------------------------------------------------------------------------

  tone(buzzer, 5000);
  delay(400);
  noTone(buzzer);
  delay(700);
}
//--------------------------------------------------------------------------------------------

void loop()
{ 
//--------------------------------------------------------------------------------------------

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
//--------------------------------------------------------------------------------------------
//Leitura e armazenamento dos dados do sensor de distancia

  VL53L1_Error status = 0;

  status = sensor.waitMeasurementDataReady();
  if (status != VL53L1_ERROR_NONE) printStatus("Error in wait data ready: ",  status);

  status = sensor.getRangingMeasurementData();
  if (status != VL53L1_ERROR_NONE) printStatus("Error in get measurement data: ",  status);

  status = sensor.clearInterruptAndStartMeasurement();
  if (status != VL53L1_ERROR_NONE) printStatus("Error in clear interrupts: ",  status);

  distancia=(float)sensor.measurementData.RangeMilliMeter + (float)sensor.measurementData.RangeFractionalPart/256.0;
  distancia=map(distancia,31,4000,0,400);
  Serial.print("Distancia: ");
  Serial.println(distancia);

//--------------------------------------------------------------------------------------------
//Leitura do ângulo de pitch

  float pitch=atan2(a.acceleration.y,sqrt(a.acceleration.x*a.acceleration.x+a.acceleration.z*a.acceleration.z))*180/PI;
  pitch=map(pitch,0,89,0,90);
  float pitch_invertido=map(pitch,0,89,90,0);

//--------------------------------------------------------------------------------------------
//Leitura e armazenamento dos dados do bluetooth

  int porcentagem_bateria=map((analogRead(carga_bateria)/1023)*45, 10, 100, 0, 100);

  if(timer1==300)
    timer1=0;
  timer1++;

  if(SerialBT.hasClient())
  {
    if(timer2==600)
      timer2=0;
    timer2++;
    
    while(porcentagem_bateria<=1)
    {
      porcentagem_bateria=map((analogRead(carga_bateria)/1023)*45, 10, 100, 0, 100);
      SerialBT.println("coloque para carregar");
    }

    while(digitalRead(carregando)==1)
    {
      porcentagem_bateria=map((analogRead(carga_bateria)/1023)*45, 10, 100, 0, 100);
      SerialBT.println("carregando");
    }

    if(porcentagem_bateria==100)
    {
      SerialBT.println("100% de carga (se estiver no carregador, retire)");
    }

    if(timer2<150)
    {
      SerialBT.print(porcentagem_bateria);
      SerialBT.println("%");
    }
    else if(timer2<300)
    {
      switch(aviso)
      {
        case 0:
          SerialBT.println("Nenhum aviso"); break;
        case 1:
          SerialBT.println("Posicione o sensor em cima do pulso e virado para frente"); break;
        case 2:
          SerialBT.println("Mantenha o braço relaxado, dobre o antebraço e posicione o sensor em cima do pulso e virado em direção ao ombro"); break;    
        case 3:
          SerialBT.println("Mantenha o braço relaxado, dobre o antebraço e posicione o sensor em cima do pulso e virado para frente"); break;    
      }
    }
    else if(timer2<450)
    {
      if(modo!=4 && modo!=5)
      {
        SerialBT.println(modo);
      }
    }
    else{
      recebido = ""; //modificar
      if(SerialBT.available()){
        char dados = SerialBT.read();
        recebido += dados;
        delay(10);
      }
    //Serial.println(recebido);
    }
  }

//--------------------------------------------------------------------------------------------
//Funcionalidade de apitar o buzzer conforme os dados do ônibus

  if(timer1==300)
    timer1=0;
  timer1++;
  if(recebido.indexOf("buzzer1"))
  { 
    if(timer1>149)
      noTone(buzzer);
    else
      tone(buzzer, 400);//2000
  } else if(recebido.indexOf("buzzer0")){
    noTone(buzzer);
  }


  //exemplo de codigo
  /*
  if(recebido.indexOf("200m"))

  if(recebido.indexOf("500m"))

  if(recebido.indexOf("1km"))

  if(recebido.indexOf("5km")

  if(recebido.indexOf("10km")
  
  if(recebido.index(">10km"))

  if(recebido.index("embarque"))

  if(recebido.index("desembarque"))

  */

//--------------------------------------------------------------------------------------------
//Botão de seleção de modos e etapas

  bool botao=digitalRead(pushButton);

  if(botao==0)
  {
    pressionado++;
    aviso=0;
  }
  
  if(pressionado>0 && pressionado<180 && botao==1)
  {
    if(modo<3)
    {
      modo++;
      vezes=modo;
    }
    else
    {
      modo=1;
      vezes=1;
    }
    pressionado=0;
  }
  
  if(pressionado==900 && modo!=5)
  {
    pressionado=0;
    modo=4;
    delay(2000);
  }

//--------------------------------------------------------------------------------------------
//Troca de modos

  while(vezes!=0)
  {
    analogWrite(vibracall_2, 0);
    analogWrite(vibracall_1, 200);
    delay(400);
    analogWrite(vibracall_1, 0);
    delay(700);
    vezes--;
  }

//--------------------------------------------------------------------------------------------
//Primeiro modo

  if(modo==1)
  {
    if(sensorParaTras==1)
      analogWrite(vibracall_2,0);
      
    if(sensorParaTras==0)
      pulsar2();
    else if(distancia>60 || distancia<1)
      analogWrite(vibracall_1, 0);
    else if(distancia>30)
      analogWrite(vibracall_1, 60);
    else if(distancia>15)
      analogWrite(vibracall_1, 100);
    else if(distancia>2)
      analogWrite(vibracall_1, 190);
    else
      pulsar1();
  }

//--------------------------------------------------------------------------------------------
//Segundo modo

  else if(modo==2)
  {
    if(sensorParaTras==1)
      analogWrite(vibracall_2,0);
    
    if(sensorParaTras==0)
      pulsar2();
    else if(distancia>200 || distancia<1)
      analogWrite(vibracall_1, 0);
    else if(distancia>100)
      analogWrite(vibracall_1, 60);
    else if(distancia>50)
      analogWrite(vibracall_1, 100);
    else if(distancia>25)
      analogWrite(vibracall_1, 190);
    else if(distancia>2)
      analogWrite(vibracall_1, 255);
    else
      pulsar1();
  }

//--------------------------------------------------------------------------------------------
//Terceiro modo

  else if(modo==3)
  {
    aviso=1;
    int cat_ad=EEPROM.read(0)+EEPROM.read(1);
    int parte_bengala=((cat_ad/cos(PI/4))-EEPROM.read(0))/4;
    int desnivel;
    if(pitch<90)
      desnivel=((distancia*cos((2*PI*pitch_invertido)/360))-cat_ad);
    else
      desnivel=0;

    if(sensorParaFrente==1)
      pulsar2();
    else if(desnivel>30)
    {
      analogWrite(vibracall_1, 0);
      analogWrite(vibracall_2, 255);
    }
    else if(desnivel>20)
    {
      analogWrite(vibracall_1, 0);
      analogWrite(vibracall_2, 190);
    }
    else if(desnivel>10)
    {
      analogWrite(vibracall_1, 0);
      analogWrite(vibracall_2, 100);
    }
    else if(desnivel>1)
    {
      analogWrite(vibracall_1, 0);
      analogWrite(vibracall_2, 60);
    }
    else if(desnivel<1 && desnivel>-1)
    {
      analogWrite(vibracall_1, 0);
      analogWrite(vibracall_2, 0);
    }
    else if(distancia<12)
    {
      pulsar1();
    }
    else if(distancia<parte_bengala)
    {
      analogWrite(vibracall_1, 255);
      analogWrite(vibracall_2, 0);
    }
    else if(distancia<(parte_bengala*2))
    {
      analogWrite(vibracall_1, 190);
      analogWrite(vibracall_2, 0);  
    }
    else if(distancia<(parte_bengala*3))
    {
      analogWrite(vibracall_1, 100);
      analogWrite(vibracall_2, 0);
    }
    else if(distancia<(parte_bengala*4))
    {
      analogWrite(vibracall_1, 60);
      analogWrite(vibracall_2, 0);
    }
  }
    
//--------------------------------------------------------------------------------------------
//Modo de pegar os dados do tamanho do braço

  else if(modo==4)
  {
    aviso=2;
    
    if(sensorParaTras==1)
    {
      pulsar2();
      espera_gravacao=5;
    }
    else if(distancia>50 || distancia<10)
    {
      pulsar1();
      espera_gravacao=5;
    }
    else
    {
      if(espera_gravacao>0)
      {
        tone(buzzer, 400);
        delay(400);
        noTone(buzzer);
        delay(700);
        espera_gravacao--;
      }
      else
      {
        EEPROM.write(0,distancia);
        EEPROM.commit();
        delay(2000);
        espera_gravacao=5;
        modo=5;
      }
    }
  }

//--------------------------------------------------------------------------------------------
//Etapa para mapear a altura entre o braço e o chão
    
  else 
  {
    aviso=3;
    
    if(sensorParaFrente==1)
    {
      pulsar2();
      espera_gravacao=5;
    }
    else if(distancia>120 || distancia<30)
    {
      pulsar1();
      espera_gravacao=5;
    }
    else
    {
      
      if(espera_gravacao>0)
      {
        tone(buzzer, 400);
        delay(400);
        noTone(buzzer);
        delay(700);
        espera_gravacao--;
      }
      else
      {
        EEPROM.write(1,distancia);
        EEPROM.commit();
        delay(2000);
        espera_gravacao=5;
        modo=1;
      }
    }
  }
}
