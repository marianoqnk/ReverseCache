//#include <String.h>
//garantizar apertura mantengo el servo hasta que detecte puerta

#include <SoftwareSerial.h> //libreria de puerto serie virtual para el GPS
#include <TinyGPS++.h> //Libreria que procesa los datos del GPS
#include <Servo.h> //Libreria para manejar el servo
#include <EEPROM.h> //Libreria para guardar los datos en la EEprom 
#include <LowPower.h> //Manejo de los modos de bajo consumo cuando se apaga solo
#include "U8glib.h" //Manero de la pantalla
#include "Logo.h" //Dibujo del travel bug

//direcciones de la EEPROM donde se guardan los datos
#define ADDR_TRIES 0     //Intentos de encontrar la posición 
#define ADDR_STAGE1 1    //Dirección de etapa uno completada
#define ADDR_STAGE2 2    //Dirección de etapa dos completada
#define ADDR_BACKDOOR 3  //Puerta trasera activada
#define ADDR_CACHE_ACTUAL 4  //Puerta trasera activada
#define ADDR_DATE 5 //4 long
#define ADDR_CONSEGUIDOS 9
#define ADDR_TOTALINTENTOS 11
#define GMT +1 //huso horario 


//Pines
#define RXPIN 3
#define TXPIN 4
#define APAGADOPIN 10
#define BACKDOORPIN 12
#define DOORCLOSED A4 //pin analogico 4
//Otros
#define VBAT75 3700
#define VBAT50 3550
#define VBAT25 3500
#define VBATLOW 3200 //tension de bateria baja en milivoltios


const int wakeUpPin = 2; 
const int servoPestillo =11;
const int batCharge=13;
const int apagado=10;


//valores de abierto y cerrado en el servo
#define CERRADO 62
#define ABIERTO  112//0 a 180

#define DIST_THRESHOLD 25.0f //25 metros la f es de float, distancia en dentro de la cual se abre la caja
#define TRIES 6              //Intentos en cada coordenada
#define DEBUG 0   //Poner a 1 para que envie datos de depuracion al puerto serie

//definicion objetos hard
TinyGPSPlus gps;
SoftwareSerial ss(RXPIN, TXPIN);
U8GLIB_PCD8544 u8g(7, 6, 9, 5, 8);		// SPI Com: SCK = 7, MOSI = 6, CS = 9, A0 = 5, Reset = 8

int intentosRestantes;
bool stage1Complete;
bool stage2Complete;    
byte cacheActual; 
float lonActual,latActual;
unsigned long lastDate;
int conseguidos;
int intentos;

int n;
static byte fase=0;

enum BATVL {BATLOW,BAT25,BAT50,BAT75,BAT100}batLvl;

Servo pestillo;
char cabecera[24]="01/01/2016 18:16";  //Primera linea
char pie[24]="N40 28.345 W03 39.345"; //Ultima linea
char mensaje[24];
char mensajeDistancia[24];

void(* resetFunc) (void) = 0;//declara la funcion reset en la direccion 0 permite hacer un reset desde el programa

const float latCaches[10] PROGMEM={ 40.326983, 
                                    40.302841,
                                    40.309366,
                                    40.320433,
                                    40.309744, 
                                    40.314251,
                                    40.326233,
                                    40.321302,
                                    40.304584, 
                                    40.322264};
const float lonCaches[10] PROGMEM={  -3.727783,
                                     -3.735738,
                                     -3.734131,
                                     -3.726842,
                                     -3.741771,
                                     -3.711600,
                                     -3.716737,
                                     -3.721991,
                                     -3.751074,
                                     -3.734849};                                    


void setup(void) 
{  
   checkBat();
   pinMode(servoPestillo,OUTPUT);
   digitalWrite(servoPestillo,HIGH);
   pinMode(apagado, OUTPUT);      // defi ne el pin de apagado como salida
   digitalWrite(apagado,HIGH); //enciende el resto
   pinMode(wakeUpPin, INPUT_PULLUP);
   pinMode(batCharge, INPUT_PULLUP);
   pinMode(BACKDOORPIN, INPUT_PULLUP);
   pinMode(DOORCLOSED, INPUT_PULLUP);
   //EEPROM.put(ADDR_CONSEGUIDOS,0);//reinicia el contador de conseguidos
   u8g.firstPage();  
   do {
      draw_logo();   
      } while( u8g.nextPage());
   Serial.begin(115200);  
   initializeEEPROM(1);
if ( PuertaTrasera() )//si se ha activado la puerta trasera abro puerta
      {    
          initializeEEPROM(0);         
          AbrePuerta();
          Terminar();
      }else if(digitalRead(DOORCLOSED))CierraPuerta();else  AbrePuerta();


if (  intentosRestantes == 0 || (stage1Complete && stage2Complete) )// Se te han acabado los intentos por hoy, poner donde hay GPS y si es un nuevo dia resetear
  {
    byte p=0;
    if(!digitalRead(DOORCLOSED))//si la puerta esta abierta espera a que se cierre
    {
      strcpy(mensaje,"Cierra caja");
      Pantalla("CERRAR");
      
      while(!digitalRead(DOORCLOSED)) //espera 50sg
          {
            delay(500);
            if(p<100)p++;else Terminar();      
          }
    }else  //if la puerta esta cerrada 
    {

    }
    
    CierraPuerta();
    ss.begin(4800);
    strcpy(mensaje,"Reiniciando");
    p=0;
    while(!gps.date.isValid())
    {
    smartdelay(500);//Tiene que haber datos validos
    if(p<100)p++;else Terminar(); 
    }
    #if DEBUG
        initializeEEPROM(0);
    #else 
      initializeEEPROM(0);
    //if(EEPROM.get(ADDR_DATE,lastDate)!=gps.date.value())initializeEEPROM(0);//last date ya tiene valor
      //else strcpy(mensaje,"Hoy no hay mas intentos");
    #endif
    Pantalla("ADIOS");
    Terminar();
  }
if(batLvl==BATLOW)
  {
      strcpy(mensaje,"Bat LOW poner a cargar");
      Pantalla("LOWBAT");  
      Terminar();
  }
n=0;

ss.begin(4800);
smartdelay(2500);//mira el GPS;



} 

void loop(void) 
{
if(!digitalRead(batCharge))
{
    cargando();
    return;    
}

// … Operacion Normal
smartdelay(500);//mira el GPS
GpsDatosToScreen(gps);
sprintf(mensaje,"Buscando SAT: %li %4li",gps.satellites.value(),gps.hdop.value());
#if DEBUG
  Serial.print("Satelites:");
  Serial.print(gps.satellites.value()); // Number of satellites in use (u32)
  Serial.print(" HDOP:");
  Serial.println(gps.hdop.value());
#endif
byte graf=n%4;
switch(graf)
{
  case 0:
    Pantalla("\\");
    break;
  case 1:
    Pantalla("|");
    break;
  case 2:
    Pantalla("/");
    break;
  case 3:
    Pantalla("-");
    break;
}
//Pantalla(n);
if(n<360)n++;else 
              {
                strcpy(mensaje,"Sin Señal GPS");
                Pantalla("NO DATA");
                Terminar();
              }


if (gps.location.isValid() && gps.hdop.value()<200)//quizas comprobar que la precisión es suficiente apox hdop/100 Pprecision(5m)
{
  if ( !stage1Complete )Estado1();
    else if ( !stage2Complete ) Estado2();
      else
        {
          //aqui no llega nunca
          strcpy(mensaje,"Game Over");
          Pantalla("FIN");
          initializeEEPROM(0);
          Terminar();
        }
}

}

void draw_logo(void)
{
 u8g.drawXBMP( 0, 0, TravelBug_width, TravelBug_height, TravelBug_bits);  
}


inline void Estado1(void)
{
float stage1Dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), latActual,lonActual);

if ( stage1Dist <DIST_THRESHOLD )
  {
    EEPROM.write( ADDR_STAGE1, 0xFF ); // estado uno completo
    strcpy(mensaje,"Estado 1.completado");

  }
else
  {
    /* Sorry, try again */
    UnIntentoMenos();
    sprintf(mensaje,"Lejos,Quedan: %i intentos",intentosRestantes);
  }
Pantalla(stage1Dist);
Terminar(stage1Dist);
}


inline void Estado2(void)
{
float stage2Dist = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),latActual,lonActual);
//He tenido que poner las variables conseguido e intentos como globales porque si no no funcionaba correctamente
if ( stage2Dist < DIST_THRESHOLD )// y es menor el margen fijado
{
  strcpy(mensaje,"Enhorabuena conseguido");
  Pantalla("BRAVO");
  delay(3000);
  EEPROM.write( ADDR_STAGE2, 0xFF );//estado dos completo
  //Reto conseguido
  EEPROM.get(ADDR_TOTALINTENTOS,conseguidos);
  strcpy(mensaje,"Lo han intendado");
  Pantalla(conseguidos);
  delay(3000);  
  EEPROM.put(ADDR_CONSEGUIDOS,++EEPROM.get(ADDR_CONSEGUIDOS, intentos));
  strcpy(mensaje,"Lo han conseguido");
  Pantalla(intentos);
  delay(3000);  
  AbrePuerta();
}
else
{
    UnIntentoMenos();
    sprintf(mensaje,"Lejos,Quedan: %i intentos",intentosRestantes);

}
Pantalla(stage2Dist);
Terminar(stage2Dist);
}
inline void cargando() //dibujo de la bateria mientras carga
{
      u8g.firstPage();  
    do {
        u8g.drawFrame(12,12,60,24);
        u8g.drawFrame(71,18,8,12);
        switch (fase)
        {
          case 0:
        
            break;
          case 1:
            u8g.drawBox(12,12,20,24);
            break;
          case 2:
            u8g.drawBox(12,12,40,24);
            break;
          case 3:
            u8g.drawBox(12,12,60,24);
            break;
        }
    } while( u8g.nextPage() ); 
    delay(1000);
    if(fase==3) fase=0;else fase++;
}
void Pantalla(char *mensaje)  // escribe un mensaje en la pantalla
{
strcpy(mensajeDistancia,mensaje);
#if DEBUG
Serial.print("Distancia:");
Serial.println(mensajeDistancia);
#endif
GpsDatosToScreen(gps);
u8g.firstPage();  
  do {
    draw();
  } while( u8g.nextPage() );
  
  
}
void Pantalla(int dato) //sobrecarga de la funcion anterior
{
itoa(dato,mensajeDistancia,10);
Pantalla(mensajeDistancia);
}

void Pantalla(float distancia )//sobrecarga de la funcion anterior
{
dtostrf(distancia,4,0,mensajeDistancia);
strcat(mensajeDistancia,"m");
Pantalla(mensajeDistancia);
}
void draw(void) { //presenta en pantalla las cadenas cabcera, mensaje y pie

  u8g.setFont(u8g_font_micro);
  u8g.drawStr( 0, 5, cabecera);
  u8g.drawLine(0, 6,84, 6);
  u8g.drawLine(0, 42,84, 42);
  u8g.drawStr( 0, 48, pie);
  u8g.drawStr( 0, 12, mensaje);
  u8g.setFont(u8g_font_10x20r);
  //u8g.drawStr( 0, 28, mensaje2);
  u8g.drawStr( 10, 40, mensajeDistancia);
  drawBatChrg();
}


static void GpsDatosToScreen(TinyGPSPlus &gps)//envia al buffer de pantalla la cabecera y el pie con los datos correspondientes
{
  char temp[11];
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid() )
    {
    //dtostrf(Temperatura(),4,1,temp);
    int mes=gps.date.month();
    int hora=gps.time.hour(); //el GPS da la hora GMT hay que corregirla en funcion del huso horario
    hora+=((mes>3 && mes<11)?GMT +1: GMT)%24; //corrige el horario de verano invierno
    sprintf(cabecera,"%02d/%02d/%04d %02d:%02d", gps.date.day(), mes, gps.date.year(), hora, gps.time.minute());
    gradosToGradosMinutos(gps.location.lat(), pie,true);
    gradosToGradosMinutos(gps.location.lng(), temp,false);
    strcat(pie,temp);    
    # if DEBUG 
    Serial.print("Tension:");
    Serial.print(batLvl);
    Serial.print(" ");
    Serial.print(cabecera); 
    Serial.print(" "); 
    Serial.println(pie);
    #endif
  }
  else
  {
  sprintf(cabecera,"Datos no validos");
  sprintf(pie,"Datos no validos");
  }

smartdelay(0);
}


static void smartdelay(unsigned long ms) //bucle de retardo en milisegundos y mientras lee el GPS
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void initializeEEPROM(bool leer)
{

if(leer)
{
  stage1Complete = (EEPROM.read(ADDR_STAGE1) != 0);
  stage2Complete = (EEPROM.read(ADDR_STAGE2) != 0); 
  intentosRestantes=EEPROM.read(ADDR_TRIES); 
  cacheActual=EEPROM.read(ADDR_CACHE_ACTUAL);  
  lonActual=pgm_read_float( &lonCaches[cacheActual] );
  latActual=pgm_read_float( &latCaches[cacheActual] );
  EEPROM.get(ADDR_DATE,lastDate);
  int conseguido;
  
  
  #if DEBUG 
  Serial.print("ST1 ");
  Serial.print(stage1Complete);
  Serial.print(" ST2 ");
  Serial.print(stage2Complete);
  Serial.print(" INTENTOS ");
  Serial.print(intentosRestantes);
  Serial.print(" ACTUAL ");
  Serial.print(cacheActual);
  Serial.print(" LAT ");
  Serial.print(latActual,6);
  Serial.print(" LON ");
  Serial.print(lonActual,6);
  Serial.print(" DATE ");
  Serial.print(lastDate);
  Serial.print(" PUERTA ");
  Serial.print(EEPROM.read( ADDR_BACKDOOR )?"OPEN":"CLOSE");
  Serial.print(" INTENTOS ");
  Serial.print(EEPROM.get(ADDR_TOTALINTENTOS, conseguido));
  Serial.print(" CONSEGUIDO ");
  Serial.println(EEPROM.get(ADDR_CONSEGUIDOS, conseguido));
  #endif
}else
{
  int total;
  EEPROM.write(ADDR_TRIES, TRIES);//intentos para este cache
  EEPROM.write(ADDR_STAGE1, 0);//Primer estado no completado
  EEPROM.write(ADDR_STAGE2, 0);//Segundo estado no completado
  randomSeed(analogRead(5));
  #if DEBUG
  cacheActual=0;//random(0, 2);
  #else
  cacheActual=0;
  #endif
  
  EEPROM.write(ADDR_CACHE_ACTUAL,cacheActual);//Selecciona cual de los 10 caches acutales a a ser
  EEPROM.put(ADDR_TOTALINTENTOS,++(EEPROM.get(ADDR_TOTALINTENTOS,total)));//numero total de intentos
  if(!digitalRead(DOORCLOSED)&& PuertaTrasera()) //pone a cero los intentos con la puerta abierta y el iman
  {
    EEPROM.put(ADDR_CONSEGUIDOS,0I);
    EEPROM.put(ADDR_TOTALINTENTOS,0I);
  }
  //CierraPuerta();
}
}

inline bool PuertaTrasera(void)
{
  //Poner un reed con imán para poder abrir sin necesidad de posicion GPS
  return !digitalRead(BACKDOORPIN);
}
void CierraPuerta(void)
{

  ss.end();
  pestillo.attach(servoPestillo);
if ( EEPROM.read( ADDR_BACKDOOR ) != 0 )
  {
      pestillo.attach(servoPestillo);
      for (byte n=ABIERTO;n!=CERRADO;n--)
      {
      pestillo.write(n);
      delay(20);
      }
      }else {pestillo.write(CERRADO);delay(2000);}
  pestillo.detach();
  EEPROM.write( ADDR_BACKDOOR, 0 );
  ss.begin(4800);
}
void AbrePuerta(void)
{
 
  strcpy(mensaje,"Abriendo Puerta");
  Pantalla("OPEN"); 
  ss.end();
  if ( EEPROM.read( ADDR_BACKDOOR ) == 0 )
  {
      pestillo.attach(servoPestillo);
      for (byte n=CERRADO;n!=ABIERTO;n++)
      {
      pestillo.write(n);
      delay(15);
      }
      }else {pestillo.write(ABIERTO);delay(2000);}
  //smartdelay(1500);
  pestillo.detach();
  //initializeEEPROM(0);//prepara para otro jugador ojo probar
  EEPROM.write( ADDR_BACKDOOR, 1 ); //Puerta abierta
  ss.begin(4800);
}

inline void UnIntentoMenos(void)
{

  EEPROM.write(ADDR_TRIES, --intentosRestantes);
  EEPROM.put(ADDR_DATE,gps.date.value());

}

inline void Terminar(){Terminar(0.0);}
void wakeUp(void){}; //funcion que se llama al despertar
void Terminar(float distancia)
{
  
delay(5000);//espera cinco segundo para que se vean los mensajes pendientes
strcpy(mensaje,"Apagando...");
Pantalla(distancia);
delay(2000);
digitalWrite( apagado,LOW );
delay(100);
pinMode(servoPestillo, INPUT); //con esto el consumo queda en 30uA si no 1mA
pinMode(7, INPUT); //Pines del display
pinMode(6, INPUT);
pinMode(9, INPUT);
pinMode(5, INPUT);
pinMode(8, INPUT);
pinMode(RXPIN, INPUT);
pinMode(TXPIN, INPUT);
pinMode(A4, INPUT);
delay(100);//espero a que se apague
attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, LOW);
LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
// Disable external pin interrupt on wake up pin.
detachInterrupt(digitalPinToInterrupt(wakeUpPin)); 
resetFunc(); //call reset 
}

/*float Temperatura() //mide la temperatura del sensor externo, tiene poca precision. Quitado por falta de sitio en pantalla
{
ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3)); //selecciona REF=1.1 y canal 8 
ADCSRA |= _BV(ADEN);  // habilita el convertidor
delay(20);            // espera a que se estabilice
ADCSRA |= _BV(ADSC);  // inicia la conversión
while (bit_is_set(ADCSRA,ADSC)); //espera al final de la conversión
return ( (ADCW - 312.31 ) / 1.22);  
}*/
inline long  readVcc() { //mide la tensión de alimentacion


    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); //selecciona la medida
  delay(2); 
  ADCSRA |= _BV(ADSC); 
  while (bit_is_set(ADCSRA,ADSC)); 
  long result = 1125300L / ADCW; // Calcula Vcc (en mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

 inline void checkBat()
 {
    long tension;
    tension=readVcc();
    batLvl=tension<VBAT75?(tension<VBAT50?(tension<VBAT25?(tension<VBATLOW?BATLOW:BAT25):BAT50):BAT75):BAT100;
  
 }
 inline void drawBatChrg()
 {
 u8g.drawFrame(73,0,10,5);
 //u8g.drawFrame(71,18,8,12);
 u8g.drawPixel(83,2);
        switch (batLvl)
        {
          case 0:
        
            break;
          case 1:
            u8g.drawBox(73,0,3,5);
            break;
          case 2:
            u8g.drawBox(73,0,5,5);
            break;
          case 3:
            u8g.drawBox(73,0,7,5);
            break;
           case 4:
            u8g.drawBox(73,0,9,5);
            break;
        } 
 }


void gradosToGradosMinutos(float valor,char cadena[],boolean nw) //true latitud false longitud
{
float minutos;
char sminutos[7],sgrados[11];

char rumbo=(valor>0.0)?((nw)?'N':'E'):((nw)?'S':'W');
valor=fabs(valor);
int grados=int(valor);
minutos=(valor-grados)*60.0;
dtostrf(minutos,6, 3, sminutos);
sprintf(sgrados,"%c%02i %s ",rumbo,grados,sminutos);
strcpy(cadena,sgrados);
}

