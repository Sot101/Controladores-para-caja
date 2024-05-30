// ON-OFF Histeresis (CHECK)

//------------------------------------------ DESCRIPCION ------------------------------------------------
// Programa de control on-off con histeresis o banda muerta, 
// Comandos en la consola:   'sp'  ->   cambia el setpoint a otro valor de adc
//                           'bm'  ->   cambia el porcentaje de banda muerta (valores de 0 - 100)                              
//------------------------------------- CONFIGURACION -----------------------------------------------------
#include <Servo.h>
Servo servo;
// PINES
int pinFotoresistencia = A3; 
int pinServo = 9;
// SETPOINT
int SetPoint = 300; 
// ANGULOS SERVO (0-90)
int angulos_servo[10] = {4, 14, 24, 34, 44, 54, 65, 75, 86, 97};
int ADC_luz_por_angulo[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// ANGULOS SERVO
int anguloCerrado = angulos_servo[0];
int anguloAbierto = angulos_servo[9] -3;
float anguloAux = 0;  // para la opción de movimiento controlado
// LUZ
int ADCLuz;
int ADCLuzFiltro;
int luz_minima = 1023;
int luz_maxima = 0;
int rango_de_luz = 1023;
// FILTRO
const float alpha = 0.8; 
// BANDA MUERTA
float porcentaje_bm = 0.15; // (porcentaje de banda muerta)
int bm = 0;

//---------------------------------------- SETUP --------------------------------------------------

void setup() {
  
  Serial.begin(9600);
  servo.attach(pinServo); 
  servo.write(anguloCerrado); 
  // MAPEAR LUZ
  Serial.println("MAPEANDO VALORES DE LUMINOSIDAD");
  for (int i = 0; i<10; i++ ){
    servo.write(angulos_servo[i]);
    delay(1000);
    ADC_luz_por_angulo[i] = muestreo_de_luz();
    Serial.print(i*10);
    Serial.print(" grados ->  ");
    Serial.println(ADC_luz_por_angulo[i]);
  }
  servo.write(anguloCerrado);
  // ____CONOCER LUZ MAX Y LUZ MIN ______
  luz_minima = ADC_luz_por_angulo[0];
  luz_maxima = ADC_luz_por_angulo[9];

  delay(500);
  //___CREAR BANDA MUERTA ____
  rango_de_luz = luz_minima - luz_maxima; 
  bm = rango_de_luz * porcentaje_bm; // porque la configuracion del divisor de voltaje da un valor de ADC mayor para luz_minima
}

//--------------------------------------- FUNCIONES ---------------------------------------------------
int muestreo_de_luz() {
  double muestreos = 0;
  for (int i = 0; i<1000; i++){
    muestreos += analogRead(pinFotoresistencia);
    delay(1);
  }
  return (muestreos / 1000);
}

int filtro_adc(int valor_adc){
  return alpha * valor_adc + (1 - alpha) * ADCLuzFiltro;
}
int grados_a_ADC(int grados){
  int grados_en_ADC;

  if (grados >= 0 && grados<= 90){
    //buscar en que intervalo de grados (0-90) está
    int N = 1;   // N es el intervalo de trabajo
    for(int i=1; i<10; i++){
      if (grados >= (i-1)*10 && grados<= (i)*10){
        N = i;
      }
    }
    grados_en_ADC = map(grados, (N-1)*10, N*10, ADC_luz_por_angulo[N-1], ADC_luz_por_angulo[N]);
  }
  else{
    grados_en_ADC = 500;
    Serial.print("ERROR: Grados fuera del rango");

  }
  
  return grados_en_ADC;
}
void movimiento_controlado(int angulo, float paso){  // se mueve controladamente hasta ese angulo
  if (anguloAux > angulo){
    anguloAux -= paso;
    
  }    
  else{
    anguloAux += paso;
  }
  if (anguloAux < anguloCerrado) anguloAux = anguloCerrado;
  if (anguloAux > anguloAbierto ) anguloAux = anguloAbierto;
  servo.write(anguloAux);

}
//------------------------------------------  LOOP  ----------------------------------------------------------------------------------------------
void loop() {
  
  // CAMBIO DE SETPOINT Y DE BANDA MUERTA POR MONITOR SERIAL
  if (Serial.available() > 0) { 
    // Read the command
    String command = Serial.readStringUntil('\n');
    // COMANDOS
    if (command == "sp") {
      // Cambiar SetPoint
      Serial.println("Nuevo SetPoint:");
      while (!Serial.available()) {} // Wait for input
      int consola = Serial.parseInt(); // Read the new SetPoint value
      Serial.print("Nuevo Setpoint: ");
      Serial.println(consola);
      SetPoint = grados_a_ADC(consola);
      delay(1500);
    } else if (command == "bm") {
      // cambiar porcentaje bm (banda muerta)
      Serial.println("Nuevo Porcentaje de Banda Muerta:");
      while (!Serial.available()) {} // Wait for input
      int numero = Serial.parseInt(); // Read the new bm value
      bm = rango_de_luz * numero / 100;
      Serial.print("Nueva Banda Muerta: ");
      Serial.println(bm);
      delay(3000);
    }
  }
  // INPUT
  ADCLuz = analogRead(pinFotoresistencia);
  ADCLuzFiltro = filtro_adc(ADCLuz);
 // VERIFICAR CONDICION
  if (ADCLuzFiltro > (SetPoint+bm)) {
    //servo.write(anguloAbierto);
    movimiento_controlado(anguloAbierto, 10);
  }
  if(ADCLuzFiltro < (SetPoint-bm)){
    //servo.write(anguloCerrado);
    movimiento_controlado(anguloCerrado, 10);
  }

  Serial.print(ADCLuzFiltro); 
  Serial.print(",");
  Serial.print(ADCLuz); 
  Serial.print(",");
  Serial.print(SetPoint);
  Serial.print(",");
  Serial.print(SetPoint + bm);
  Serial.print(",");
  Serial.println(SetPoint - bm);
}


