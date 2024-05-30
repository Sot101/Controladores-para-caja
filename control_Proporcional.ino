// PROPORCIONAL (CHECK)

#include <Servo.h>
Servo servo;
//------------------------------------- CONFIGURACION -----------------------------------------------------
// PINES
int pinFotoresistencia = A3; 
int pinServo = 9;
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
int ADCLuzAnterior;
int luz_minima = 1023;
int luz_maxima = 0;
// FILTRO
float alpha = 0.3; 
// PROPORCIONAL
float error = 0;
float kp = 1.3;      // 0.9 (bien)   -   1.2   1.3 (bien)  1.4
// SETPOINT
int SetPoint = 300; 
float angulo_servo = anguloCerrado;
//---------------------------------------- SETUP --------------------------------------------------

void setup() {

  
  Serial.begin(9600);
  servo.attach(pinServo); 
  servo.write(anguloCerrado); 
  delay(1500);

  // MAPEAR LUZ
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

  delay(3000);

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



float filtro_adc(int valor_adc) {
  ADCLuzFiltro = alpha * valor_adc + (1 - alpha) * ADCLuzFiltro;
  return ADCLuzFiltro;
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

int ADC_a_grados_servo(float adc){
  float grados_de_servo;
  // convertir a grados servo
  grados_de_servo = (adc/(luz_minima - luz_maxima)) * (anguloAbierto - anguloCerrado);

  return grados_de_servo;
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
//------------------------------------------  LOOP  -------------------------------------------------
void loop() {
  // INPUT
  
  ADCLuz = analogRead(pinFotoresistencia);
  ADCLuzFiltro = filtro_adc(ADCLuz);
  
  // ERROR
  error = SetPoint - ADCLuzFiltro;
  float errorp = (error/SetPoint)*100;
  // SEÑAL DE CONTROL
  float s_control = -kp * error;
  float cambio_de_grados = ADC_a_grados_servo(s_control);
  // MAPEARLA a GRADOS SERVO
  angulo_servo +=  cambio_de_grados;
  if (angulo_servo <= anguloCerrado) angulo_servo = anguloCerrado;
  if (angulo_servo >= anguloAbierto) angulo_servo = anguloAbierto;
  // ESCRIBIR ANGULO
  if (angulo_servo >= anguloCerrado && angulo_servo <= anguloAbierto){
    servo.write(angulo_servo);
  }    
  else{
    servo.write(angulos_servo[0]);
    Serial.println("ERROR al escribir angulo");
  }




  // CAMBIO DE SETPOINT Y KP
  if (Serial.available() > 0) { 
    // Read the command
    String command = Serial.readStringUntil('\n');
    // Process the command
    if (command == "sp") {
      // Cambiar SetPoint
      Serial.println("Nuevo SetPoint:");
      while (!Serial.available()) {} // Wait for input
      int consola = Serial.parseInt(); // Read the new SetPoint value
      Serial.print("Nuevo Setpoint: ");
      Serial.println(consola);
      SetPoint = grados_a_ADC(consola);

    } else if (command == "kp") {
      // cambiar porcentaje bm (banda muerta)
      Serial.println("Nueva kp:");
      while (!Serial.available()) {} // Wait for input
      float numero = Serial.parseFloat(); // Read the new bm value
      kp = numero;
    }
    else if (command == "fi") {
      // cambiar porcentaje bm (banda muerta)
      Serial.println("Nueva alpha:");
      while (!Serial.available()) {} // Wait for input
      float numero = Serial.parseFloat(); // Read the new bm value
      alpha = numero;
    }
  }
  


  // EXTRA
  
  Serial.print("ADC: ");
  Serial.print(ADCLuzFiltro);
  
  Serial.print("    - Setpoint (ADC): ");
  Serial.print(SetPoint);
  Serial.print("    - Error(ADC): ");
  Serial.print(error);
  Serial.print("    - KP: ");
  Serial.print(kp);
  Serial.print("    - Error(ADC): ");
  Serial.print(error);
  Serial.print("    - Error(%): ");
  Serial.print(errorp);
  Serial.println("%"); 
  /*
  Serial.print("    - SControl: ");
  Serial.print(s_control);
  Serial.print("    - GradosS: ");
  Serial.print(angulo_servo);
  Serial.print("    - CambioG: ");
  Serial.print(cambio_de_grados);
  Serial.print("    - Luz Max: ");
  Serial.print(luz_maxima);
  Serial.print("    - Luz Min: ");
  Serial.println(luz_minima); 
 */

  Serial.print(ADCLuzFiltro); 
  Serial.print(",");
  Serial.print(ADCLuz); 
  Serial.print(",");
  Serial.println(SetPoint);
}
