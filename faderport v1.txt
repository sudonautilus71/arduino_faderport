 
   #include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MIDIUSB.h"
#include <CapacitiveSensor.h>

// Dirección I2C del LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD con dirección I2C 0x27 y tamaño de 16x2
// Crear una instancia del sensor capacitivo
CapacitiveSensor capSensor = CapacitiveSensor(4, 14); // Pin 4 envía, pin 14 recibe

// Pines del potenciómetro y motor
#define potA0 A0 // Potenciómetro para el fader
#define enA 15
#define in1 5
#define in2 6

int valorDAW = 8192; // Valor de DAW para Pitch Bend
const int deadZone = 100; // Zona muerta para evitar movimientos innecesarios
int valorFader = 0;
bool potTocado = false; // Estado del sensor capacitivo
int threshold = 500; // Umbral del sensor capacitivo (ajustable según tu hardware)

unsigned long previousMillis = 0;
const long interval = 50; // Intervalo para leer el sensor capacitivo
// Pines del encoder
const int encoderPinA = 7;   // Pin A del encoder (para detectar el movimiento)
const int encoderPinB = 8;   // Pin B del encoder (para determinar la dirección del movimiento)

// Variable de control del panorama MIDI
int valorPanorama = 64;      // Valor inicial del panorama (entre 0 y 127)
int ccPanorama = 10;
int valorZoomH = 64;      // Valor inicial del panorama (entre 0 y 127)
int ccZoomH = 20;
int valorZoomV = 64;      // Valor inicial del panorama (entre 0 y 127)
int ccZoomV = 21;
int valorEnc = 64;      // Valor inicial del panorama (entre 0 y 127)
int ccEnc = 22;
// Número de Control Change para el panorama
// pines lcd 2=SDA  3=SCL
// Pines de los pulsadores
const int buttonPin1 = A1;   // Pulsador 1 en A1 (para activar el menú y borrar el modo)
const int buttonPin3 = 9;    // Pulsador 3 en pin 9 (para seleccionar opciones del menú)
const int buttonPin2 = A2;    // Pulsador 2 en pin 10 ()
const int buttonPin4 = A3;    // Pulsador 4 en pin 16 ()
const int buttonPin5 = 16;    // Pulsador 5 en pin A2 ()
const int buttonPin6 = 10;    // Pulsador 6 en pin A3 ()

// Variables para el debounce (para evitar múltiples lecturas erróneas al presionar un botón)
unsigned long lastDebounceTime = 0;  // Almacena el tiempo del último cambio
unsigned long debounceDelay = 5;     // Tiempo de debounce (5 ms)

// Variables para el encoder
volatile int encoderPos = 0;  // Posición del encoder, se modifica por interrupciones
volatile bool encoderMoved = false; // Bandera para indicar que el encoder se movió
int lastStateA = LOW;         // Último estado del pin A
int lastStateB = LOW;         // Último estado del pin B

// Variables para los pulsadores
int buttonState1 = HIGH;      // Estado actual de pulsador 1
int buttonState3 = HIGH;      // Estado actual de pulsador 3

// Array de opciones del menú
String menu[10] = {
  "PAN", "TRACK", "RW/FF", "ZOOM H", "ZOOM V",
  "ENCODER", "Opcion 7", "Opcion 8", "Opcion 9", "Opcion 10",
};

// Variables para manejar el menú
unsigned long lastMenuUpdateTime = 0;  // Última actualización del menú
unsigned long menuUpdateInterval = 20;  // Intervalo de actualización del menú (20 ms)
int currentMenuIndex = 0;  // Índice de la opción actual en el menú
bool menuActivo = false;    // Bandera para controlar si el menú está activo
bool opcionSeleccionada = false;  // Bandera para indicar si se seleccionó una opción

// Enumeración de modos disponibles
enum Modo { MODO_PAN, MODO_TRACK, MODO_RW, MODO_ZOOM_H, MODO_ZOOM_V, MODO_ENC, NINGUNO };
Modo modoActual = NINGUNO;  // Variable que almacena el modo actual
// Definimos el tiempo de debounce
const unsigned long debounceTime = 200; // Tiempo de rebote en milisegundos

// Variables para controlar los pulsadores y su estado
unsigned long lastPressTime1 = 0;
unsigned long lastPressTime2 = 0;
unsigned long lastPressTime3 = 0;
unsigned long lastPressTime4 = 0;
unsigned long lastPressTime5 = 0;
unsigned long lastPressTime6 = 0;

// Incluye las librerías y demás configuración del programa...

// Configuración inicial
void setup() {
  Serial.begin(9600);  // Iniciar comunicación serial para depuración
    Serial.begin(115200);
    pinMode(potA0, INPUT);
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);  // Configurar pin A del encoder como entrada con pull-up
  pinMode(encoderPinB, INPUT_PULLUP);  // Configurar pin B del encoder como entrada con pull-up
  pinMode(buttonPin1, INPUT_PULLUP);  // Configurar pulsador 1 como entrada con pull-up

  pinMode(buttonPin2, INPUT_PULLUP);  // Configurar pulsador 2 como entrada con pull-up
  pinMode(buttonPin3, INPUT_PULLUP);  // Configurar pulsador 3 como entrada con pull-up
  pinMode(buttonPin4, INPUT_PULLUP);  // Configurar pulsador 4 como entrada con pull-up
  pinMode(buttonPin5, INPUT_PULLUP);  // Configurar pulsador 5 como entrada con pull-up
  pinMode(buttonPin6, INPUT_PULLUP);  // Configurar pulsador 6 como entrada con pull-up


  lcd.init();  // Inicializar LCD
  lcd.backlight();  // Encender retroiluminación del LCD
  lcd.setCursor(0, 0);  // Posicionar el cursor en la primera línea, primera columna
  lcd.print("F1 para MENU");  // Mostrar mensaje inicial

  // Configurar interrupciones para el encoder
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, CHANGE);  // Interrupción en cambio de pin A
  attachInterrupt(digitalPinToInterrupt(encoderPinB), encoderISR, CHANGE);  // Interrupción en cambio de pin B
}

// Bucle principal
void loop() {
   manejarPulsadores(); // Llamar a la función que gestiona los pulsadores

  // Mantener el resto de las funciones existentes

   // Recibir mensajes MIDI de Pitch Bend
    recibirMensajesMidi();

    // Leer el sensor capacitivo en intervalos regulares
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        leerSensorCapacitivo();
    }

    // Controlar el motor basado en el valor de Pitch Bend y el estado del potenciómetro
    controlMotor();
  // Leer los estados de los pulsadores
  buttonState1 = digitalRead(buttonPin1);
  buttonState3 = digitalRead(buttonPin3);

  // Detectar si el pulsador A1 se presiona y borrar el modo actual
  if (buttonState1 == LOW && millis() - lastDebounceTime > debounceDelay) {
    lastDebounceTime = millis();  // Actualizar el tiempo del último cambio
    borrarModoActual();  // Llamar a la función para borrar el modo actual
    lcd.clear();  // Limpiar la pantalla
    lcd.setCursor(0, 0);  // Colocar el cursor en la primera fila
    lcd.print("Modo Borrado");  // Mostrar mensaje de borrado
    delay(500);  // Esperar 500 ms para que se lea el mensaje
    lcd.clear();  // Limpiar la pantalla nuevamente
    lcd.setCursor(0, 0);  // Colocar el cursor en la primera fila
    lcd.print("F1 para MENU");  // Mostrar el mensaje de inicio
  }

  // Activar el menú si se presiona el botón 1
  if (buttonState1 == LOW && !menuActivo) {
    menuActivo = true;  // Activar el menú
    opcionSeleccionada = false;  // Restablecer la opción seleccionada
    currentMenuIndex = 0;  // Iniciar en la primera opción del menú
    lcd.clear();  // Limpiar la pantalla
  }

  // Si el menú está activo, actualizar las opciones cada cierto tiempo
  if (menuActivo && !opcionSeleccionada) {
    if (millis() - lastMenuUpdateTime >= menuUpdateInterval) {
      actualizarMenu();  // Actualizar el menú
      lastMenuUpdateTime = millis();  // Actualizar el tiempo de la última actualización
    }
    if (buttonState3 == LOW) {  // Si se presiona el botón 3, seleccionar la opción
      seleccionarOpcionMenu();
      delay(200);  // Esperar para evitar múltiples lecturas rápidas
    }
  }

  // Ejecutar el código para manejar el encoder (solo si se movió)
  if (encoderMoved) {
    leerEncoder();  // Llamar a la función para leer el encoder
    encoderMoved = false;  // Resetear el flag de movimiento
  }
}

// Función para borrar el modo actual
void borrarModoActual() {
  modoActual = NINGUNO;  // Resetear el modo a "NINGUNO"
}

// Función para manejar las interrupciones del encoder
void encoderISR() {
  int stateA = digitalRead(encoderPinA);  // Leer el estado del pin A
  int stateB = digitalRead(encoderPinB);  // Leer el estado del pin B

  // Si el estado del pin A ha cambiado
  if (stateA != lastStateA) {
    // Si el pin A está en HIGH, actualizamos la posición del encoder
    if (stateA == HIGH) {
      encoderPos += (stateB == LOW) ? 1 : -1;  // Incrementar o decrementar según la dirección
      encoderMoved = true;  // Indicar que el encoder se movió
    }
    lastStateA = stateA;  // Actualizar el estado del pin A
  }
}

// Actualizar el menú y avanzar o retroceder en las opciones
void actualizarMenu() {
  // Avanzar o retroceder en el menú dependiendo de la posición del encoder
  if (encoderPos > 0) {
    currentMenuIndex = (currentMenuIndex + 1) % 10;  // Avanzar al siguiente ítem en el menú
    encoderPos = 0;  // Resetear la posición del encoder
  } else if (encoderPos < 0) {
    currentMenuIndex = (currentMenuIndex - 1 + 10) % 10;  // Retroceder al ítem anterior
    encoderPos = 0;  // Resetear la posición del encoder
  }
  // Mostrar la opción actual del menú en el LCD
  lcd.setCursor(0, 1);
  lcd.print("                ");  // Limpiar la línea 2
  lcd.setCursor(0, 1);  // Colocar el cursor en la segunda línea
  lcd.print(menu[currentMenuIndex]);  // Mostrar la opción seleccionada
}

// Seleccionar opción y activar el modo correspondiente
void seleccionarOpcionMenu() {
  lcd.clear();  // Limpiar la pantalla
  lcd.setCursor(0, 0);  // Colocar el cursor en la primera fila
  lcd.print(menu[currentMenuIndex]);  // Mostrar el nombre de la opción seleccionada
  opcionSeleccionada = true;  // Marcar la opción como seleccionada
  menuActivo = false;  // Desactivar el menú
  activarModo(static_cast<Modo>(currentMenuIndex));  // Activar el modo correspondiente
}

// Función para activar un modo específico y desactivar los otros
void activarModo(Modo nuevoModo) {
  modoActual = nuevoModo;  // Establecer el nuevo modo
}

// Función para manejar el encoder y enviar valores MIDI
void leerEncoder() {
  Serial.print("Encoder Position: ");
  Serial.println(encoderPos);  // Imprimir la posición del encoder para depuración

  // Dependiendo del modo actual, enviar diferentes mensajes MIDI
  switch (modoActual) {
    case MODO_PAN:
      valorPanorama = constrain(valorPanorama + ((encoderPos > 0) ? 3 : -3), 0, 127);  // Modificar el panorama
      enviarPanorama(valorPanorama);  // Enviar el valor de panorama por MIDI
      encoderPos = 0;  // Resetear la posición del encoder
      break;
    case MODO_TRACK:
      enviarNota((encoderPos > 0) ? 0x2F : 0x2E);  // Enviar nota para control de pistas
      encoderPos = 0;  // Resetear la posición del encoder
      break;
    case MODO_RW:
      enviarNota((encoderPos > 0) ? 0x0C : 0x15);  // Enviar nota para RW/FF
      encoderPos = 0;  // Resetear la posición del encoder
      break;
    case MODO_ZOOM_H:
     valorZoomH = constrain(valorZoomH + ((encoderPos > 0) ? 1 : -1), 0, 127);  // Modificar el panorama
      enviarZoomH(valorZoomH);  // Enviar el valor de panorama por MIDI
      encoderPos = 0;  // Resetear la posición del encoder
      break;
    case MODO_ENC:
     valorEnc = constrain(valorEnc + ((encoderPos > 0) ? 1 : -1), 0, 127);  // Modificar el panorama
      enviarEnc(valorEnc);  // Enviar el valor de panorama por MIDI
      encoderPos = 0;  // Resetear la posición del encoder
      break;
//      enviarNota((encoderPos > 0) ? 0x28 : 0x1A);  // Enviar nota para zoom horizontal
//      encoderPos = 0;  // Resetear la posición del encoder
//      break;
    case MODO_ZOOM_V:
      valorZoomV = constrain(valorZoomV + ((encoderPos > 0) ? 1 : -1), 0, 127);  // Modificar el panorama
      enviarZoomV(valorZoomV);  // Enviar el valor de panorama por MIDI
      encoderPos = 0;  // Resetear la posición del encoder
      break;
//        enviarNota((encoderPos > 0) ? 0x36 : 0x44);  // Enviar nota para zoom vertical
//        encoderPos = 0;  // Resetear la posición del encoder
//        break;
//      default:
//        break;
  }
}

// Función para enviar el valor de panorama en MIDI
void enviarPanorama(int valor) {
  midiEventPacket_t panControl = {0x09, 0xB0 | 0, ccPanorama, (uint8_t)valor};  // Paquete MIDI de Control Change
  MidiUSB.sendMIDI(panControl);  // Enviar el paquete MIDI
  MidiUSB.flush();  // Asegurar que se envíe el paquete
  Serial.print("Enviando MIDI Control Change Panorama: ");
  Serial.println(valor);  // Mostrar el valor enviado
}
void enviarZoomH(int valor) {
  midiEventPacket_t ZoomHControl = {0x09, 0xB0 | 0, ccZoomH, (uint8_t)valor};  // Paquete MIDI de Control Change
  MidiUSB.sendMIDI(ZoomHControl);  // Enviar el paquete MIDI
  MidiUSB.flush();  // Asegurar que se envíe el paquete
  Serial.print("Enviando MIDI Control Change Panorama: ");
  Serial.println(valor);  // Mostrar el valor enviado
}
void enviarZoomV(int valor) {
  midiEventPacket_t ZoomVControl = {0x09, 0xB0 | 0, ccZoomV, (uint8_t)valor};  // Paquete MIDI de Control Change
  MidiUSB.sendMIDI(ZoomVControl);  // Enviar el paquete MIDI
  MidiUSB.flush();  // Asegurar que se envíe el paquete
  Serial.print("Enviando MIDI Control Change Panorama: ");
  Serial.println(valor);  // Mostrar el valor enviado
}
void enviarEnc(int valor) {
  midiEventPacket_t EncControl = {0x09, 0xB0 | 0, ccEnc, (uint8_t)valor};  // Paquete MIDI de Control Change
  MidiUSB.sendMIDI(EncControl);  // Enviar el paquete MIDI
  MidiUSB.flush();  // Asegurar que se envíe el paquete
  Serial.print("Enviando MIDI Control Change Panorama: ");
  Serial.println(valor);  // Mostrar el valor enviado
}
// Función para enviar una nota MIDI
void enviarNota(uint8_t nota) {
  midiEventPacket_t note = {0x09, 0x90, nota, 0x7F};  // Paquete MIDI de nota
  MidiUSB.sendMIDI(note);  // Enviar el paquete MIDI
  MidiUSB.flush();  // Asegurar que se envíe el paquete
  Serial.print("Enviando nota MIDI: ");
  Serial.println(nota);  // Mostrar la nota enviada
}
void recibirMensajesMidi() {
    while (true) {
        midiEventPacket_t rx = MidiUSB.read();
        if (rx.header == 0) break;

        if ((rx.byte1 & 0xF0) == 0xE0) {
            // Leer el valor de Pitch Bend desde el DAW
            int valorPitchBend = (rx.byte3 << 7) | rx.byte2;
            valorDAW = map(valorPitchBend, 0, 16383, -8192, 8191);
            Serial.print("Recibido desde DAW: ");
            Serial.println(valorDAW);
        }
    }
}

void controlMotor() {
    valorFader = analogRead(potA0); // Leer el valor del potenciómetro minimo  Value: -8151 máximo  Value: 8191
    int valorFaderMapped = map(valorFader, 0, 1023, -8151, 8191); // Mapear a rango MIDI

    // Si el potenciómetro está tocado, enviar su valor al DAW
    if (potTocado) {
        analogWrite(enA, 0); // Detener el motor si está tocado
        enviarValorADaw(valorFaderMapped); // Enviar valor al DAW
        return; // Terminar la función si está tocado
    }

    // Si el motor necesita ajustarse para alcanzar el valor del DAW
    if (abs(valorFaderMapped - valorDAW) > deadZone) {
        int motorSpeed = map(abs(valorFaderMapped - valorDAW), deadZone, 8191, 100, 255);

        // Mover el motor en la dirección correcta según la diferencia entre fader y DAW
        if (valorFaderMapped < valorDAW) {
            digitalWrite(in1, HIGH); // Girar el motor en un sentido
            digitalWrite(in2, LOW);
        } else {
            digitalWrite(in1, LOW);  // Girar el motor en sentido contrario
            digitalWrite(in2, HIGH);
        }

        analogWrite(enA, motorSpeed); // Controlar la velocidad del motor
    } else {
        // Detener el motor si está cerca de la posición deseada
        analogWrite(enA, 0);
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
    }
}

void enviarValorADaw(int valorFader) {
    // Convertir el valor del fader en Pitch Bend MIDI
    int valorPitchBend = map(valorFader, -8192, 8191, 0, 16383);

    // Crear el paquete MIDI para el Pitch Bend
    midiEventPacket_t pitchBend = {0x09, 0xE0, (uint8_t)(valorPitchBend & 0x7F), (uint8_t)((valorPitchBend >> 7) & 0x7F)};
    MidiUSB.sendMIDI(pitchBend);  // Enviar el paquete MIDI
    MidiUSB.flush();  // Asegurar que el mensaje se envíe

    // Mostrar el valor enviado en el monitor serie
    Serial.print("Enviado al DAW: ");
    Serial.println(valorFader);
}

void leerSensorCapacitivo() {
    long total = capSensor.capacitiveSensor(30); // Leer el valor del sensor capacitivo
    potTocado = total > threshold; // Actualizar el estado del sensor capacitivo

    // Mostrar el valor en el monitor serie
//    Serial.print("Valor del sensor capacitivo: ");
//    Serial.print(total);
//    Serial.print(" - Tocado: ");
//    Serial.println(potTocado ? "Sí" : "No");
}
void manejarPulsadores() {
  // Leer el estado de los pulsadores y comprobar si ha pasado el tiempo de debounce

  // Pulsador 1
  if (digitalRead(buttonPin1) == LOW && (millis() - lastPressTime1) > debounceTime) {
    Serial.println("Pulsador 1 presionado (A1)");
    enviarNotaMidi(120); // Nota MIDI no común
    lastPressTime1 = millis(); // Actualizar el tiempo de la última presión
  }

  // Pulsador 2
  if (digitalRead(buttonPin2) == LOW && (millis() - lastPressTime2) > debounceTime) {
    Serial.println("Pulsador 2 presionado (10)");
    enviarNotaMidi(121); // Nota MIDI no común
    lastPressTime2 = millis();
  }

  // Pulsador 3
  if (digitalRead(buttonPin3) == LOW && (millis() - lastPressTime3) > debounceTime) {
    Serial.println("Pulsador 3 presionado (9)");
    enviarNotaMidi(122); // Nota MIDI no común
    lastPressTime3 = millis();
  }

  // Pulsador 4
  if (digitalRead(buttonPin4) == LOW && (millis() - lastPressTime4) > debounceTime) {
    Serial.println("Pulsador 4 presionado (16)");
    enviarNotaMidi(123); // Nota MIDI no común
    lastPressTime4 = millis();
  }

  // Pulsador 5
  if (digitalRead(buttonPin5) == LOW && (millis() - lastPressTime5) > debounceTime) {
    Serial.println("Pulsador 5 presionado (A2)");
    enviarNotaMidi(124); // Nota MIDI no común
    lastPressTime5 = millis();
  }

  // Pulsador 6
  if (digitalRead(buttonPin6) == LOW && (millis() - lastPressTime6) > debounceTime) {
    Serial.println("Pulsador 6 presionado (A3)");
    enviarNotaMidi(125); // Nota MIDI no común
    lastPressTime6 = millis();
  }
}

// Función para enviar un paquete MIDI con la nota
void enviarNotaMidi(byte nota) {
  midiEventPacket_t note = {0x09, 0x90, nota, 0x7F};  // Paquete MIDI de nota
  MidiUSB.sendMIDI(note);  // Enviar el paquete MIDI
  MidiUSB.flush();         // Asegurar que se envíe el paquete
}
