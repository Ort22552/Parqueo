#include <WiFi.h>
#include <Wire.h>

const char* ssid = "POCOX3";
const char* password = "Sexo2312";

WiFiServer server(80);

// Variable global que almacena el estado del parqueo
volatile byte estado_parqueos = 0;

void setup() {
  Serial.begin(115200);

  // Conexión WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi conectado. Dirección IP: ");
  Serial.println(WiFi.localIP());
  server.begin();

  // Configurar I2C como esclavo en dirección 0x08
  Wire.begin(0x08); // Dirección esclavo
  Wire.onReceive(recibirDatos); // Handler para datos entrantes
}

void loop() {
  WiFiClient client = server.available();
  estado_parqueos=random(0,9);
  if (client) {
    Serial.println("Cliente conectado");
    
    while (client.connected()) {
      if (client.available()) {
        String request = client.readStringUntil('\r');
        Serial.println("Petición: " + request);
        client.readStringUntil('\n'); // limpia el buffer

        String html = generarHTML(estado_parqueos);

        // Enviar respuesta HTTP
        client.println("HTTP/1.1 200 OK");
        client.println("Content-type:text/html");
        client.println("Connection: close");
        client.println();
        client.println(html);
        break;
      }
    }
    client.stop();
    Serial.println("Cliente desconectado");
  }
}

void recibirDatos(int cantidad) {
  if (cantidad >= 1) {
    estado_parqueos = Wire.read();  // Leer el byte enviado por el maestro
    Serial.print("Nuevo estado recibido: ");
    Serial.println(estado_parqueos, BIN);
  }
}

String generarHTML(byte estado) {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Parqueo Digital</title><style>";
  html += "body { font-family: Arial; background-color: #f4f4f4; margin: 20px; }";
  html += "table { width: 100%; border-collapse: collapse; }";
  html += "th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }";
  html += "th { background-color: #4CAF50; color: white; }</style></head><body>";
  html += "<h1>Parqueo Digital</h1><table><tr>";

  for (int i = 0; i < 4; i++) {
    html += "<th>Parqueo " + String(i + 1) + "</th>";
  }
  html += "</tr><tr>";

  for (int i = 0; i < 4; i++) {
    bool ocupado = bitRead(estado, i);
    html += "<td style='color:" + String(ocupado ? "red" : "green") + "'>";
    html += ocupado ? "Ocupado" : "Libre";
    html += "</td>";
  }

  html += "</tr></table></body></html>";
  return html;
}

