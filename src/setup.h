// Set the IP address and port to match the server you're connecting to
IPAddress serverIP(192, 168, 50, 93);
const uint16_t serverPort = 10000;
const char* ssid = "ZhouLab";
const char* password = "ZhouRobotics917";

// Create an instance of the WiFiServer class
WiFiServer server(serverPort);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  

  //////////// Motor setup ////////////
  motorSetup();
  AFMS.begin();


  //////////// Wifi setup ////////////
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());
  // Start the server
  server.begin();
  

  //////////// Sensor Setup ////////////
  sensorSetup();

  delay(2000);

}


