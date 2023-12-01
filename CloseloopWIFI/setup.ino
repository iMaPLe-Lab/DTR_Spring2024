
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


