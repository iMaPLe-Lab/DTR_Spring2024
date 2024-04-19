// Set the IP address and port to match the server you're connecting to
IPAddress serverIP(192, 168, 50, 93);
const uint16_t serverPort = 10000;
const char* ssid = "ZhouLab";
const char* password = "ZhouRobotics917";

// Create an instance of the WiFiServer class
WiFiServer server(serverPort);

struct __attribute__((packed)) Data {
    int16_t seq;
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    int16_t l2_trigger;
    int16_t r2_trigger;
    bool r1_button;
    bool l1_button;
    bool x_button;
    bool y_button;
} data;

Data readControllerData(WiFiClient& client) {
    static int16_t seqExpected = 0;

    // if data available from client read and display it
    int length;
    if ((length = client.available()) > 0) {
        //str = client.readStringUntil('\n');  // read entire response
        // Serial.printf("Received length %d - ", length);
        // if data is correct length read and display it
        if (length >= sizeof(data)) {
            client.readBytes((char*)&data, sizeof(data));
            // Serial.printf("seq %d left_x %d left_y %d right_x %d right_y %d l2_trigger %d r2_trigger %d r1_button %d l1_button %d x_button %d y_button %d \n", 
            //             (short)data.seq, (short)data.left_x, (short)data.left_y, (short)data.right_x, (short)data.right_y, (short)data.l2_trigger, (short)data.r2_trigger,
            //              (bool)data.r1_button, (bool)data.l1_button, (bool)data.x_button, (bool)data.y_button);
            if (data.seq != seqExpected)  // check sequence number received
                Serial.printf("Error! seq expected %d received %d\n", seqExpected, data.seq);
            seqExpected = data.seq;  // set sequence number ready for next data
            seqExpected++;
        } else {
            while (client.available()) Serial.print((char)client.read());  // discard corrupt packet
            Serial.printf("corrupt packet, expected %d bytes \n", sizeof(data));
        }
            
    }
    return data;
}
int mapValue(int16_t value) {
  // Normalize the input value from -100 - 100 to 0 - 200
  int normalizedValue = value + 100;

  // Now map from 0 - 200 to 0 - 999
  return map(normalizedValue, 0, 200, 0, 700);
}

int setleftandrightSpeeds(int16_t input) {
    int normalizedValue = input;
    // Normalize input from -99 - 100 to -100 - 100 for easier calculations
    return map(normalizedValue, -100, 100, -100, 100);
}

void processData(Data data, int* ptrLX, int* ptrLY, int* ptrRX, int* ptrRY,int* ptrR2, bool* ptrX) {
  *ptrLX = data.left_x;
  *ptrLY = data.left_y;
  *ptrRX = data.l2_trigger;//mapValue(data.right_x); //L2 zero -100 , full 100 Trust 
  *ptrRY = setleftandrightSpeeds(-data.right_x);//setleftandrightSpeeds(data.right_y); //left right left 100 right -99 /
  *ptrR2 = data.r2_trigger;
  *ptrX = data.x_button;
}

