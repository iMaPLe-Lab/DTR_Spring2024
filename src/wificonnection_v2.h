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

String readControllerData(WiFiClient& client) {
    static int16_t seqExpected = 0;

    // if data available from client read and display it
    int length;
    if ((length = client.available()) > 0) {
        //str = client.readStringUntil('\n');  // read entire response
        Serial.printf("Received length %d - ", length);
        // if data is correct length read and display it
        if (length == sizeof(data)) {
            client.readBytes((char*)&data, sizeof(data));
            Serial.printf("seq %d left_x %d left_y %d right_x %d right_y %d l2_trigger %d r2_trigger %d r1_button %d l1_button %d x_button %d y_button %d \n", 
                        (short)data.seq, (short)data.left_x, (short)data.left_y, (short)data.right_x, (short)data.right_y, (short)data.l2_trigger, (short)data.r2_trigger,
                         (bool)data.r1_button, (bool)data.l1_button, (bool)data.x_button, (bool)data.y_button);
            if (data.seq != seqExpected)  // check sequence number received
                Serial.printf("Error! seq expected %d received %d\n", seqExpected, data.seq);
            seqExpected = data.seq;  // set sequence number ready for next data
            seqExpected++;
        } else {
            while (client.available()) Serial.print((char)client.read());  // discard corrupt packet
            Serial.printf("corrupt packet, expected %d bytes \n", sizeof(data));
        }
            
    }
    return "success";
}
