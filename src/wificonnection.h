

String readInteger(WiFiClient& client) {
  byte buffer[5];

  // Wait for 5 bytes to be available
  while (client.available() < 5) {
    delay(1);  // Small delay to wait for data
  }

  // Read 5 bytes from the client
  client.readBytes(buffer, 5);

  // Convert the first byte to an integer (assuming it's in ASCII format)
  int firstNumber = buffer[0];  // Convert from HEX to int (only if it's an ASCII digit)
  // Serial.print((char)firstNumber);
  // Combine the next four bytes into a 32-bit integer
  int32_t secondNumber = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);

  // Perform sign extension if the number is negative
  if (buffer[4] & 0x80) {
    secondNumber |= 0xFF000000;  // Sign extension
  }

  // Create a string from both numbers
  String result = String(firstNumber) + "," + String(secondNumber);

  return result;
}


void processData(String data, int* ptrLY, int* ptrRX, int* ptrTru, int* ptrBreak, int* ptrCat, int* ptrAtt) {
  int commaIndex = data.indexOf(',');  // Find the position of the comma
  if (commaIndex == -1) return;        // If there's no comma, exit the function

  // Split the string into the identifier and value parts
  String idStr = data.substring(0, commaIndex);
  String valueStr = data.substring(commaIndex + 1);

  // Convert the identifier to an integer
  int id = idStr.toInt();

  // Convert the value part to an integer
  int value = valueStr.toInt();

  // Use a switch-case to determine the action based on the identifier
  switch (id) {
    case 89:  // left_y Robot up and down motion Range -100 to 100
      // Serial.print("left_y:");
      *ptrLY= value;
      // Serial.println(leftJoystickY);
      break;
    case 88:  // right_x Robot Yaw motion, -100 to 100

      *ptrRX= value;
      // Serial.print("right_x:");
      // Serial.println(rightJoystickX);
      break;
    case 83:  // x_button stop capture
      // Serial.print("x_button:");
      if(value ==1) *ptrCat=0;
      
      // Serial.println(value);
      break;
    case 70:  // l2_trigger Backward
      // Serial.print("l2_trigger:");
      *ptrBreak = value;
      // Serial.println(value);
      break;
    case 66:  // r2_trigger Forward
      *ptrTru = value;
      // Serial.print("r2_trigger:");
      // Serial.println(value);
      break;
    case 67:  // r1_button Start capture
      if(value ==1) *ptrCat=1;
      // Serial.print("r1_button:");
      // Serial.println(value);
      break;
    case 82:  // l1_button Stop capture
      if(value ==1) *ptrCat=3;
      // Serial.print("l1_button:");
      // Serial.println(value);
      break;
    case 79:
      if(value ==1) *ptrCat=2;
      // Serial.print("O_button:");
      // Serial.println(value);
      break;

    case 65:
      *ptrAtt = value;
    default:
      // Serial.print("Unknown ID:");
      // Serial.println(value);
      break;
  }
}