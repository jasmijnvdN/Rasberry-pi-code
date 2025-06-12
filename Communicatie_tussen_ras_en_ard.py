# -*- coding: utf-8 -*-
"""
Created on Wed Jun 11 11:46:20 2025

@author: jivdn
"""

#Arduino
String inputString = "";
bool receiving = false;
int messageCount = 0;

void setup() {
  Serial.begin(57600);  #57600 baud rate (zelfde als bij python code)
  delay(2000);          #tijd om op te starten
  Serial.println("yes");  #signaal voor rasberry pi om te starten
}

void loop() {
  if (Serial.available()) {
    inputString = Serial.readStringUntil('\n');
    inputString.trim();

    if (receiving) {
      Serial.print("Arduino received: ");
      Serial.println(inputString);
      messageCount++;

      // Send "stop" after 5 messages
      if (messageCount >= 5) {
        Serial.println("stop");
        receiving = false;
        messageCount = 0;
      }
    }

    // Wait for "start" command from Raspberry Pi
    if (inputString == "start") {
      receiving = true;
    }
  }
}


#PYTHON
import serial
import time

# Open seriele communicatie naar arduino
ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1) #maakt verbinding met de Arduino via de usb port
time.sleep(2)  #wacht voor arduino

try:
    while True:
        if ser.in_waiting > 0:
            msg = ser.readline().decode().strip()
            print(f"From Arduino: {msg}") #wacht op "yes" van arduino

            if msg.lower() == "yes":
                print("Arduino is ready. Sending data...")
                ser.write(b"start\n")  #Arduino krijgt berichten terug
                time.sleep(0.5)

                while True:
                    # Send data to Arduino
                    ser.write(b"data from pi\n") #stuurt berichten terug
                    print("Sent: data from pi")  
                    time.sleep(1)

                    # wacht op "stop" van Arduino
                    if ser.in_waiting > 0:
                        response = ser.readline().decode().strip()
                        print(f"Arduino says: {response}")
                        if response.lower() == "stop":
                            print("Received 'stop'. Ending transmission.")
                            break
                break  #exit
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    ser.close()
    print("Serial connection closed.")
