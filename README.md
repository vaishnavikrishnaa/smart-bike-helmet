Accidents are a major concern on roads, and providing immediate assistance can help save lives. To address this issue, we have developed a project called Smart Bike Helmet.
This helmet is designed using Embedded C and integrates multiple sensors, including GPS, gyroscope, touch sensor, pulse sensor, and an 8-array IR sensor.

Pulse Sensor for heart rate monitoring

MPU6050 Accelerometer & Gyroscope for motion analysis

Touch Sensor for helmet usage verification

GPS for location tracking

IR Sensor Array
    * Detects the position of the biker or surrounding obstacles.
    * Can help determine if the biker has fallen or if there are potential hazards nearby.


All these sensors are connected to an ESP32 microcontroller, which processes the data.

Once the code is uploaded to ESP32, Through the wifi connection the sensor readings are collected and stored on ThingSpeak, our chosen cloud platform.


The system detects key safety parameters:

GPS Tracking:# Providing the exact location of the rider in case of an accident.

Pulse Rate Monitoring: #Detecting abnormal heart rates in case of emergencies.

Helmet Detection: #Ensuring the rider wears the helmet before starting.

Accident Detection: I#dentifying crashes using motion anomalies and impact force.  we have created a website that displays the biker's data, helping detect whether an accident has occurred. This enables timely intervention and assistance.  If the helmet is not worn, an alert is generated.
* If an abnormal gyroscopic change, high acceleration, or abnormal pulse rate is detected, an accident is assumed.
* The system sends data to ThingSpeak, and an alert can be sent for emergency response.
  All components of our Smart Bike Helmet are securely placed inside the helmet, except for the GPS module, which remains external to ensure accurate latitude and longitude detection. The pulse sensor continuously monitors vital signs for accident detection, while the touch sensor verifies helmet usage and enhances accident detection accuracy. Additionally, our system intelligently differentiates between actual accidents and false helmet falls, as the accident status is only triggered when multiple sensor conditions in the code’s logic are met, ensuring reliable and precise detection.  The Smart Bike Helmet is a step towards smarter and safer commuting, leveraging IoT technology to enhance rider security and provide critical accident response capabilities.

Stay safe, ride smart!
![image](https://github.com/user-attachments/assets/1fb27053-c79f-4a98-af60-dc99c7f37e48)
![image](https://github.com/user-attachments/assets/34468430-6df1-42d2-9fff-f5f75d5083b3)
![image](https://github.com/user-attachments/assets/f31da567-d46f-4e25-8e38-85619460def1)
![image](https://github.com/user-attachments/assets/8ffe8952-15f1-4636-b42f-733857d4e32e)



