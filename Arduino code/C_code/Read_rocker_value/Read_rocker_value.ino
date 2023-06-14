/*
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         Read_rocker_value
* @author       Jessica
* @version      V1.0
* @date         2019.8.21
* @brief        Read rocker value
* @details
* @par History  
*
*/
int VRX = A1; //X axis of rocker connected to analog A1 input
int VRY = A0; //Y axis of rocker connected to analog A0 input
int x = 0, y = 0; //Define variables x,y

/*
* Function       setup
* @author        Jessica
* @date          2019.8.21
* @brief         Initial configuration
* @param[in]     void
* @retval        void
* @par History   no
*/
void setup()
{
  pinMode(VRX, INPUT);
  pinMode(VRY, INPUT);
  Serial.begin(9600);
}

/*
* Function       loop
* @author        Jessica
* @date          2019.8.21
* @brief         Read the analog value of the joystick and print it to the serial port. The range of the analog value is 0 on the left and 700 on the right.
* @param[in]     void
* @retval        void
* @par History   no
*/
void loop()
{
  x = analogRead(VRX); //Read the analog voltage value of A0 and assign it to x
  y = analogRead(VRY); //Read the analog voltage value of A1 and assign it to y
  //Serial.print("x: "); 
  //Serial.println(x);   
  Serial.print("y: ");
  Serial.println(y);
  delay(1000);//delay 1s
}
