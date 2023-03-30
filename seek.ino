int outPinl = 8;
int outPinr = 9;
int right=0;
int left = 0;
int in1 = 5;
int in2 = 4;
// motor two
int in3 = 7;
int in4 = 6;
int in5 = 3;
int autonomous=1;
int control = 0; 

int DIR_L_F = 13;    // Left Direction pin that will indicate forewards movement (1 for forewards, 0 for backwards).
int DIR_L_B = 12;    // Left Direction pin that will indicate backwards movement (1 for backwards, 0 for forewards).
int PWM_L = 11;      // Speed controll pin. *** This pin must be plugged into an output pin on the arduino that is labled PWM ***.

int DIR_R_F = 4;     // Right Direction pin that will indicate forewards movement (1 for forewards, 0 for backwards)
int DIR_R_B = 2;     // Left Direction pin that will indicate backwards movement (1 for backwards, 0 for forewards).
int PWM_R = 3;       // Speed controll pin. *** This pin must be plugged into an output pin on the arduino that is labled PWM ***.


int trigPin = 11; // 'trig' pin defintion
int echoPin = 10; // 'echo' pin definition
double pingTime; // variable to hold
double DIST_SCALE = 1.085767; // **MODIFY TO YOUR OWN**
double SPEED_OF_SOUND = 331.0; // Approx. speed of sound
double convert = 1000000.0; // factor to scale pingTime

void GetBTCommand();                   //Outputs a list containing the raw controller inputs. This function is given a list pointer, and a character teling it which app is being used. 
void UnicaTranslateCommand();          //Gets used by GetBTCommand() when '#' is sent as an input. This function decodes the string message and fills the "TranslatedCommand" list.  
void BluetoothPadTranslateCommand();   //Gets used by GetBTCommand() when '\n' is sent as an input. This function decodes the string message and fills the "TranslatedCommand" list. 
void SimpleMapInput();                 //A pre write function That takes in TransatedCommand as an imput. It uses the Angle and Power inputs (first 2 indexes of the input list) and output both motor speed and direction information for the left and right motors.
void ExecuteCommand_L298N();           //Takes in a 4 element list (Left Direction, Right Direction, Left Speed, Right Speed) and applies the correct outputs to the 6 pins used to drive the motors. (these pins are determined by you at the begining of the code) 

float ControllerInput[8];    // Angle, speed, button 1-4 (Android)   or    Angle, speed, slider value, channel 0-3 (Windows) 
float MotorOutputs[4];       // Left Direction, Right Direction, Left Speed, Right Speed.


void setup() {
  Serial.begin(9600);
  pinMode(outPinl, INPUT);
  pinMode(outPinr, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(trigPin, OUTPUT); // Set pin modes
  pinMode(echoPin, INPUT);

  pinMode(DIR_L_F, OUTPUT);
  pinMode(DIR_L_B, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  pinMode(DIR_R_F, OUTPUT);
  pinMode(DIR_R_B, OUTPUT);
  pinMode(PWM_R, OUTPUT);
}



void loop() {

  digitalWrite(trigPin, LOW); // Set pin to low before pulse
  delayMicroseconds(2000);
  digitalWrite(trigPin, HIGH); // Send ping
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW); // Set pin to low again

  pingTime = pulseIn(echoPin, HIGH); // Receive the echo from the pulse 
  double dist = DIST_SCALE*0.5*(SPEED_OF_SOUND*(pingTime/convert)); // Convert to distance

  Serial.println("Distance is: "); // Show distance in serial monitor
  Serial.println(dist);

  if (dist >=1.0){
    autonomous = 0;
    control = 1; 
  }

  if (autonomous ==1){

    int outl = digitalRead(outPinl);
    int outr = digitalRead(outPinr);
    if (outl == 1){
      Serial.println("Black surface left");  
      left = 1;

    }

    if (outl == 0){
      Serial.println("White surface");
      left = 0;
    }

    if (outr == 1){
      Serial.println("Black surface ");  
      right = 1;

    }

    if (outr == 0){
      Serial.println("White surface");
      right = 0;
    }

    if (left==1){
    Serial.println("left");
    digitalWrite(DIR_L_F, HIGH);
    digitalWrite(DIR_L_B, LOW);
    digitalWrite(DIR_R_B, HIGH);
    digitalWrite(DIR_R_F, LOW);
    analogWrite(PWM_L, 200);
    analogWrite(PWM_R, 100);
  }
  else if (right ==1){
    Serial.println("right");
    digitalWrite(DIR_R_F, HIGH);
    digitalWrite(DIR_R_B, LOW);

    analogWrite(PWM_L, 100);
    analogWrite(PWM_R, 200);
  }
  else{
    digitalWrite(DIR_L_F, HIGH);
    digitalWrite(DIR_R_F, HIGH);
    analogWrite(PWM_L, 200);
    analogWrite(PWM_R, 200);
  }
  }

  


  if (control ==1){
    GetBTCommand('#', ControllerInput);  // '\n' for Windows and '#' for android 
    SimpleMapInput(MotorOutputs, ControllerInput);
    ExecuteCommand_L298N(MotorOutputs);
  }

}




void GetBTCommand(char border,float * ans ){
  char data;                        //Variable to store the data
  String Command = "";              //variable to be returned
  while(true){
    if ( Serial.available()){       //Checks the availability of Serial port
      data = Serial.read();         // Read the data and stores it in variable. 
      if(data == border){           // Different commands are separated by # or \n characters. If you read in a # or \n it is clear that a command has just ended.
        if(border == '#'){
          UnicaTranslateCommand(ans , Command);
          return; 
        }else{
          BluetoothPadTranslateCommand(ans , Command);
          return;
        }
      }else{
        Command = Command + String(data); //Append the character to the end of the command string 
      }
    }
  }
  Serial.print("lalalalala");
}




void BluetoothPadTranslateCommand(float * ans , String Input){
  int X;
  int Y;
  int S;
  int C = Input.substring(0,1).toInt();
  int start = 2;
  int count = 2;
  int i = 0;
  
  while(i<2){
    count ++;

    if(Input.substring(count,count+1)==","){
      if(i==0){
        X=Input.substring(start,count).toInt();
        start = count + 1;
        i++;
      }else if(i==1){
        Y=Input.substring(start,count).toInt();
        start = count + 1;
        i++;
        S=Input.substring(start,Input.length()).toInt();
      }
    }
  }


  float angle = 180.0*atan2(Y,X)/3.14;
  if(Y<0){
    angle+=360.0;
  }
  
  float power = sqrt((X*X)+(Y*Y));
  if(power>100){
    power = 100.0;
  }
  power = power * 2.55;

  ans[0]=angle;
  ans[1]=power;
  ans[2]=S;
  for(int i = 3; i<7; i++){
    if((C+3) == i){
      ans[i]=1;
    }else{
      ans[i]=0;
    }
  }  
}




void UnicaTranslateCommand(float * ans , String Input){
  float angle = float(Input.substring(0,3).toInt());      //First get the abc numbers out of the command and then convert to integer.
  float power = float(Input.substring(3,6).toInt()*2.55); //Next get the def numbers out of the command and then convert to integer. Finally convert this 0-100 value into a 0-255 value.
  int button = float(Input.substring(6).toInt());         //Finally get the g number out of the command and then convert to integer.
  ans[0]=angle;
  ans[1]=power;
  for(int i = 2; i<7; i++){
    if(button==(i-1)){
      ans[i]=1;
    }else{
      ans[i]=0;
    }
  }

  Serial.println(ans[0]);
  Serial.println(ans[1]);
  Serial.println(ans[2]);
  Serial.println(ans[3]);
  Serial.println(ans[4]);
  Serial.println(ans[5]);
  Serial.println(ans[6]);
  Serial.println("");
  
}




void SimpleMapInput(float * ans, float * Input){
  float angle = Input[0];
  float power = Input[1];
  int L_direction;
  int R_direction;
  float L_speed;
  float R_speed;


  // define direction for L motor (corrasponds with motor A on the motor driver)
  if(angle<=180 ||angle>=330){
    L_direction = 1;
  }else{
    L_direction = 0;
  }

  // define direction for R motor (corrasponds with motor B on the motor driver)
  if(angle<=200){
    R_direction = 1;
  }else{
    R_direction = 0;
  }

  // define L speed
  if(angle>=90 && angle<=270){
    L_speed = abs(((angle-90.0)/90.0)-1.0)*power; //The angle will range from 90 -> 270 and will output a L_Speed range of 1 -> -1, multiplied by the the power variable.
  }else if(angle<90){
    L_speed = (0.5+(angle/180.0))*power; //The angle will range from 0 -> 90 and will output a L_Speed range of 0.5 -> 1, multiplied by the the power variable.
  }else{
    L_speed = abs(0.7 + (((angle-360.0)/90.0)*1.7))*power; //The angle will range from 270 -> 360 and will output a L_Speed range of -1 -> 0.5, multiplied by the the power variable.
  }

  // define R speed
  if(angle<=90){
    R_speed = (angle/90.0)*power;
  }else if(angle>=270){
    R_speed = abs((angle-360)/90)*power;
  }else if(angle>90 && angle<180){
    R_speed = (1-((angle-90)/180))*power;
  }else{
    R_speed = abs(0.7 - (((angle-180.0)/90.0)*1.7))*power;
  }


  ans[0] = (L_direction);
  ans[1] = (R_direction);
  ans[2] = (L_speed);
  ans[3] = (R_speed);
}




void ExecuteCommand_L298N(float * Command){
  
  if(Command[0]>0){
    digitalWrite(DIR_L_F, HIGH);
    digitalWrite(DIR_L_B, LOW);
  }else{
    digitalWrite(DIR_L_B, HIGH);
    digitalWrite(DIR_L_F, LOW);
  }

  if(Command[1]>0){
    digitalWrite(DIR_R_F, HIGH);
    digitalWrite(DIR_R_B, LOW);
  }else{
    digitalWrite(DIR_R_B, HIGH);
    digitalWrite(DIR_R_F, LOW);
  }
  
  // Next set speed:
  analogWrite(PWM_L, Command[2]);
  analogWrite(PWM_R, Command[3]);
}
