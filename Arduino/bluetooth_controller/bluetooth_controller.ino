char t;


int ENB = 5;
int IND = 9;
int INC = 6;

int ENA = 7;
int INA = 8;
int INB = 13;

int QR = (140);
int QL = (140);
void setup() {
  
pinMode(ENB,OUTPUT);   //left motors forward
pinMode(IND,OUTPUT);   //left motors reverse
pinMode(INC,OUTPUT);   //right motors forward
pinMode(ENA,OUTPUT);   //right motors reverse
pinMode(INA,OUTPUT);   //Led
pinMode(INB,OUTPUT);
Serial.begin(9600);


}
 
void loop() {
if(Serial.available()){
  t = Serial.read();
  Serial.println(t);
  //Serial.println(Q);
}
 
if(t == 'F'){            //move forward(all motors rotate in forward direction)
  analogWrite(ENA,QR);
  analogWrite(ENB,QL);
  digitalWrite(IND,HIGH);
  digitalWrite(INA,HIGH);
  digitalWrite(INC,LOW);
  digitalWrite(INB,LOW);
  Serial.println(t);
}
 
else if(t == 'B'){      //move reverse (all motors rotate in reverse direction)
  analogWrite(ENA,QR);
  analogWrite(ENB,QL);
  digitalWrite(IND,LOW);
  digitalWrite(INA,LOW);
  digitalWrite(INC,HIGH);
  digitalWrite(INB,HIGH);
  Serial.println(t);
}
 
else if(t == 'L'){      //turn right (left side motors rotate in forward direction, right side motors doesn't rotate)
  analogWrite(ENA,QR);
  analogWrite(ENB,QL);
  digitalWrite(IND,HIGH);
  digitalWrite(INA,LOW);
  digitalWrite(INC,LOW);
  digitalWrite(INB,HIGH);
  Serial.println(t);
}
 
else if(t == 'R'){      //turn left (right side motors rotate in forward direction, left side motors doesn't rotate)
  analogWrite(ENA,QR);
  analogWrite(ENB,QL);
  digitalWrite(IND,LOW);
  digitalWrite(INA,HIGH);
  digitalWrite(INC,HIGH);
  digitalWrite(INB,LOW);
  Serial.println(t);
}
 
else if(t == 'S'){      //STOP (all motors stop)
  analogWrite(ENA,QR);
  analogWrite(ENB,QL);        
  digitalWrite(IND,LOW);
  digitalWrite(INA,LOW);
  digitalWrite(INC,LOW);
  digitalWrite(INB,LOW);
  Serial.println(t);
}
//delay(Q);
}
