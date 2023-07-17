else if ( control == 'A' )
    {
      while ( 1 )
      {
        Angle = GetAngle() ;
        //Serial.println(Angle) ;
        if ( car.available() == 1 )
           break ;           
      }      
      InputAngle = GetInput() ;
       while ( InputAngle > 360 )
      {
        InputAngle -= 360 ;
      }
      while ( InputAngle < -360 )
      {
        InputAngle += 360 ;
      }
         InitialAngle = Angle ;
      FinalAngle = InitialAngle + InputAngle ;
      while (1)
      {
 if ( InputAngle > 0 )
      {
        analogWrite(pwm1,80);
        analogWrite(pwm2,80);
        digitalWrite(motor1_left,HIGH);
        digitalWrite(motor1_right,LOW);
        digitalWrite(motor2_left,LOW);
        digitalWrite(motor2_right,HIGH); 
        Angle = GetAngle();
        if(Angle>=FinalAngle)  
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            break;
          }
      }
      else if ( InputAngle < 0 )
     {
        analogWrite(pwm1,80);
        analogWrite(pwm2,80);
        digitalWrite(motor1_left,LOW);
        digitalWrite(motor1_right,HIGH);
        digitalWrite(motor2_left,HIGH);
        digitalWrite(motor2_right,LOW);  
         angle = GetAngle();
        if(Angle<=FinalAngle)  
        {
            digitalWrite(motor1_left,LOW);
            digitalWrite(motor1_right,LOW);
            digitalWrite(motor2_left,LOW);
            digitalWrite(motor2_right,LOW); 
            break;
          } 
      }
      }
  }
