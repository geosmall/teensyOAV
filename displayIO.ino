// **************************************************************************
// * teensyOAV - a port of David Thompsen's OpenAero code to the teensy 4
// *
// * John Ihlein  July 2021
// **************************************************************************

// **************************************************************************
// OpenAero code by David Thompson, included open-source code as per quoted references.
//
// **************************************************************************
// *             GNU GPL V3 notice
// **************************************************************************
// * Copyright (C) 2016 David Thompson
// * 
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// * 
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// * GNU General Public License for more details.
// * 
// * You should have received a copy of the GNU General Public License
// * along with this program. If not, see <http://www.gnu.org/licenses/>.
// * 
// * tl;dr - all derivative code MUST be released with the source code!
// *
// **************************************************************************

// ************************************************************
// *Code
// ************************************************************

void displayInOut(void)
{
  uint8_t  i = 0;
  int16_t  temp = 0;
  uint16_t uTemp = 0;
  int8_t   pos1, pos2, pos3;
  int16_t  outputs[MAX_OUTPUTS];
  int16_t  inputs[MAX_RC_CHANNELS];
  float    tempf1 = 0.0;
  
  // While back button not pressed
  while(digitalRead(BUTTON1) != 0)
  {
    if (frame_100Hz) {
      frame_100Hz = false;
      updateTransition();
    }
    
    if (frame_500Hz) {
      frame_500Hz = false;
      tasks500Hz();  
    }
  
    if (frame_10Hz) {
      frame_10Hz = false;
      
      updateServos();     // Transfer config.channel[i].value data to servoOut[i]

      // Re-span numbers from internal values (2500 to 5000) to microseconds
      for (i = 0; i < MAX_OUTPUTS; i++)
      {
        temp = servoOut[i];  // Promote to 16 bits

        // Check for motor marker and ignore if set
        if (config.channel[i].motorMarker != MOTOR)
        {
          // Scale servo from 2500~5000 to 875~2125
          temp = ((temp - 3750) >> 1) + SERVO_CENTER; // SERVO_CENTER = 1500
        }
        else
        {
          // Scale motor from 2500~5000 to 1000~2000
          temp = ((temp << 2) + 5) / 10;  // Round and convert
        }
    
        servoOut[i] = (uint16_t)temp;
      }

      // Check limits in microsecond units.
      for (i = 0; i < MAX_OUTPUTS; i++)
      {
        temp = servoOut[i];
      
        // Enforce min, max travel limits
        if (temp > config.limits[i].maximum)
        {
          temp = config.limits[i].maximum;
        }

        else if (temp < config.limits[i].minimum)
        {
          temp = config.limits[i].minimum;
        }
      
        servoOut[i] = temp;
      }
    
      // Servos are now in microsecond units.
    
      // Check for motor flags if throttle is below arming minimum
      if (monopolarThrottle < THROTTLEIDLE)  // THROTTLEIDLE = 50
      {
        // For each output
        for (i = 0; i < MAX_OUTPUTS; i++)
        {
          // Check for motor marker
          if (config.channel[i].motorMarker == MOTOR)
          {
            // Set output to minimum pulse width (1000us)
            servoOut[i] = MOTORMIN;
          }
        }
      }

      // Convert outputs to percentages (center and divide by 4)
      for (i = 0; i < MAX_OUTPUTS; i++)
      {
        outputs[i] = (int16_t)servoOut[i];
        outputs[i] = outputs[i] - SERVO_CENTER;
      
        // Round correctly in both directions
        if (outputs[i] >= 0)
        {
          outputs[i] = (outputs[i] + 2) / 5;  // Convert to percentages +/-500 = +/-100%
        }
        else
        {
          outputs[i] = (outputs[i] - 2) / 5; 
        }
      }

      // Convert inputs to percentages (center and divide by 10)
      for (i = 0; i < MAX_RC_CHANNELS; i++)
      {
        tempf1    = (float)rcInputs[i];
        inputs[i] = (int16_t)(tempf1 / 10.0f);
      }

      u8g2.clearBuffer();
    
      // Column 1
      lcdDisplayText(478, 0,  3);
      lcdDisplayText(479, 0, 13);
      lcdDisplayText(480, 0, 23);
      lcdDisplayText(481, 0, 33);
      lcdDisplayText(482, 0, 43);
      lcdDisplayText(483, 0, 53);

      itoa((monopolarThrottle / 20), pBuffer, 10);  u8g2.drawStr(25,  3, pBuffer);
      itoa(inputs[AILERON],          pBuffer, 10);  u8g2.drawStr(25, 13, pBuffer);
      itoa(inputs[ELEVATOR],         pBuffer, 10);  u8g2.drawStr(25, 23, pBuffer);
      itoa(inputs[RUDDER],           pBuffer, 10);  u8g2.drawStr(25, 33, pBuffer);
      itoa(inputs[GEAR],             pBuffer, 10);  u8g2.drawStr(25, 43, pBuffer);
      itoa(inputs[AUX1],             pBuffer, 10);  u8g2.drawStr(25, 53, pBuffer);
    
      // Column 2
      lcdDisplayText(484, 48,  3);
      lcdDisplayText(485, 48, 13);
      lcdDisplayText(477, 48, 23);
      lcdDisplayText(468, 48, 33);
      lcdDisplayText(469, 48, 43);
      lcdDisplayText(470, 48, 53);   
    
      itoa(inputs[AUX2], pBuffer,10);  u8g2.drawStr(71,  3, pBuffer);
      itoa(inputs[AUX3], pBuffer,10);  u8g2.drawStr(71, 13, pBuffer);
      itoa(outputs[0],   pBuffer,10);  u8g2.drawStr(57, 33, pBuffer);
      itoa(outputs[1],   pBuffer,10);  u8g2.drawStr(57, 43, pBuffer);
      itoa(outputs[2],   pBuffer,10);  u8g2.drawStr(57, 53, pBuffer);
    
      // Column 3
      lcdDisplayText(471, 94,  3);
      lcdDisplayText(472, 94, 13);
      lcdDisplayText(473, 94, 23);
      lcdDisplayText(474, 94, 33);
      lcdDisplayText(475, 94, 43);
      lcdDisplayText(476, 88, 53);

      itoa(outputs[3], pBuffer, 10);  u8g2.drawStr(104,  3, pBuffer);
      itoa(outputs[4], pBuffer, 10);  u8g2.drawStr(104, 13, pBuffer);
      itoa(outputs[5], pBuffer, 10);  u8g2.drawStr(104, 23, pBuffer);
      itoa(outputs[6], pBuffer, 10);  u8g2.drawStr(104, 33, pBuffer);
      itoa(outputs[7], pBuffer, 10);  u8g2.drawStr(104, 43, pBuffer);

      // Display the transition number as 1.00 to 2.00
      uint8_t xLoc = 104;  // X location of transition display
      uint8_t yLoc = 53;   // Y location of transition display

      uTemp = transition + 100;
      temp = uTemp / 100;  // Display whole decimal part first
    
      itoa(temp,pBuffer,10);  u8g2.drawStr(xLoc, yLoc, pBuffer);
    
      pos1 = u8g2.getStrWidth(pBuffer);
    
      uTemp = uTemp - (temp * 100);  // Now display the parts to the right of the decimal point

      lcdDisplayText(268, (xLoc + pos1 + 1), yLoc);
    
      pos3 = u8g2.getStrWidth(".");
      pos2 = u8g2.getStrWidth("0");
    
      if (uTemp >= 10)
      {
        itoa(uTemp, pBuffer, 10);  u8g2.drawStr((xLoc + pos1 + pos3 + 3), yLoc, pBuffer);
      }
      else
      {
        lcdDisplayText(269, (xLoc + pos1 + pos3 + 3), yLoc);
        itoa(uTemp, pBuffer, 10);  u8g2.drawStr((xLoc + pos1 + pos2 + pos3 + 4), yLoc, pBuffer);
      }

      u8g2.sendBuffer();
    }
  }
}
