/*********************************************
 * 1. Only for raspberry PI Pico
 * 2. Only suitable for 125M crystal oscillator
 * 3. date：2021-12-3
 * 4. programmer: jieliang mo
 * 5. https://github.com/earlephilhower/arduino-pico/
 * 6. https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 *********************************************/

#include"ir.h"

/////////////////////////////////////////////////////////
IR::IR(int p){
  pin_ = p;
  pinMode(pin_, INPUT);
}
/////////////////////////////////////////////////////////
boolean IR::IRStart(void){
  if (digitalRead(pin_) == HIGH) return false;

  unsigned long start = micros();
  while (digitalRead(pin_) == LOW) {
    if (micros() - start > 12000) return false;
  }

  unsigned long lowUs = micros() - start;
  if (lowUs < 4000 || lowUs > 10000) return false;

  start = micros();
  while (digitalRead(pin_) == HIGH) {
    if (micros() - start > 6000) return false;
  }
  return true;
}
/////////////////////////////////////////////////////////
int IR::getByte(void){  //接收32位红外数据（地址、地址反码、数据、数据反码）
    int Byte = 0;
    for(uint8_t i=0; i<8; i++){
        unsigned long start = micros();
        while(digitalRead(pin_)==LOW) {
          if (micros() - start > 1500) return -1;
        }

        start = micros();
        while(digitalRead(pin_)==HIGH) {
          if (micros() - start > 2500) return -1;
        }

        if(micros() - start > 1000)
            Byte |= 1 << i;
    }
    return Byte;	
}
/////////////////////////////////////////////////////////
int IR::getKey(void){
    int key[4] = {0, 0, 0, 0};
    if(IRStart() == false){     //判断红外引导脉冲
        //delay(108);             //One message frame lasts 108 ms.
        return -1;
    }
    else{
        for(uint8_t i=0; i<4; i++)
                key[i] = getByte();                                //接收32位红外数据（地址、地址反码、数据、数据反码）
        if(key[0] + key[1] == 0xff && key[2] + key[3] == 0xff)     //校验接收数据是否正确
            return key[2];
        else
            return -1;
    }	
}


