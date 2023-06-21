#include "HardwareSerial.h"
#include "Arduino.h"
#include "WString.h"

/* Instrukce:

XYOTTTTTTTT\n

X - pozice X
Y - pozice Y
O - orientace
T - čas v milisekndách (libovolná délka)

*/

enum class Direction { LEFT, UP, RIGHT, DOWN};

struct Position {
  public:
    byte x;
    byte y;
    Direction orientation;
};

struct DanceInstruction{
  public:
    Position instruction_position;
    unsigned long time;
    bool row_first;
};

class Choreography{
  private:
    bool hasNextStatus = true;
    String rowChars = "ABCDEFGHI";
    DanceInstruction initState;
    byte lastByte;


    void printInstruction(DanceInstruction di){
      Serial.print("X: ");
      Serial.print(di.instruction_position.x);
      Serial.print(" Y: ");
      Serial.print(di.instruction_position.y);
      Serial.print("\tO: ");
      switch(di.instruction_position.orientation){
        case Direction::UP    : Serial.print("U"); break;
        case Direction::DOWN  : Serial.print("D"); break;
        case Direction::LEFT  : Serial.print("L"); break;
        case Direction::RIGHT : Serial.print("R"); break;
      }
      Serial.print("\tT: ");
      Serial.print(di.time);
      Serial.print('\n');
    }

    bool isRow(char c){
      return rowChars.indexOf(c) != -1;
    }

    byte translateCollumnsToCoords(char c){
      return (byte) (c - 'A') + 1;
    }

    void eatWhiteSpaces(){
      while(isWhitespace(Serial.peak())){
        Serial.read();
      }
    }

    long serialReadLong(){
      unsigned long output = 0;
      while(isDigit(Serial.peak())){
        output *= 10;
        output += Serial.read();
      }
      return output;
    }

  public:

    Choreography(){
      eatWhiteSpaces();
      char a = (char) Serial.read();
      char b = (char) Serial.read();
      char c = (char) Serial.read();

      if(isRow(a)){
        initState.instruction_position.x = a;
        initState.instruction_position.y = b;
      } else {
        initState.instruction_position.x = b;
        initState.instruction_position.y = a;
      }

       switch(c){
        case 'W' : initState.orientation = Direction::UP;    break;
        case 'E' : initState.orientation = Direction::RIGHT; break;
        case 'N' : initState.orientation = Direction::LEFT;  break;
        case 'S' : initState.orientation = Direction::DOWN;  break;
      }    
      initState.time = 0;
    }

    bool hasNext(){
      return Serial.available();
    }

    DanceInstruction getNextInstruction(){
      DanceInstruction newInstruction;
      if(!hasNext()){
        newInstruction.instruction_position.x = -1;
        newInstruction.instruction_position.y = -1;
        newInstruction.time = 0;
        Serial.println("NO more instructions");
        return newInstruction;
      }
      eatWhiteSpaces();
    

      byte a = Serial.read();
      byte b = Serial.read();
      if (isRow(a)){
        newInstruction.instruction_position.x = translateCollumnsToCoords(a);
        newInstruction.instruction_position.y = b;
        newInstruction.row_first = true;
      } else {
        newInstruction.instruction_position.x = translateCollumnsToCoords(b);
        newInstruction.instruction_position.y = a;
        newInstruction.row_first = false;
      }

      eatWhiteSpaces();
      Serial.read(); // zahoď T
      newInstruction.time = serialReadLong();
      printInstruction(newInstruction);
      return newInstruction;
    }

    DanceInstruction getInitialState(){
      
    }

};
