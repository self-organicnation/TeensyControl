#ifndef com_h
#define com_h

#define NBCHAR 20  // char size for text

void initCom();
void printSerialCommandsAvailable();
bool readSerial();
void addCharToSerialBuffer(char c);
void parseSerialBuffer();
void printCmdVal( String cmd, int32_t val);
void printCmdValVal( String cmd, int32_t val1, int32_t val2);

#endif