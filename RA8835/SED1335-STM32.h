#ifndef SED1335_STM32_H
#define SED1335_STM32_H

void GLCD_InitPorts(void);
void GLCD_WriteCommand(unsigned char);
void GLCD_WriteData(unsigned char);
unsigned char GLCD_ReadData(void);
char GLCD_ReadByteFromROMMemory(char *);

#endif /* SED1335_STM32_H */
