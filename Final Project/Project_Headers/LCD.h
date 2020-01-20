/*
 * LCD.H
 *
 */

#ifndef LCD_H_
#define LCD_H_

void LCD_Config();
void LCD_Strobe ();
void LCD_Command (int Command);

void Delay1min(int n);
void DataByte();
void Nop();

#endif /* LCD_H_ */
