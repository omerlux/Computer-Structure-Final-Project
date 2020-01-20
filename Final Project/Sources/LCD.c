#include "TFC.h"
#include "mcg.h"


void LCD_Config(){
	
	//---------------------------------------------------------------------------------------------
	//Enable Clocks to all ports - page 206, enable clock to Ports
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
	SIM_SCGC6 |= SIM_SCGC6_DAC0_MASK;
	
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK; //Enable the Clock to the PIT Modules
	// Timer 0
	//PIT_TCTRL0 = PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK; //Enabale PIT and inrpt
	//PIT_MCR |= PIT_MCR_FRZ_MASK; 						  // Stop  PIT in debug mode
	//enable_irq(INT_PIT-16); 							  //Enable PIT IRQ on the NVIC
	//set_irq_priority(INT_PIT-16,0);  					  // Interrupt priority = 0 = MAX
	
	DAC0_C0 |= DAC_C0_DACEN_MASK + DAC_C0_DACRFS_MASK + DAC_C0_DACTRGSEL_MASK + DAC_C0_LPEN_MASK; 	// DAC Enable, DAC Reff select, DAC trigger software
	DAC0_DAT0L =0xff;
	DAC0_DAT0H =0x05; //Brightness setup
	
  //PIT_MCR &= ~PIT_MCR_MDIS_MASK; //0 - Clock for standard PIT timers is enabled
  	
	//------------------Initiate PORTS----------------------------------------------------------------
	
	PORTE_PCR1 = PORT_PCR_MUX(1);
	PORTA_PCR16 = PORT_PCR_MUX(1);
	PORTD_PCR3 = PORT_PCR_MUX(1);
	PORTC_PCR17 = PORT_PCR_MUX(1);
	PORTD_PCR2 = PORT_PCR_MUX(1);
	PORTC_PCR16 = PORT_PCR_MUX(1);
	PORTD_PCR0 = PORT_PCR_MUX(1);
	PORTC_PCR13 = PORT_PCR_MUX(1);
	PORTD_PCR5 = PORT_PCR_MUX(1);
	PORTC_PCR12 = PORT_PCR_MUX(1);
	PORTA_PCR13 = PORT_PCR_MUX(1);
		
	GPIOE_PDDR|= PORT_LOC(1);
	GPIOA_PDDR|= PORT_LOC(16);
	GPIOD_PDDR|= PORT_LOC(3);
	GPIOC_PDDR|= PORT_LOC(17);
	GPIOD_PDDR|= PORT_LOC(2);
	GPIOC_PDDR|= PORT_LOC(16);
	GPIOD_PDDR|= PORT_LOC(0);
	GPIOC_PDDR|= PORT_LOC(13);
	GPIOD_PDDR|= PORT_LOC(5);
	GPIOC_PDDR|= PORT_LOC(12);
	GPIOA_PDDR|= PORT_LOC(13);
	
	GPIOE_PDDR|= PORT_LOC(30); // For DAC PTE30
	
  	//------------------Initiate LCD----------------------------------------------------------------
	
	GPIOE_PCOR = PORT_LOC(1);  // RS  = '0'
	GPIOA_PCOR = PORT_LOC(16); // R/W = '0'
	GPIOD_PCOR = PORT_LOC(3);  // E   = '0'
	//Delay1ms(15);
	DataByte(0x3f);
	LCD_Strobe();
	//Delay1ms(5);
	DataByte(0x3f);
	LCD_Strobe();
	//Delay1ms(1);
	DataByte(0x3f);
	LCD_Strobe ();
	LCD_Command (0x3c);	
	LCD_Command (0x0f);	
	LCD_Command (0x01);	
	LCD_Command (0x06);	
	LCD_Command (0x80);	
	LCD_Command (0x02);	
}
//---------------------------All other routine---------------------------------

void LCD_Strobe (){
	GPIOD_PSOR = PORT_LOC(3);  // E   = '1'
	Nop();
	Nop();
	GPIOD_PCOR = PORT_LOC(3);  // E   = '0'
}

void Delay1ms(int n){
	int i;
	for(i=0;i<=10000*n;i++);
	//loop 10000*n 
}

void LCD_Command (int Command){
	//Delay1ms(5);
	DataByte(Command);
	LCD_Strobe ();
}

void LCD_Data(char char_value)
{
	//Delay1ms(5);
	DataByte(0x00);
	GPIOE_PSOR = PORT_LOC(1);	// RS  = '0'
	DataByte(char_value);
	LCD_Strobe ();
	GPIOE_PCOR = PORT_LOC(1); 	// RS  = '0'
}

void DataByte(int data) {
	int BytePart = data;
		if(BytePart%2==0)
			GPIOC_PCOR = PORT_LOC(17);
		else 
			GPIOC_PSOR = PORT_LOC(17);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOD_PCOR = PORT_LOC(2);
		else 
			GPIOD_PSOR = PORT_LOC(2);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOC_PCOR = PORT_LOC(16);
		else 
			GPIOC_PSOR = PORT_LOC(16);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOD_PCOR = PORT_LOC(0);
		else 
			GPIOD_PSOR = PORT_LOC(0);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOC_PCOR = PORT_LOC(13);
		else 
			GPIOC_PSOR = PORT_LOC(13);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOD_PCOR = PORT_LOC(5);
		else 
			GPIOD_PSOR = PORT_LOC(5);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOC_PCOR = PORT_LOC(12);
		else 
			GPIOC_PSOR = PORT_LOC(12);
		BytePart = BytePart/2;
		
		if(BytePart%2==0)
			GPIOA_PCOR = PORT_LOC(13);
		else 
			GPIOA_PSOR = PORT_LOC(13);
}

void Nop(){
	
}


