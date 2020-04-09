#include <pic.h>
#include <stdlib.h>
#include <math.h>


void Inicializa();	//funcion para configurar el pic
void PID();			//funcion calcula el valor del PWM
void Constantes();	//obtiene valor de las constantes

bit do_PID;
signed char en0, en1, en2, en3, off_set;
unsigned char temp;
short int temp_int;
unsigned short int ki, kd, kp;
signed int SumE_Min, SumE_Max, SumE, integral_term, derivative_term;
signed long Cn;

void main()
{
Inicializa();                        			
Constantes();						
while(1)                             	
 {	
 if(do_PID){
  PID();
  }
 }
}

void Inicializa()
{
    //Inicializa puertos de entrada-salida
	PORTA = 0;				
	TRISA = 0b00101101; 				
	PORTC = 0;
	TRISC = 0b00000011;	
	//Sin comparador	
	CMCON0 = 0x07;			
	//Frecuencia interna de 8MHz
	IRCF0 = 1;				
	IRCF1 = 1;				
	IRCF2 = 1;				

	CCP1CON = 0b01001100;	// Full bridge PWM
	ECCPAS = 0;				// Sin Auto_shutdown
	PR2 = 0x3F;				// Periodo de PWM a 31.2KHz
	T2CON = 0;				// Timer 2 apagado sin preescalador
	CCPR1L = 0;				// Valor del Ciclo de Trabajo del PWM
	TMR2ON = 1;				// Inicia Timer 2

	ANSEL = 0b00110101;		// ConfigurR AN0,AN2,AN4 and AN5 as analog
	//AN0=SENSOR
	//AN2=TEMRINO Proporcional
	//AN4=TERMINO INTEGRAL
	//AN5=TERMINO DERIVATIVO

	VCFG = 0;				// Rango para entrada analogica de Vdd a Vss
	ADFM = 1;				// justificado a la derecha
	//ADC Prescalado de 16 (Fosc/16)
	ADCS0 = 1;				
	ADCS1 = 0;
	ADCS2 = 1;
	//Canal AN0 seleccionado
	CHS0 = 0;				
	CHS1 = 0;
	CHS2 = 0;
	//ADC empieza a leer
	ADON = 1;				

	en0=en1=en2=en3=0;
	ki=kd=0;
	kp=off_set=0;
	temp_int=integral_term=derivative_term=0;
	SumE_Max=30000;
	SumE_Min=1-SumE_Max;
	//do_PID=1 entra en el ciclo
    do_PID = 1;				
	T0CS = 0;				// Timer0 como temporizador
	TMR0 = 10;				// Valor cargado a TIMER0
	//Preescalado del Timer=16 equivalente a 256Hz
    PSA = 0;				
	PS0 = 0;				
	PS1 = 0;
	PS2 = 1;

	INTCON = 0;	//Limpia el registro de interrupciones
	PIE1 = 0; 	
	T0IE = 1;	//activa modo de interrupcion cuando hay desbordamiento del TIMER0
	GIE = 1;	//activa interrupcion global
	return;
}


void Constantes()
{
 	//AN2 como entrada analogica
	ANS2 = 1;				
	ANS4 = 1;				
	ANS5 = 1;				

	ADFM = 1;				// justificado a la derecha
	
	//Canal AN4 seleccionado
	CHS0 = 0;				
	CHS1 = 0;
	CHS2 = 1;

	//Retardo
	temp = 200;				
	while(temp){
		temp--;
	}
	//empieza a leer ADC
	GODONE = 1;
	while(GODONE);{
		temp = 0;			// Esperar hasta cargar valor del ADC
	}
	ki = ADRESH << 8;		// Almacenar valor en costante integral
	ki = ki + ADRESL;

    //Canal AN5 seleccionado
	CHS0 = 1;				
	CHS1 = 0;
	CHS2 = 1;
	
	//Retardo
	temp = 200;				
	while(temp){
		temp--;
	}
	//empieza a leer ADC
    GODONE = 1;
	while(GODONE);{
		temp = 0;			// Esperar hasta cargar valor del ADC
	}
	kd = ADRESH << 8;		// Almacenar valor en constante derivativo
	kd = kd + ADRESL;

	//Canal AN2 seleccionado
	CHS0 = 0;				
	CHS1 = 1;
	CHS2 = 0;

	//Retardo
	temp = 200;				
	while(temp){
		temp--;
	}
	//empieza a leer ADC
	GODONE = 1;
	while(GODONE);{
		temp = 0;			// Esperar hasta carar valor del ADC
	}
	kp = ADRESH << 8;		// Almacenar valor en constante proporcional
	kp = kp + ADRESL;

	//Canal AN0 seleccionado
	CHS0 = 0;				
	CHS1 = 0;
	CHS2 = 0;	
}


void PID()								
{
	integral_term = derivative_term = 0;
	
// Calcula el termino Integral
	SumE = SumE + en0;			//se le suma el valor del error						
	if(SumE > SumE_Max){						
		SumE = SumE_Max;
	}
	if(SumE < SumE_Min){						
		SumE = SumE_Min;
	}											
	integral_term = SumE / 256;					// Dividido por la frecuencia de muestreo 256Hz
	integral_term = integral_term * ki;			// Multiplicado por la constante integral
	integral_term = integral_term / 16;			// Dividido por el factor de escala del ADC=16

// Calcula el termino Derivativo
	derivative_term = en0 - en3;
	if(derivative_term > 120){					
		derivative_term = 120;
	}
	if(derivative_term < -120){					
		derivative_term = -120;
	} 											
	derivative_term = derivative_term * kd;		//Multiplicado por la constante derivativa
	derivative_term = derivative_term >> 5;  	

	if(derivative_term > 120){					
		derivative_term = 120;
	}
	if(derivative_term < -120){
		derivative_term = -120;
	}
										
	Cn = en0 + integral_term + derivative_term;	// Suma total de terminos
	Cn = Cn * kp / 1024;						// Multiplicado por la constante proporcional y dividido por la resolucion del ADC (10bits => 0-1024 valores)
 
	if(Cn >= 1000)								
	{
		Cn = 1000;
	}
    if(Cn <= -1000)
	{
		Cn = -1000;
	}  
	if(Cn == 0){			
		//PWM valor de cero	
		DC1B1 = DC1B1 = 0;	
		CCPR1L = 0;
	}
	if(Cn > 0){					// El motor avanza hacia adelante y toma el valor de Cn
		P1M1 = 0;				
		temp = Cn;
		if(temp^0b00000001){
			DC1B0 = 1;
		}
		else{
			DC1B0 = 0;
		}
		if(temp^0b00000010){
			DC1B1 = 1;
		}
		else{
			DC1B1 = 0;
		}	
		CCPR1L = Cn >> 2;		
		off_set = off_set +1;	//Off_set es utilizado para ajustar el angulo del sistema del pendulo
		if(off_set > 55){		
			off_set = 55;
		}	
	}

	else {						// El motor avanza hacia atras y toma el valor de Cn
		P1M1 = 1;				
		temp_int = abs(Cn);		// Valor absoluto de Cn para manejar solo valores positivos
		temp = temp_int;		
		if(temp^0b00000001){
			DC1B0 = 1;
		}
		else{
			DC1B0 = 0;
		}
		if(temp^0b00000010){
			DC1B1 = 1;
		}
		else{
			DC1B1 = 0;
		}	
		CCPR1L = temp_int >> 2;	
		off_set = off_set -1;
		if(off_set < -55){
			off_set = -55;
		}
	}
	en3 = en2;		// Valores del Error
	en2 = en1;
	en1 = en0;
	en0 = 0;
	//Termina PID, sale del ciclo en la funcion void y espera a que el Timer se desborde para continuar con una nueva lectura y un nuevo ciclo
	do_PID = 0;				

	RA4 = 0;				

	return;
}


//Funcion Interrupcion para el caso del desbordamiento del Timer0
void interrupt Isr()
{

	//T0IF = Timer0 se ha desbordado 
	//T0IE = activado la interrupcion del Timer0
	if(T0IF&&T0IE){
		TMR0 = 10;				// carga valor del timer
		T0IF = 0;				// limpia el bit y marca que Timer0 no ha sido desbordado
		RA4 = 1;

	  	temp_int = 0;
		temp_int = ADRESH << 8;	// Almacena el valor del ADC en el termino temp_int
		temp_int = temp_int + ADRESL - 512;
		en0 = temp_int + off_set/8;				
	
		do_PID = 1;				// le da un valor de 1 a la variable para entrar en un nuevo ciclo y obtener los valores de PID
		GODONE = 1;				// Empieza a leer ADC
	}
	else
	{
		PIR1 = 0;
		RAIF = 0;	//Ningun pin del puerto A ha cambiado de estado
		INTF = 0;	//no ha ocurrido una interrupcion en la entrada RA2
	}
	if(temp_int > 180){ 		//Si el erroe es demasiado grande
		
		DC1B0 = DC1B1 = 0;		// Detien PWM
		CCPR1L = 0;	
		en0 = en1 = en2 = en3 = off_set = 0; 		// Limpia valor de las constantes
		Cn = integral_term = derivative_term = SumE = RA4 = 0;
		do_PID = 0;				// Sale del ciclo
	}
	if(temp_int < -180){ 		//Si el error es demasiado grande (negativo)
		
		DC1B0 = DC1B1 = 0;		// Detiene PWM
		CCPR1L = 0;	
		en0 = en1 = en2 = en3 = off_set = 0; 		// Limpia valor de las constantes
		Cn = integral_term = derivative_term = SumE = RA4 = 0;
		do_PID = 0;				// Sale del ciclo
	}

}
