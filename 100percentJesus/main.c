/*
 * 100percentJesus.c
 *
 * Created: 20/06/2018 19:23:04
 * Author : Mateus Sousa
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define USART_BAUDRATE FOSC/16/BAUD-1

#define VEL_NORMAL  0x1E
#define VEL_CURVA 0xff
#define PARE    0x00
#define cod_PARE  0x01
#define cod_NORMAL  0x02
#define cod_RE    0x03

char flag_partida = 0x00, motor_state = 0x02, sensor_state = 0x00;

void setup_pin(){
	// Configura o RX e TX
	DDRD = 0b11111110;
	PORTD = 0x00;
	
	// Configura a entrada dos sensores digitais
	DDRB = 0x00;
	PORTB = 0b00110011;
	
	//Enable Pin Change INT[1]
	PCICR = 0b00000001;
	
	/*Pin Change Interrupt MASK*/
	/*bits 23 to 16*/
	PCMSK2 = 0x00;
	/*bits 14 to 08*//*Doesnt exist PCINT15*/
	PCMSK1 = 0x00;
	/*bits 07 to 00*/
	PCMSK0 = 0b00110011;
	
}

void setup_usart(unsigned int ubrr){
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
	
}

void setup_pwm_timer(){
	
	//  TIMER 0 Config, PWM fast
	TCCR0A = 0b11110011;
	TCCR0B = 0x04;
	TIMSK0 = 0x00;
	OCR0A = 0xFF;
	OCR0B = 0xFF;
	TCNT0 = 0x00;
	
}

unsigned char USART_Receive( void ){
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) );
	/* Get and return received data from buffer */
	return UDR0;
}

ISR (PCINT0_vect){
	
	if((~PINB) & 1){
		motor_state = cod_PARE;
	}else if((~PINB) & 2){
		motor_state = cod_PARE;
	}else{
		motor_state = cod_NORMAL;
	}
	
	if(((~PINB) & (1 << PINB4)) && ((~PINB) & (1 << PINB5))){
		motor_state = cod_RE;
	}else if((~PINB) & (1 << PINB4)){
		sensor_state = 0x02;
	}else if((~PINB) & (1 << PINB5)){
		sensor_state = 0x01;
	}else{
		sensor_state = 0x00;
	}
	
}

void configuration_motor(){
	PORTD &= ~(1 << 2);
	PORTD &= ~(1 << 3);
	OCR0A = VEL_NORMAL;
	OCR0B = VEL_NORMAL;
}

void machine_leitura(){
	if(motor_state == cod_NORMAL){
		switch(sensor_state){
			case 0x00: // Go straight
				OCR0A = VEL_NORMAL;
				OCR0B = VEL_NORMAL;
			break;
			case 0x01: // Curve to the left
				OCR0A = VEL_CURVA;
				OCR0B = VEL_NORMAL;
			break;
			case 0x02: // Curve to the right
				OCR0A = VEL_NORMAL;
				OCR0B = VEL_CURVA;
			break;
			default:
			break;
		}
	} else if(motor_state == cod_RE){
		switch(sensor_state){
			case 0x01: // Curve to the left
				PORTD |= (1 << 2);
				PORTD &= ~(1 << 3);
				OCR0A = VEL_CURVA;
				OCR0B = VEL_NORMAL;
			break;
			case 0x02: // Curve to the right
				PORTD &= ~(1 << 2);
				PORTD |= (1 << 3);
				OCR0A = VEL_NORMAL;
				OCR0B = VEL_CURVA;
			break;
			default:
			break;
		}
		while((((~PINB) & (1 << PINB4)) && ((~PINB) & (1 << PINB5))));
			motor_state = cod_NORMAL;
			PORTD &= ~(1 << 2);
			PORTD &= ~(1 << 3);
	}else{
		OCR0A = PARE;
		OCR0B = PARE;
		while(((~PINB) & 1));
		motor_state = cod_NORMAL;
	}
}

int main(void){
	
	setup_pin();
	setup_usart(USART_BAUDRATE);
	
	while(USART_Receive() != '1');
	
	setup_pwm_timer();
	configuration_motor();
	sei();

	while (1){
		machine_leitura();
	}
}

