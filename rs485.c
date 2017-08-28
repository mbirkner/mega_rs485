#include "rs485.h"
#include "tiny485_syscfg.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#define BAUD 57600
#define USE_U2X 1
#define UBRR_CALC(baud,f_osc,u2x) (((uint32_t)(f_osc)/(8*(2-(u2x))*(uint32_t)(baud)))-1)
#define BACKLIGHT PC0
#define LED1 PD7

/* inquiry:
     <SOH> <addr> <payload> <EOT>
   answer:
     <STX> <payload> <ETX>
*/

enum ASCII_CHARS {
	ASCII_SOH = '\001',
	ASCII_STX = '\002',
	ASCII_ETX = '\003',
	ASCII_EOT = '\004',
	ASCII_ESC = '\033',
	ASCII_LOW = '\040'  /* escape everything < ASCII_LOW */
};

/*
 *                   +-------+
 *                   |       |
 *                   v       | data(ignored)
 *   RS485_RX_SKIP_REPLY ----+
 *    |      ^
 *    | ETX  | STX
 *    |      |
 *    +--> IDLE  <-------------------+<--------------------+
 *           |                       |      +-----+        |
 *           | SOH               EOT |      |     |        |
 *           v                       |      v     | data   |
 *        RS485_RX_ADDR  ---> RS485_RX_PAYLOAD ---+ (store)|
 *           |           addr                              |
 *           |                                        EOT  |
 *           +--------------> RS485_RX_SKIP_PAYLOAD -------+
 *                    != addr      |   ^
 *                                 |   | data(ignored)
 *                                 +---+
 *
 * Any STX received jumps directly to SKIP_REPLY.
 * Any SOH received jumps directly to RX_ADDR.
 * Any EOT, ETX, or undefined character < 0x20 jumps directly to IDLE.
 * To allow data or addr < 0x20 to be transmitted, it has to be escaped
 * by ESC (0x13, dec 27, \033) and the character be XORed with 0x20.
 * e.g. <0x01> is transmitted as <0x13 0x21>.
 * 
 */


#define RS485_ESC_FLAG 0x80  /* set this bit in flags if ESC detected */
enum rs485_rx_state {
	RS485_RX_IDLE,
	RS485_RX_ADDR,
	RS485_RX_PAYLOAD,
	RS485_RX_SKIP_PAYLOAD,
	RS485_RX_SKIP_REPLY,
};

/* driver enable / receiver not-enable connected to port PD2 */
#define PORT_DEnRE  PORTD
#define BIT_DEnRE   (1<<PD2)
#define DDR_DEnRE   DDRD

unsigned char rs485_rxbuf[RS485_MAX_MSGSIZE];
#define RS485_RXBUFP_BUSY 0x80
static unsigned char rs485_rxbufp;

enum rs485_rx_state rs485_rx_state;

/* receive complete */
ISR(USART_RX_vect)
{
	enum rs485_rx_state state = rs485_rx_state;
	unsigned char p = rs485_rxbufp;
	unsigned char c;

	PORTD ^= (1<<LED1);
	c = UDR0;

	switch (c) {
	case ASCII_SOH:
		/* state != RS485_RX_IDLE error, ignored */
		state = RS485_RX_ADDR;
		break;
	case ASCII_EOT:
		/* state != RS485_RX_PAYLOAD && state != RS485_RX_SKIP_PAYLOAD
		   would be an error, but is ignored */
		/* only mark message as received when it was addressed to us */
		if (state == RS485_RX_PAYLOAD) { /* mark messages as received */
			p |= RS485_RXBUFP_BUSY;
		}
		state = RS485_RX_IDLE;
		break;
	case ASCII_STX:
		/* state != RS485_RX_IDLE would be an error, ignored */
		state = RS485_RX_SKIP_REPLY;
		break;
	case ASCII_ETX:
		/* state != RS485_RX_SKIP_REPLY would be an error, ignored */
		state = RS485_RX_IDLE;
		break;
	case ASCII_ESC:
		state |= RS485_ESC_FLAG;
		break;
	default:
		if (state == RS485_RX_SKIP_REPLY || state == RS485_RX_SKIP_PAYLOAD)
			goto out;

		/* received a non-control character, only valid during address */
		/* or payload, check if we had a ESC just before this character */
		if (state & RS485_ESC_FLAG) {
			c ^= ASCII_LOW;
			state &= ~RS485_ESC_FLAG;
		}
		/* handle address, payload, skip_payload or skip_reply */
		if (state == RS485_RX_ADDR) {
			/* our address, and we are not still busy */
			if (c == tiny485_syscfg.nodeaddr && !(p & RS485_RXBUFP_BUSY))
			{
				state = RS485_RX_PAYLOAD;
				p = 0;
			} else {
				state = RS485_RX_SKIP_PAYLOAD;
			}
		} else if (state == RS485_RX_PAYLOAD) {
			if (p >= RS485_MAX_MSGSIZE) {
				/* flag error */
				state = RS485_RX_IDLE;
			} else {
				rs485_rxbuf[p++] = c;
			}
		} else {
			/* received non-character while not waiting for
			   address or payload */
			state = RS485_RX_IDLE;
		}
	}
out:
	rs485_rx_state = state;
	rs485_rxbufp = p;
}

unsigned char
rs485_poll() {
	unsigned char p;

	cli();
	p = rs485_rxbufp;
	if (p & RS485_RXBUFP_BUSY)
		p &= ~RS485_RXBUFP_BUSY;
	else
		p = RS485_POLL_NOMSG;
	sei();
	return p;
}

void
rs485_rxok() {
	unsigned char p;
	cli();
	p = rs485_rxbufp;
	if (p & RS485_RXBUFP_BUSY)
		rs485_rxbufp = 0;
	PORTD &= ~(1<<LED1);	
	sei();
}

enum rs485_tx_state {
	RS485_TX_STX,
	RS485_TX_PAYLOAD,
	RS485_TX_ESC,
	RS485_TX_ETX,
	RS485_TX_END
};

unsigned char rs485_txbuf[RS485_MAX_MSGSIZE];
static enum rs485_tx_state rs485_tx_state;
static unsigned char rs485_txbuflen;
static unsigned char rs485_txbufp;

/* transmit complete */
ISR(USART_TX_vect)
{	
	enum rs485_tx_state state = rs485_tx_state;
	if (state == RS485_TX_END)
		PORT_DEnRE &= ~BIT_DEnRE;     /* disable RS485 TX */
		
	PORTC &= ~(1<<BACKLIGHT);	
}

/* data register empty */
ISR(USART_UDRE_vect)
{
	enum rs485_tx_state state = rs485_tx_state;
	unsigned char l = rs485_txbuflen;
	unsigned char p = rs485_txbufp;
	unsigned char c = 0xba;
	PORTC ^= (1<<BACKLIGHT);
	switch (state) {
	case RS485_TX_STX:
		PORT_DEnRE |= BIT_DEnRE;
		c = ASCII_STX;
		p = 0;
		if (l)
			state = RS485_TX_PAYLOAD;
		else
			state = RS485_TX_ETX;
		break;
	case RS485_TX_PAYLOAD:
	case RS485_TX_ESC:
		c = rs485_txbuf[p];
		if (state == RS485_TX_ESC) {
			state = RS485_TX_PAYLOAD;
			c ^= ASCII_LOW;
		} else {
			if (c <= ASCII_EOT || c == ASCII_ESC) {
				state = RS485_TX_ESC;
				c = ASCII_ESC;
				goto write_udr;
			}
		}
		p++;
		if (p >= l)
			state = RS485_TX_ETX;
		break;
	case RS485_TX_ETX:
		c = ASCII_ETX;
		state = RS485_TX_END;
		break;
	case RS485_TX_END:
		l = 0;  /* mark buffer as sent */
		UCSR0B &= ~_BV(UDRIE0); /* disable data register empty interrupt */
		goto no_write_udr;
	}
write_udr:
	UDR0 = c;

no_write_udr:
	rs485_tx_state = state;
	rs485_txbufp = p;
	rs485_txbuflen = l;
}

void
rs485_start_tx(unsigned char len)
{
	cli();
	rs485_tx_state = RS485_TX_STX; /* set to start tx */
	rs485_txbuflen = len;
	UCSR0B |=  _BV(UDRIE0); /* enable data register empty interrupt */
	sei();
}

void
rs485_init()
{
	uint16_t ubrr = UBRR_CALC(BAUD,F_CPU,USE_U2X);
 	DDRC |= (1<<BACKLIGHT);
 	DDRD |= (1<<LED1);
	/* PD0 = RxD, PD1 = TxD */
	DDRD  &= ~_BV(0);  /* PD0 = input */
	PORTD |=  _BV(0);  /* weak pullup on RxD */
	//DDRD |= (1<<PD7);
	//PORTD &= ~(1<<PD7);
	/* PA1 = RX485 Tx Enable */
	DDR_DEnRE  |=  BIT_DEnRE;
	PORT_DEnRE &= ~BIT_DEnRE;

	UBRR0L =  ubrr       & 0x00ff;
	UBRR0H = (ubrr >> 8) & 0x00ff;
	UCSR0A = _BV(TXC0) | (USE_U2X?_BV(U2X0):0); /* clear tx complete interrupt, just in case */
	UCSR0B = 0;
	UCSR0C = _BV(UCSZ01)|_BV(UCSZ00); /* 8 bit */
	UCSR0B |= _BV(TXCIE0)|_BV(RXCIE0)|_BV(RXEN0)|_BV(TXEN0);
}
