

#include "Marlin.h"
#include "MarlinSerial.h"

#ifndef AT90USB


#if defined(UBRRH) || defined(UBRR0H) || defined(UBRR1H) || defined(UBRR2H) || defined(UBRR3H)

#if UART_PRESENT(SERIAL_PORT)
  ring_buffer rx_buffer  =  { { 0 }, 0, 0 };
#endif

FORCE_INLINE void store_char(unsigned char c)
{
  int i = (unsigned int)(rx_buffer.head + 1) % RX_BUFFER_SIZE;

  
  
  
  
  if (i != rx_buffer.tail) {
    rx_buffer.buffer[rx_buffer.head] = c;
    rx_buffer.head = i;
  }
}



#if defined(M_USARTx_RX_vect)
  
  
  SIGNAL(M_USARTx_RX_vect)
  {
    unsigned char c  =  M_UDRx;
    store_char(c);
  }
#endif



MarlinSerial::MarlinSerial()
{

}



void MarlinSerial::begin(long baud)
{
  uint16_t baud_setting;
  bool useU2X = true;

#if F_CPU == 16000000UL && SERIAL_PORT == 0
  
  
  
  if (baud == 57600) {
    useU2X = false;
  }
#endif
  
  if (useU2X) {
    M_UCSRxA = 1 << M_U2Xx;
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  } else {
    M_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  
  M_UBRRxH = baud_setting >> 8;
  M_UBRRxL = baud_setting;

  sbi(M_UCSRxB, M_RXENx);
  sbi(M_UCSRxB, M_TXENx);
  sbi(M_UCSRxB, M_RXCIEx);
}

void MarlinSerial::end()
{
  cbi(M_UCSRxB, M_RXENx);
  cbi(M_UCSRxB, M_TXENx);
  cbi(M_UCSRxB, M_RXCIEx);  
}



int MarlinSerial::peek(void)
{
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    return rx_buffer.buffer[rx_buffer.tail];
  }
}

int MarlinSerial::read(void)
{
  
  if (rx_buffer.head == rx_buffer.tail) {
    return -1;
  } else {
    unsigned char c = rx_buffer.buffer[rx_buffer.tail];
    rx_buffer.tail = (unsigned int)(rx_buffer.tail + 1) % RX_BUFFER_SIZE;
    return c;
  }
}

void MarlinSerial::flush()
{









  rx_buffer.head = rx_buffer.tail;
}









void MarlinSerial::print(char c, int base)
{
  print((long) c, base);
}

void MarlinSerial::print(unsigned char b, int base)
{
  print((unsigned long) b, base);
}

void MarlinSerial::print(int n, int base)
{
  print((long) n, base);
}

void MarlinSerial::print(unsigned int n, int base)
{
  print((unsigned long) n, base);
}

void MarlinSerial::print(long n, int base)
{
  if (base == 0) {
    write(n);
  } else if (base == 10) {
    if (n < 0) {
      print('-');
      n = -n;
    }
    printNumber(n, 10);
  } else {
    printNumber(n, base);
  }
}

void MarlinSerial::print(unsigned long n, int base)
{
  if (base == 0) write(n);
  else printNumber(n, base);
}

void MarlinSerial::print(double n, int digits)
{
  printFloat(n, digits);
}

void MarlinSerial::println(void)
{
  print('\r');
  print('\n');  
}

void MarlinSerial::println(const String &s)
{
  print(s);
  println();
}

void MarlinSerial::println(const char c[])
{
  print(c);
  println();
}

void MarlinSerial::println(char c, int base)
{
  print(c, base);
  println();
}

void MarlinSerial::println(unsigned char b, int base)
{
  print(b, base);
  println();
}

void MarlinSerial::println(int n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned int n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(long n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(unsigned long n, int base)
{
  print(n, base);
  println();
}

void MarlinSerial::println(double n, int digits)
{
  print(n, digits);
  println();
}



void MarlinSerial::printNumber(unsigned long n, uint8_t base)
{
  unsigned char buf[8 * sizeof(long)]; 
  unsigned long i = 0;

  if (n == 0) {
    print('0');
    return;
  } 

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

void MarlinSerial::printFloat(double number, uint8_t digits) 
{ 
  
  if (number < 0.0)
  {
     print('-');
     number = -number;
  }

  
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  print(int_part);

  
  if (digits > 0)
    print("."); 

  
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    print(toPrint);
    remainder -= toPrint; 
  } 
}



MarlinSerial MSerial;

#endif 
#endif 
