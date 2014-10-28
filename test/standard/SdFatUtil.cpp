#include "Marlin.h"

#ifdef SDSUPPORT
#include "SdFatUtil.h"





int SdFatUtil::FreeRam() {
  extern int __bss_end;
  extern int* __brkval;
  int free_memory;
  if (reinterpret_cast<int>(__brkval) == 0) {

    free_memory = reinterpret_cast<int>(&free_memory)
                  - reinterpret_cast<int>(&__bss_end);
  } else {

    free_memory = reinterpret_cast<int>(&free_memory)
                  - reinterpret_cast<int>(__brkval);
  }
  return free_memory;
}






void SdFatUtil::print_P( PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) MYSERIAL.write(c);
}






void SdFatUtil::println_P( PGM_P str) {
  print_P( str);
  MYSERIAL.println();
}





void SdFatUtil::SerialPrint_P(PGM_P str) {
  print_P(str);
}





void SdFatUtil::SerialPrintln_P(PGM_P str) {
  println_P( str);
}
#endif
