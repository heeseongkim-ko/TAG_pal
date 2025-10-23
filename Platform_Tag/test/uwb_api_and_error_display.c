
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#include "UWB_routines.h"
#include "dw3000_regs.h"
//************************************************************************simple api
/*
uint32_t dw3000_read_deviceID (void)
{
  uint32_t result = 0;
  result = dw3000_read_register(DEV_ID_ID, 0x00, DEV_ID_LEN);

  return result;
}
*/
/* write test
  uint32_t Indirect_pointer_A = dw3000_read_register(0x00, 0x1C, 4);
  printf("READ Indirect pointer A = 0x%08lX\n\r", Indirect_pointer_A); 

  dw3000_write_register(0x00, 0x1C, tx_msg, 4);
 
  uint32_t write_read_Indirect_pointer_A = dw3000_read_register(0x00, 0x1C, 4);
  printf("WRITE READ Indirect pointer A = 0x%08lX\n\r", write_read_Indirect_pointer_A);  
  */
//************************************************************************error display