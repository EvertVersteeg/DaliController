/*
 * debug.h
 *
 *  Created on: Aug 27, 2021
 *      Author: evers
 */

#ifndef SRC_DEBUG_H_
#define SRC_DEBUG_H_


int _write(int file, char *ptr, int len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  int i=0;
  for(i=0 ; i<len ; i++)
    ITM_SendChar((*ptr++));
  return len;
}


#endif /* SRC_DEBUG_H_ */
