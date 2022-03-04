/*
 * other.h
 *
 *  Created on: 2 мар. 2022 г.
 *      Author: jet
 */

#ifndef OTHER_H_
#define OTHER_H_

#include <stdint.h>
#include <ctype.h>

int ip4_aton(const char *cp, uint32_t *addr);
void ip4Parse( char * str );

#endif /* OTHER_H_ */
