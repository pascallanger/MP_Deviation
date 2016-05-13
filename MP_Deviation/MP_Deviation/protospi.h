#ifndef _SPIPROTO_H_
#define _SPIPROTO_H_

#include "target.h"

#define PROTOSPI_pin_set(io) digitalWrite(io, HIGH)
#define PROTOSPI_pin_clear(io) digitalWrite(io, LOW)
#define PROTOSPI_xfer(byte) spi_transfer(byte)
#define PROTOSPI_read3wire() spi_read3wire()

#endif // _SPIPROTO_H_
