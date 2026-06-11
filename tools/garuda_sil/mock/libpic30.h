/* mock libpic30.h — delay macros become no-ops on the host. */
#ifndef MOCK_LIBPIC30_H
#define MOCK_LIBPIC30_H
#define __delay_us(x) do { (void)(x); } while (0)
#define __delay_ms(x) do { (void)(x); } while (0)
#define __delay32(x)  do { (void)(x); } while (0)
#endif
