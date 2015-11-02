#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ftdi.h>
#include <libusb.h>
#include <stdarg.h>

#include "spi.h"

#define SPI_CLOCK_FREQ 1000000

#define LOW	0
#define HIGH	1

    // pins defs
#define PIN_FCSN	(1 << 3)	/* CS */	// nRF2LE1 Chip select
#define PIN_FMISO	(1 << 2)	/* MISO */
#define PIN_FMOSI	(1 << 1)	/* MOSI */
#define PIN_FSCK	(1 << 0)	/* SCK */
#define PIN_RESET	(1 << 4)	/* JTAG_RST   */	// nRF24LE1 Reset
#define PIN_PROG	(1 << 6)	/* JTAG_DBGRQ */	// nRF24LE1 Program
#define PINS_OUT	(PIN_PROG|PIN_RESET|PIN_FCSN|PIN_FSCK|PIN_FMOSI)

#define BYTES_PER_BIT 3
#define FTDI_READ_FIFO_SIZE 384
    //#define delay_ns(t)
#define delay_us(t) usleep(t)
#define delay_ms(t) usleep(t * 1000)
#define delay_s(t) sleep(t)

static void spi_send_buf( const unsigned char *buf, int size );

static struct ftdi_context *ftdi;
static uint8_t pin_state = 0;

static void digitalWrite(unsigned char pin, int value)
{
    if (value)
        pin_state |= pin;
    else
        pin_state &= ~pin;
    
    unsigned char buf[9];
    int i = 0;
    
    buf[i++] = SET_BITS_LOW;
    buf[i++] = pin_state;
    buf[i++] = PINS_OUT;
    
    spi_send_buf(buf, i);
}

static void prog_begin()
{
    digitalWrite(PIN_PROG, HIGH);
    digitalWrite(PIN_RESET, LOW);
    delay_us(1); // we need only 0.2 us
    digitalWrite(PIN_RESET, HIGH);
    delay_ms(2); // we need to wait at least 1.5 ms before send commands
}

static void prog_end()
{
    digitalWrite(PIN_PROG, LOW);
    digitalWrite(PIN_RESET, LOW);
    delay_us(1);
    digitalWrite(PIN_RESET, HIGH);
}

void spi_fail( int code, const char *fmt, ... )
{
    va_list args;
    
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    
    fprintf(stderr, ": %s\n", ftdi_get_error_string(ftdi) );
    
    ftdi_free( ftdi );
    
    exit( code );
}

static void spi_send_buf( const unsigned char *buf, int size )
{
    if( ftdi_write_data( ftdi, (unsigned char *) buf, size ) < 0 )
    {
        spi_fail( -12, "ftdi_write_data (size=%d) failed", size );
    }
}

static void spi_get_buf(unsigned char *buf, int size)
{
    int r;
    
    while (size > 0)
    {
        r = ftdi_read_data(ftdi, buf, size);
        
        if (r < 0) {
            spi_fail( -13, "ftdi_read_data failed" );
        }
        if (r == 0) {
            spi_fail( -14, "ftdi_read_data got 0 bytes" );
        }
        
        
        buf += r;
        size -= r;
    }
}

int spi_begin(uint8_t bus, uint8_t port)
{
    int ret;
    unsigned char buf[512];
    int clock_5x = 1;
    uint32_t mpsse_clk = 0;

    ftdi = ftdi_new();
    if (ftdi == 0) {
        fprintf(stderr, "ftdi_new failed\n");
        return -1;
    }
    
    ftdi_set_interface( ftdi, INTERFACE_A );
    
    if (bus > 0) {
        struct ftdi_device_list *list = NULL;
        struct ftdi_device_list *p;
        
        ret = ftdi_usb_find_all(ftdi, &list, 0, 0);
        if (ret < 0) {
            spi_fail(-2, "unable to list devices: %d", ret);
        }
        
        p = list;
        while (p) {
            if (bus == libusb_get_bus_number(p->dev) &&
                port == libusb_get_port_number(p->dev)) {
                ret = ftdi_usb_open_dev(ftdi, p->dev);
                break;
            }
            p = p->next;
        }
        
        ftdi_list_free(&list);
        
        if (!p) {
            fprintf(stderr, "dev on bus %i and port %i not found\n",
                    bus, port);
            ftdi_free(ftdi);
            return -3;
        }
    } else
        ret = ftdi_usb_open(ftdi, 0x0403, 0x8a98); // TUMPA
    
    if (ret < 0 && ret != -5) {
        spi_fail( -4, "unable to open ftdi device: %d", ret );
    }
    
    if (ftdi_usb_reset(ftdi) < 0) {
        spi_fail( -6, "reset failed");
    }
    
    
    if (ftdi_set_latency_timer(ftdi, 1) < 0) {
        spi_fail( -7, "Unable to set latency timer" );
    }
    
    if (ftdi_write_data_set_chunksize(ftdi, 256)) {
        spi_fail( -8, "Unable to set chunk size" );
    }
    
    if (ftdi_set_bitmode(ftdi, 0x00, BITMODE_RESET) < 0) {
        spi_fail( -9, "Unable to reset bitmode" );
    }
    
    
    if (ftdi_set_bitmode(ftdi, PINS_OUT, BITMODE_MPSSE) < 0) {
        spi_fail( -10, "Unable to set bitmode to SPI" );
    }
    
    if (ftdi_usb_purge_buffers( ftdi ) < 0) {
        spi_fail( -11, "purge buffers failed" );
    }
    
    
    if (clock_5x) {
        printf("Enabling 5x clock divisor\n");
        buf[0] = EN_DIV_5;          /* Disable divide-by-5. */
        
        spi_send_buf( buf, 1 );
        
        printf("Disabling 5x clock divisor\n");
        buf[0] = DIS_DIV_5;          /* Disable divide-by-5. */
        
        spi_send_buf( buf, 1 );
        
        mpsse_clk = 60000000;
    } else {
        mpsse_clk = 12000000;
    }
    
    
    printf( "mpsse_clk = %u\n", mpsse_clk );
    
    uint16_t divisor = ((mpsse_clk / SPI_CLOCK_FREQ ) / 2) - 1;
    
    printf("divisor = %u\n", divisor);
    
    buf[0] = TCK_DIVISOR;          /* command "set divisor" */
    buf[1] = divisor & 0xff;
    buf[2] = divisor >> 8;
    
    uint32_t freq = (mpsse_clk / ((1 + divisor) * 2));
    
    printf( "freq = %u\n", freq );
    
    spi_send_buf( buf, 3 );
    
    /* Disconnect TDI/DO to TDO/DI for loopback. */
    
    buf[0] = LOOPBACK_END;
    
    spi_send_buf( buf, 1 );
    
    digitalWrite(PIN_PROG, LOW);
    digitalWrite(PIN_FSCK, LOW);
    digitalWrite(PIN_FCSN, HIGH);
    digitalWrite(PIN_FMOSI, LOW);
    digitalWrite(PIN_RESET, HIGH);
    
    prog_begin();
    
    return 0;
}

void spi_end()
{
    prog_end();
    
//    ftdi_disable_bitbang(ftdi);
    ftdi_free(ftdi);
}

int spi_transfer( uint8_t *data, size_t len )
{
    uint8_t *end = data + len;
    
    digitalWrite(PIN_FCSN, LOW);
    
    while( data < end )
    {
        uint8_t buf[ 256 ];
        
        int i = 0;
        int chunk = end - data;
        
        if(chunk > (sizeof(buf) - 3) ) { chunk = sizeof(buf) - 3; }
        
        buf[ i++ ] = MPSSE_DO_READ | MPSSE_DO_WRITE | MPSSE_WRITE_NEG;
        buf[ i++ ] = chunk - 1;
        buf[ i++ ] = 0;
        
        memcpy( buf + i, data, chunk );
        
        
        spi_send_buf( buf, i + chunk );
        
        spi_get_buf( data, chunk );
        
        data += chunk;
    }
    
    digitalWrite(PIN_FCSN, HIGH);
    
    return len;
}


