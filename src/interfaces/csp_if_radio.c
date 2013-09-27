/*
  Cubesat Space Protocol - A small network-layer protocol designed for Cubesats
  Copyright (C) 2012 GomSpace ApS (http://www.gomspace.com)
  Copyright (C) 2012 AAUSAT3 Project (http://aausat3.space.aau.dk)

  Radio interface using astrodev radios.
  Copyright (C) 2013 Nanosatisfi (http://www.nanosatisfi.com)
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <csp/csp_platform.h>
#include <csp/csp_interface.h>
#include <csp/interfaces/csp_if_radio.h>
#include <csp/arch/csp_malloc.h>

#define RADIO_MODE_NOT_STARTED 0
#define RADIO_MODE_STARTED 1
#define RADIO_MODE_ESCAPED 2
#define RADIO_MODE_ENDED 3

#define FEND  0xC0
#define FESC  0xDB
#define TFEND 0xDC
#define TFESC 0xDD

#define TNC_DATA			0x00

static csp_radio_putstr_f radio_tx = NULL;

static int csp_radio_tx (csp_packet_t * packet, uint32_t timeout)
{
    int ret = CSP_ERR_NONE;
    int txbufin = 0;
    uint8_t *txbuf = csp_malloc(packet->length + 30);

    if (txbuf == NULL)
        return CSP_ERR_NOMEM;

    /* Save the outgoing id in the buffer */
    packet->id.ext = csp_hton32(packet->id.ext);
    packet->length += sizeof(packet->id.ext);

    txbuf[txbufin++] = FEND;
    txbuf[txbufin++] = TNC_DATA;

    for (unsigned int i = 0; i < packet->length; i++) {
        if (((unsigned char *) &packet->id.ext)[i] == FEND) {
            ((unsigned char *) &packet->id.ext)[i] = TFEND;
            txbuf[txbufin++] = FESC;
        } else if (((unsigned char *) &packet->id.ext)[i] == FESC) {
            ((unsigned char *) &packet->id.ext)[i] = TFESC;
            txbuf[txbufin++] = FESC;
        }
        txbuf[txbufin++] = ((unsigned char *) &packet->id.ext)[i];

    }
    txbuf[txbufin++] = FEND;
    csp_buffer_free(packet);

    if (radio_tx(txbuf, txbufin) != 0) {
        ret = CSP_ERR_TIMEDOUT;
    }

    csp_free(txbuf);

    return ret;
}

void csp_radio_rx (uint8_t *buf, int len)
{
    static csp_packet_t * packet;
    static int length = 0;
    static volatile unsigned char *cbuf;
    static int mode = RADIO_MODE_NOT_STARTED;
    static int first = 1;

    while (len) {
        switch (mode) {
        case RADIO_MODE_NOT_STARTED:
            if (*buf == FEND) {
                packet = csp_buffer_get(csp_if_radio.mtu);

                if (packet != NULL) {
                    cbuf = (unsigned char *)&packet->id.ext;
                } else {
                    cbuf = NULL;
                }

                mode = RADIO_MODE_STARTED;
                first = 1;
            }
            break;
        case RADIO_MODE_STARTED:
            if (*buf == FESC)
                mode = RADIO_MODE_ESCAPED;
            else if (*buf == FEND) {
                if (length > 0) {
                    mode = RADIO_MODE_ENDED;
                }
            } else {
                if (cbuf != NULL)
                    *cbuf = *buf;
                if (first) {
                    first = 0;
                    break;
                }
                if (cbuf != NULL)
                    cbuf++;
                length++;
            }
            break;
        case RADIO_MODE_ESCAPED:
            if (*buf == TFESC) {
                if (cbuf != NULL)
                    *cbuf++ = FESC;
                length++;
            } else if (*buf == TFEND) {
                if (cbuf != NULL)
                    *cbuf++ = FEND;
                length++;
            }
            mode = RADIO_MODE_STARTED;
            break;
        }

        len--;
        buf++;

        if (mode == RADIO_MODE_ENDED) {
            if (packet == NULL) {
                mode = RADIO_MODE_NOT_STARTED;
                length = 0;
                continue;
            }

            packet->length = length;
            csp_if_radio.frame++;

            if (packet->length >= CSP_HEADER_LENGTH && packet->length <= csp_if_radio.mtu + CSP_HEADER_LENGTH) {
                /* Strip the CSP header off the length field before converting to CSP packet */
                packet->length -= CSP_HEADER_LENGTH;

                /* Convert the packet from network to host order */
                packet->id.ext = csp_ntoh32(packet->id.ext);

                /* Send back into CSP, notice calling from task so last argument must be NULL! */
                csp_new_packet(packet, &csp_if_radio, NULL);
            } else {
                csp_log_warn("Weird radio frame received! Size %u\r\n", packet->length);
                csp_buffer_free(packet);
            }

            mode = RADIO_MODE_NOT_STARTED;
            length = 0;
        }
    }
}

int csp_radio_init (csp_radio_putstr_f radio_putstr_f)
{
    radio_tx = radio_putstr_f;

    /* Register interface */
    csp_route_add_if(&csp_if_radio);

    return CSP_ERR_NONE;
}

/** Interface definition */
csp_iface_t csp_if_radio = {
    .name = "RADIO",
    .nexthop = csp_radio_tx,
    .mtu = 255,
};
