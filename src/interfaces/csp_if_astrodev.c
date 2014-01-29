/**
 * CSP Radio interface using astrodev radios.
 *
 * @author Francisco Soto <francisco@nanosatisfi.com>
 *
 * Copyright (C) 2013 Nanosatisfi (http://www.nanosatisfi.com)
 */

#include <inttypes.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <csp/csp.h>
#include <csp/csp_endian.h>
#include <csp/csp_platform.h>
#include <csp/csp_interface.h>
#include <csp/interfaces/csp_if_astrodev.h>
#include <csp/arch/csp_malloc.h>

#define ASTRODEV_PACKET_SIZE 255

typedef struct __attribute__((__packed__)) {
    uint8_t dst_callsign[6];
    uint8_t dst_ssid;
    uint8_t src_callsign[6];
    uint8_t src_ssid;
    uint8_t digi_callsign[6];
    uint8_t digi_ssid;
    uint8_t control;
    uint8_t pid;
} ax25_header_t;

static int csp_astrodev_tx (csp_iface_t *interface,
                            csp_packet_t *packet, uint32_t timeout) {
    int ret = CSP_ERR_NONE;
    int txbufin = packet->length + CSP_HEADER_LENGTH;
    uint8_t *txbuf = csp_malloc(txbufin);
    csp_astrodev_handle_t *driver = interface->driver;

    if (txbuf == NULL)
        return CSP_ERR_NOMEM;

    /* Save the outgoing id in the buffer */
    packet->id.ext = csp_hton32(packet->id.ext);

    memcpy(txbuf, &packet->id.ext, txbufin);

    /* The packet goes straigth to the radio. */
    if (driver->radio_tx(txbuf, txbufin) != 0) {
        interface->tx_error++;
        ret = CSP_ERR_TIMEDOUT;
    }
    else {
        csp_buffer_free(packet);
    }

    csp_free(txbuf);

    return ret;
}

void csp_astrodev_rx (csp_iface_t *interface,
                      uint8_t *buf, int len, void *xTaskWoken) {
    csp_packet_t *packet;
    ax25_header_t radio_header;

    if (len < (int)sizeof(ax25_header_t) + CSP_HEADER_LENGTH) {
        csp_log_warn("Weird radio frame received! Size %u\r\n", len);
    }

    memcpy(&radio_header, buf, sizeof(ax25_header_t));

    /* Strip off the AX.25 header. */
    buf += sizeof(ax25_header_t);

    /* Remove the header size */
    len -= sizeof(ax25_header_t);

    /* Remove trailing two bytes */
    len -= (sizeof(uint8_t) * 2);

    packet = csp_buffer_get(interface->mtu);

    if (packet != NULL) {
        memcpy(&packet->id.ext, buf, len);

        packet->length = len;

        if (packet->length >= CSP_HEADER_LENGTH &&
            packet->length <= interface->mtu + CSP_HEADER_LENGTH) {

            /* Strip the CSP header off the length field before converting to CSP packet */
            packet->length -= CSP_HEADER_LENGTH;

            /* Convert the packet from network to host order */
            packet->id.ext = csp_ntoh32(packet->id.ext);

            csp_new_packet(packet, interface, xTaskWoken);
        }
        else {
            csp_log_warn("Weird radio frame received! Size %u\r\n", packet->length);
            interface->frame++;
            csp_buffer_free(packet);
        }
    }
    else {
        interface->frame++;
    }
}

void csp_astrodev_init (csp_iface_t *csp_iface, csp_astrodev_handle_t *csp_astrodev_handle,
                       csp_astrodev_putstr_f astrodev_putstr, const char *name) {

    csp_astrodev_handle->radio_tx = astrodev_putstr;

    csp_iface->driver = csp_astrodev_handle;
    csp_iface->mtu = ASTRODEV_PACKET_SIZE - CSP_HEADER_LENGTH;
    csp_iface->nexthop = csp_astrodev_tx;
    csp_iface->name = name;

    /* Register interface */
    csp_route_add_if(csp_iface);
}
