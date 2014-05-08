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

// To provide information to beacon for some of its algorithms.
// Note: this gets reset externally.
csp_id_t latest_csp_transfer_id[NUM_ASTRODEV_MODULES] = {{0x0},{0x0}};

static int csp_astrodev_tx (csp_iface_t *interface,
                            csp_packet_t *packet, uint32_t timeout) {
    int ret = CSP_ERR_NONE;

    int txbufin = packet->length + CSP_HEADER_LENGTH;

    uint8_t *txbuf = csp_malloc(txbufin);
    csp_astrodev_handle_t *driver = interface->driver;

    if (txbuf == NULL)
        return CSP_ERR_NOMEM;

    if (driver->module < NUM_ASTRODEV_MODULES)
        latest_csp_transfer_id[driver->module] = packet->id;

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

    csp_astrodev_handle_t* handle =
        (csp_astrodev_handle_t*) (interface->driver);
    ax25_header_t radio_header;

    if (handle->use_ax25_header) {

        if (len < (int)sizeof(ax25_header_t) + CSP_HEADER_LENGTH) {
            csp_log_warn("Length less than minimum expected! Size %u,"
                         " expected %u (with ax25); dropping message\r\n", len,
                         (int)sizeof(ax25_header_t) + CSP_HEADER_LENGTH);
            return;
        }

        memcpy(&radio_header, buf, sizeof(ax25_header_t));

        /* Strip off the AX.25 header. */
        buf += sizeof(ax25_header_t);

        /* Remove the header size */
        len -= sizeof(ax25_header_t);

    } else {
        if (len < CSP_HEADER_LENGTH) {
            csp_log_warn("Length less than minimum expected! Size %u,"
                         " expected %u; dropping message\r\n", len,
                         CSP_HEADER_LENGTH);
            return;
        }
    }

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

            if (handle->module < NUM_ASTRODEV_MODULES)
                latest_csp_transfer_id[handle->module] = packet->id;
        }
        else {
            csp_log_warn("Packet length %u did not meed specifications. Must be >="
                         "%u and <= %u.  dropping message\r\n",
                         packet->length, CSP_HEADER_LENGTH,
                         interface->mtu + CSP_HEADER_LENGTH);
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
