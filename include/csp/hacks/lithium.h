#ifndef LEMUR_1_LITHIUM_HACK
#define LEMUR_1_LITHIUM_HACK

//Lithium on Lemur-1 doesn't decode first two bytes correctly.  Add two byte radio frame header to
//compensate.  :O
typedef struct {
    uint16_t header;
    uint8_t data[0];
} radio_frame_t;

#endif