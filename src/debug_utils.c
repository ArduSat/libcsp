#include <stdio.h>
#include <csp/csp.h>
  
/* Prints the binary contents of packet to stderr */
void debug_inspect_packet(char * msg, csp_packet_t * packet) {
  int i,j;
  fprintf(stderr, "%s", msg);
  for (i = 0; i < packet->length; i++) {
  	if (i % 8 == 0) { fprintf(stderr, "\n"); }
  	for (j = 7; j >= 0; j--) {
  		fprintf(stderr, "%d", (packet->data[i] >> j) & 0x01);
  	}
  	fprintf(stderr, " ");
  }
  fprintf(stderr, "\n");
}