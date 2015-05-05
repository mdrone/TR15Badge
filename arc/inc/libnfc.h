#ifndef __LIBNFC_H__
#define __LIBNFC_H__

#define PN532_FIFO_SIZE 64
#define PN532_MAX_PAYLAOADSIZE 264
#define PN532_MAX_PACKET_SIZE (PN532_MAX_PAYLAOADSIZE+11)

#define CLONE   0
#define LIBNFC  1

extern uint8_t main_menu;

typedef enum {
	STATE_IDLE = 0,
	STATE_PREFIX = -1,
	STATE_PREFIX_EXT = -2,
	STATE_HEADER = -3,
	STATE_WAKEUP = -4,
	STATE_FIFOFLUSH = -5,
	STATE_PAYLOAD = -6,
	STATE_FLOWCTRL = -7
} PN532_State;

typedef struct {
	uint32_t last_seen;
	uint16_t reserved;
	uint16_t pos;
	uint16_t expected;
	uint8_t data_prev;
	uint8_t wakeup;
	uint8_t crc;
	uint8_t tfi;
	PN532_State state;
	uint8_t data[PN532_MAX_PACKET_SIZE + 1];
} PN532_Packet;


void packet_init(PN532_Packet * pkt, uint8_t reserved, uint8_t tfi);
void packet_reset(PN532_Packet * pkt);
int packet_put(PN532_Packet * pkt, uint8_t data);
void dump_packet(uint8_t * data, int count);
void rfid_hexdump(const void *buffer, int size);
void get_firmware_version(void);
void loop_libnfc_rfid(void);

#endif/*__LIBNFC_H__*/
