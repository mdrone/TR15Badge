#ifndef __ARC_H__
#define __ARC_H__

#define MIFARE_KEY_SIZE     6
#define MIFARE_CARD_SIZE    1024
#define BLOCKS              64
#define BLOCK_SIZE          16
#define SECTORS             16
#define ACCESS_BYTES        4
#define KEYS                24

#define READ    0
#define WRITE   1

extern uint8_t submenu;
extern uint8_t status;

void loop_clone_rfid(void (*f)(void));
void dump_mifare_card(void);

#endif/*__ARC_H__*/
