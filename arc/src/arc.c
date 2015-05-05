#define _XOPEN_SOURCE 9001
#include <openbeacon.h>
#include "arc.h"
#include "libnfc.h"

uint8_t mifare_card[MIFARE_CARD_SIZE];

uint8_t default_keys[][MIFARE_KEY_SIZE] = 
    { 
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    {0x0f, 0x5f, 0xb2, 0x9d, 0xdc, 0x10}, 
    {0x25, 0xe5, 0xb3, 0x47, 0x75, 0x06}, 
    {0x63, 0x15, 0xd5, 0x6b, 0x21, 0xf4}, 
    {0x66, 0x47, 0x0d, 0xe8, 0xaa, 0x11}, 
    {0x7a, 0x46, 0x38, 0x61, 0xb1, 0xec}, 
    {0x7c, 0x56, 0x37, 0xd4, 0x02, 0x40}, 
    {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5}, 
    {0xa2, 0x7d, 0x38, 0x04, 0xc2, 0x59}, 
    {0xbc, 0x0b, 0x0c, 0x6b, 0xb4, 0xec}, 
    {0xc8, 0x27, 0x32, 0x52, 0x23, 0xb3}, 
    {0xc8, 0xb4, 0x70, 0xc4, 0x8f, 0x77}, 
    {0xca, 0x0f, 0xb8, 0x30, 0x93, 0xc6}, 
    {0xfe, 0x39, 0xef, 0x4d, 0x55, 0xe1},
    {0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7}, // NFCForum content key
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // Blank key
    {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5},
    {0x4d, 0x3a, 0x99, 0xc3, 0x51, 0xdd},
    {0x1a, 0x98, 0x2c, 0x7e, 0x45, 0x9a},
    {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff},
    {0x71, 0x4c, 0x5c, 0x88, 0x6e, 0x97},
    {0x58, 0x7e, 0xe5, 0xf9, 0x35, 0x0f},
    {0xa0, 0x47, 0x8c, 0xc3, 0x90, 0x91},
    {0x53, 0x3c, 0xb6, 0xc7, 0x23, 0xf6},
    {0x8f, 0xd0, 0xa4, 0xf2, 0x56, 0xe9}
    };

uint8_t access_bytes[ACCESS_BYTES] = 
    {
    0xFF, 0x07, 0x80, 0x69 /* AB rw Access */
    };

uint8_t key_b[MIFARE_KEY_SIZE] = 
    {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF /* B Access Key */
    };

int turn_rf_off (uint8_t *data, uint8_t size) {
    /* wait 0.5s */
    pmu_wait_ms(5);

    /* turning field off */
    data[0] = PN532_CMD_RFConfiguration;
    data[1] = 0x01;	/* CfgItem = 0x01 */
    data[2] = 0x00;	/* RF Field = off */
    return rfid_execute(&data[0], 3, size);
}

int mifare_reader_init(uint8_t *data, uint8_t size)
{

	/* User Manual S.97 141520.pdf */
	data[0] = PN532_CMD_SAMConfiguration; /* 0x14 */
	data[1] = 0x01;		/* Normal Mode */
	return rfid_execute(&data[0], 2, size);
}

void dump_mifare_card (void)
{
    for (uint8_t i = 0; i < BLOCKS; i++) {
        debug_printf("Block: %2d", i);
        rfid_hexdump(&mifare_card[i*BLOCK_SIZE], BLOCK_SIZE);
    }
}

int initiator_init(uint8_t *data, uint8_t size)
{
    data[0] = PN532_CMD_InListPassiveTarget; /* 0x4a */
    data[1] = 0x01;	/* MaxTg - maximum cards */
    data[2] = 0x00;	/* BrTy - 106 kbps type A */
    return rfid_execute(&data[0], 3, size);
}

int mifare_authenticate_block(uint8_t *data, uint8_t size, uint8_t block)
{

    data[0] = PN532_CMD_InDataExchange; /* 0x40 */
    data[1] = 0x01;	/* card 1 */
    data[2] = 0x60;	/* MIFARE authenticate A */
    data[3] = block;
    return rfid_execute(&data[0], 14, size);

}

void set_uid(uint8_t *data, int oid)
{
    /* MIFARE NFCID1 */
    memcpy(&data[10], &oid, sizeof(oid));
}

void set_key(uint8_t *data, uint8_t keyindex)
{
    /* MIFARE default key 6*0xFF */
    memcpy(&data[4], &default_keys[keyindex], MIFARE_KEY_SIZE);
}


int mifare_read_block(uint8_t *data, uint8_t size, uint8_t block)
{

    data[0] = PN532_CMD_InDataExchange; /* 0x40 */
    data[1] = 0x01;	    /* card 1 */
    data[2] = 0x30;	    /* MIFARE read 16 bytes */
    data[3] = block;    /* block 1 */
    return rfid_execute(&data[0], 4, size);

}

int mifare_write_block(uint8_t *data, uint8_t size, uint8_t block)
{

    data[0] = PN532_CMD_InDataExchange; /* 0x40 */
    data[1] = 0x01;	    /* card 1 */
    data[2] = 0xA0;	    /* MIFARE write 16 bytes */
    data[3] = block;    /* block 1 */
    return rfid_execute(&data[0], 20, size);

}


void loop_clone_rfid(void (*f)(void))
{
    uint8_t data[80];
    uint8_t keyindex = 0;
    uint8_t block = 0;
    uint8_t tries = 0;
    int res, oid;

	get_firmware_version();

    while (block < BLOCKS) {
        if ( READ != main_menu) { break; }
        res = mifare_reader_init(data, sizeof(data));

        if (tries >= KEYS) {
            block += 1;
        }

        if (res >= 0) {
            res = initiator_init(data, sizeof(data));

            if (res >= 11) {
                if (0x00 == data[3] && data[6] >= 0x04) {
                    memcpy(&oid, &data[7], sizeof(oid));
                    if (0x00 == block) {
                        debug_printf("MIFARE_CARD_ID:");
                        rfid_hexdump(&oid, sizeof(oid));
                    }

                    set_uid(data, oid);
                    set_key(data, keyindex);

                    res = mifare_authenticate_block(data, sizeof(data), block);

/*
                    debug_printf("res:");
                    rfid_hexdump(&res, sizeof(res));

                    debug_printf("data:");
                    rfid_hexdump(&data[0], sizeof(data));
*/

                    if (0x41 == data[0] && 0x00 == data[1]) {
                        debug_printf("Auth Succeeded.\n");
                        tries = 0;

                        switch (submenu) {
                            case READ:
                                res = mifare_read_block(data, sizeof(data), block);

                                if (res == 18) {
                                    debug_printf("Block:");
                                    rfid_hexdump(&block, sizeof(block));
                                    debug_printf("Data:");
                                    rfid_hexdump(&data[2], BLOCK_SIZE);
                                    debug_printf("Key:");
                                    rfid_hexdump(&default_keys[keyindex], MIFARE_KEY_SIZE);

                                    memcpy(&mifare_card[block*BLOCK_SIZE], &data[2], BLOCK_SIZE);
                                    if (0x00 == (block+1) % 4) {
                                        memcpy(&mifare_card[block*BLOCK_SIZE], &default_keys[keyindex], MIFARE_KEY_SIZE);
                                        memcpy(&mifare_card[block*BLOCK_SIZE+6], &access_bytes[0], ACCESS_BYTES);
                                        memcpy(&mifare_card[block*BLOCK_SIZE+10], &key_b[0], MIFARE_KEY_SIZE);
                                    }
                                }
                            break;
                            case WRITE:
                                memcpy(&data[4], &mifare_card[block*BLOCK_SIZE], BLOCK_SIZE);
                                res = mifare_write_block(data, sizeof(data), block);
                                debug_printf("res:");
                                rfid_hexdump(&res, sizeof(res));
                            break;
                        }
                        if (BLOCKS-1 == block) {
                            status = 1;
                        }
                        block += 1;
                    } else if (0x41 == data[0] && 0x14 == data[1]) {
                        debug_printf("Auth Failed.\n");
                        keyindex = (keyindex + 1) % KEYS;
                        tries += 1;
                    }
                }
            }
        }
        turn_rf_off(data, sizeof(data));
    }
    if (0x01 == status) {
        (*f)();
    }
    main_menu = LIBNFC;
}
