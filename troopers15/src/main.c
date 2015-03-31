/***************************************************************
 *
 * OpenBeacon.org - main file for OpenPCD2 libnfc interface
 *
 * Copyright 2012 Milosch Meriac <meriac@openbeacon.de>
 *
 ***************************************************************

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; version 2.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

 */
/*
 Minor Additions for the Troopers Badge
 Changes:
    General:
    - in core added a wait: chipselect and spi_txrx core/openbeacon/src/rfid.c
    Libnfc:
    - copy&paste openpcd2-libnfc
    Standalone (read):
    - read is mainly copy&paste of openpcd2
    - added a few loops to more or less ensure that the communication doesn't
      collapse
    Emulate:
    - read is mainly copy&paste of openpcd2-emulate
    - hardcoded UID
    - added a few loops to more or less ensure that the communication doesn't
      collapse


 The Communication looks more or less like this:
     PCD: Coupling Device (Reader, Initiator)                                                           
     PICC: Proximity IC Card (Tag, Target)                                                          

    Due to laziness and timing-issues
    +---+ every command loops till data is received
        | a Button press interrupts every loop
    ^   |
    |___|
                                                            
 Szenario 1:
 It is assumed that both parties (PCD, PICC) are emulated by the Badge.

                                                            
           Initiator                                    Target           
              |                                           |             
              |                                           |             
 I. InListPassiveTarget    --activate->             TgInitAsTarget       
              |                                           |             
              |                                      TgSetData          
 II.   InDataExchange      --send    ->              TgGetData        Authentication
              |                                           |           for Block #1
              |                                           |           (we actually don't check the key)
              |                                           |             
              |                                           |             
 III.  InDataExchange      <-send    --              TgSetData        Send Data (just some bytes)
              |                                           |             
              |                                           |             
---------------------------------------------------------------------------

To save you time with the manual and to play with the tamashell (libnfc-tool):

The Szenario 1 with pn53x Commands:
    Step I.
        Initiator: execute 0x4a, 0x01, 0x00
            translates to "Listen for one Target" (0x4a InListPassiveTarget)

        Target: execute 0x8c, MODE, MIFARE, FELICA
            translates to "Hey Im a Mifare Card please activate me" (0x8c TgInitAsTarget)
            Mode x00 == all, x01 == Passive, x02 == DEP, x04 == PICC
            example used: 0x8c 0x05 0x04 0x00 0xDE 0xC0 0xDE 0x20 [30 times 0x00]
                                                |   |    |
                                                ----------- These are the three Bytes we
                                                            can alter as UID, the first Byte
                                                            is fixed to 0x08 by the pn532
    Step II.
        Initiator: execute 0x40, 0x01, 0x60, 0x01, KEY, UID (0x40 InDataExchange)
            translates to "I want to Authenticate for the first block"
            example used: 0x40, 0x01, 0x60, 0x01, [6 times 0xFF], UID

        Target: execute  0x8E (TgSetData)
                execute  0x86 (TgGetData)
            the answer looks like this: 0x87, 0x01, 0x60, 0x01 .. [Data from Initiator]

    Step III.
        Initiator: execute 0x40, 0x01, 0x30, 0x01, KEY, UID
                                        |     |
                                       Read  Block
        Target: check request and prepare response
            execute 0x8E (TgSetData)

*/

#include <openbeacon.h>
#include "iap.h"
#include "rfid.h"
#include "usbserial.h"

#define PN532_FIFO_SIZE 64
#define PN532_MAX_PAYLAOADSIZE 264
#define PN532_MAX_PACKET_SIZE (PN532_MAX_PAYLAOADSIZE+11)

#define NOTHING 4
#define EMULATE 0
#define READ 1
#define LIBNFC 2
#define SAVE 3

#define SAVEUID 0
#define SAVEBLOCK 1

#define UIDPROFILE 0
#define FIRSTPROFILE 16
#define SECONDPROFILE 32
#define THIRDPROFILE 48

short main_menu = LIBNFC;
short main_lock = 1;
short temp_main_menu = LIBNFC;

short profile_lock = 1;
short profile = UIDPROFILE;
short temp_profile = UIDPROFILE;
short profile_led_flashed = 0;

short tell_me_what_to_save = SAVEBLOCK;

static uint8_t payload[80];

void check_profile_leds (void);

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

static PN532_Packet buffer_put, buffer_get;

static void packet_init(PN532_Packet * pkt, uint8_t reserved, uint8_t tfi)
{
	memset(pkt, 0, sizeof(*pkt));
	pkt->reserved = reserved;
	pkt->tfi = tfi;
	pkt->data_prev = 0x01;
}

static void packet_reset(PN532_Packet * pkt)
{
	packet_init(pkt, pkt->reserved, pkt->tfi);
}

static int packet_put(PN532_Packet * pkt, uint8_t data)
{
	PN532_State res;
	uint8_t len, lcs;
	const uint8_t prefix[] = { 0x00, 0x00, 0xFF };

	res = pkt->state;

	switch (pkt->state) {
	case STATE_WAKEUP:
		{
			debug("\nWAKEUP\n");
			pmu_wait_ms(50);
			res = STATE_IDLE;
			/* intentionally no 'break;' */
		}

	case STATE_IDLE:
		{
			/* TODO: WTF? - need to wait for one character */
			debug_printf(".");

			/* if needed, delete packet from previous run */
			if (pkt->pos) {
				packet_reset(pkt);
				break;
			}

			/* scan for 0x00+0xFF prefix */
			if (data == 0xFF && pkt->data_prev == 0x00) {
				memcpy(&pkt->data[pkt->reserved], prefix,
				       sizeof(prefix));
				/* add size of reserved+prefix to packet pos */
				pkt->pos = pkt->reserved + sizeof(prefix);
				/* expect at least a short frame */
				pkt->expected = pkt->pos + 2;
				/* switch to prefix reception mode */
				res = STATE_FLOWCTRL;
				break;
			}

			/* scan for HSU wakeup */
			if (data == 0x55 && pkt->data_prev == 0x55)
				/* wait for three times 0x00 */
				pkt->wakeup = 3;
			else if (pkt->wakeup) {
				if (data)
					pkt->wakeup = 0;
				else {
					pkt->wakeup--;
					if (!pkt->wakeup) {
						res = STATE_WAKEUP;
						break;
					}
				}
			}

			break;
		}

	case STATE_FLOWCTRL:
		{
			pkt->data[pkt->pos++] = data;
			if (pkt->pos >= pkt->expected) {
				lcs = pkt->data[pkt->pos - 1];
				len = pkt->data[pkt->pos - 2];

				/* detected extended frame */
				if (len == 0xFF && lcs == 0xFF) {
					debug("IR: extended frame\n");
					/* expect three more bytes for extended frame */
					pkt->expected += 4;
					res = STATE_PREFIX_EXT;
					break;
				}

				/* detected ACK frame */
				if (len == 0xFF && lcs == 0x00) {
					res = pkt->pos;
					break;
				}

				/* detected NACK frame */
				if (len == 0x00 && lcs == 0xFF) {
					res = pkt->pos;
					break;
				}

				pkt->expected++;
				res = STATE_PREFIX;
			}

			break;
		}

	case STATE_PREFIX:
		{
			pkt->data[pkt->pos++] = data;
			if (pkt->pos >= pkt->expected) {
				lcs = pkt->data[pkt->pos - 2];
				len = pkt->data[pkt->pos - 3];

				if (len == 0x01 && lcs == 0xFF) {
					pkt->expected += len;
					pkt->crc = pkt->data[pkt->pos - 1];
					res = STATE_PAYLOAD;
					break;
				}

				/* if valid short packet */
				if (((uint8_t) (len + lcs)) == 0) {
					pkt->expected += len;

					/*detect oversized packets */
					if (pkt->expected >
					    PN532_MAX_PACKET_SIZE) {
						packet_reset(pkt);
						res = STATE_IDLE;
					} else {
						/* check for TFI */
						if (pkt->data[pkt->pos - 1] ==
						    pkt->tfi) {
							/* maintain CRC including TFI */
							pkt->crc = pkt->tfi;
							res = STATE_PAYLOAD;
						} else {
							packet_reset(pkt);
							res = STATE_IDLE;
						}
					}

					break;
				}
			}
			break;
		}

	case STATE_PREFIX_EXT:
		{
			/* TODO: add extended frame support */
			debug("IR: extended frame is not yet supported\n");
			packet_reset(pkt);
			res = STATE_IDLE;
			break;
		}

	case STATE_PAYLOAD:
		{
			pkt->data[pkt->pos++] = data;
			pkt->crc += data;

			if (pkt->pos >= pkt->expected) {
				if (pkt->crc) {
					debug("IR: packet CRC error [0x%02X]\n",
					      pkt->crc);
					packet_reset(pkt);
					res = STATE_IDLE;
				} else
					res = pkt->pos;
			}
			break;
		}

	default:
		{
			debug("IR: unknown state!!!\n");
			packet_reset(pkt);
			res = STATE_IDLE;
		}
	}

	pkt->data_prev = data;
	pkt->state = (res > 0) ? STATE_IDLE : res;

	return res;
}

void dump_packet(uint8_t * data, int count)
{
	int i;
	for (i = 0; i < count; i++)
		debug_printf("%c%02X", 6 == i % 7 ? '*' : ' ', *data++);
	debug_printf("\n");
}

/* standalone START */
#define MIFARE_KEY_SIZE 6
const unsigned char mifare_key[MIFARE_KEY_SIZE] =
    { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
unsigned char test_signal = 0;

static void rfid_hexdump(const void *buffer, int size)
{
	int i;
	const unsigned char *p = (unsigned char *)buffer;

	for (i = 0; i < size; i++) {
		if (i && ((i & 3) == 0))
			debug_printf(" ");
		debug_printf(" %02X", *p++);
	}
	debug_printf(" [size=%02i]\n", size);
}

/* get firmware version */
static void get_firmware_version(void)
{
	int i;
	uint8_t data;
	data = PN532_CMD_GetFirmwareVersion;

	while (1) {
		if (((i = rfid_write(&data, sizeof(data))) == 0) &&
		    ((i = rfid_read(buffer_get.data, PN532_FIFO_SIZE))) > 0)
			break;

		debug_printf("fw_res=%i\n", i);
		pmu_wait_ms(490);
		GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
		pmu_wait_ms(10);
		GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);
	}

	if (buffer_get.data[1] == 0x32)
		debug_printf("PN532 firmware version: v%i.%i\n",
			     buffer_get.data[2], buffer_get.data[3]);
	else
		debug("Unknown firmware version\n");
}

static void loop_read_rfid(void)
{
	int res, old_test_signal = -1;
	static unsigned char data[80], ultralightid[16], bus, signal;
	static unsigned char oid[4];

	/* fully initialized */
	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);

	debug_printf("in read\n");

	/* User Manual S.97 141520.pdf */
	data[0] = PN532_CMD_SAMConfiguration;
	data[1] = 0x01;		/* Normal Mode */
	res = rfid_execute(&data, 2, sizeof(data));

	/* show card response on U.FL */
	test_signal = (25 << 3) | 2;
	/* enable debug output */
	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);

	while (1) {
		if (main_menu != READ) {
			break;
		}
        check_profile_leds ();
		/* detect cards in field */
		data[0] = PN532_CMD_InListPassiveTarget; /* 0x4a */
		data[1] = 0x01;	/* MaxTg - maximum cards */
		data[2] = 0x00;	/* BrTy - 106 kbps type A */

		if (((res = rfid_execute(&data, 3, sizeof(data))) >= 11)
		    && (data[1] == 0x01) && (data[2] == 0x01)) {
			/* only for Mifare Ultralight cards */
			if (data[3] == 0 && data[4] == 0x44) {
				debug_printf("\nULTRALIGHT_READ:");

				for (int i = 0; i < 16; i++) {
				    if (i < data[6]) {
                        ultralightid[i] = data[6+i];
                    } else {
                        ultralightid[i] = 0x00;
                    }
                }

				data[0] = PN532_CMD_InDataExchange; /* 0x40 */ 
				data[1] = 0x01;	/* card 1 */
				data[2] = 0x30;	/* ULTRALIGHT read 16 bytes */
				data[3] = 0x04;	/* block 1 */

				/* MIFARE Read */
				res = rfid_execute(&data, 4, sizeof(data));

				if (res == 18) {
					rfid_hexdump(&data[2], 16);
					/* save */
					if (tell_me_what_to_save == SAVEUID) {
                        if (profile == UIDPROFILE) {
                            temp_profile = FIRSTPROFILE;
                            memcpy(&payload[temp_profile], &ultralightid, sizeof(ultralightid));	/* "memorize" first block */
                        } else {
                            temp_profile = profile;
                            memcpy(&payload[temp_profile], &ultralightid, sizeof(ultralightid));	/* "memorize" first block */
                        }
                    } else {
					    if (profile == UIDPROFILE) {
                            memcpy(&payload[FIRSTPROFILE], &data[2], 16);	/* "memorize" first block */
                        } else {
                            memcpy(&payload[profile], &data[2], 16);	/* "memorize" first block */
                        }
                    }

                }
				else
					debug_printf(" failed [%i]\n", res);
			} else
				/* only for Mifare Classic cards */
			if (data[3] == 0 && data[4] == 4 && data[6] >= 4) {
				memcpy(oid, &data[7], sizeof(oid));

				res = 0;
				data[0] = PN532_CMD_InDataExchange; /* 0x40 */
				data[1] = 0x01;	/* card 1 */
				data[2] = 0x60;	/* MIFARE authenticate A */
				data[3] = 0x01;	/* block 1 */
				/* MIFARE NFCID1 */
				memcpy(&data[10], oid, sizeof(oid));
				/* MIFARE default key 6*0xFF */
				memcpy(&data[4], mifare_key, MIFARE_KEY_SIZE);

				if (res <= 0) {
					/* MIFARE Authenticate */
					if (main_menu != READ) {
						break;
					}
					res =
					    rfid_execute(&data, 14,
							 sizeof(data));
				}

				if (res > 0) {
					res = 0;
					rfid_hexdump(&data, res);

					data[0] = PN532_CMD_InDataExchange; /* 0x40 */
					data[1] = 0x01;	/* card 1 */
					data[2] = 0x30;	/* MIFARE read 16 bytes */
					data[3] = 0x01;	/* block 1 */

					/* MIFARE Read */
					if (res <= 0) {
						if (main_menu != READ) {
							break;
						}
						res =
						    rfid_execute(&data, 14,
								 sizeof(data));
					}

					debug_printf("\nMIFARE_READ:");
					if (res == 18) {
						rfid_hexdump(&data[2], 16);
                        /* save */
                        if (tell_me_what_to_save == SAVEUID) {
                            if (profile == UIDPROFILE) {
                                temp_profile = FIRSTPROFILE;
                                memcpy(&payload[temp_profile], &oid, sizeof(oid));	/* "memorize" first block */
                            } else {
                                temp_profile = profile;
                                memcpy(&payload[temp_profile], &oid, sizeof(oid));	/* "memorize" first block */
                            }
                            /* fill rest with zeros */
                            for (int i = sizeof(oid); i < 16; i++) {
                                payload[temp_profile+i] = 0x00;
                            }
                        } else {
                            if (profile == UIDPROFILE) {
                                memcpy(&payload[FIRSTPROFILE], &data[2], 16);	/* "memorize" first block */
                            } else {
                                memcpy(&payload[profile], &data[2], 16);	/* "memorize" first block */
                            }
                        }
						//memcpy(&payload[profile], &data[2], 16);	/* "memorize" first block */
					} else
						debug_printf(" failed [%i]\n",
							     res);
				} else
					debug_printf("AUTH failed [%i]\n", res);

				debug_printf("MIFARE_CARD_ID:");
				rfid_hexdump(oid, sizeof(oid));
			} else {
				debug_printf("\nCARD_TYPE:");
				rfid_hexdump(&data[3], 3);
				debug_printf("CARD_ID:");
				rfid_hexdump(&data[7], data[6]);
			}

			/* blink LED to indicate card */
			GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
			pmu_wait_ms(50);
			GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);
		} else {
			GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
			if (res != -8)
				debug_printf("PN532 error res=%i\n", res);
		}

		/* wait 0.5s */
		pmu_wait_ms(500);

		/* turning field off */
		data[0] = PN532_CMD_RFConfiguration;
		data[1] = 0x01;	/* CfgItem = 0x01 */
		data[2] = 0x00;	/* RF Field = off */
		rfid_execute(&data, 3, sizeof(data));

		if (test_signal != old_test_signal) {
			old_test_signal = test_signal;

			/* enable test singal output on U.FL sockets */
			signal = test_signal & 0x7;
			bus = (test_signal >> 3) & 0x1F;

			rfid_write_register(0x6328, 0xFC);
			/* select test bus signal */
			rfid_write_register(0x6321, signal);
			/* select test bus type */
			rfid_write_register(0x6322, bus);
			/* debug_printf("UPDATED_DEBUG_OUTPUT\n"); */
		}
		/* display current test signal ID */
		/* debug_printf("TEST_SIGNAL_ID: %02i.%i\n", bus, signal); */
	}
}
/* standalone END */

/* libnfc START */
static void loop_libnfc_rfid(void)
{
	int t, count, res;
	uint8_t data, *p;

	debug_printf("in libnfc\n");

	packet_init(&buffer_get, 0, 0xD5);
	packet_init(&buffer_put, 1, 0xD4);

	/* run RFID loop */
	t = 0;
	while (1) {
		if (main_menu != LIBNFC) {
			break;
		}
        check_profile_leds ();

		if (!GPIOGetValue(PN532_IRQ_PORT, PN532_IRQ_PIN)) {
			GPIOSetValue(LED_PORT, LED_BIT, (t++) & 1);

			data = 0x03;

			spi_txrx(SPI_CS_PN532 | SPI_CS_MODE_SKIP_CS_DEASSERT,
				 &data, sizeof(data), NULL, 0);

			while (!GPIOGetValue(PN532_IRQ_PORT, PN532_IRQ_PIN)) {
			    check_profile_leds();
				spi_txrx((SPI_CS_PN532 ^ SPI_CS_MODE_SKIP_TX) |
					 SPI_CS_MODE_SKIP_CS_ASSERT |
					 SPI_CS_MODE_SKIP_CS_DEASSERT, NULL, 0,
					 &data, sizeof(data));

				if ((res = packet_put(&buffer_get, data)) > 0) {
					/* add termination */
					buffer_get.data[res++] = 0x00;
					p = buffer_get.data;
					count = res;
					while (count--) {
			            check_profile_leds();
						usb_putchar(*p++);
					}
					usb_flush();
#ifdef  DEBUG
					debug("RX: ");
					dump_packet(buffer_get.data, res);
#endif
				}
			}

			spi_txrx(SPI_CS_PN532 | SPI_CS_MODE_SKIP_CS_ASSERT,
				 NULL, 0, NULL, 0);
		}

		while ((res = usb_getchar()) >= 0) {
            check_profile_leds();
			if ((count =
			     packet_put(&buffer_put, (uint8_t) res)) > 0) {
				GPIOSetValue(LED_PORT, LED_BIT, (t++) & 1);
				buffer_put.data[0] = 0x01;
				buffer_put.data[count++] = 0x00;
				spi_txrx(SPI_CS_PN532, buffer_put.data, count,
					 NULL, 0);
#ifdef  DEBUG
				debug("TX: ");
				dump_packet(&buffer_put.data[1], count - 1);
#endif				 /*DEBUG*/
				    break;
			} else {
				switch (count) {
				case STATE_WAKEUP:
					/* reset PN532 */
					GPIOSetValue(PN532_RESET_PORT,
						     PN532_RESET_PIN, 0);
					pmu_wait_ms(100);
					GPIOSetValue(PN532_RESET_PORT,
						     PN532_RESET_PIN, 1);
					pmu_wait_ms(400);
					count = 0;
					break;
				case STATE_FIFOFLUSH:
					/* flush PN532 buffers */
					buffer_put.data[0] = 0x01;
					memset(&buffer_put.data[1], 0,
					       PN532_FIFO_SIZE);
					spi_txrx(SPI_CS_PN532, buffer_put.data,
						 PN532_FIFO_SIZE + 1, NULL, 0);
					break;
				}
			}
		}
	}
}
/* libnfc END */

/* emulate START */
#define MODE_ALL      0
#define MODE_PASSIVE (1 << 0)
#define MODE_DEP     (1 << 1)
#define MODE_PICC    (1 << 2)
/* http://www.nxp.com/documents/application_note/AN10833.pdf */
#define MF_MINI_1         0x04
#define MF_MINI_2         0x00
#define MF_CLASSIC_1K_1   0x04
#define MF_CLASSIC_1K_2   0x00
#define MF_CLASSIC_4K_1   0x02
#define MF_CLASSIC_4K_2   0x00
#define MF_ULTRALIGHT_1   0x44
#define MF_ULTRALIGHT_2   0x00

#define MF_SAK_MINI       0x09
#define MF_SAK_CLASSIC_1K 0x08
#define MF_SAK_CLASSIC_4K 0x18
#define SAK_ISO_14443_4_COMPLIANT 0x20
static int target_init(unsigned char* data, unsigned int size) {
    int res = -1;
    while(res < 0) {
        check_profile_leds();
        data[0] = PN532_CMD_TgInitAsTarget;	/* 0x8C */
        data[1] = MODE_PASSIVE | MODE_PICC;
        /* 6 Bytes Mifare */
        data[2] = MF_MINI_1;	/* SENS_RES */
        data[3] = MF_MINI_2;
        /*prefix= 0x08 */
        data[4] = 0xDE;	/* three Bytes UID (NFCID1); first Byte is prefixed by the pn532-chip (0x08) */
        data[5] = 0xC0;
        data[6] = 0xDE;
        data[7] = SAK_ISO_14443_4_COMPLIANT; //MF_SAK_MINI;	/* SEL_RES */

        /* 18 Bytes FeliC + 10 Bytes NFCID3t + 1 Byte Len(GT) + 1 Byte Len(TK) */
        for (unsigned int i = 8; i < size; i++) {
            data[i] = 0x00;
        }

        res = rfid_execute(data, 38, size);

	if (main_menu != EMULATE) {
            return -2;
        }
    }
    /* data: 0x8D + 1 Byte Mode + n Byte Initator CMD */
    return res;
}

/* http://www.nxp.com/documents/data_sheet/MF1S503x.pdf */
#define MF_AUTH_A 0x60
#define MF_AUTH_B 0x61
#define MF_READ   0x30
#define RATS      0xe0
#define DESELECT  0xc2
#define PPSS      0xd0
#define HLTA      0xd0
static int process_cmd(unsigned char* data, unsigned int size) {
    if(size) {
        switch(data[1]) {
        case MF_AUTH_A:
        case MF_AUTH_B: {
	        debug_printf("Auth initiated\n");
                return 0;
            } break;
        /* http://wg8.de/wg8n1344_17n3269_Ballot_FCD14443-4.pdf */
        case RATS: {
                if(data[2] & 0x0F) {
	            debug_printf("invalid RATS CID\n");
                    return -2;
                }
	        debug_printf("answer RATS\n");
                data[0] = 1; /* Len(ATS) == 0 */
                return 1;
            }
        case MF_READ: {
	        debug_printf("send 1st block\n");
                memcpy(data, &payload[profile], 16);
                return 16;
            }
        case HLTA:
        case DESELECT: {
	        debug_printf("deselected/halt\n");
                return -1;
            }
        }

        if(data[1] & PPSS) {
            /* send data[0] back */
            data[0] = data[1];
            return 1;
        }
        /* unknown cmd */
        return -2;
    }

    return 0;
}

static void loop_emulate_rfid(void)
{
	int res;
	static unsigned char data[80];

	debug_printf("in emulate\n");

	/* User Manual S.97 141520.pdf */
	data[0] = PN532_CMD_SAMConfiguration;	/* 0x14 */
	data[1] = 0x01;		/* Normal Mode */
	res = rfid_execute(&data, 2, sizeof(data));

	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);

    int was_in_get = 0;
	while (1) {
		if (main_menu != EMULATE) {
			break;
		}
        check_profile_leds ();

        res = target_init(data, sizeof(data));
        while(res >= 0) {
            check_profile_leds();
            /* skip first byte (response type) */
            res = process_cmd(data+1, res-2);
            if(res > 0) {
                data[0] = PN532_CMD_TgSetData; /* 0x8E */
                if (was_in_get == 1) { was_in_get = 0;
                rfid_hexdump (&data, sizeof(data));
                }
                if(rfid_execute(&data, res+1, sizeof(data)) < 0) {
                    /* error during send */
                    break;
                }
                /* data[0] == 0x8F data[1] == Status */
                if(res == 2 && data[1]) {
                    debug_printf("Error occurred during TgSetData: %02X\n", data[1]);
                    if (0x29 == data[1]) { break; }
                }
            }

            if(res >= 0) {
                data[0] = PN532_CMD_TgGetData; /* 0x86 */
                res = rfid_execute(&data, 1, sizeof(data));

                pmu_wait_ms(5);

                /* data[0] == 0x87 data[1] == Status */

                /* Handle Auth explicitly */
                if(data[2] == 0x60) {
                    data[0] = PN532_CMD_TgSetData; /* 0x8E */
                    if((res = rfid_execute(&data, 1, sizeof(data))) < 0) {
                        /* error during send */
                        break;
                    }
                }
                if(res == 2 && data[1]) {
                    debug_printf("Error occurred during TgGetData: %02X\n", data[1]);
                    /* break out (reinitiate) on Error (most of the time if released or invalid d,state) */
                    /* 0x29: Released; 0x25: Invalid device State */
                    break;
                }
            }
            pmu_wait_ms(5);
        }
        pmu_wait_ms(25);
	}
}
/* emulate END */

void
libnfc_leds (void)
{
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(1, 10, LED_ON);
    GPIOSetValue(0, 7, LED_ON);
}

void
emulate_leds (void)
{
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(1, 10, LED_OFF);
    GPIOSetValue(0, 7, LED_ON);
}

void
read_leds (void)
{
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(1, 10, LED_ON);
    GPIOSetValue(0, 7, LED_OFF);
}

/* If EEPROM is supported */
void
save_leds (void)
{
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(1, 10, LED_ON);
    GPIOSetValue(0, 7, LED_ON);
}

void
leds_on (void)
{
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
    GPIOSetValue(1, 10, LED_ON);
    GPIOSetValue(0, 7, LED_ON);
}
void
leds_off (void)
{
    GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);
    GPIOSetValue(1, 10, LED_OFF);
    GPIOSetValue(0, 7, LED_OFF);
}

void
flash_light (int i)
{
    if (profile_led_flashed == 0) {
        for (int n = 0; n < 3; n++) {
            GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
            pmu_wait_ms(40*i);
            GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);
            pmu_wait_ms(40*i);
        }
        debug_printf("Profile %x\n", temp_profile);
        debug_printf("Active Profile %x\n", profile);
    }
    profile_led_flashed = 1;
    GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
}

void
menu_lock_toggle (void)
{
    if (main_lock == 1) {
        main_lock = 0;
        temp_main_menu = main_menu;
    } else {
        main_lock = 1;
        main_menu = temp_main_menu;
        debug_printf("confirmed\n");
    }
	debug_printf("Lock: %i, Menu: %i\n", main_lock, main_menu);
}

void
profile_lock_toggle (void)
{
    if (profile_lock == 1) {
        profile_lock = 0;
        temp_profile = profile;
    } else {
        profile = temp_profile;
        profile_lock = 1;
        debug_printf("confirmed\n");
    }
	debug_printf("Lock: %i, profile: %i\n", profile_lock, profile);
}

/* Set and Handle Interrupts */
void WAKEUP_IRQHandlerPIO2_0(void)
{
	debug_printf("Menu (Pressed 2_0)\n");
	/* Clear pending IRQ */
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO2_0;
	
    temp_main_menu = (temp_main_menu + 1) % SAVE;
    main_lock = 0;

    switch (temp_main_menu) {
    case EMULATE:
        emulate_leds();
        break;
    case READ:
        read_leds();
        break;
    case LIBNFC:
        libnfc_leds();
    case NOTHING:
        break;
    }
    debug_printf("Press OK to Lock Menu %x\n", temp_main_menu);
}

void
check_profile_leds (void)
{
    switch (temp_profile) {
    case UIDPROFILE:
        flash_light (1);
        break;
    case FIRSTPROFILE:
        flash_light (3);
        break;
    case SECONDPROFILE:
        flash_light (6);
        break;
    case THIRDPROFILE:
        flash_light (9);
        break;
    }
}

void WAKEUP_IRQHandlerPIO0_1(void)
{
    debug_printf("Profile (Pressed 0_1)\n");
    LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO0_1;
    switch (temp_profile) {
    case UIDPROFILE:
        temp_profile = FIRSTPROFILE;
        break;
    case FIRSTPROFILE:
        temp_profile = SECONDPROFILE;
        break;
    case SECONDPROFILE:
        temp_profile = THIRDPROFILE;
        break;
    case THIRDPROFILE:
        temp_profile = UIDPROFILE;
        break;
    }
    profile_led_flashed = 0;
}

void WAKEUP_IRQHandlerPIO1_0(void)
{
    debug_printf("OK (Pressed 1_0)\n");
    LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_0;
    if (temp_main_menu != main_menu) {
        menu_lock_toggle ();
    } else if (temp_profile != profile) {
        profile_lock_toggle ();
    }
}

void ButtonInit(void)
{
	NVIC_EnableIRQ(WAKEUP_PIO2_0_IRQn);

	LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO2_0);
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO2_0;
	LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO2_0;

	NVIC_EnableIRQ(WAKEUP_PIO1_0_IRQn);

	LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO1_0);
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO1_0;
	LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO1_0;

	NVIC_EnableIRQ(WAKEUP_PIO0_1_IRQn);

	LPC_SYSCON->STARTAPRP0 = (LPC_SYSCON->STARTAPRP0 & ~STARTxPRP0_PIO0_1);
	LPC_SYSCON->STARTRSRP0CLR = STARTxPRP0_PIO0_1;
	LPC_SYSCON->STARTERP0 |= STARTxPRP0_PIO0_1;
}

int main(void)
{

	/* Initialize GPIO (sets up clock) */
	GPIOInit();

	ButtonInit();

	/* Set LED port pin to output */
	GPIOSetDir(LED_PORT, LED_BIT, 1);
	GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);

	GPIOSetDir(1, 1, 1);
	GPIOSetDir(1, 10, 1);
	GPIOSetDir(0, 7, 1);

	GPIOSetValue(1, 1, LED_OFF);
	GPIOSetValue(1, 10, LED_OFF);
	GPIOSetValue(0, 7, LED_OFF);

	/* UART setup */
	UARTInit(115200, 0);

	/* CDC USB Initialization */
	usb_init();

	/* Init Power Management Routines */
	pmu_init();

	/* Init RFID SPI interface */
	rfid_init();

	//SystemInit();

	debug_printf("OpenPCD2 vTROOPERS15\n");

	/* show LED to signal initialization */
	GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
	pmu_wait_ms(500);

	/* reset FIFO buffers */
	packet_init(&buffer_get, 0, 0xD5);
	packet_init(&buffer_put, 1, 0xD4);

    /* UID */
    TDeviceUID uid;
    debug_printf ("UID:");
    iap_read_uid (&uid);
    rfid_hexdump(&uid, DEVICE_UID_MEMBERS*4);

	/* init payload (first block to replay) */
    memcpy(&payload[UIDPROFILE], &uid, 16);	/* "memorize" uid as first block */

	get_firmware_version();

	debug_printf ("You have passed the Test\n");
	debug_printf ("What Test?\n");
	debug_printf ("... the Debuginterfacetest\n");

    debug_printf ("          __________________/\\__________________\n");
    debug_printf ("         /                                      \\\n");
    debug_printf ("        (             Welcome to the             )\n");
    debug_printf ("        |            TROOPERS15 BADGE            |\n");
    debug_printf ("        |                                        |\n");
    debug_printf ("        |   For more information please visit:   |\n");
    debug_printf ("        `                                        ´\n");
    debug_printf ("         `      https://wwww.insinuator.net/    ´\n");
    debug_printf ("          `     https://www.troopers.de/       ´\n");
    debug_printf ("           `                                  ´\n");
    debug_printf ("            `                                ´\n");
    debug_printf ("             `                              ´\n");
    debug_printf ("              `                            ´\n");
    debug_printf ("                `                        ´\n");
    debug_printf ("                  `                    ´\n");
    debug_printf ("                     `      __      ´\n\n");
    debug_printf ("          __________________/\\__________________\n");
    debug_printf ("         /                                      \\\n");
    debug_printf ("        (                                        )\n");
    debug_printf ("        |               +-----+           LED0   |\n");
    debug_printf ("        |               |     |           LED1   |\n");
    debug_printf ("        |   MENU        |  0 EMULATE             |\n");
    debug_printf ("        `               |  1 READ         OK     ´\n");
    debug_printf ("         `              |  2 LIBNFC             ´\n");
    debug_printf ("          `             |     |                ´\n");
    debug_printf ("           `            +-----+               ´\n");
    debug_printf ("            `                                ´\n");
    debug_printf ("             `     RESET                    ´\n");
    debug_printf ("              `       PROFILE              ´\n");
    debug_printf ("                `                        ´\n");
    debug_printf ("                  `      PR_LED        ´\n");
    debug_printf ("                     `      __      ´\n\n");

    while (1) {
    check_profile_leds ();
    switch (main_menu) {
            case EMULATE:
                emulate_leds();
                loop_emulate_rfid();
                break;
            case READ:
                read_leds();
                loop_read_rfid();
                break;
            case LIBNFC:
                libnfc_leds();
                loop_libnfc_rfid();
                break;
            case NOTHING:
                pmu_wait_ms(500);
                break;
            }
        }
        return 0;
}
