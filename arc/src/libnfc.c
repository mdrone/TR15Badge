#define _XOPEN_SOURCE 9001
#include <openbeacon.h>
#include "libnfc.h"
#include "usbserial.h"


void packet_init(PN532_Packet * pkt, uint8_t reserved, uint8_t tfi)
{
	memset(pkt, 0, sizeof(*pkt));
	pkt->reserved = reserved;
	pkt->tfi = tfi;
	pkt->data_prev = 0x01;
}

void packet_reset(PN532_Packet * pkt)
{
	packet_init(pkt, pkt->reserved, pkt->tfi);
}

int packet_put(PN532_Packet * pkt, uint8_t data)
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

void rfid_hexdump(const void *buffer, int size)
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
void get_firmware_version(void)
{
	int i;
	uint8_t data;
	uint8_t output[PN532_FIFO_SIZE];
	data = PN532_CMD_GetFirmwareVersion;

	while (1) {
		if (((i = rfid_write(&data, sizeof(data))) == 0) &&
		    ((i = rfid_read(output, PN532_FIFO_SIZE))) > 0)
			break;

		debug_printf("fw_res=%i\n", i);
		pmu_wait_ms(490);
		GPIOSetValue(LED_PORT, LED_BIT, LED_ON);
		pmu_wait_ms(10);
		GPIOSetValue(LED_PORT, LED_BIT, LED_OFF);
	}

	if (output[1] == 0x32)
		debug_printf("PN532 firmware version: v%i.%i\n",
			     output[2], output[3]);
	else
		debug("Unknown firmware version\n");
}

/* libnfc START */
void loop_libnfc_rfid(void)
{
	int t, count, res;
	uint8_t data, *p;

	get_firmware_version();

	debug_printf("in libnfc\n");

    PN532_Packet buffer_put, buffer_get;

	packet_init(&buffer_get, 0, 0xD5);
	packet_init(&buffer_put, 1, 0xD4);

	/* run RFID loop */
	t = 0;
	while (1) {
        if ( LIBNFC != main_menu) { break; }

		if (!GPIOGetValue(PN532_IRQ_PORT, PN532_IRQ_PIN)) {
			GPIOSetValue(LED_PORT, LED_BIT, (t++) & 1);

			data = 0x03;

			spi_txrx(SPI_CS_PN532 | SPI_CS_MODE_SKIP_CS_DEASSERT,
				 &data, sizeof(data), NULL, 0);

			while (!GPIOGetValue(PN532_IRQ_PORT, PN532_IRQ_PIN)) {
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
