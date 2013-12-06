/*
 * avrdude - A Downloader/Uploader for AVR device programmers
 * Copyright (C) 2003-2004  Theodore A. Roth  <troth@openavr.org>
 * Copyright 2007 Joerg Wunsch <j@uriah.heep.sax.de>
 * Copyright 2008 Klaus Leidinger <klaus@mikrocontroller-projekte.de>
 * Copyright 2009 Sascha Warner <swarner@mibtec.de>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* avrootloader.c 814 2009-09-23 sascha_warner  - Not a real ID, just for reference FIXME */

/*
 * avrdude interface for the very nice and feature-rich bootloader avrootloader
 * by Hagen Reddmann.
 * TODO: Encryption, versioning
 */

#include "ac_cfg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <sys/time.h>
#include <unistd.h>

#include "avrdude.h"
#include "avr.h"
#include "config.h"
#include "crc16.h"
#include "pgm.h"
#include "avrootloader.h"
#include "serial.h"

#define VERSION_OFFSET_FROM_END 3			// position of the bottloader version in the first reply
#define SIG_OFFSET_FROM_END 4				// ...chip signature... 
#define BOOTPAGES_OFFSET_FROM_END 1			// ...reserved pages...
#define SPAMDELAY 20 * 1000					// how long do we wait before sending another INIT message
#define SELECTDELAY 50						// how long do we wait for an answer if we tried to INIT
#define INIT_TRIALS 100						// how often do we try to contact the bootloader before giving up
#define CMD_INIT 1
#define CMD_WRITEFLASH 2
#define CMD_ERASEPAGES 3
#define CMD_SENDBUF 4
#define CMD_VERIFYFLASH 5
#define CMD_WRITEE 6


#define CRYPT 1
#define CRYPTFLASH 2
#define CRYPTEE 4
#define VERSIONING 8

/*
 * Private data for this programmer.
 */
struct pdata
{
  unsigned char sigbytes[3];
  char has_auto_incr_addr;
  unsigned char devcode;
  unsigned int buffersize;
  unsigned char test_blockmode;
  unsigned char use_blockmode;
  unsigned char * internalbuf;
  unsigned char * internaleeprombuf;
  unsigned int eeprompos;
  unsigned int eepromaddr;
  unsigned int maxdelay;
  unsigned int page_size;
  unsigned int bootpages;
  unsigned int current_page_vrfy;
  unsigned int nbytes;
  unsigned char features;
  unsigned char trig[255];
  unsigned char key[255];
  unsigned char * eeprom;
};

#define PDATA(pgm) ((struct pdata *)(pgm->cookie))

static void avrootloader_setup(PROGRAMMER * pgm)
{
	if ((pgm->cookie = malloc(sizeof(struct pdata))) == 0)
	{
		fprintf(stderr,
			"%s: avrootloader_setup(): Out of memory allocating private data\n",
			progname);
		exit(1);
	}
	memset(pgm->cookie, 0, sizeof(struct pdata));
	PDATA(pgm)->test_blockmode = 1;
}

static void avrootloader_teardown(PROGRAMMER * pgm)
{
	if (PDATA(pgm)->eeprom != 0)
		free(PDATA(pgm)->eeprom);

	if (PDATA(pgm)->internalbuf != 0)
		free(PDATA(pgm)->internalbuf);

	free(pgm->cookie);
}


static int avrootloader_send(PROGRAMMER * pgm, char * buf, size_t len)
{
	return serial_send(&pgm->fd, (unsigned char *)buf, len);
}


static int avrootloader_recv(PROGRAMMER * pgm, char * buf, size_t len)
{
  int rv;

	if (serial_probe(&pgm->fd, PDATA(pgm)->maxdelay) > 0)
	{
		rv = serial_recv(&pgm->fd, (unsigned char *)buf, len);
		if (rv < 0)
		{
			fprintf(stderr,
				"%s: avrootloader_recv(): programmer is not responding\n",
				progname);
			exit(1);
		}
	}
	else
	{
		fprintf(stderr,
			"%s: avrootloader_recv(): programmer is not responding, select timed out\n",
			progname);
		exit(1);
	}
	return rv;
}


static int avrootloader_drain(PROGRAMMER * pgm, int display)
{
  return serial_drain(&pgm->fd, display);
}

static void avrootloader_vfy_cmd_sent(PROGRAMMER * pgm, char * errmsg)
{
  unsigned char c = 0;
  
	avrootloader_recv(pgm, &c, 1);
  
	switch (c)
	{
		case 0xc0:
			fprintf(stderr, "%s: Verification error: %s\n",	progname, errmsg);
			exit(-1);
			break;

		case 0xc1:
			fprintf(stderr, "%s: Unknown command error: %s\n", progname, errmsg);
			exit(-1);
			break;

		case 0xc2:
			fprintf(stderr, "%s: CRC error: %s\n", progname, errmsg);
			exit(-1);
			break;

		case 0xc3:
			fprintf(stderr, "%s: Boundary error: %s\n", progname, errmsg);
			exit(-1);
			break;

		case 0xc4:
			fprintf(stderr, "%s: Decryption error: %s\n", progname, errmsg);
			exit(-1);
			break;
		
		case 0xc5:
			fprintf(stderr, "%s: Programming error: %s\n", progname, errmsg);
			exit(-1);
			break;

		case 0xc6:
			fprintf(stderr, "%s: Wrong version error: %s\n", progname, errmsg);
			exit(-1);
			break;

		case 0x30:
			return;
			break;
			
		default:
			fprintf(stderr, "%s: Unknown error: %s, Code 0x%x\n", progname, errmsg, c & 0xff);
			exit(-1);
	}
 }



/*
 * issue the 'chip erase' command to the AVR device
 */
static int avrootloader_chip_erase(PROGRAMMER * pgm, AVRPART * p)
{
  //avrootloader_send(pgm, "e", 1);
  //avrootloader_vfy_cmd_sent(pgm, "chip erase");

  /*
   * avrootloader firmware may not delay long enough SW FIXME needed for avrootloader? I dont think so...
   */
  //usleep (p->chip_erase_delay);
/*
  fprintf(stderr, "\n AVR CHIP_ERASE\n");
	erase[1] = (m->size - BootPages) / page_size; // 'FIXME static FLASH and bootloader sizes
	crc = 0;
	for (i = 0; i < sizeof(erase) - 2; i++)
		crc = calcCRC16r(crc, erase[i], 0xa001);

	erase[2] = (crc & 0xff);
	erase[3] = (crc >> 8);

	avrootloader_send(pgm, erase, sizeof(erase));
	avrootloader_vfy_cmd_sent(pgm, "Erase remaining chip");
*/
  return 0;
}

static void avrootloader_leave_prog_mode(PROGRAMMER * pgm)
{}

/*
 * issue the 'program enable' command to the AVR device
 */
static int avrootloader_program_enable(PROGRAMMER * pgm, AVRPART * p)
{
  return -1;
}

/*
 * transmit an AVR device command and return the results; 'cmd' and
 * 'res' must point to at least a 4 byte data buffer
 */
static int avrootloader_cmd(PROGRAMMER * pgm, unsigned char cmd[4], 
                      unsigned char res[4])
{
	return 0; // FIXME
  char buf[5];

  /* FIXME: Insert version check here */

  buf[0] = '.';                 /* New Universal Command */
  buf[1] = cmd[0];
  buf[2] = cmd[1];
  buf[3] = cmd[2];
  buf[4] = cmd[3];

  avrootloader_send (pgm, buf, 5);
  avrootloader_recv (pgm, buf, 2);

  res[0] = 0x00;                /* Dummy value */
  res[1] = cmd[0];
  res[2] = cmd[1];
  res[3] = buf[0];

  return 0;
}

static int avrootloader_send_cmd(PROGRAMMER * pgm, unsigned char cmd, unsigned int parambytes ,char * params)
{
char helo[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0d};
char writeflash[4] = {0x01, 0x01, 0xc0, 0x50};
char setbuf[6] = {0xfe, 0x00, 0x0f, 0x00, 0x34, 0x18};
char erase[4] = {0x02, 0x76, 0x80, 0x86};
char writeeeprom[4] = {0x05, 0x01, 0xc2, 0x90};
char vrfyflash[4] = {0x03, 0x01, 0xc1, 0x30};

unsigned int i = 0;
unsigned short tmp = 0;
unsigned char crc[2] = {0, 0};

	switch (cmd)
	{
		case CMD_INIT:
			tmp = 0;
			for (i = 0; i < strlen(helo); i++)
				tmp = calcCRC16r(tmp, helo[i], 0xa001);
			for (i = 0; i < strlen(PDATA(pgm)->key); i++)
				tmp = calcCRC16r(tmp, PDATA(pgm)->key[i], 0xa001);
	
			crc[0] = tmp & 0xff;
			crc[1] = (tmp >> 8) & 0xff;	

			avrootloader_send(pgm, helo, sizeof(helo));
			avrootloader_send(pgm, PDATA(pgm)->key, strlen(PDATA(pgm)->key));
			avrootloader_send(pgm, crc, sizeof(crc));
			break;

		case CMD_WRITEFLASH:
			avrootloader_send_cmd(pgm, CMD_SENDBUF, parambytes, params);
			avrootloader_send(pgm, writeflash, sizeof(writeflash));
			avrootloader_vfy_cmd_sent(pgm, "WRITE FLASH");			
			break;
			
		case CMD_WRITEE:
			
			avrootloader_send_cmd(pgm, CMD_SENDBUF, parambytes, params);
			avrootloader_send(pgm, writeeeprom, sizeof(writeeeprom));
			avrootloader_vfy_cmd_sent(pgm, "WRITE EEPROM");			
			break;
			
		case CMD_ERASEPAGES:
			erase[1] = *params;
			tmp = 0;
			for (i = 0; i < sizeof(erase) - 2; i++)
				tmp = calcCRC16r(tmp, erase[i], 0xa001);

			erase[2] = tmp & 0xff;
			erase[3] = tmp >> 8;

			avrootloader_send(pgm, erase, sizeof(erase));
			avrootloader_vfy_cmd_sent(pgm, "ERASE REMAINING FLASH");
			break;
		
		case CMD_VERIFYFLASH:
			avrootloader_send_cmd(pgm, CMD_SENDBUF, parambytes, params);
			avrootloader_send(pgm, vrfyflash, sizeof(vrfyflash));
			avrootloader_vfy_cmd_sent(pgm, "VERIFY FLASH");
			break;

		case CMD_SENDBUF:
			setbuf[3] = (parambytes - 2) & 0xff;
			setbuf[2] = (parambytes - 2) >> 8;
			tmp = 0;
			for (i = 0; i < (sizeof(setbuf) - 2); i++)
				tmp = calcCRC16r(tmp, setbuf[i], 0xa001);
			setbuf[4] = tmp & 0xff;
			setbuf[5] = (tmp >> 8) & 0xff;
			
			tmp = 0;
			for (i = 0; i < parambytes - 2; i++)
				tmp = calcCRC16r(tmp, params[i], 0xa001);
			
			params[parambytes - 2] = tmp & 0xff;
			params[parambytes - 1] = (tmp >> 8) & 0xff;

			avrootloader_send(pgm, setbuf, sizeof(setbuf));
			avrootloader_send(pgm, params, parambytes);
			avrootloader_vfy_cmd_sent(pgm, "FILL BUFFER");
			break;

		default:
			return -1;
			break;
	}
return 0;
}



/*
 * initialize the AVR device and prepare it to accept commands
 */
static int avrootloader_initialize(PROGRAMMER * pgm, AVRPART * p) 
{
  char rcv[265];
  char tmp;
  unsigned int i = 0;
  unsigned int errcnt = 0;

	memset(rcv, 0, sizeof(rcv));

	while (((tmp != '0') || (i == 0)) && (i < sizeof(rcv)))
	{
		if (i == 0)
		{
			usleep(SPAMDELAY);
			avrootloader_send_cmd(pgm, CMD_INIT, 0, &tmp);
		}
	
		if (serial_probe(&pgm->fd, SELECTDELAY) > 0)
		{
			avrootloader_recv(pgm, &tmp, 1);	// this damn thing blocks us 5s!
			rcv[i] = tmp;						// thats why we introduced serial_probe

			if (i < strlen(PDATA(pgm)->trig))
			{
				if (rcv[i] == PDATA(pgm)->trig[i])
					i++;
				else
				{
					i = 0;
					errcnt++;
				}
			}
			else
				i++;
		}
		else
			errcnt++;

		if (errcnt > INIT_TRIALS)
			{
				fprintf(stderr, " %s: avrootloader_initialize() timeout while contacting bootloader \n", progname);
				exit(-1);
			}		
	}

	if ((rcv[i - 1] & 0xf0) != 0x30)
	{
		fprintf(stderr, " %s: avrootloader_initialize() unexpected bootloader response 0x%x\n", progname, rcv[i - 1]);
 		exit(-1);
	}
 
	if (rcv[i - VERSION_OFFSET_FROM_END] != 5)
	{
		fprintf(stderr, " %s: avrootloader_initialize() unexpected bootloader version %u\n", progname, rcv[i - VERSION_OFFSET_FROM_END]);
		//exit(-1);
	}
 
	PDATA(pgm)->features = rcv[i] & 0x0f;
	PDATA(pgm)->bootpages = rcv[i - BOOTPAGES_OFFSET_FROM_END];
	PDATA(pgm)->sigbytes[0] = 0x1e;
	PDATA(pgm)->sigbytes[1] = rcv[i - (SIG_OFFSET_FROM_END + 1)];
	PDATA(pgm)->sigbytes[2] = rcv[i - SIG_OFFSET_FROM_END];

	printf("\nEntering programming mode...\n");

	return 0;
}


static void avrootloader_disable(PROGRAMMER * pgm)
{}


static void avrootloader_enable(PROGRAMMER * pgm)
{}

static int avrootloader_parseextparms(PROGRAMMER * pgm, LISTID extparms)
{
  LNODEID ln;
  const char *extended_param;
  int rv = 0;
  char loader[11] = { 'B', 'O', 'O', 'T', 'L', 'O', 'A', 'D', 'E', 'R' , 0x00};
  char back[12] = { '(', 'c', ')', ' ', '2', '0', '0', '9', ' ', 'H' ,'R' , 0x00};
  unsigned int i;

	for (i = 0; i < sizeof(loader); i++)
		PDATA(pgm)->key[i] = loader[i];

	for (i = 0; i < sizeof(back); i++)
		PDATA(pgm)->trig[i] = back[i];	

	for (ln = lfirst(extparms); ln; ln = lnext(ln)) 
	{
		extended_param = ldata(ln);

		if (strncmp(extended_param, "bootid=", strlen("bootid=")) == 0) 
		{
			char bootid[255] = {0};
			if (sscanf(extended_param, "bootid=%s", bootid) != 1)
			{
				fprintf(stderr,
					"%s: avrootloader_parseextparms(): invalid devcode '%s'\n",
					progname, extended_param);
				rv = -1;
				continue;
			}
			else
				fprintf(stderr, 
					"%s: set bootloader signature to '%s'\n",
					progname, bootid);
				
			if (verbose >= 2) 
				fprintf(stderr,
					"%s: avrootloader_parseextparms(): devcode overwritten as %s\n",
					progname, bootid);
			//PDATA(pgm)->devcode = devcode;

			continue;
		}

		if (strncmp(extended_param, "no_blockmode", strlen("no_blockmode")) == 0)
		{
			if (verbose >= 2) 
			{
				fprintf(stderr,
					"%s: avrootloader_parseextparms(-x): no testing for Blockmode\n",
					progname);
			}
			
			PDATA(pgm)->test_blockmode = 0;
			continue;
		}

		if (strncmp(extended_param, "trig=", strlen("trig=")) == 0)
		{
			sscanf(extended_param, "trig=%s", PDATA(pgm)->trig);
			
			if (verbose >= 2) 
			{
				fprintf(stderr,
					"%s: avrootloader_parseextparms(-x): triggering on %s\n",
					progname, PDATA(pgm)->trig);
			}
			continue;
		} 

		if (strncmp(extended_param, "key=", strlen("key=")) == 0)
		{
			sscanf(extended_param, "key=%s", PDATA(pgm)->key);
			
			if (verbose >= 2) 
			{
				fprintf(stderr,
					"%s: avrootloader_parseextparms(-x): sending key '%s'\n",
					progname, PDATA(pgm)->key);
			}
			continue;
		} 

		fprintf(stderr,
			"%s: avrootloader_parseextparms(): invalid extended parameter '%s'\n",
			progname, extended_param);
		rv = -1;
	}
	return rv;
}


static int avrootloader_open(PROGRAMMER * pgm, char * port)
{
  /*
   *  If baudrate was not specified use 115.200 Baud so we might flash fast
   */
	if(pgm->baudrate == 0)
		pgm->baudrate = 115200; // SW: was 19200

	strcpy(pgm->port, port);
	serial_open(port, pgm->baudrate, &pgm->fd);

  /*
   * drain any extraneous input
   */
	avrootloader_drain (pgm, 0);
	
	return 0;
}

static void avrootloader_close(PROGRAMMER * pgm)
{
	avrootloader_leave_prog_mode(pgm);

	serial_close(&pgm->fd);
	pgm->fd.ifd = -1;
}


static void avrootloader_display(PROGRAMMER * pgm, const char * p)
{
	return;
}


static void avrootloader_set_addr(PROGRAMMER * pgm, unsigned long addr)
{
  unsigned char cmd[6];
  unsigned short crcx = 0;
  unsigned int i;
  
	cmd[0] = 0xff;
	cmd[1] = (addr >> 16) & 0xff;
	cmd[2] = (addr >> 8 ) & 0xff;
	cmd[3] = addr & 0xff;

	for (i = 0; i < sizeof(cmd) - 2; i++)
		crcx = calcCRC16r(crcx, cmd[i], 0xa001);
  
	cmd[4] = (unsigned char) crcx & 0xff;
	cmd[5] = (unsigned char) (crcx >> 8) & 0xff;
  
	avrootloader_send(pgm, cmd, sizeof(cmd));
	avrootloader_vfy_cmd_sent(pgm, "SET ADDRESS");
}


static int avrootloader_write_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                             unsigned long addr, unsigned char value)
{
	if (strcmp(m->desc, "flash") == 0) 
		return -2; 	// not supported, why do you think we have paged_write?
					// we need to fill at least one flashpage anyway and we are
					// not going to rewrite the same page for each byte we get.
					
	else if (strcmp(m->desc, "eeprom") == 0)
	{
		unsigned char buf[m->page_size];
		
		memset(buf, 0xff, m->page_size);
		
		// buffer is page aligned
		buf[addr % m->page_size] = value;
		
		// we need to align here cause we are going to write one whole page
		avrootloader_set_addr(pgm, (addr / m->page_size) * m->page_size);
		//avrootloader_vfy_cmd_sent(pgm, "SET ADDRESS");
		
		avrootloader_send_cmd(pgm, CMD_WRITEE, m->page_size, buf);
	}
	else
		return avr_write_byte_default(pgm, p, m, addr, value);
	
	return 0;
}

static int avrootloader_read_byte_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                  unsigned long addr, unsigned char * value)
{
  unsigned int i;
  unsigned short crc = 0;
  unsigned int written = 0;
  char setbuf[6] = {0xfe, 0x00, 0x0f, 0x00, 0x34, 0x18};
  char vrfyflash[4] = {0x03, 0x01, 0xc1, 0x30};
  unsigned int bufsize = 0;

	if (PDATA(pgm)->internalbuf == 0)
	{
		fprintf(stderr, " %s: avrootloader_read_byte_flash() Reading is not supported by this bootloader - only verify works\n", progname);
		exit(-1);
	}

	bufsize = (p->sram - m->page_size);

	if (bufsize == 0)
		bufsize = m->page_size;

	char buf[bufsize + 2];

	if (PDATA(pgm)->current_page_vrfy != (addr / m->page_size))
	{
		avrootloader_set_addr(pgm, addr >> 1);
		PDATA(pgm)->current_page_vrfy = addr / m->page_size;

		for (i = 0; i < 4; i++)
			crc = calcCRC16r(crc, setbuf[i], 0xa001);
		setbuf[4] = crc & 0xff;
		setbuf[5] = crc >> 8;

		while (written < PDATA(pgm)->nbytes)
		{
			if ((written + bufsize) > PDATA(pgm)->nbytes)
			{
				memset(buf, 0xff, bufsize);
				memcpy(buf, m->buf + written, PDATA(pgm)->nbytes - written);
			}
			else
				memcpy(buf, m->buf + written, bufsize);
	
			crc = 0;
			for (i = 0; i < bufsize; i++)
				crc = calcCRC16r(crc, buf[i], 0xa001);

			buf[bufsize] = (crc & 0xff);
			buf[(bufsize + 1)] = (crc >> 8);
	
			PDATA(pgm)->maxdelay = 5000;
			avrootloader_send(pgm, setbuf, sizeof(setbuf));
			avrootloader_send(pgm, buf, sizeof(buf)); 
			avrootloader_vfy_cmd_sent(pgm, "FILL BUFFER");

			avrootloader_send(pgm, vrfyflash, sizeof(vrfyflash));
			avrootloader_vfy_cmd_sent(pgm, "VERIFY FLASH");
	
			written += bufsize;
			report_progress (written, PDATA(pgm)->nbytes, NULL);
		}
	}
	return (buf[addr % PDATA(pgm)->page_size]);
}


static int avrootloader_read_byte_eeprom(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                                   unsigned long addr, unsigned char * value)
{
  char readeeprom[4] = {0x04, 0x00, 0x02, 0xc0};
  unsigned int i;
  unsigned short tmp;
  unsigned char crc[2] = {0, 0};
  unsigned int bytesread = 0;
  unsigned int bufsize = 0;
  
	bufsize = avr_locate_mem(p, "flash")->page_size * 2;
 
	if (PDATA(pgm)->eeprom == 0)
	{
		PDATA(pgm)->eeprom = malloc(m->size);
		if (PDATA(pgm)->eeprom == 0)
		{
			fprintf(stderr,"\nError allocating memory: avrootloader_read_byte_eeprom\n");
			exit(-1);
		}	
		memset(PDATA(pgm)->eeprom, 0xff, m->size);

		PDATA(pgm)->maxdelay = 5000;

		while (bytesread < m->size)
		{
			avrootloader_send(pgm, readeeprom, sizeof(readeeprom));
			avrootloader_recv(pgm, PDATA(pgm)->eeprom + bytesread, bufsize);
			avrootloader_recv(pgm, crc, sizeof(crc));
			
			tmp = 0;
			for (i = 0; i < bufsize; i++)
				tmp = calcCRC16r(tmp, PDATA(pgm)->eeprom[i + bytesread], 0xa001);
			
			
			if (((tmp & 0xff) != crc[0]) || ((tmp >> 8) != crc[1]))
			{
				fprintf(stderr, "\navrootloader: Error in EEPROM CRC - please retry\n");
				exit(-1);
			}
			
			avrootloader_vfy_cmd_sent(pgm, "READ EEPROM");
			bytesread += bufsize;
		}
	}
	*value = PDATA(pgm)->eeprom[addr - m->offset];
//	free(PDATA(pgm)->eeprom);

	return 1;
}


static int avrootloader_read_byte(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                            unsigned long addr, unsigned char * value)
{
	if (strcmp(m->desc, "flash") == 0)
		return avrootloader_read_byte_flash(pgm, p, m, addr, value);

	if (strcmp(m->desc, "eeprom") == 0)
		return avrootloader_read_byte_eeprom(pgm, p, m, addr, value);
 
 	// This bootloader cannot read any fuses (no bootloader can set at all)
	
	*value = 1; 
	return 1;
 }

static int avrootloader_paged_write_flash(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                                    int page_size, int n_bytes)
{
  unsigned int addr = 0;
  unsigned int bufsize = 0;
  unsigned int written = 0;
  unsigned char param_eraseprog = 1;
  unsigned int tmp = 0;

	if (n_bytes < (p->sram - page_size))
		while (bufsize < n_bytes)
			bufsize += page_size;
	else
		bufsize = (p->sram - page_size);

	if (bufsize == 0)
		bufsize = page_size;

	char buf[bufsize + 2];

	PDATA(pgm)->page_size = page_size;
	PDATA(pgm)->nbytes = n_bytes;

	if ((PDATA(pgm)->internalbuf = malloc(n_bytes + page_size)) == 0)
	{
	    fprintf(stderr,
	    "%s: avrootloader__paged_write_flash(): Out of memory allocating verify buffer\n",
	    progname);
		exit(1);	
	}
	memset(PDATA(pgm)->internalbuf, 0xff, n_bytes + page_size);
	memcpy(PDATA(pgm)->internalbuf, m->buf, n_bytes);	

	avrootloader_set_addr(pgm, addr >> 1); // fixme REALLY DIV2?

	PDATA(pgm)->maxdelay = m->max_write_delay * bufsize;
	while (written < n_bytes)
	{
		if ((written + bufsize) > n_bytes)
		{
			memset(buf, 0xff, bufsize);
			memcpy(buf, m->buf + written, n_bytes - written);
		}
		else
			memcpy(buf, m->buf + written, bufsize);
		
		avrootloader_send_cmd(pgm, CMD_WRITEFLASH, sizeof(buf), buf);

		written += bufsize;
		report_progress (written, n_bytes, NULL);
	}
//	free(PDATA(pgm)->internalbuf);


// The erase command 0x02 takes one parameter, which describes the num of
// pages to be erased, beginning at the address we currently are at.
// (i.e. the program end after flashing)
// But keep the reserved space for the bootloader in mind. Page size is the real
// pagesize of the chip.
// pages_to_be_erased = (chipsize - totally_written - bootloadersize) / page_size

	if (param_eraseprog > 0)
	{
		tmp = (m->size - written - (PDATA(pgm)->bootpages * page_size)) / page_size;
		PDATA(pgm)->maxdelay = m->max_write_delay * tmp * page_size;
		avrootloader_send_cmd(pgm, CMD_ERASEPAGES, sizeof(tmp), (unsigned char *) &tmp);
	}
	return n_bytes;
}

static int avrootloader_paged_write_eeprom(PROGRAMMER * pgm, AVRPART * p,
                                     AVRMEM * m, int page_size, int n_bytes)
{
  unsigned int written = 0;
  unsigned int bufsize = 0;

	if (n_bytes < (p->sram - page_size))
		while (bufsize < n_bytes)
			bufsize += page_size;
	else
		bufsize = (p->sram - page_size);

	if (bufsize == 0)
		bufsize = page_size;

	char buf[bufsize + 2];

	avrootloader_set_addr(pgm, m->offset); // fixme REALLY DIV2?
	PDATA(pgm)->maxdelay = (m->max_write_delay * n_bytes);

	while (written < n_bytes)
	{
		if ((written + bufsize) > n_bytes)
		{
			memset(buf, 0xff, bufsize);
			memcpy(buf, m->buf + written, n_bytes - written);
		}
		else
			memcpy(buf, m->buf + written, bufsize);
		
		avrootloader_send_cmd(pgm, CMD_WRITEE, sizeof(buf), buf);
		written += bufsize;
		report_progress (written, n_bytes, NULL);
	}

	return n_bytes;
}


static int avrootloader_paged_write(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m,
                              int page_size, int n_bytes)
{
  int rval = 0;

	if (PDATA(pgm)->use_blockmode == 0)
	{
		if (strcmp(m->desc, "flash") == 0)
			rval = avrootloader_paged_write_flash(pgm, p, m, page_size, n_bytes);
		else if (strcmp(m->desc, "eeprom") == 0)
			rval = avrootloader_paged_write_eeprom(pgm, p, m, page_size, n_bytes);
		else
			rval = -2;
	}
	
/*	if (PDATA(pgm)->use_blockmode == 1)
	{
		unsigned int addr = 0;
		unsigned int max_addr = n_bytes;
		char *cmd;
		unsigned int blocksize = PDATA(pgm)->buffersize;

		if (strcmp(m->desc, "flash") && strcmp(m->desc, "eeprom"))
			rval = -2;

		if (m->desc[0] == 'e')
			blocksize = 1;		// Write to eeprom single bytes only
		
		avrootloader_set_addr(pgm, addr);

		cmd = malloc(4 + blocksize);
		if (!cmd)
			rval = -1;
    
		cmd[0] = 'B';
		cmd[3] = toupper(m->desc[0]);

		while (addr < max_addr)
		{
			if ((max_addr - addr) < blocksize)
				blocksize = max_addr - addr;

			memcpy(&cmd[4], &m->buf[addr], blocksize);
			cmd[1] = (blocksize >> 8) & 0xff;
			cmd[2] = blocksize & 0xff;

			avrootloader_send(pgm, cmd, 4 + blocksize);
			// avrootloader_vfy_cmd_sent(pgm, "write block");

			addr += blocksize;

			report_progress (addr, max_addr, NULL);
		}

		free(cmd);
		rval = addr;
	}
	*/
	return rval;
}


static int avrootloader_paged_load(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m, 
                             int page_size, int n_bytes)
{
  unsigned int addr = 0;
  unsigned int written = 0;
  unsigned int bufsize = 0;
  char readeeprom[4] = {0x04, 0x00, 0x02, 0xc0};
  unsigned int i;
  unsigned short tmp;
  unsigned char crc[2] = {0, 0};


	if (strcmp(m->desc, "flash") == 0)
	{
		if (PDATA(pgm)->internalbuf == 0)
		{
			fprintf(stderr, " %s: avrootloader_read_byte_flash() Reading is not supported by this bootloader - only verify works\n", progname);
			exit(-1);
		}

		if (n_bytes < (p->sram - page_size))
			while (bufsize < n_bytes)
				bufsize += page_size;
		else
			bufsize = (p->sram - page_size);

		if (bufsize == 0)
			bufsize = page_size;
		
		char buf[bufsize + 2];

			avrootloader_set_addr(pgm, addr >> 1);

			memcpy(m->buf, PDATA(pgm)->internalbuf, n_bytes);
			
			PDATA(pgm)->maxdelay = 5000;

			while (written < n_bytes)
			{
				if ((written + bufsize) > n_bytes)
				{
					memset(buf, 0xff, bufsize);
					memcpy(buf, PDATA(pgm)->internalbuf + written, n_bytes - written);
				}
				else
					memcpy(buf, PDATA(pgm)->internalbuf + written, bufsize);
				
				avrootloader_send_cmd(pgm, CMD_VERIFYFLASH, sizeof(buf), buf);							
				
				written += bufsize;
				report_progress (written, n_bytes, NULL);
			}
			return n_bytes;
	}
	else if (strcmp(m->desc, "eeprom") == 0)
	{
  
		bufsize = avr_locate_mem(p, "flash")->page_size * 2;
 
		PDATA(pgm)->maxdelay = 5000;

		while (written < n_bytes)
		{
			avrootloader_send(pgm, readeeprom, sizeof(readeeprom));
			avrootloader_recv(pgm, m->buf + written, bufsize);
			avrootloader_recv(pgm, crc, sizeof(crc));
			
			tmp = 0;
			for (i = 0; i < bufsize; i++)
				tmp = calcCRC16r(tmp, m->buf[i + written], 0xa001);
			
			
			if (((tmp & 0xff) != crc[0]) || ((tmp >> 8) != crc[1]))
			{
				fprintf(stderr, "\navrootloader: Error in EEPROM CRC - please retry\n");
				exit(-1);
			}
			
			avrootloader_vfy_cmd_sent(pgm, "READ EEPROM");
			written += bufsize;
		}
	}
	return n_bytes;
}


/* Signature byte reads are always 3 bytes. */

static int avrootloader_read_sig_bytes(PROGRAMMER * pgm, AVRPART * p, AVRMEM * m)
{
	if (m->size < 3)
	{
		fprintf(stderr, "%s: memsize too small for sig byte read", progname);
		return -1;
	}
	
	m->buf[0] = PDATA(pgm)->sigbytes[0];
	m->buf[1] = PDATA(pgm)->sigbytes[1];
	m->buf[2] = PDATA(pgm)->sigbytes[2];
  
	return 3;
}

const char avrootloader_desc[] = "Hagen Reddmanns extended capabilities serial programmer";

void avrootloader_initpgm(PROGRAMMER * pgm)
{
	strcpy(pgm->type, "avrootloader");

	/*
	* mandatory functions
	*/
	pgm->initialize     = avrootloader_initialize;
	pgm->display        = avrootloader_display;
	pgm->enable         = avrootloader_enable;
	pgm->disable        = avrootloader_disable;
	pgm->program_enable = avrootloader_program_enable;
	pgm->chip_erase     = avrootloader_chip_erase;
	pgm->cmd            = avrootloader_cmd;
	pgm->open           = avrootloader_open;
	pgm->close          = avrootloader_close;

	/*
	* optional functions
	*/

	pgm->write_byte = avrootloader_write_byte;
	pgm->read_byte = avrootloader_read_byte;

	pgm->paged_write = avrootloader_paged_write;
	pgm->paged_load = avrootloader_paged_load;

	pgm->read_sig_bytes = avrootloader_read_sig_bytes;

	pgm->parseextparams = avrootloader_parseextparms;
	pgm->setup          = avrootloader_setup;
	pgm->teardown       = avrootloader_teardown;
}
