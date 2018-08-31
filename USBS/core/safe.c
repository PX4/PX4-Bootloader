/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V1.0.0
* Date               : 30/8/2018
* Description        : safe connect.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
*******************************************************************************/
/************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *   Copyright (c) 2010 libopencm3 project
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * LICENSE NOTE FOR EXTERNAL LIBOPENCM3 LIBRARY:
 *
 *   The PX4 development team considers libopencm3 to be
 *   still GPL, not LGPL licensed, as it is unclear if
 *   each and every author agreed to the LGPS -> GPL change.
 *
 ***********************************************************************/

/*
 * SAFE interface for the bootloader.
 */

#include "usbs_hw_config.h"
#include "safe.h"
#include <string.h>
//#include "cdcacm.h"
#include "uart.h"


#define PACK_LEN    210
#define PACK_HEAD   0x7E
    struct  pack
    {
         uint8_t head;
         uint8_t len;
         uint8_t count;
         uint8_t data[PACK_LEN-5];
         uint16_t crc16;
         uint8_t recv;
    }_pack;
    char recv_buf[512];
    uint16_t recv_buf_len=0;
    uint8_t pack_en[PACK_LEN];
    uint8_t recv_count=0;
    uint8_t recv_last=0;
    //uint8_t recv_data[128];
    //uint16_t recv_len=0;
    //uint16_t recv_rindex=0;
    uint8_t bl_buf[64];
	uint8_t bl_buf_len=0;
static unsigned int head, tail;
static uint8_t rx_buf[512];
void safe_buf_put(uint8_t b)
{
	//uart_cout(&b, 1);
	unsigned next = (head + 1) % sizeof(rx_buf);
	if (next != tail) {
		rx_buf[head] = b;
		head = next;
	}
	/*b = (uint8_t)head;
	uart_cout(&b, 1);
	b = (uint8_t)tail;
	uart_cout(&b, 1);*/
}
int safe_buf_get(void)
{
	//uint8_t b=0;
	int	ret = -1;
	if (tail != head) {
		ret = rx_buf[tail];
		tail = (tail + 1) % sizeof(rx_buf);
		/*b = (uint8_t)ret;
		uart_cout(&b, 1);*/
	}
	return ret;
}

int decode(struct pack *_p, const char _buf[], const uint16_t lenght);  // 解码
uint16_t crc16(const uint8_t pack[], const uint16_t lenght);

void safe_init(void)
{
	memset(recv_buf, 0, sizeof(recv_buf));
	memset(pack_en, 0, sizeof(pack_en));
	memset(bl_buf, 0, sizeof(bl_buf));
	memset(&_pack, 0, sizeof(_pack));
	recv_buf_len=0;
	recv_count=0;
	recv_last=0;
	//recv_len=0;
	//recv_rindex=0;
	head = 0;
	tail = 0;
	bl_buf_len=0;
}

uint8_t* get_pack(void)
{
    return pack_en;
}

void recvSerialData(const uint8_t data)
{
    uint16_t count=0;
    recv_buf[recv_buf_len++] = data;
	if(recv_buf_len>=(sizeof(recv_buf)-1)) recv_buf_len=0;
    if(0==decode(&_pack, recv_buf, recv_buf_len))
    {
		//usb_cout((uint8_t *)recv_buf, 7);
		//uart_cout((uint8_t *)recv_buf, 7);
		memset(recv_buf, 0, sizeof(recv_buf));
		recv_buf_len = 0;
        //count = recv_last;
		count = recv_count;
        count++;
        if(count>255) count=1;
        if((count==_pack.count) || (0==_pack.count))
        {
            //memcpy(recv_data, _pack.data, _pack.len);
            //recv_len = _pack.len;
			//recv_rindex=0;
			for(count=0; count<_pack.len; count++) safe_buf_put(_pack.data[count]);
            recv_last = recv_count;  // update last
            recv_count = _pack.count;
			if(0==_pack.count) recv_last=0;
			//uart_cout((uint8_t *)&recv_len, sizeof(recv_len));
			//uart_cout((uint8_t *)&recv_rindex, sizeof(recv_rindex));
			//uart_cout((uint8_t *)&recv_last, sizeof(recv_last));
			//ACK
			uint8_t buf=0;
			int _size = encode(&buf, 0, 0x01);
			uart_cout(get_pack(), _size);
            return;
        }
		else if(_pack.count==recv_count)
		{
			uint8_t buf=0;
			int _size = encode(&buf, 0, 0x02);
			uart_cout(get_pack(), _size);
			_size = encode(bl_buf, bl_buf_len, 0);
			uart_cout(get_pack(), _size);
		}
		else
		{
			//ACK
			uint8_t buf=0;
			int _size = encode(&buf, 0, 0x10);
			uart_cout(get_pack(), _size);
		}
    }
}
/*int readSerialData(void)
{
    int	uart_in;
    uart_in = -1;
    if(recv_rindex<recv_len)
    {
		//uart_cout((uint8_t *)&recv_len, sizeof(recv_len));
		//uart_cout((uint8_t *)&recv_rindex, sizeof(recv_rindex));
        uart_in = recv_data[recv_rindex++];
    }
    return uart_in;
}*/
int decode(struct pack *_p, const char _buf[], const uint16_t lenght)  // 解码
{
#define DECODE_HEAD_LEN    3
    /**
   | 1B | 1B|  1B | xB | 2B |
   |0x7E|len|count|DATA|crc|
     */
    uint16_t i=0;
    uint16_t crc=0;
    const char *_data=NULL;
    for(i=0; i<lenght; i++)
    {
        if(PACK_HEAD == _buf[i])
        {
            _data = &_buf[i];
            _p->head = _data[0];
            _p->len  = _data[1];
            _p->count = _data[2];
            if((_p->len+i+DECODE_HEAD_LEN+2) > lenght) return -1;
            memcpy(_p->data, &_data[DECODE_HEAD_LEN], _p->len);
            crc = _data[DECODE_HEAD_LEN+_p->len]&0xFF;
            crc = (crc << 8) + (_data[DECODE_HEAD_LEN+_p->len+1]&0xFF);
            _p->crc16 = crc;
            crc = crc16((const uint8_t*)&_data[0], DECODE_HEAD_LEN+_p->len);
            if(crc != _p->crc16) return -2;
            _p->recv = 1;
            return 0;
        }
    }
    return -3;
}
void save_bl_ack(const uint8_t _data[], const uint8_t lenght)
{
	bl_buf_len = lenght;
	if(bl_buf_len>=(sizeof(bl_buf))) bl_buf_len=(sizeof(bl_buf)-1);
	memcpy(bl_buf, _data, bl_buf_len);
}
int encode(const uint8_t _data[], const uint8_t lenght, const uint8_t status)  // 编码
{
#define ENCODE_HEAD_LEN    5
    /**
  ack:
   | 1B | 1B|  1B |  1B  |  1B  | xB |crc|
   |0x7E|len|count| last |status|DATA|crc|
     */
    uint16_t crc = 0;
    memset(pack_en, 0, sizeof(pack_en));
    pack_en[0] = PACK_HEAD;
    pack_en[1] = lenght;
    pack_en[2] = recv_count;
    pack_en[3] = recv_last;
    pack_en[4] = status;
    memcpy(&pack_en[ENCODE_HEAD_LEN], _data, lenght);
    crc = crc16(pack_en, lenght+ENCODE_HEAD_LEN);
    pack_en[ENCODE_HEAD_LEN+lenght] = (crc>>8)&0xFF;
    pack_en[ENCODE_HEAD_LEN+lenght+1] = (crc)&0xFF;

	//usb_cout((uint8_t *)pack_en, (lenght+ENCODE_HEAD_LEN+2));
    return (lenght+ENCODE_HEAD_LEN+2);
}

uint16_t crc16(const uint8_t pack[], const uint16_t lenght)
{
    uint16_t crc=0;
    uint16_t sum=0;
    uint16_t _len=0;
    uint16_t i=0;

    _len=lenght - (lenght&0x1); // 2字节对对齐
    sum = 0;
    for(i=0; i<_len; i+=2)
    {
        crc = pack[i];
        crc = (crc<<8) + pack[i+1];
        sum += crc;
    }
    if(lenght&0x1)
    {
        crc = pack[i];
        crc = (crc<<8) + 0xFF;
        sum += crc;
    }
    return sum;
}


