/******************** (C) COPYRIGHT 2018 merafour ********************
* Author             : 冷月追风@merafour.blog.163.com
* Version            : V2.0.0
* Date               : 25/2/2019
* Description        : TEA encryption algorithm.
********************************************************************************
* merafour.blog.163.com
* merafour@163.com
* github.com/Merafour
* gitee.com/merafour
*******************************************************************************/

#include "encrypt.h"
//#include "flash.h"
#include <string.h>
/**
* 该函数用于对 flash进行编程.
*/
extern int flash_program(const uint32_t word[], const uint32_t len, const uint32_t flash_start_addr, const uint32_t flash_end_addr);
/**
* 段定义,reset_rb2为加密相关代码所在段,reset_rb1用于计算段的其实地址,reset_rb3用于计算段的结束地址.
* 由于段会按照其命名规则顺序存放,因此查看 map文件会看到(注:以下所有地址由具体的编译链接决定):
    0x08000208   0x08000208   0x00000010   Code   RO         2664    .reset_rb1          encrypt.o
    0x08000218   0x08000218   0x000001bc   Code   RO         2665    .reset_rb2          encrypt.o
    0x080003d4   0x080003d4   0x00000010   Code   RO         2666    .reset_rb3          encrypt.o
	即通过将代码段 reset_rb1,reset_rb2,reset_rb3全部通过写0(擦除之后为0的写1)操作擦除代码,从而避免
	复杂的对 flash的自编程操作.
	最终,这三个段中的函数的地址如下:
    encryption_start                         0x08000209   Thumb Code    12  encrypt.o(.reset_rb1)
    encoding                                 0x08000219   Thumb Code   136  encrypt.o(.reset_rb2)
    UserLicense                              0x080002a1   Thumb Code   130  encrypt.o(.reset_rb2)
    _encryption                              0x08000323   Thumb Code   120  encrypt.o(.reset_rb2)
    encryption_end                           0x080003d5   Thumb Code    12  encrypt.o(.reset_rb3)
	从中我们看到我们加密的子入口函数 _encryption的位置仅在 encryption_end之前,因此写0(或写1)的时候
	仅需保留 encryption_end函数以及 _encryption函数的返回指令即可在加密代码被擦除后正常返回。
	encryption_start与 encryption_end可以不包含有意义的操作，但是不能被编译器优化掉。
* 以上主要通过段把相关代码集合到一起,然后利用链接器的相关规则让代码按照我们需要的顺序存放,从而以简
    单的方式获取加密部分代码的大小.
* 当然也可以使用上面的 encoding函数作为起始地址,但是当函数增加时未必能够确保 encoding函数放在段的最前面.
* 若想只定义一个段那必须充分了解连接器的链接规则,然后通过对函数特殊命名从而放在段的首尾.
*/
#define PASSWD_SEC_START __attribute__ ((section(".reset_rb1"))) 
#define PASSWD_SEC __attribute__ ((section(".reset_rb2"))) 
#define PASSWD_SEC_END __attribute__ ((section(".reset_rb3"))) 

#define     FLASH_ADDR      (0x08000000)
extern uint32_t __Vectors_Size;
extern uint32_t __Vectors[];
static uint32_t passwd[8*3+1+2];
static uint32_t passwd_addr;
/**
                __Vectors
				0       0xFFFF0000                        ; flag 1
				1       0xFFFFFFFF                        ; flag 2
				2       0x04db2615                        ; License
				3       0xFFFFFFFF                        ; passwd 1
...             4-10 B
				11     0xFFFFFFFF                        ; passwd 2
...             12-18 B
				19     0xFFFFFFFF                        ; passwd 3
...             20-26 B
以上是加密信息存储的位置,通过在中断向量表尾部增加数据,主要是0xFFFFFFFF(或0x00000000)用来占位:
                DCD     HASH_RNG_IRQHandler               ; Hash and Rng
                DCD     FPU_IRQHandler                    ; FPU
				DCD     0xFFFF0000                        ; flag 1
				DCD     0xFFFFFFFF                        ; flag 2
				DCD     0x04db2615                        ; License
				DCD     0xFFFFFFFF                        ; passwd 
				...
				然后通过对上述被占位区域写0(或写1)的方式写入加密数据.
*/
/**
 * 静态变量，防止栈溢出
*/
static uint32_t erase[512];
/**
* 定义用于覆盖代码的值,擦除之后为 0xFF请定义 ERASE0xFF,擦除之后为 0x00请定义 ERASE0x00.
* 通常flash在擦除之后为全1,写入数据实际上为是把 1编程为 0,如 STM32F1,F4.
* 经测试, STM32L0在擦除之后为全0，写入数据实际上是把 0编程为 1.
*/
#if defined  (ERASE0xFF)
const uint8_t erase_cover = 0x00;
#elif defined  (ERASE0x00)
const uint8_t erase_cover = 0xFF;
#else
 #error "Please define ERASE0xFF or ERASE0x00"
#endif

uint8_t get_flag(const uint8_t pos)
{
	volatile uint32_t index;
	volatile uint32_t _first;
	volatile uint32_t _end;
	volatile uint32_t _flag=0;
	volatile uint32_t i;
	_first = ((uint32_t)(&__Vectors_Size));
	_first = (_first-4*8*3-4*2)/4-3;
	_end = ((uint32_t)(&__Vectors_Size))/4;
	for(index=_first; index<_end; index++)
	{
		if(0xFFFF0000 == __Vectors[index])
		{
			passwd_addr = (uint32_t)(&__Vectors[index]);
			break;
		}
	}
	for(i=0; i<(sizeof(passwd)/4); i++)
	{
		passwd[i] = __Vectors[index+i];
	}
	_flag = passwd[1];
	_flag = (_flag>>pos)&0xFF;
	return _flag;
}

#define UID0    (0x63066cd9U)
#define UID1    (0x56b3c423U)
#define UID2    (0xc60cd9b2U)
//int erase_flag=0; // 通过该变量检测是否强行跳过了加密校验代码
PASSWD_SEC void encoding(uint32_t sign[8], volatile uint32_t uid[3])
{
	uint32_t order = 0x29d9c998; //(uint32_t)(&clock_setup.flash_config);
    sign[0] = (order&uid[1])|     ((~uid[0])&UID2);
    sign[1] = (order&uid[0])|     (uid[1]&(~UID2));
    sign[2] = uid[0]^UID1^order;
	sign[3] = uid[1]^((order+uid[0])|(~UID2));
	//printf("order0: %08X %08X %08X\r\n", (unsigned int)order, (unsigned int)uid[0], (unsigned int)uid[1]);
	order = order + (uid[0]&uid[1]);
	//printf("order1: %08X\r\n", (unsigned int)order);
	order = ((order<<12)&0xFFFFF000) | ((order>>12)&0x000FFFFF);
	//printf("order2: %08X\r\n", (unsigned int)order);
	sign[4] = (sign[0]^order)|     (uid[1]^(~sign[2]));
	sign[5] = (sign[0]|UID2) &     (sign[2]+order);  // 
	sign[6] = (~sign[0]|UID0)&     (sign[2]+order);   // 
	sign[7] = (sign[1]|UID1) &     (~(sign[2]+order)); // 
	//erase_flag = 1;
}

PASSWD_SEC void UserLicense(uint32_t sign[], volatile uint32_t uid[3], uint32_t License)
{
#define User_UID0    (0xc3037ea1U)
#define User_UID1    (0xb6a6831aU)
#define User_UID2    (0x2f6fb587U)
 //uint32_t *sign = (uint32_t *)(flash);
 //uint32_t License = 0x5cb36a04; //sign[40];
	//uint32_t License = 0x4ba34568U; //sign[13];
#if 0
		sign[0] = ((~uid[0])&uid[2]);
		sign[1] = (uid[1]&(~uid[2]));
		sign[2] = uid[0]^uid[1]^License;
		sign[3] = uid[0]^(sign[2]|(~License));
#else
    sign[0] = (uid[2]&uid[1])|((~uid[0])&uid[2]);
    sign[1] = (uid[2]&uid[0])|(uid[1]&(~uid[0]));
    sign[2] = User_UID0^License^uid[2];
		sign[3] = User_UID1^(License|(~uid[2]));
#endif
		sign[4] = (uid[0]+User_UID1+User_UID2) | License;
		//License = License + User_UID1&User_UID2 + sign[3];
		License = License + (User_UID1&User_UID2) + sign[3];
		License = ((License<<16)&0xFFFF0000) | ((License>>16)&0x0000FFFF);
		
		sign[5] = (sign[0]+License)|(User_UID1&(~uid[2]));
		sign[6] = (sign[1]+sign[4])|(User_UID2&(License));
		sign[7] = (sign[2]+sign[5])|(User_UID0&(~uid[2]));
		//match_flag = 1;
		//match_flag.skip1 = BOOT_COERCE_SKIP;
}
PASSWD_SEC_START void encryption_start(void)
{
	passwd[1] |= 0xFF;
}
PASSWD_SEC_END void encryption_end(void)
{
	passwd[1] |= 0xFF;
}
PASSWD_SEC void eaencoding(void)
{
	passwd[1] |= 0xFF;
}
PASSWD_SEC void ezencoding(void)
{
	passwd[1] |= 0xFF;
}
PASSWD_SEC void _encryption(uint32_t uid[3])
{
	uint32_t *sign;
	uint32_t *app_sign;
	uint32_t License;
	uint32_t encrypt_start;
	uint32_t encrypt_end;
	uint32_t encrypt_size;
	sign = (uint32_t *)(&passwd[3]);
	app_sign = (uint32_t *)(&passwd[8+3]);
	License = passwd[2];
	// erase License
	passwd[2] = erase_cover;
	passwd[2] = (passwd[2]<<8)|erase_cover;
	passwd[2] = (passwd[2]<<16)|erase_cover;
	passwd[2] = (passwd[2]<<24)|erase_cover;
	encryption_start();
	encryption_end();
	encoding(sign, uid);
	UserLicense(app_sign, uid, License);
	passwd[1] &= 0xFFFFFF00;
	passwd[1] |= 0x55;
	flash_program(passwd, sizeof(passwd)/4, passwd_addr, passwd_addr+512);
	encrypt_start = (uint32_t)(&encryption_start)&0xFFFFFFFC;
	encrypt_start += 4;
	encrypt_end = (uint32_t)(&encryption_end)&0xFFFFFFFC;
	/**
	* 这里需要保留返回指令,大小为 encryption_end函数的大小加上返回指令
	* 0x08为测试值.
	*/
	encrypt_size = encrypt_end-encrypt_start-0x08;
	memset(erase, erase_cover, sizeof(erase));
	flash_program(erase, encrypt_size, encrypt_start, encrypt_end);
	//HAL_NVIC_SystemReset();
}

void encryption(void (*read_uid)(uint32_t uid[3]))
{
	uint32_t uid[3];
	read_uid(uid);
	if(0x55!=get_flag(0))
	{
		_encryption(uid);
	}
}


