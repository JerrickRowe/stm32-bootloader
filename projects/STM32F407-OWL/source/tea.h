#ifndef __TEA_h__
#define __TEA_h__

#include <stdint.h>
// TEA加密函数
void btea_encrypt(unsigned char* buf, unsigned char* key);
// TEA解密函数
void				 btea_decrpyt(unsigned char* buf, unsigned char* key);
extern unsigned char TEA_key[16];

typedef struct {
	uint32_t id[3];
} ChipID;

ChipID Get_ChipID(void);
#endif
