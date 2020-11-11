#include "TEA.h"
#include <stdio.h>


#define MX                (z>>5^y<<2)+(y>>3^z<<4)^(sum^y)+(k[p&3^e]^z)
#define DELTA             0x9e3779b9
#define S_LOOPTIME        16        //5
#define BLOCK_SIZE        512         //PAGE_SIZE,根据你所要加密的数据包长度修改此参数(单位:字节)



uint32_t idAddr[]={ 0x1FFFF7AC,/*STM32F0唯一ID起始地址*/
                    0x1FFFF7E8,/*STM32F1唯一ID起始地址*/
                    0x1FFF7A10,/*STM32F2唯一ID起始地址*/
                    0x1FFFF7AC,/*STM32F3唯一ID起始地址*/ 
                    0x1FFF7A10,/*STM32F4唯一ID起始地址*/
                    0x1FF0F420,/*STM32F7唯一ID起始地址*/
                    0x1FF80050,/*STM32L0唯一ID起始地址*/
                    0x1FF80050,/*STM32L1唯一ID起始地址*/
                    0x1FFF7590,/*STM32L4唯一ID起始地址*/
                    0x1FF0F420
}; /*STM32H7唯一ID起始地址*/
 

//TEA密钥
unsigned char TEA_key[16]=
{ 
    0x01,0x05,0x03,0x04,0x05,0x06,0x07,0x08,
    0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10
};

//数据缓冲区
unsigned char TX_buffer[32];
unsigned char RX_buffer[32];

/*******************
  TEA加密解密算法
*******************/

/* 
*key  maybe 128bit =16 Bytes.
*buf  maybe BLOCK_SIZE
*/

void btea_encrypt( unsigned char* buf, unsigned char* key )
{
    unsigned char n=BLOCK_SIZE/4;
    unsigned long *v=(unsigned long *)buf;
    unsigned long *k=(unsigned long *)key;
    unsigned long z = v[n - 1],y = v[0],sum = 0,e ;
    unsigned char p,q ;
    // Coding Part 
    
    q = S_LOOPTIME + 52 / n ;
    while ( q-- > 0 )
    {
        sum += DELTA ;
        e = sum >> 2 & 3 ;
        for ( p = 0 ; p < n - 1 ; p++ )
          y = v[p + 1],
          z = v[p] += MX;
          y = v[0] ;
        z = v[n - 1] += MX;
    }
}


/*
*key  maybe 128bit =16Bytes.
*buf  maybe BLOCK_SIZE
inbuf == outbuf == buf
*/

void btea_decrpyt( unsigned char* buf, unsigned char* key )
{
    unsigned char n=BLOCK_SIZE/4;
    unsigned long *v=(unsigned long *)buf;
    unsigned long *k=(unsigned long *)key;
    unsigned long z = v[n - 1],y = v[0],sum = 0,e ;
    unsigned char  p,q ;
    
    //Decoding Part...
    q = S_LOOPTIME + 52 / n ;
    sum = q * DELTA ;
    while ( sum != 0 )
    {
        e = sum >> 2 & 3 ;
        for ( p = n - 1 ; p > 0 ; p-- )
            z = v[p - 1],
            y = v[p] -= MX;
            z = v[n - 1] ;
        y = v[0] -= MX;
        sum -= DELTA ;
    }
}
/* 获取MCU的唯一ID */

ChipID Get_ChipID(void)
{
  ChipID chipid = {0};
  
  chipid.id[0] = *(uint32_t *)(0x1FFF7A10 + 0x00);
  chipid.id[1] = *(uint32_t *)(0x1FFF7A10 + 0x04);
  chipid.id[2] = *(uint32_t *)(0x1FFF7A10 + 0x08);
  
  return chipid;
}

/*
int main(){
    unsigned char buffer[15] = "1234567890";
    btea_encrypt(buffer,TEA_key);
    printf("encrypt:%s\r\n",buffer);
    btea_decrpyt(buffer,TEA_key);
    printf("decrpyt:%s\r\n",buffer);
}
*/
