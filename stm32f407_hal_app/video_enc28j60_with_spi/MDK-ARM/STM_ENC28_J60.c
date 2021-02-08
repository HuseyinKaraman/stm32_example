#include "STM_ENC28_J60.h"

extern SPI_HandleTypeDef hspi1; // sonradan eklendi // bunun olmasi sart ayni confg i aliyoruz...
static uint8_t Enc28_Bank;  // ekledik...
uint8_t dataWatch8;
uint16_t dataWatch16;

// h dosyasindaki tüm fonskiyonlari ekledik ve yukariya h i bildirdik..
// burda içerigini tanimlayacagiz ::::

uint8_t ENC28_readOp(uint8_t oper,uint8_t addr) // okumak için fonksiyon tanimladik .
{
uint8_t spiData[2];
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET); // cs=0
// spiData[0] = (oper| (addr & ADDR_MASK)); // açiklamasi : sayfa 28 de spi komutlari ie ilgili...
spiData[0] =((oper<<5)&0xE0)|(addr & ADDR_MASK); // yukardaki pital ve write da aynsini yaptik 
	// nedennnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn
	// birlestirme yaptik ilk 3bit(oper)/5bit(adres); 
	
HAL_SPI_Transmit(&hspi1,spiData,1,100); // okumak istedigimiz adres degeri gitti
if(addr & 0x80) // neden 2 mask ??????????????????????????????
	{
		//2 kez çagirmamiz gerekiyormus.... receive i
		HAL_SPI_Receive(&hspi1, &spiData[1], 1, 100);
	}
HAL_SPI_Receive(&hspi1,&spiData[1],1,100); // veri aliyoruz...
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET); // cs=1

return spiData[1];  // okudugumuz degeri gönderiyoruz..
}

void ENC28_writeOp(uint8_t oper,uint8_t addr,uint8_t data) // yazmak için fonksiyon tanimladik/tek farki herhangi parametre döndürmeyecak olmasi.
{// geri veri dönmedigi için void

uint8_t spiData[2];
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET); // cs=0
//spiData[0]=(oper<<5)|addr;  // açiklamasi : sayfa 28 de spi komutlari ie ilgili...birlestirme yaptik..ilk 3 bit yazma komutu son 3 bit adres. sonraki gelen ise veri(spidata[1]) 
	spiData[0] = ((oper<<5)&0xE0)|(addr & ADDR_MASK); //(oper| (addr & ADDR_MASK));
	// nedennnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn
	spiData[1]=data;	
HAL_SPI_Transmit(&hspi1,spiData,2,100); // yazmak istedigimiz adres degerine datayi yolladik.
HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET); // cs=1

}


uint8_t ENC28_readReg8(uint8_t addr) 
{// bu fonksiyonlar ile oper i bildirmeme gerek kalmaz

ENC28_setBank(addr); 
return ENC28_readOp(ENC28_READ_CTRL_REG,addr); 
// ENC28_READ_CTRL_REG .h dosyasinda tanimli. ERDPTL adresi 0x00 okuma yaptigimiz reg.
// ENC28_READ_CTRL_REG 000aaaaa     , a=control register address
}
	
void ENC28_writeReg8(uint8_t addr,uint8_t data)
{
ENC28_setBank(addr);
ENC28_writeOp(ENC28_WRITE_CTRL_REG,addr,data);
// ENC28_WRITE_CTRL_REG .h dosyasinda tanimli. EWRPTL bu register a data yazabiliriz.
// ENC28_WRITE_CTRL_REG 0x02 => 010aaaaa dddddddd
}

uint16_t ENC28_readReg16(uint8_t addr) // bu fonksiyonlar ile oper i bildirmeme gerek kalmaz
{
//	uint16_t data16;
//	data16= ENC28_readReg8(addr); // 8 bit ile adresten degeri okuduk.// low bits
//  data16+=(ENC28_readReg8(addr+1)<<8); //bir sonraki adresten  high bits leri okuduk. ve 8 bit kaydirdik
//	// ve low high bitleri birlestirdik.
//	return data16;
	return ENC28_readReg8(addr) + (ENC28_readReg8(addr+1) << 8);
}

void ENC28_writeReg16(uint8_t addrL,uint16_t data)
{
//ENC28_writeReg8(addrL,data&0xFF);  // low bits  //   0xFF=1111 1111 (ilk 8 bit sifir)
//ENC28_writeReg8(addrL+1,(data>>8)&0xFF); // high bits  //  ilk 8 biti son 8 e kaydirdik..

	ENC28_writeReg8(addrL, data);
	ENC28_writeReg8(addrL+1, data >> 8);
}


void ENC28_setBank(uint8_t addr)  // istenen bank a gitmemizi saglayacak..
{// 4 bank mevcut 
	/*  BSEL1:BSEL0: Bank Select bits // ECON1: ETHERNET CONTROL REGISTER 1 
      11 = SPI accesses registers in Bank 3
			10 = SPI accesses registers in Bank 2
			01 = SPI accesses registers in Bank 1
			00 = SPI accesses registers in Bank 0  */
//	if((addr & BANK_MASK)!=Enc28_Bank) // addr ile bank_mask in AND lenmesi o adresin bankini verir.
//	{
//		ENC28_writeOp(ENC28_BIT_FIELD_CLR,ECON1,ECON1_BSEL1|ECON1_BSEL0);
//		ENC28_writeOp(ENC28_BIT_FIELD_SET,ECON1,Enc28_Bank>>5);// bsel1-bsel0
//		Enc28_Bank= addr & BANK_MASK; // suanki banki atadik.. eger if e ayni bank gelirse degisiklik yapmiyoruz.
//	 // The Bit Field Set (BFS) command is used to set up to
//  // 8 bits in any of the ETH Control registers.
//	}
	
	if ((addr & BANK_MASK) != Enc28_Bank) 
	{
		ENC28_writeOp(ENC28_BIT_FIELD_CLR, ECON1, ECON1_BSEL1|ECON1_BSEL0);
		Enc28_Bank = addr & BANK_MASK;
    ENC28_writeOp(ENC28_BIT_FIELD_SET, ECON1, Enc28_Bank>>5);
	}
}

void ENC28_Init(void) 
{
	// (1): Disable the chip CS pin
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET); // cs=1
	HAL_Delay(1);
  // (2): Perform soft reset to the ENC28J60 module
 // ENC28_writeOp(ENC28_SOFT_RESET,0x1F,0x00);  // sayfa 28 de en son gönderilen bitler..
		ENC28_writeOp(ENC28_SOFT_RESET, 0, ENC28_SOFT_RESET); // yukardakini degistik.
	// onun kodlarinda farkli...... System Reset Command (Soft Reset)************************
	//  a = control register address = 0x1F
	HAL_Delay(2);
	
	// (3) : wait until clock is ready 
	// syf :66 =>  estat reg.. estat_clkrdy 0.bit
	/*CLKRDY: Clock Ready
		1= OST has expired; PHY is ready
		0= OST is still counting; PHY is not ready */ 
	while(!ENC28_readOp(ENC28_READ_CTRL_REG,ESTAT)& ESTAT_CLKRDY); // son bit 1 ise clk hazir dir !
	
	
	// (4): Initialize RX and TX buffer size
	/*The Ethernet buffer contains transmit and receive
memory used by the Ethernet controller. The entire
buffer is 8 Kbytes, divided into separate receive and
transmit buffer spaces. */ // syf :19
	ENC28_writeReg16(ERXST,RXSTART_INIT);    // datalar neye göre yazildi ??
	ENC28_writeReg16(ERXND,RXSTOP_INIT);
	
	ENC28_writeReg16(ETXST,TXSTART_INIT);
	ENC28_writeReg16(ETXND,TXSTOP_INIT);
	
	ENC28_writeReg16(ERXRDPT,RXSTART_INIT); // sifirladik
	ENC28_writeReg16(ERXWRPT,RXSTART_INIT); // sifirladik 
	
	dataWatch16 = ENC28_readReg16(ERXND);
	
	//(5): Receive buffer filters syf:50
	ENC28_writeReg8(ERXFCON,ERXFCON_UCEN|ERXFCON_ANDOR|ERXFCON_CRCEN);
	// 11100000 bitini atadik.. sadece yerel mac adresini kabul edecek açiklamalar syf 50 de..
	dataWatch8 = ENC28_readReg8(ERXFCON);
	// (6) MAC Control Register 1 syf:36 //MAC Initialization Settings 6.5 syf : 36 'da..
	ENC28_writeReg8(MACON1,MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS|MACON1_PASSALL);	
	// eger Tx,Rx paus un ne oldugunu bilmiyorsan google dan bak..
	//Flow Control – Pause Frames diye bakabilirsin 
	dataWatch8 = ENC28_readReg8(ERXFCON);
	// (7) MAC Control Register 3 syf:37
	ENC28_writeOp(ENC28_BIT_FIELD_SET,MACON3,MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
  
	// (8): NON/Back to Back gap syf:36
	ENC28_writeReg16(MAIPG,0x0C12); // NonBackToBack gap  6. 7. madde
	ENC28_writeReg8(MABBIPG,0x12); // BackToBack gap 5. madde
 
 //(9): Set Maximum framelenght 
	ENC28_writeReg16(MAMXFL,MAX_FRAMELEN); // set maximum frame lengt (any packet bigger will be discarded)
 
 //(10): Set the MAC address of the device
	ENC28_writeReg8(MAADR6,MAC_6);
	ENC28_writeReg8(MAADR5,MAC_5);
	ENC28_writeReg8(MAADR4,MAC_4);
	ENC28_writeReg8(MAADR3,MAC_3);
	ENC28_writeReg8(MAADR2,MAC_2);
	ENC28_writeReg8(MAADR1,MAC_1);


  dataWatch8 = ENC28_readReg8(MAADR1);
	dataWatch8 = ENC28_readReg8(MAADR2);
	dataWatch8 = ENC28_readReg8(MAADR3);
	dataWatch8 = ENC28_readReg8(MAADR4);
	dataWatch8 = ENC28_readReg8(MAADR5);
	dataWatch8 = ENC28_readReg8(MAADR6);
	
	if(dataWatch8==MAC_6)HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
/* ***********************Advanced Initilisations*************************** */
}
