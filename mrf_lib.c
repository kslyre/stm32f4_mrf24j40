#include "mrf_lib.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "tm_stm32f4_delay.h"
#include <stdio.h>

static uint8_t mrf_rx_buf[127];
static uint8_t ignore_bytes = 0;

volatile uint8_t rx_flag;

static mrf_rx_info_t mrf_rx_info;

uint8_t mrf_read_short(uint8_t address) {
    mrf_select();
    // 0 top for short addressing, 0 bottom for read
    SPI_write(address<<1 & 0x7E);
    uint8_t res = SPI_write(0x0);
    mrf_deselect();
    return res;
}

uint8_t mrf_read_long(uint16_t address) {
    mrf_select();
    uint8_t ahigh = address >> 3;
    uint8_t alow = address << 5;
    SPI_write(0x80 | ahigh);  // high bit for long
    SPI_write(alow);
    uint8_t res = SPI_write(0);
    mrf_deselect();
    return res;
}

void mrf_write_short(uint8_t address, uint8_t data) {
    mrf_select();
    // 0 for top address, 1 bottom for write
    SPI_write((address<<1 & 0x7E) | 0x01); // 7f
    SPI_write(data);
    mrf_deselect();
}

void mrf_write_long(uint16_t address, uint8_t data) {
    mrf_select();
    uint8_t ahigh = address >> 3;
    uint8_t alow = address << 5;
    SPI_write(0x80 | ahigh);  // high bit for long
    SPI_write(alow | 0x10);  // last bit for write
    SPI_write(data);
    mrf_deselect();
}

/*
void mrf_write_short(uint8_t addr, uint8_t data){
	mrf_select();
	SPI_write(((addr<<1) & 0x7f) | 0x01);
	SPI_write(data);
	mrf_deselect();
}

uint8_t mrf_read_short(uint8_t addr){
	uint8_t data;
	mrf_select();
	SPI_write(((addr<<1) & 0x7e));
	data = SPI_write(0);
	mrf_deselect();
	return data;
}

void mrf_write_long(uint16_t addr, uint8_t data){
	mrf_select();
	SPI_write(((addr>>3) & 0x7f) | 0x80);
	SPI_write(((addr<<5) & 0xe0) | 0x1f);
	SPI_write(data);
	mrf_deselect();
}

uint8_t mrf_read_long(uint16_t addr){
	uint8_t data;
	mrf_select();
	SPI_write(((addr>>3) & 0x7f) | 0x80);
	SPI_write(((addr<<5) & 0xe0));
	data = SPI_write(0);
	mrf_deselect();
	return data;
}*/

uint8_t SPI_write(uint8_t data) {
    SPI2->DR = data;
	while ((SPI2->SR & SPI_SR_TXE) == 0)
        ;
  while ((SPI2->SR & SPI_SR_RXNE) == 0)
        ;
    return SPI2->DR;
	/*SPI_I2S_SendData(SPI2, data);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
	while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE) == RESET); 
  return SPI_I2S_ReceiveData(SPI2);*/
}

void mrf_select(void) {
  GPIO_LOW(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
	Delay(10);
}

void mrf_deselect(void) {
	Delay(10);
  GPIO_HIGH(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);
	Delay(10);
}

void mrf_reset(void) {
    GPIO_LOW(PORT_MRF_RESET, PIN_MRF_RESET);
    Delayms(10);
    GPIO_HIGH(PORT_MRF_RESET, PIN_MRF_RESET);
    Delayms(10);
}

void set_rx_flag(void)
{
	rx_flag = 0;
}

uint8_t get_rx_flag(void)
{
	return rx_flag;
}

mrf_rx_info_t get_rx_info(void)
{
	return mrf_rx_info;
}
	
uint8_t * get_rx_buf(void)
{
	return mrf_rx_buf;
}

void mrf_set_ignorebytes(uint8_t count) {
    ignore_bytes = count;
}

void mrf_set_interrupts(void) {
    mrf_write_short(MRF_INTCON, 0xF6);
}

void mrf_set_channel(char chan) {
	uint8_t channel;
	
	// indian code
	switch(chan)
	{
		case 12:
			channel = 0x13;
			break;
		case 13:
			channel = 0x23;
			break;
		case 14:
			channel = 0x33;
			break;
		case 15:
			channel = 0x43;
			break;
		case 16:
			channel = 0x53;
			break;
		case 17:
			channel = 0x63;
			break;
		case 18:
			channel = 0x73;
			break;
		case 19:
			channel = 0x83;
			break;
		case 20:
			channel = 0x93;
			break;
		case 21:
			channel = 0xA3;
			break;
		case 22:
			channel = 0xB3;
			break;
		case 23:
			channel = 0xC3;
			break;
		case 24:
			channel = 0xD3;
			break;
		case 25:
			channel = 0xE3;
			break;
		case 26:
			channel = 0xF3;
			break;
		default:
			channel = 0x03;
			break;
	}
  
	
	
	
	//mrf_write_long(MRF_RFCON0, chan<<4);
	mrf_write_long(MRF_RFCON0, channel);
	mrf_write_short(MRF_RFCTL, 0x04); //  Reset RF state machine.
  mrf_write_short(MRF_RFCTL, 0x00); //
	Delayms(1); 					// Delay at least 192usec
}

void mrf_init(void) {

    mrf_write_short(MRF_SOFTRST, 0x7);
    while (mrf_read_short(MRF_SOFTRST) & (0x7 != 0)) {
        ;
    }
    mrf_write_short(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
    mrf_write_short(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.

    mrf_write_long(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
    mrf_write_long(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
    mrf_write_long(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
    mrf_write_long(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
    mrf_write_long(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
    mrf_write_long(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
    mrf_write_long(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.
		
    //  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and Nonbeacon-Enabled Networks”):
    mrf_write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
    mrf_write_short(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
    mrf_write_short(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.
    mrf_set_interrupts();
    mrf_set_channel(CHAN11);
		
		//while((mrf_read_long(MRF_RFSTATE) & 0xa0) != 0xa0){};
    // Program the short MAC Address, 0xffff
    //load the short address of the device with 0xffff which means that it will be ignored upon receipt
    mrf_write_short(MRF_SADRL, 0x12);
    mrf_write_short(MRF_SADRH, 0x34);
    //load the pan address also with 0xffff;
    mrf_write_short(MRF_PANIDL, 0xff);
    mrf_write_short(MRF_PANIDH, 0xff);
 
	char i;
  //Program Long MAC Address
  for(i=0;i<8;i++)
  {
		mrf_write_short(MRF_EADR0+i,0x11);
  }
	
	mrf_write_short(MRF_RXMCR, 0x21);		// Promiscuous mode
}

void mrf_interrupt_handler(void) {
    uint8_t last_interrupt = mrf_read_short(MRF_INTSTAT);
    if (last_interrupt & MRF_I_RXIF) {
        rx_flag = 1;
        mrf_write_short(MRF_BBREG1, 0x04);  // RXDECINV - disable receiver
        uint8_t frame_length = mrf_read_long(0x300);  // read start of rxfifo

        uint16_t frame_control = mrf_read_long(0x301);
        frame_control |= mrf_read_long(0x302) << 8;
        mrf_rx_info.frame_type = frame_control & 0x07;
        mrf_rx_info.pan_compression = (frame_control >> 6) & 0x1;
        mrf_rx_info.ack_bit = (frame_control >> 5) & 0x1;
        mrf_rx_info.dest_addr_mode = (frame_control >> 10) & 0x3;
        mrf_rx_info.frame_version = (frame_control >> 12) & 0x3;
        mrf_rx_info.src_addr_mode = (frame_control >> 14) & 0x3;
        mrf_rx_info.sequence_number = mrf_read_long(0x303);

        for (int i = 0; i <= frame_length - 4; i++) {
            mrf_rx_buf[i] = mrf_read_long(0x304 + i);
        }
        mrf_rx_info.frame_length = frame_length - 3 - 2;
        mrf_rx_info.lqi = mrf_read_long(0x300 + frame_length + 1);
        mrf_rx_info.rssi = mrf_read_long(0x300 + frame_length + 2);

        mrf_write_short(MRF_BBREG1, 0x00);  // RXDECINV - enable receiver
    }
}									 

void setup_gpios(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = PIN_MRF_RESET;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_MRF_CHIPSELECT;
    GPIO_Init(PORT_MRF_CHIPSELECT, &GPIO_InitStructure);
    GPIO_HIGH(PORT_MRF_CHIPSELECT, PIN_MRF_CHIPSELECT);

    GPIO_InitStructure.GPIO_Pin = PIN_MRF_RESET; //PIN_MRF_CHIPSELECT;
    GPIO_Init(PORT_MRF_RESET, &GPIO_InitStructure);
    GPIO_LOW(PORT_MRF_RESET, PIN_MRF_RESET);
}

void setup_usart(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    
    GPIO_InitTypeDef usart;
    usart.GPIO_Pin = GPIO_Pin_2; /*| GPIO_Pin_3;*/
		usart.GPIO_PuPd = GPIO_PuPd_NOPULL;
    usart.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &usart);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
   // GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
    
    USART_ClockInitTypeDef usart_clocks;
    USART_ClockStructInit(&usart_clocks);
    usart_clocks.USART_Clock = USART_Clock_Enable;
    USART_ClockInit(USART2, &usart_clocks);
    
    USART_InitTypeDef usart_init;
    usart_init.USART_BaudRate = 115200; //57600; //9600;//115200; 
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_Mode = USART_Mode_Rx;
    USART_Init(USART2, &usart_init);
    USART_Cmd(USART2, ENABLE);
}

void setup_spi(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  
  //GPIO_PinAFConfig(PORT_SPI, GPIO_PinSource12, GPIO_AF_SPI2);
  GPIO_PinAFConfig(PORT_SPI, GPIO_PinSource13, GPIO_AF_SPI2);
  GPIO_PinAFConfig(PORT_SPI, GPIO_PinSource14, GPIO_AF_SPI2);
  GPIO_PinAFConfig(PORT_SPI, GPIO_PinSource15, GPIO_AF_SPI2);

  GPIO_InitTypeDef gpio;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
  gpio.GPIO_Speed = GPIO_Speed_25MHz;
  gpio.GPIO_Pin = /*GPIO_Pin_12 |*/ GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(PORT_SPI, &gpio);
  
  SPI_InitTypeDef spi;
  spi.SPI_Mode = SPI_Mode_Master;
  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi.SPI_DataSize = SPI_DataSize_8b;
  spi.SPI_NSS = SPI_NSS_Soft;
  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
  spi.SPI_CPOL = SPI_CPOL_Low;
  spi.SPI_CPHA = SPI_CPHA_1Edge;
  spi.SPI_FirstBit = SPI_FirstBit_MSB;
  
  SPI_Init(SPI2, &spi);
  SPI_Cmd(SPI2, ENABLE);
	
	SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
}

void setup_irq(void)
{
  GPIO_InitTypeDef mrf_irq;
  mrf_irq.GPIO_Mode = GPIO_Mode_IN;
  mrf_irq.GPIO_PuPd = GPIO_PuPd_NOPULL;
  //mrf_irq.GPIO_Pin = GPIO_Pin_11;
	mrf_irq.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOB, &mrf_irq);
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
  //  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource11);

    EXTI_InitTypeDef exti;
    exti.EXTI_Line = EXTI_Line3;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = EXTI3_IRQn;
  //nvic.NVIC_IRQChannel = USART2_IRQn;  //EXTI15_10_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
  nvic.NVIC_IRQChannelSubPriority = 0x01;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
}

void EXTI3_IRQHandler(void) {
//void EXTI15_10_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line3) != RESET) {
		mrf_interrupt_handler();
    EXTI_ClearITPendingBit(EXTI_Line3);
    EXTI_ClearFlag(EXTI_Line3);   
	}
}

void USART2_IRQHandler(void) {
  if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
    mrf_interrupt_handler();
    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
}
 
void mrf_start(void)
{
  setup_gpios();
  setup_usart();
  setup_spi();
  setup_irq();

  mrf_reset();
  mrf_deselect();

  mrf_init();
  mrf_set_ignorebytes(0);
	
	rx_flag = 0;
}
