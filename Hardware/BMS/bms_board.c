#include "bms_board.h"
#include "bms_config.h"
#include "stm32f4xx.h"
#include <string.h>
#define RING 256u
typedef struct {volatile uint16_t head,tail;volatile bool overflow,busy;uint8_t rx[RING],tx[RING];uint32_t stamp[RING];volatile uint16_t txpos,txlen;} uart_t;
static uart_t serial[3];
static USART_TypeDef *const usart[3]={USART1,USART2,USART3};
static volatile uint32_t ticks,pre_start;
static volatile bool pre_on,pre_expired;
static void pin(GPIO_TypeDef *g,uint16_t p,GPIOMode_TypeDef mode,GPIOOType_TypeDef type,GPIOPuPd_TypeDef pull)
{GPIO_InitTypeDef x;GPIO_StructInit(&x);x.GPIO_Pin=p;x.GPIO_Mode=mode;x.GPIO_OType=type;x.GPIO_PuPd=pull;x.GPIO_Speed=GPIO_Speed_50MHz;GPIO_Init(g,&x);}
static void irq(uint8_t ch,uint8_t priority)
{NVIC_InitTypeDef n;n.NVIC_IRQChannel=ch;n.NVIC_IRQChannelPreemptionPriority=priority;n.NVIC_IRQChannelSubPriority=0;n.NVIC_IRQChannelCmd=ENABLE;NVIC_Init(&n);}
uint32_t board_ms(void){return ticks;}
uint16_t board_flash_kb(void){return *(const volatile uint16_t*)0x1fff7a22u;}
void board_us(uint32_t us){uint32_t t=TIM2->CNT;while((uint32_t)(TIM2->CNT-t)<us){}}
void SysTick_Handler(void)
{
    ticks++;
    if(pre_on&&ticks-pre_start>=BMS_PRECHARGE_HARD_MS){GPIO_ResetBits(GPIOC,GPIO_Pin_7);pre_on=false;pre_expired=true;}
}
void board_precharge(bool on)
{
    uint32_t primask=__get_PRIMASK();__disable_irq();
    if(on&&!pre_on&&!pre_expired){pre_start=ticks;pre_on=true;GPIO_SetBits(GPIOC,GPIO_Pin_7);}
    if(!on){GPIO_ResetBits(GPIOC,GPIO_Pin_7);pre_on=false;}
    if(!primask)__enable_irq();
}
bool board_precharge_expired(void){return pre_expired;}
void board_buzzer(bool on){GPIO_WriteBit(GPIOC,GPIO_Pin_9,on?Bit_RESET:Bit_SET);}
void board_esp_reset(bool reset){GPIO_WriteBit(GPIOC,GPIO_Pin_10,reset?Bit_RESET:Bit_SET);}
bool board_key(void){return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)==Bit_RESET;}
bool board_afe_alert(void){return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)!=Bit_RESET;}
void board_wake(bool on){GPIO_WriteBit(GPIOC,GPIO_Pin_6,on?Bit_SET:Bit_RESET);}
void board_emergency_off(void){GPIO_ResetBits(GPIOC,GPIO_Pin_7);pre_on=false;GPIO_SetBits(GPIOC,GPIO_Pin_5);}
static uint16_t scl(unsigned bus){(void)bus;return GPIO_Pin_6;}
static uint16_t sda(unsigned bus){(void)bus;return GPIO_Pin_7;}
static void line(uint16_t p,bool high){GPIO_WriteBit(GPIOB,p,high?Bit_SET:Bit_RESET);board_us(4);}
static bool clock_high(unsigned bus)
{uint32_t t=TIM2->CNT;GPIO_SetBits(GPIOB,scl(bus));while(!GPIO_ReadInputDataBit(GPIOB,scl(bus)))if((uint32_t)(TIM2->CNT-t)>1000u)return false;board_us(4);return true;}
static void stop(unsigned bus){line(scl(bus),false);line(sda(bus),false);(void)clock_high(bus);line(sda(bus),true);}
static bool start(unsigned bus)
{line(sda(bus),true);if(!clock_high(bus))return false;if(!GPIO_ReadInputDataBit(GPIOB,sda(bus)))return false;line(sda(bus),false);line(scl(bus),false);return true;}
static bool put(unsigned bus,uint8_t value)
{unsigned i;bool ack;for(i=0;i<8;i++){line(sda(bus),(value&0x80)!=0);if(!clock_high(bus))return false;line(scl(bus),false);value<<=1;}line(sda(bus),true);if(!clock_high(bus))return false;ack=!GPIO_ReadInputDataBit(GPIOB,sda(bus));line(scl(bus),false);return ack;}
static bool get(unsigned bus,uint8_t *value,bool ack)
{unsigned i;uint8_t v=0;line(sda(bus),true);for(i=0;i<8;i++){if(!clock_high(bus))return false;v=(uint8_t)((v<<1)|GPIO_ReadInputDataBit(GPIOB,sda(bus)));line(scl(bus),false);}line(sda(bus),!ack);if(!clock_high(bus))return false;line(scl(bus),false);line(sda(bus),true);*value=v;return true;}
bool board_i2c(unsigned bus,uint8_t address,const uint8_t *tx,unsigned nt,uint8_t *rx,unsigned nr)
{
    unsigned i;bool ok=false;if(bus!=0||(!nt&&!nr))return false;
    if(!start(bus)){/* Recover a stuck slave, never loop indefinitely. */for(i=0;i<9;i++){line(scl(bus),false);if(!clock_high(bus))break;}stop(bus);if(!start(bus))goto done;}
    if(nt){if(!put(bus,(uint8_t)(address<<1)))goto done;for(i=0;i<nt;i++)if(!put(bus,tx[i]))goto done;}
    if(nr){if(nt&&!start(bus))goto done;if(!put(bus,(uint8_t)((address<<1)|1)))goto done;for(i=0;i<nr;i++)if(!get(bus,&rx[i],i+1<nr))goto done;}
    ok=true;
done:stop(bus);return ok;
}
static bool adc_read(uint8_t channel,uint16_t *value)
{uint32_t t;ADC_RegularChannelConfig(ADC1,channel,1,ADC_SampleTime_480Cycles);ADC_ClearFlag(ADC1,ADC_FLAG_EOC);ADC_SoftwareStartConv(ADC1);t=TIM2->CNT;while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))if((uint32_t)(TIM2->CNT-t)>1000u)return false;*value=ADC_GetConversionValue(ADC1);return true;}
bool board_adc(uint16_t *pack_raw,uint16_t *ntc_raw)
{unsigned i;uint16_t a,b;uint32_t x=0,y=0;for(i=0;i<8;i++){if(!adc_read(ADC_Channel_0,&a)||!adc_read(ADC_Channel_1,&b))return false;x+=a;y+=b;}*pack_raw=(uint16_t)(x/8);*ntc_raw=(uint16_t)(y/8);return true;}
static void uart_irq(unsigned p)
{
    USART_TypeDef *u=usart[p];uart_t *q=&serial[p];uint32_t sr=u->SR;
    /* Read SR then DR exactly once: discard framing/noise/parity errors too,
       so a damaged byte cannot reach either command parser. */
    if(sr&(USART_SR_RXNE|USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE)){
        uint8_t v=(uint8_t)u->DR;
        if(sr&(USART_SR_ORE|USART_SR_NE|USART_SR_FE|USART_SR_PE))q->overflow=true;
        else {uint16_t h=(uint16_t)((q->head+1)&(RING-1));if(h!=q->tail){q->rx[q->head]=v;q->stamp[q->head]=ticks;q->head=h;}else q->overflow=true;}
    }
    if(USART_GetITStatus(u,USART_IT_TXE)){if(q->txpos<q->txlen)USART_SendData(u,q->tx[q->txpos++]);else{USART_ITConfig(u,USART_IT_TXE,DISABLE);USART_ITConfig(u,USART_IT_TC,ENABLE);}}
    if(USART_GetITStatus(u,USART_IT_TC)){USART_ITConfig(u,USART_IT_TC,DISABLE);if(p==1)GPIO_ResetBits(GPIOC,GPIO_Pin_12);q->busy=false;}
}
void USART1_IRQHandler(void){uart_irq(0);}void USART2_IRQHandler(void){uart_irq(1);}void USART3_IRQHandler(void){uart_irq(2);}
bool board_uart_send(unsigned p,const uint8_t *data,unsigned n)
{uart_t *q;if(p>2||!n||n>RING)return false;q=&serial[p];if(q->busy)return false;memcpy(q->tx,data,n);q->txlen=(uint16_t)n;q->txpos=0;q->busy=true;if(p==1)GPIO_SetBits(GPIOC,GPIO_Pin_12);USART_ClearFlag(usart[p],USART_FLAG_TC);USART_ITConfig(usart[p],USART_IT_TXE,ENABLE);return true;}
bool board_uart_get(unsigned p,uint8_t *v,uint32_t *stamp)
{uart_t *q;if(p>2)return false;q=&serial[p];if(q->head==q->tail)return false;*v=q->rx[q->tail];*stamp=q->stamp[q->tail];q->tail=(uint16_t)((q->tail+1)&(RING-1));return true;}
bool board_uart_overflow(unsigned p)
{bool v;uint32_t mask;if(p>2)return true;mask=__get_PRIMASK();__disable_irq();v=serial[p].overflow;serial[p].overflow=false;if(!mask)__enable_irq();return v;}
bool board_links_idle(void)
{unsigned i;for(i=0;i<3;i++)if(serial[i].busy||serial[i].head!=serial[i].tail)return false;return (CAN1->TSR&CAN_TSR_TME)==CAN_TSR_TME;}
void board_links_standby(bool standby)
{GPIO_WriteBit(GPIOC,GPIO_Pin_5,standby?Bit_SET:Bit_RESET);if(standby)GPIO_ResetBits(GPIOC,GPIO_Pin_12);/* No RS485 power switch exists on this board. */}
bool board_can_receive(uint8_t data[8])
{
    CanRxMsg m;unsigned n=3;
    if(CAN1->RF0R&CAN_RF0R_FOVR0){
        while(n--&&CAN_MessagePending(CAN1,CAN_FIFO0))CAN_FIFORelease(CAN1,CAN_FIFO0);
        CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);return false;
    }
    if(!CAN_MessagePending(CAN1,CAN_FIFO0))return false;
    CAN_Receive(CAN1,CAN_FIFO0,&m);
    if(m.IDE!=CAN_Id_Standard||m.RTR!=CAN_RTR_Data||m.StdId!=0x321||m.DLC!=8)return false;
    memcpy(data,m.Data,8);return true;
}
bool board_can_send(uint16_t id,const uint8_t data[8])
{CanTxMsg m;if((CAN1->TSR&CAN_TSR_TME)==0)return false;memset(&m,0,sizeof(m));m.StdId=id;m.IDE=CAN_Id_Standard;m.RTR=CAN_RTR_Data;m.DLC=8;memcpy(m.Data,data,8);return CAN_Transmit(CAN1,&m)!=CAN_TxStatus_NoMailBox;}
void board_watchdog_start(void){IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);IWDG_SetPrescaler(IWDG_Prescaler_64);IWDG_SetReload(2000);IWDG_ReloadCounter();IWDG_Enable();}
void board_watchdog_feed(void){IWDG_ReloadCounter();}
void board_init(void)
{
    USART_InitTypeDef u;ADC_InitTypeDef a;ADC_CommonInitTypeDef ac;CAN_InitTypeDef c;CAN_FilterInitTypeDef f;RCC_ClocksTypeDef clocks;uint32_t timer_hz;unsigned i;
    SystemCoreClockUpdate();NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOG,ENABLE);
    /* Visible before LCD reset/ID waits and any optional peripheral probing. */
    GPIO_ResetBits(GPIOG,GPIO_Pin_15);pin(GPIOG,GPIO_Pin_15,GPIO_Mode_OUT,GPIO_OType_PP,GPIO_PuPd_NOPULL);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);RCC_GetClocksFreq(&clocks);
    timer_hz=clocks.PCLK1_Frequency;if((RCC->CFGR&RCC_CFGR_PPRE1)!=0)timer_hz*=2;
    TIM2->CR1=0;TIM2->PSC=timer_hz/1000000u-1;TIM2->ARR=0xffffffffu;TIM2->EGR=1;TIM2->CNT=0;TIM2->CR1=1;
    GPIO_SetBits(GPIOC,GPIO_Pin_5|GPIO_Pin_9|GPIO_Pin_10);GPIO_ResetBits(GPIOC,GPIO_Pin_6|GPIO_Pin_7);GPIO_ResetBits(GPIOC,GPIO_Pin_12);
    pin(GPIOC,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9,GPIO_Mode_OUT,GPIO_OType_PP,GPIO_PuPd_NOPULL);
    pin(GPIOC,GPIO_Pin_10,GPIO_Mode_OUT,GPIO_OType_OD,GPIO_PuPd_NOPULL);
    pin(GPIOC,GPIO_Pin_4|GPIO_Pin_8|GPIO_Pin_11,GPIO_Mode_IN,GPIO_OType_PP,GPIO_PuPd_UP);
    pin(GPIOC,GPIO_Pin_12,GPIO_Mode_OUT,GPIO_OType_PP,GPIO_PuPd_NOPULL);
    GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);pin(GPIOB,GPIO_Pin_6|GPIO_Pin_7,GPIO_Mode_OUT,GPIO_OType_OD,GPIO_PuPd_NOPULL);
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CYCCNT=0;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    SysTick_Config(SystemCoreClock/1000u);NVIC_SetPriority(SysTick_IRQn,0);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_USART1,ENABLE);RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2|RCC_APB1Periph_USART3|RCC_APB1Periph_CAN1,ENABLE);
    pin(GPIOA,GPIO_Pin_0|GPIO_Pin_1,GPIO_Mode_AN,GPIO_OType_PP,GPIO_PuPd_NOPULL);
    ADC_CommonStructInit(&ac);ac.ADC_Prescaler=ADC_Prescaler_Div4;ADC_CommonInit(&ac);ADC_StructInit(&a);ADC_Init(ADC1,&a);ADC_Cmd(ADC1,ENABLE);
    pin(GPIOA,GPIO_Pin_9|GPIO_Pin_10,GPIO_Mode_AF,GPIO_OType_PP,GPIO_PuPd_UP);GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);
    pin(GPIOA,GPIO_Pin_2|GPIO_Pin_3,GPIO_Mode_AF,GPIO_OType_PP,GPIO_PuPd_UP);GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
    pin(GPIOB,GPIO_Pin_10|GPIO_Pin_11,GPIO_Mode_AF,GPIO_OType_PP,GPIO_PuPd_UP);GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);
    USART_StructInit(&u);u.USART_BaudRate=115200;
    for(i=0;i<3;i++){USART_Init(usart[i],&u);USART_ITConfig(usart[i],USART_IT_RXNE,ENABLE);USART_Cmd(usart[i],ENABLE);}irq(USART1_IRQn,3);irq(USART2_IRQn,2);irq(USART3_IRQn,3);
    pin(GPIOA,GPIO_Pin_11|GPIO_Pin_12,GPIO_Mode_AF,GPIO_OType_PP,GPIO_PuPd_UP);GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1);
    CAN_StructInit(&c);c.CAN_ABOM=ENABLE;c.CAN_NART=ENABLE;c.CAN_Prescaler=6;c.CAN_BS1=CAN_BS1_11tq;c.CAN_BS2=CAN_BS2_2tq;
    if(CAN_Init(CAN1,&c)==CAN_InitStatus_Success){memset(&f,0,sizeof(f));f.CAN_FilterMode=CAN_FilterMode_IdMask;f.CAN_FilterScale=CAN_FilterScale_32bit;f.CAN_FilterIdHigh=(uint16_t)(0x321u<<5);f.CAN_FilterMaskIdHigh=0xffe0;f.CAN_FilterMaskIdLow=6;f.CAN_FilterActivation=ENABLE;CAN_FilterInit(&f);GPIO_ResetBits(GPIOC,GPIO_Pin_5);}
}
