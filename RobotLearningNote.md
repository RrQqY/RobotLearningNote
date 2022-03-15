# Robot Learning Note

此文档为本人在机器人学习过程中遇到的各种知识模块的笔记集合。

## STM32

### 固件库

#### 外设的初始化和设置

1. 调用以下一个函数来使能外设的时钟：

   ```c
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_PPPx, ENABLE); 
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_PPPx, ENABLE); 
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PPPx, ENABLE); 
   ```

   <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\image-20220201210713643.png" alt="image-20220201210713643" style="zoom:67%;" />

2. 声明一个初始化结构 PPP_InitTypeDef

   ```c
   PPP_InitTypeDef PPP_InitStructure; 
   ```

   这里 PPP_InitStructure 是一个位于内存中的工作变量，用来初始化一个或者多个外设 PPP。

3. 为变量 PPP_InitStructure 的各个结构成员填入允许的值。可以采用以下 2 种方式：

   a）设置整个结构体：

   ```c
   PPP_InitStructure.member1 = val1; 
   PPP_InitStructure.member2 = val2; 
   PPP_InitStructure.memberN = valN; 
   // 以上步骤可以合并在同一行里，用以优化代码大小：
   PPP_InitTypeDef PPP_InitStructure = { val1, val2,.., valN} 
   ```

   b）仅设置结构体中的部分成员：

   这种情况下，用户应当首先调用函数 PPP_SturcInit(..)来初始化变量PPP_InitStructure，然后再修改其中需要修改的成员。这样可以保证其他成员的值（多为缺省值）被正确填入。

   ```c
   PPP_StructInit(&PPP_InitStructure); 
   PP_InitStructure.memberX = valX; 
   PPP_InitStructure.memberY = valY; 
   ```

4. 调用函数 PPP_Init(..)来初始化外设 PPP。

5. 调用函数 PPP_Cmd(..)来使能之，然后可以通过调用一系列函数来使用外设。

   ```c
   PPP_Cmd(PPP, ENABLE); 
   ```

- 可以调用函数 PPP_Deinit(..)来把外设 PPP 的所有寄存器复位为缺省值：

  ```c
  PPP_DeInit(PPP);
  ```

#### GPIO

- **GPIO_Init**：初始化外设 GPIOx 寄存器

  GPIO_InitTypeDef 定义如下：

  ```c
  typedef struct { 
  	u16 GPIO_Pin; 
  	GPIOSpeed_TypeDef GPIO_Speed; 
  	GPIOMode_TypeDef GPIO_Mode; 
  } GPIO_InitTypeDef; 
  ```

  - GPIO Pin：GPIO管脚

  - GPIO Speed：管脚速率，可以取值：GPIO_Speed_10MHz；GPIO_Speed_2MHz；GPIO_Speed_50MHz。

  - GPIO_Mode：管脚工作状态，可以取值：

    <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\image-20220201210824302.png" alt="image-20220201210824302" style="zoom:67%;" />

  ```c
  // 初始化一个GPIO管脚示例如下
  #define GPIO_LED_PIN_Green   GPIO_Pin_0                // GPIO管脚
  #define GPIO_LED_PORT_Green  GPIOB                     // GPIO PORT
  #define GPIO_LED_CLK_Green   RCC_APB2Periph_GPIOB      // GPIO时钟
  
  RCC_APB2PeriphClockCmd(GPIO_LED_CLK_Green, ENABLE);    // 1、打开时钟
  	
  GPIO_InitTypeDef GPIO_InitStruct_Green;    // 2、定义初始化结构体
  	
  GPIO_InitStruct_Green.GPIO_Pin = GPIO_LED_PIN_Green;    // 3、修改初始化结构体数据
  GPIO_InitStruct_Green.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct_Green.GPIO_Speed = GPIO_Speed_50MHz;
  	
  GPIO_Init(GPIO_LED_PORT_Green, &GPIO_InitStruct_Green);    // 4、调用 GPIO_Init 初始化结构体。形参为PORT和GPIO_InitStruct指针
  GPIO_Init(GPIO_LED_PORT_Blue, &GPIO_InitStruct_Blue);
  GPIO_Init(GPIO_LED_PORT_Red, &GPIO_InitStruct_Red);
  ```

- **GPIO_ReadInputDataBit**：读取指定端口管脚的输入

  ```c
  ReadValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);    // 形参为PORT和GPIO_Pin管脚
  ```

- **GPIO_SetBits** / **GPIO_ResetBits** / **GPIO_WriteBit**：设置 / 清除 / 设置或清除指定的数据端口位

  ```c
  GPIO_SetBits(GPIOA, GPIO_Pin_10 | GPIO_Pin_15);              // 设置管脚输出为1。形参为形参为PORT和GPIO_Pin管脚
  GPIO_ResetBits(GPIOA, GPIO_Pin_10 | GPIO_Pin_15);            // 设置管脚输出为0。形参为形参为PORT和GPIO_Pin管脚
  GPIO_WriteBits(GPIOA, GPIO_Pin_10 | GPIO_Pin_15, Bit_SET);   // 设置管脚输出。形参为形参为PORT、GPIO_Pin管脚和待写入的值（Bit_SET或Bit_RESET）
  ```

- **GPIO_Write**：向指定 GPIO 数据端口写入数据（一次性给多个IO口赋值）

  ```c
  GPIO_Write(GPIOA, 0x1101);    // 形参为PORT和待写入的值
  ```

#### I2C

- **I2C_ Init**：根据 I2C_InitStruct 中指定的参数初始化外设 I2Cx 寄存器

  I2C_InitTypeDef 定义如下：

  ```
  typedef struct { 
  	u16 I2C_Mode; 
  	u16 I2C_DutyCycle; 
  	u16 I2C_OwnAddress1; 
  	u16 I2C_Ack; 
  	u16 I2C_AcknowledgedAddress; 
  	u32 I2C_ClockSpeed; 
  } I2C_InitTypeDef;
  ```

  - I2C_Mode：I2C 的工作模式。可以取值：

    <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\image-20220201210903827.png" alt="image-20220201210903827" style="zoom:67%;" />

  - I2C_DutyCycle：设置 I2C 的占空比。可以取值：

    <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\image-20220201210923069.png" alt="image-20220201210923069" style="zoom:67%;" />

  - I2C_OwnAddress1：设置第一个设备自身地址。

  - I2C_Ack：使能或者失能应答。可以取值：I2C_Ack_Enable；I2C_Ack_Disable。

  - I2C_AcknowledgedAddres：定义应答 7 位地址还是 10 位地址。可以取值：I2C_AcknowledgeAddress_7bit / I2C_AcknowledgeAddress_10bit。

  - I2C_ClockSpeed：设置时钟频率，不能高于 400KHz。

  ```c
  // 初始化一个GPIO管脚示例如下
  // I2C硬件宏定义
  #define I2Cx  I2C1    // 选择I2C外设
  #define I2C_APBxClock_FUN  RCC_APB1PeriphClockCmd    //打开时钟函数封装
  #define I2C_CLK  RCC_APB1Periph_I2C1    // I2C时钟
  #define I2C_GPIO_APBxClock_FUN  RCC_APB2PeriphClockCmd
  #define I2C_GPIO_CLK  RCC_APB2Periph_GPIOB
  #define I2C_SCL_PORT  GPIOB
  #define I2C_SCL_PIN  GPIO_Pin_6
  #define I2C_SDA_PORT  GPIOB
  #define I2C_SDA_PIN  GPIO_Pin_7
  
  #define I2C_Speed 400000           // I2C 快速模式
  #define I2Cx_OWN_ADDRESS7  0X0A    //这个地址只要与 STM32 外挂的 I2C 器件地址不一样即可
  
  // I2C的GPIO设置
  static void I2C_GPIO_Config(void) {
  	GPIO_InitTypeDef GPIO_InitStructure;    // 声明GPIO结构变量
  
  	// 打开与 I2C 有关的时钟
  	I2C_APBxClock_FUN ( I2C_CLK, ENABLE );
  	I2C_GPIO_APBxClock_FUN ( I2C_GPIO_CLK, ENABLE );
      
  	// I2C_SCL、I2C_SDA的GPIO管脚设置
  	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // 开漏输出
  	GPIO_Init(I2C_SCL_PORT, &GPIO_InitStructure);
  
  	GPIO_InitStructure.GPIO_Pin = I2C_SDA_PIN;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD; // 开漏输出
  	GPIO_Init(I2C_SDA_PORT, &GPIO_InitStructure);
  }
  
  // I2C模式设置
  static void I2C_Mode_Configu(void) {
  	I2C_InitTypeDef I2C_InitStructure;    // 声明I2C结构变量
  
  	// I2C 配置
  	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;    // 高电平数据稳定，低电平数据变化 SCL 时钟线的占空比
  	I2C_InitStructure.I2C_OwnAddress1 =I2Cx_OWN_ADDRESS7;
  	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable ;
  	I2C_InitStructure.I2C_AcknowledgedAddress = 	I2C_AcknowledgedAddress_7bit;    // I2C 的寻址模式
  	I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;    // 通信速率
  
  	I2C_Init(EEPROM_I2Cx, &I2C_InitStructure);    //I2C 初始化
  	I2C_Cmd(EEPROM_I2Cx, ENABLE);    //使能 I2C
  }
  
  // I2C总体设置
  void I2C_Init(void) {
  	I2C_GPIO_Config();
  	I2C_Mode_Configu();
  }
  ```

- **I2C_ Cmd**：使能或者失能 I2C 外设

  ```c
  I2C_Cmd(I2C1, ENABLE);    // 形参为I2C外设和设置状态
  ```

#### SysTick

- 使用SysTick延迟指定毫秒

  ```c
  void SysTick_Delay_ms(uint32_t ms) {
  	SysTick_Config(72000);    // 配置为ms的基本单位72  clk=72M时，1 ms=72000*(1/72M)
  	uint32_t i;
  	for(i=0; i<ms; i++)       // for循环us次，每次都判断systick的countflag置1，即systick已经计到0才进行下一次循环
  		while( !((SysTick->CTRL) & (1<<16)) );    // 等待systick的countflag置1，即systick已经计到0
  	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;    // 关闭systick使能
  }
  ```

#### TIM1（高级定时器）

高级定时器（TIM1、TIM8）具有以下功能：定时、输出比较（PWM输出）、输入捕获（脉冲宽度测量、PWM输入（只能由通道1/2捕获））、互补输出（脉冲反向输出）

##### PWM输出

1. 配置TIM1需要的GPIO管脚

   ```c
   // TIM1 输出比较通道
   #define ADVANCE_TIM_CH1_GPIO_CLK  RCC_APB2Periph_GPIOA
   #define ADVANCE_TIM_CH1_PORT  GPIOA
   #define ADVANCE_TIM_CH1_PIN  GPIO_Pin_8
   
   // TIM1 输出比较通道的互补通道
   #define ADVANCE_TIM_CH1N_GPIO_CLK  RCC_APB2Periph_GPIOB
   #define ADVANCE_TIM_CH1N_PORT  GPIOB
   #define ADVANCE_TIM_CH1N_PIN  GPIO_Pin_13
   
   // TIM1 输出比较通道的刹车通道
   #define ADVANCE_TIM_BKIN_GPIO_CLK  RCC_APB2Periph_GPIOB
   #define ADVANCE_TIM_BKIN_PORT  GPIOB
   #define ADVANCE_TIM_BKIN_PIN  GPIO_Pin_12
   
   static void ADVANCE_TIM_GPIO_Config(void) {
       GPIO_InitTypeDef GPIO_InitStructure;
   	// 输出比较通道 GPIO 初始化
   	RCC_APB2PeriphClockCmd(ADVANCE_TIM_CH1_GPIO_CLK, ENABLE);
   	GPIO_InitStructure.GPIO_Pin = ADVANCE_TIM_CH1_PIN;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   	GPIO_Init(ADVANCE_TIM_CH1_PORT, &GPIO_InitStructure);
   
   	// 输出比较通道互补通道 GPIO 初始化
   	RCC_APB2PeriphClockCmd(ADVANCE_TIM_CH1N_GPIO_CLK, ENABLE);
   	GPIO_InitStructure.GPIO_Pin = ADVANCE_TIM_CH1N_PIN;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   	GPIO_Init(ADVANCE_TIM_CH1N_PORT, &GPIO_InitStructure);
   
   	// 输出比较通道刹车通道 GPIO 初始化
   	RCC_APB2PeriphClockCmd(ADVANCE_TIM_BKIN_GPIO_CLK, ENABLE);
   	GPIO_InitStructure.GPIO_Pin = ADVANCE_TIM_BKIN_PIN;
   	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   	GPIO_Init(ADVANCE_TIM_BKIN_PORT, &GPIO_InitStructure);
       
   	// BKIN 引脚默认先输出低电平
   	GPIO_ResetBits(ADVANCE_TIM_BKIN_PORT,ADVANCE_TIM_BKIN_PIN);
   }
   ```

2. 定时器模式配置

   ```c
   // TIM1的时钟宏定义
   #define ADVANCE_TIM  TIM1
   #define ADVANCE_TIM_APBxClock_FUN  RCC_APB2PeriphClockCmd
   #define ADVANCE_TIM_CLK  RCC_APB2Periph_TIM1
   
   // PWM信号设置
   #define ADVANCE_TIM_PERIOD  (256-1)                   // 周期
   #define ADVANCE_TIM_PSC  (2000-1)                     // 预分频
   #define ADVANCE_TIM_PULSE  4                          // 占空比
   #define ADVANCE_TIM_IRQ  TIM1_UP_IRQn
   #define ADVANCE_TIM_IRQHandler  TIM1_UP_IRQHandler
   
   #define TIM_OCxInit  TIM_OC1Init                      // 通道初始化函数，这里设置为1通道
   
   static void ADVANCE_TIM_Mode_Config(void) { 
   	ADVANCE_TIM_APBxClock_FUN(ADVANCE_TIM_CLK,ENABLE);    // 开启定时器时钟, 即内部时钟 CK_INT=72M
   	
       /*--------------------时基结构体初始化-------------------------*/
   	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   	TIM_TimeBaseStructure.TIM_Period=ADVANCE_TIM_PERIOD;        // 自动重装载寄存器的值，累计 TIM_Period+1 个频率后产生一个更新或者中断
   	TIM_TimeBaseStructure.TIM_Prescaler= ADVANCE_TIM_PSC;       // 驱动 CNT 计数器的时钟 = Fck_int/(psc+1)
   	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;       // 时钟分频因子 ，配置死区时间时需要用到
   	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;   // 计数器计数模式，设置为向上计数
   	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;              // 重复计数器的值，没用到不用管
   	TIM_TimeBaseInit(ADVANCE_TIM, &TIM_TimeBaseStructure);      // 初始化定时器
   
      /*--------------------输出比较结构体初始化-------------------*/
   	TIM_OCInitTypeDef TIM_OCInitStructure;
   	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                 // 配置为 PWM 模式 1
   	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;     // 输出使能
   	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;   // 互补输出使能
   	TIM_OCInitStructure.TIM_Pulse = ADVANCE_TIM_PULSE;                // 设置占空比大小
   	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;         // 输出通道电平极性配置
   	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;       // 互补输出通道电平极性配置
   	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;        // 输出通道空闲电平极性配置
   	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;    // 互补输出通道空闲电平极性配置
   	TIM_OCxInit(ADVANCE_TIM, &TIM_OCInitStructure);
   	TIM_OC1PreloadConfig(ADVANCE_TIM, TIM_OCPreload_Enable);
   
   	/*-------------------刹车和死区结构体初始化-------------------*/
   	// 有关刹车和死区结构体的成员具体可参考 BDTR 寄存器的描述
   	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
   	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
   	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
   	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
   	// 输出比较信号死区时间配置，具体如何计算可参考 BDTR:UTG[7:0] 的描述，这里配置的死区时间为 152ns
   	TIM_BDTRInitStructure.TIM_DeadTime = 11;
   	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
   	// 当 BKIN 引脚检测到高电平的时候，输出比较信号被禁止，就好像是刹车一样
   	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
   	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
   	TIM_BDTRConfig(ADVANCE_TIM, &TIM_BDTRInitStructure);
   
   	TIM_Cmd(ADVANCE_TIM, ENABLE);                // 使能计数器
   	TIM_CtrlPWMOutputs(ADVANCE_TIM, ENABLE);     // 主输出使能，当使用的是通用定时器时，这句不需要
   }
   ```

3. 总体初始化

   ```c
   void ADVANCE_TIM_Init() {
   	ADVANCE_TIM_GPIO_Config();
       ADVANCE_TIM_Mode_Config();
   }
   ```

4. 修改占空比（通过修改输出比较寄存器的值修改）

   ​		初始化控制 RGB 灯用的定时器，它被配置为向上计数，TIM_Period 被配置为 255，即定时器每个时钟周期计数器加 1，计数至 255 时溢出，从 0 开始重新计数；而代码中的 PWM 通道配置，当计数器的值小于输出比较寄存器 CCRx 的值时，PWM 通道输出低电平。所以上述代码配置把输出脉冲的单个周期分成了 256 份，而输出比较寄存器 CCRx 配置的值即该脉冲周期内PWM 通道输出低电平的时间份数，所以修改CCRx 的值，即可控制输出 [0:255] 种输出速度。

   ```c
   #define PWM_CCRx  CCR1    // 通道比较寄存器，应与选择的输出通道一致
   
   void SetColorValue(uint8_t speed) {    // speed为[0,T]的值
   	// 根据参数值修改定时器的比较寄存器值
   	ADVANCE_TIM->PWM_CCRx = speed;
   }
   ```

##### PWM输入捕获

- 每个TIM对应四个输入通道，用于捕获一个PWM信号。PWM输入捕获只能使用CH1和CH2两个通道。其中，每个通道会被两个捕获通道捕获，TI1FP1捕获周期，TI2FP2捕获占空比。

  输入通道用来信号输入，捕获通道用来捕获输入信号。一个输入通道的信号可以同时输入给两个捕获通道，只有一路输入信号(CH1）却占用了两个捕获通道（IC1和IC2）。

  <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\image-20220201211813263.png" alt="image-20220201211813263" style="zoom: 67%;" />

- 输入捕获可以对输入的信号的上升沿，下降沿或者双边沿进行捕获，常用的有测量输入信号的脉宽和测量PWM输入信号的频率和占空比这两种。输入捕获的原理是，当捕获到信号的跳变沿的时候，把计数器CNT的值锁存到捕获寄存器CCR中，把前后两次捕获到的CCR寄存器中的值相减，就可算出脉宽或者频率。

- 代码：

  ```c
  #define LEFTWHEEL_TIM_IN_GPIO_CLK    RCC_APB2Periph_GPIOC
  #define LEFTWHEEL_TIM_IN_PORT        GPIOA
  #define LEFTWHEEL_TIM_IN_PIN         GPIO_Pin_6
  
  #define LEFTWHEEL_TIM_IN                  TIM3
  #define LEFTWHEEL_TIM_IN_APBxClock_FUN    RCC_APB1PeriphClockCmd
  #define LEFTWHEEL_TIM_IN_CLK              RCC_APB1Periph_TIM3
  #define LEFTWHEEL_TIM_IN_PWM_CHANNEL      TIM_Channel_1
  
  // 中断相关宏定义
  #define LEFTWHEEL_TIM_IN_IRQ      TIM3_IRQn           // 左轮中断源
  
  #include "AdvanceTim.h"
  // GPIO 初始化 
  static void ADVANCE_TIM_GPIO_Config(void) {
      GPIO_InitTypeDef GPIO_InitStructure;
  	RCC_APB2PeriphClockCmd(LEFTWHEEL_TIM_IN_GPIO_CLK, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = LEFTWHEEL_TIM_IN_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(LEFTWHEEL_TIM_IN_PORT, &GPIO_InitStructure);
  }
  
  // TIM 属性初始化 
  static void ADVANCE_TIM_Mode_Config(void) { 
  	LEFTWHEEL_TIM_IN_APBxClock_FUN(LEFTWHEEL_TIM_OUT_CLK, ENABLE);    // 开启定时器时钟, 即内部时钟 CK_INT=72M
  	
      /*--------------------时基结构体初始化-------------------------*/
  	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
      TIM_TimeBaseStructure.TIM_Period = 1000-1;                    // 自动重装载寄存器的值，累计 TIM_Period+1 个频率后产生一个更新或者中断
  	TIM_TimeBaseStructure.TIM_Prescaler= 72-1;                    // 驱动 CNT 计数器的时钟 = Fck_int/(psc+1)
  	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;         // 时钟分频因子 ，配置死区时间时需要用到
  	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;     // 计数器计数模式，设置为向上计数
  	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;                // 重复计数器的值，没用到不用管	
  	TIM_TimeBaseInit(LEFTWHEEL_TIM_IN, &TIM_TimeBaseStructure);
  	
  	/*--------------------输入捕获结构体初始化-------------------*/
  	TIM_ICInitTypeDef TIM_ICInitStructure;
  	// 捕获通道 IC1 配置 
      TIM_ICInitStructure.TIM_Channel = LEFTWHEEL_TIM_IN_PWM_CHANNEL;          // 选择捕获通道
      TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;               // 设置捕获的边沿
      TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;          // 设置捕获通道的信号来自于哪个输入通道，有直连和非直连两种
      TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;                    // 1 分频，即捕获信号的每个有效边沿都捕获
      TIM_ICInitStructure.TIM_ICFilter = 0x0;                                  // 不滤波
  	
      TIM_PWMIConfig(LEFTWHEEL_TIM_IN, &TIM_ICInitStructure);                   // 初始化 PWM 输入模式
      // 当工作做 PWM 输入模式时，只需要设置触发信号的那一路即可（用于测量周期）。另外一路（用于测量占空比）会由硬件自带设置，不需要再配置
      TIM_SelectInputTrigger(LEFTWHEEL_TIM_IN, TIM_TS_TI1FP1);                 // 选择输入捕获的触发信号
      TIM_SelectSlaveMode(LEFTWHEEL_TIM_IN, TIM_SlaveMode_Reset);              // 选择从模式:复位模式
      TIM_SelectMasterSlaveMode(LEFTWHEEL_TIM_IN,TIM_MasterSlaveMode_Enable);  // PWM 输入模式时，从模式必须工作在复位模式，当捕获开始时，计数器 CNT 会被复位
      
  	TIM_ITConfig(LEFTWHEEL_TIM_IN, TIM_IT_CC1, ENABLE);                      // 使能捕获中断，这个中断针对的是主捕获通道（测量周期那个）
      TIM_ClearITPendingBit(LEFTWHEEL_TIM_IN, TIM_IT_CC1);                     // 清除中断标志位
      TIM_Cmd(LEFTWHEEL_TIM_IN, ENABLE);                                       // 使能高级控制定时器，计数器开始计数
  }
  
  // 中断优先级配置
  static void ADVANCE_TIM_NVIC_Config(void) {
      NVIC_InitTypeDef NVIC_InitStructure; 
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);		          // 设置中断组为0
      NVIC_InitStructure.NVIC_IRQChannel = LEFTWHEEL_TIM_IN_IRQ; 	  // 设置中断来源
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	  // 设置抢占优先级
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 设置响应优先级	
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);
  	
  	NVIC_InitStructure.NVIC_IRQChannel = RIGHTWHEEL_TIM_IN_IRQ;   // 设置中断来源  
  	NVIC_Init(&NVIC_InitStructure);
  }
  
  // TIM 总初始化
  void ADVANCE_TIM_Init() {
  	ADVANCE_TIM_GPIO_Config();
  	ADVANCE_TIM_NVIC_Config();
      ADVANCE_TIM_Mode_Config();
  }
  ```

#### USART



## Linux

### Linux基础命令

- $ cat file1 file2 ...：将文件file1, file2等的内容拼接起来并显示，然后退出。

  $ cat >> file：从键盘读取数据，将数据追加到已有文件中。

  $ cat file：显示file文件内容

- $ ls：显示指定目录内容，缺省参数为当前目录，ls -l显示详细列表，ls -F显示文件类型信息。

- $ cp：拷贝文件。

  $ cp file1 file2：将文件file1拷贝到文件file2。

  $ cp file1 ... fileN dir：将多个文件file1到fileN拷贝到目录dir。

- $ mv：重命名文件。

  $ mv file1 file2：将文件名从file1重命名为file2。

  $ mv file1 ... fileN dir：将多个文件移动到某个目录。

- $ touch file：创建一个文件，如果文件已经存在，则该命令会更新文件的时间戳。

- $ rm file：用来删除文件，文件一旦被删除通常无法恢复。

- $ echo s：将参数s显示到标准输出。

  $ echo s > file：将s内容显示到file文件

- 目录之间使用/分割，以/开头的路径叫绝对路径，不以/开头的路径叫相对路径。

  ..代表一个目录的上层目录。

  .代表当前目录。通常不需使用.，而是直接使用目录名来访问当前目录下的子目录。

- $ cd dir：切换当前的工作目录为dir。如果不带dir，cd命令会跳到你的个人根目录。

  $ cd ..：切换到上一目录。

- $ mkdir dir：创建一个名为dir的新目录。

  $ rmdir dir：删除一个目录。rmdir只能删除空目录。可以使用rm -rf来删除一个目录及其中的所有内容。

- $ grep s dir：显示文件和输入流dir中和参数s匹配的行的内容。

  $ grep s /etc/*：查看目录/etc中所有包含s的文件。

  $ grep -i：不区分大小写。$ grep -v：反转匹配，显示所有不匹配的行。

- $ less file1：当要查看的文件过大或者内容多得需要滚动屏幕的时候，less将内容分屏显示，按空格查看下一屏，b键查看上一屏，q键退出。

- $ pwd：显示当前目录的绝对路径。


- $ diff file1 file2：查看两个文件之间的不同。


- $ file file：获取一个文件的格式信息。


- $ find [目录名] [选项] [查找条件]：在指定目录中寻找符合选项条件的文件。


- $ head -n file/ $ tail -n file命令显示文件的前n行内容，tail命令显示文件的最后n行内容，n参数缺省默认10行。如果要从第n行开始显示所有内容，使用tail +n。


- $ sort：将文件内的所有行按照字幕顺序排序，使用-n选项按照数字顺序排序那些以数字开头的行，使用-r选项反转排序。


- $ passwd username：更改用户密码。


- 命令路径PATH：存放路径，路径之间以冒号分隔。

  $ PATH=dir:$PATH：将路径dir加入到PATH的最前面。

  $ PATH=$PATH:dir：将路径加入到PATH变量的最后面。

- $ info [指令]：查看指令相关帮助信息。


- $ man [指令]：查看指令相关手册。


- $ clear：刷新屏幕，并保留历史命令操作记录。


- $ command > file：将命令的执行结果输出到文件（默认终端屏幕）。如果文件file不存在，则创建一个新的file文件；如果file文件已经存在，Shell会清空文件内容。

  $ command >> file：将命令的输出结果加入到文件尾部

  $ command1| command2：使用管道字符 | 将一个命令的执行结果输出到另一个命令。命令1的正确输出作为命令2的操作对象。

- 常见错误：

  No such file or directory：访问一个不存在的文件或者目录。

  File exists：新建文件的名称和现有的一个文件或者目录重名。

  Not a directory, Is a directory：把文件当作目录或者反过来，把目录当作文件。

  No space left on device：硬盘空间不足。

  Permission denied：读或写一个没有权限访问的文件和目录时。

  Operation not permitted：试图终止一个无权终止的进程时。

  Segmentation fault, Bus error：分段故障，总线繁忙，常因程序输入数据有问题。

- $ ps：列出所有正在运行的进程。PID：进程ID。TTY：进程所在的终端设备。STAT：进程状态，就是进程在内存中的状态。TIME：进程目前为止所用CPU时长。COMMAND：进程名。


- $ kill pid：终止一个进程。

  $ kill -STOP pid：被暂停的进程仍然停留在内存，等待被继续执行。

  $ kill -CONT pid：继续执行进程

- 在命令行末尾添加&操作符将进程设置为后台运行。


- $ chmod file：更改文件权限。

  $ chmod g+r file：为用户组（g）加上可读权限（r）。

  $ chmod o+r file：为其他用户（o）加上可读权限（r）。

- $ ln -s target linkname：创建符号链接，相当于文件的别名。linkname参数是符号链接名称，target参数是要指向的目标路径，-s选项表示这是一个符号链接。


- $ gunzip file.gz：解压缩.gz文件。

  $ gzip file：压缩文件并删除原有文件。

  $ gzip -l file：查看压缩文件内容。

  $ gzip -kd file：解压以.gz文件且保留原文件。

  $ gzip -k file：压缩文件且保留原文件。

- $ tar：多个文件的压缩与解压。

  $ tar cvf archive.tar file1 file2 ...：archive.tar参数是生成的归档文件名，file1 file2 ...是要归档的文件和目录列表。选项c代表创建文件，选项v用来显示详细的命令执行信息，选项f代表文件，后面需要指定一个归档文件名，如果不指定归档文件名，则归档到磁带设备，如果文件名为－，则归档到标准输入或者输出。

  $ tar xvf archive.tar：解压缩tar文件。选项x代表解压模式。还可以只解压归档文件中的某几个文件，只需要在命令后面加上这些文件的文件名。

  $ zcat file：显示压缩包中文件的内容。

  其他压缩/解压缩命令：bzip2/bunzip2，xz/unxz。

- $ sudo：以root用户身份执行命令。


- $ ping：用于检测主机。


- $ htop：显示系统中正在运行的进程的实时状态。


- $ ps：显示当前进程的状态，参数：-A 列出所有的进程；-w 显示加宽可以显示较多的资讯；-au 显示较详细的资讯；-aux 显示所有包含其他使用者的行程。


- $ history n：显示历史执行过的n条命令。! number执行第几条命令；! command从最近的命令查到以command开头的命令执行；!!执行上一条 。


- $ ifconfig

  $ iwconfig

- $ apt-get install software_name：安装软件 

  $ sudo apt-get update：下载最新的软件列表

  $ sudo apt-get upgrade：安装更新

- Ctrl+C：结束当前命令进程。


- $ stty -echo #：设置输入字符不回显。

  $ stty echo #：设置输入字符回显。

### Linux操作案例

#### 串口登录

1. 使用数据线连接Linux开发板，“win+X”快捷键打开设备管理器，检查串口
2. 使用Putty软件串口登录开发板（记得将波特率改为115200）

#### 获取从机IP地址

1. 使用串口登录Linux开发板，并确保开发板和电脑在同一局域网内
2. 在命令窗口内输入ifconfig指令，获取开发板在局域网下IP地址

#### SSH远程登录

1. 已知IP地址时，使用Putty软件SSH选项远程登录开发板

#### WIFI配置

1. 切换到root用户：su root

2. 开启wifi：nmcli r wifi on

3. 扫描附近wifi：nmcli dev wifi

4. 连接特定wifi：sudo nmcli dev wifi connect “WIFINAME” password “PASSWORD” ifname wlan0

5. sudo ifconfig wlan0 down

   sudo ifconfig wlan0 up：重启网卡设备

#### 开机自启动脚本文件

1. 编写service文件

   - 编写如下样式的.service文件

   ```shell
   [Unit]
   Description=yourDescription
   After=network.service
   [Service]
   Type=simple
   User=root
   Group=root
   WorkingDirectory=yourSrcPath
   ExecStart=/yourInstallPath/NAME.sh
   PrivateTmp=true
   Restart=on-failure
   [Install]
   WantedBy=multi-user.target
   ```

   - 使用指令：sudo cp NAME.service /etc/systemd/system/NAME.service将service文件转移到/etc/systemd/system文件夹下
   - Type指定了我的类型是simple；after指定了启动network.service服务后开始启动我的服务；ExecStart指定了执行/usr/bin/NAME.sh；WorkingDirectory指定了工作空间；PrivateTmp指定了开启独立的进程空间，

2. 编写sh脚本

   - 如下形式编写.sh脚本文件

     ```shell
     #!/bin/bash
     sleep 10
     cd /home/pi/Documents/LCDDisplay
     sudo python lcd_clock.py
     ```

   - #!/bin/bash：制定脚本解释器

     sleep 10：等待一段时间，防止启动失败

     sudo python lcd_clock.py：执行制定py文件

3. 开启服务

   - sudo systemctl start NAME.service：开启开机启动服务

     sudo systemctl enable NAME.service：激活开机启动服务

     sudo systemctl status NAME.service：查看开机启动服务状态

### Ubuntu系统使用技巧

#### 双系统切换

- 在开机时长按F2，进入boot界面。

  若要进入ubuntu系统，在boot选项中将ubuntu设置为第一启动选项，在advance中将SATA设置为AHCI。

  若要进入win系统，在boot选项中将win设置为第一启动选项，在advance中将SATA设置为RAID。

  然后按F10保存修改并启动。



## Git

### 提交修改

<img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\2679306e3b75dc221476e3350c0dfca.jpg" alt="2679306e3b75dc221476e3350c0dfca" style="zoom: 33%;" />

- 初始化Git仓库：

  ①创建一个空目录：

  ​    $ mkdir dir

  ​    $ cd learngit

  ​    $ pwd

  ②使用git init命令把当前目录变为git仓库。

- 把文件添加到库：

  ①把文件放到库目录下

  ②使用git add file命令，将文件添加到暂存区

  ③使用git commit -m “instruction”命令，把暂存区所有文件提交到仓库，引号内为此次提交的说明。

- git status：显示仓库当前的状态（是否有需提交的文件，是否有文件被修改但未提交）

- git diff：查看文件修改内容。

  git diff HEAD -- file：查看工作区和版本库里面最新版本的区别。

- git log：显示从最近到最远的提交日志

  git log --pretty=oneline：显示从最近到最远的提交日志（每次提交在一行中显示）

  git log --graph --pretty=oneline --abbrev-commit：显示分支合并图。

- git reflog：显示使用的每次命令（找回版本号）

- git reset -hard edition：将当前版本设置为edition版本（HEAD为当前版本，HEAD^为上一版本，HEAD^^为上两版本，HEAD~n为上n版本；edition也可是版本号）。

  git reset HEAD file：把暂存区的修改撤销掉，重新放回工作区。

- git checkout -- file：直接丢弃工作区的修改，将工作区文件还原到提交前版本。

  git checkout -b branch：创建并切换到branch分支。

  git checkout -b branch origin/branch：在本地创建和远程分支对应的分支。

  git checkout branch：切换到branch分支。

- git branch：查看当前分支。

  git branch -d branch：删除branch分支。

  git branch -D branch：强行删除一个没有被合并过的分支。

  git branch --set-upstream-to branch origin/branch：创建本地分支和远程分支的链接关系。

- git merge branch：将branch分支合并到当前分支。

  git merge --no-ff branch：禁用Fast forward（不覆盖分支信息，保留历史有分支），将branch分支合并到当前分支。

- git switch -c branch：创建并切换到branch分支。

  git switch branch：切换到branch分支。

- git rm file：从版本库中删除文件（删除后需commit提交修改）

- ssh-keygen -t rsa -C "youremail@example.com"：创建SSH Key。

- git remote add origin git@github:path/repo-name.git：关联github远程库（一个本地库可以关联到两个远程库，但需要起不一样的远程库名称，把origin默认名改掉）。

  git remote / git remote -v：查看远程库信息。

  git remote rm <name>：解除本地和远程库的绑定关系。

- git push -u origin branch：第一次向github远程库推送branch分支的所有内容。

  git push origin branch：向github远程库推送branch分支的所有本地提交。

  git push origin tagname：向github远程库推送某个标签。

  git push origin --tags：向github远程库推送所有未推送的标签。

  git push origin :refs/tags/tagname：删除github远程库中的tagname标签。

- git clone git@github.com:path/repo-name.git：克隆一个远程仓库到本地。

- git stash：保存当前工作状态，清理工作区，方便处理其他工作。

  git stash list：显示stash保存的工作状态。

  git stash apply：恢复保存的工作状态，不删除stash内容。

  git stash drop：删除stash内容。

  git stash pop：恢复保存的工作状态，并删除stash内容。

- git cherry-pick edition：将一个特定的提交复制到当前分支，edition为提交的版本号。

- git pull：把最新的提交从远程库对应分支抓下来，并在本地合并。

- git rebase：把本地未push的分支提交历史整理成直线。

- 创建标签：

  ①切换到需要打标签的分支上：git switch branch

  ②使用命令git tag tagname edition：为版本号为edition的commit打一个新标签tagname（edition缺省则默认打在最新提交的commit上）。

  git tag：查看所有标签。

  git tag -a tagname -m "instruction" edition：创建带有说明的标签。

  git tag -d tagname：删除tagname标签。

- git show tagname：显示标签说明文字。

- git config --global alias.abbr order：设置order命令的别名为abbr



## SSH

- ssh-keygen：在本地计算机上生成 RSA 密钥对。

  ssh-keygen -t rsa -b 4096 -f ~/.ssh/user_ca -C user_ca

- ssh-copy-id username@remote_host：将公钥复制到服务器。

- ssh remote_host / ssh username@remote_host：连接到远程服务器。

  ssh username@remote_host command_to_run：在远程服务器上运行单个命令。

  ssh -p port_num username@remote_host：指定端口号，登录到具有不同端口的服务器。

- eval $(ssh-agent)

  ssh-add：启动代理程序并将私钥添加到代理，无需再次重新输入密码。



## Vim

- 模式：编辑模式、指令模式

  i：进入编辑模式

   ‘: ’：进入指令模式

  esc：退出编辑模式

- 移动光标：在移动命令前加上数字表示执行次数

  H：将光标向左移动一个字符。

  j：将光标向下移动一行。

  k：将光标向上移动一行。

  l：将光标向右移动一个字符。

  0：将光标移动到行首。

  $：将光标移动到行尾。

  w：向前移动一个字。

  b：向后移动一个字。

  G：移动到文件末尾。

  gg：移动到文件的开头。

  `.：移至最后一次编辑。

- 删除和撤销（删除的文本存储在准备好粘贴回文档的缓冲区内）：

  x：将删除一个字符。

  d：开始删除操作。

  dw：将删除一个单词。

  d0：将删除到一行的开头。

  d$：将删除到行尾。

  dgg：将删除到文件的开头。

  dG：将删除到文件末尾。 

  u：将撤消最后一个操作。

  Ctrl-r：将重做最后一次撤消。

- 查找和替换：

  /text：搜索text文本。

  ?text：反向搜索text文本。

  n：再次查找上一次搜索的单词。

  N：反向再次查找上一次搜索的单词。

  :%s/ text / replacement text /g：在整个文档中搜索text文本并将其替换为替换文本。

  :%s/ text / replacement text /gc：在替换文本之前，搜索整个文档并确认。 

- 复制和粘贴



## Matlab

### 基础知识

- 变量：

  变量命名：大小写敏感；首字母必须为英文；可以包含下划线、数字，不能包含空格、标点

  系统预设变量：ans、eps、pi、inf、NaN、i、j等

  全局变量：在函数前用global将变量声明为全局变量

- 表达式：

  如果一个指令过长可以在结尾加上...（代表此行指令与下一行连续）

  若不想让 MATLAB 每次都显示运算结果，可在运算式结尾加上分号；

  注释：百分号%

- 常用运算：

  基本算术运算：+ - * / ^

- 文件：

  - .m文件：用于编写代码

  - .m文件（函数）：

    函数文件名需要与函数名一致

    ```matlab
    % demo函数fun1.m
    function Y= fun1(x)    % 函数定义：function[out1, out2, ..] = functioname(in1, in2, ..)
    % 第一行帮助行，说明函数作用及有关内容
    Y =(x^3 - 2*x^2 + x - 6.3)/(x^2 + 0.05*x - 3.14);
    ```

    建立函数库：将保存有自定义函数文件的目录添加到matlab搜索路径中

  - .mat文件：保存数据

- 控制语句：

  ```matlab
  % 循环语句：
  for x = array
  	{commands}
  end
  
  while expression
  {commands}
  end
  
  % 判断语句：
  if expression1
  	commands evaluated if expression1 is True
  elseif expression2
  	commands evaluated if expression2 is True
  ……
  else
  	commands evaluated if no other expression is True
  end
  
  % 开关语句：
  switch 表达式（％可以是标量或字符串）
  case expression1
  	commands1
  case expression1
  	commands2
  ……
  otherwise
  	commandsn
  end
  ```

- 数据类型：

  double：矩阵、多维数组等

  char：字符串变量和矩阵

  sparse：稀疏矩阵

  struct：结构体型

  cell：单元型变量

### 常用命令

- help：查看帮助

- who：显示当前变量

  whos：显示当前变量的详细信息

- clear：清空工作空间的变量和函数

  clc：清空命令行所有命令

- load：将文件读取到工作区间

  save：将变量存储到文件中

- quit/exit：退出matlab

- which：定位函数或文件

- path：获取或设置搜索路径

- echo：命令回显

  cd：改变当前工作目录

  pwd：显示当前工作目录

  dir：显示目录内容

- 数学运算：

  | 函数名             | 用途                       | 函数名    | 用途                |
  | ------------------ | -------------------------- | --------- | ------------------- |
  | abs                | 求绝对值或模               | mod       | 求余数              |
  | sqrt               | 求二次方根                 | fix       | 向最接近0取整       |
  | real / imag        | 求实部 / 虚部              | floor     | 向最接近-oo取整     |
  | conj               | 求共轭复数                 | round     | 四舍五入取整        |
  | sin / cos / tan    | 求正弦 / 余弦 / 正切       | exp / log | 自然指数 / 自然对数 |
  | asin / acos / atan | 求反正弦 / 反余弦 / 反正切 |           |                     |

- 矩阵生成：

  | 函数名      | 用途                                |
  | ----------- | ----------------------------------- |
  | zeros(m, n) | 生成m×n的全0矩阵                    |
  | ones(m, n)  | 生成m×n的全1矩阵                    |
  | rand(m, n)  | 生成m×n的均匀分布的[0, 1)的随机矩阵 |
  | randn(m, n) | 生成m×n的正态分布的随机矩阵         |
  | eye(m, n)   | 生成m×n的单位矩阵                   |

  注：当生成矩阵时只输入一个形参n时，产生n×n的方阵

- 矩阵运算：

  | 函数名          | 用途                        |
  | --------------- | --------------------------- |
  | det(X)          | 计算方阵X的行列式           |
  | rank(X)         | 计算矩阵X的秩               |
  | inv(X)          | 计算方阵X的逆               |
  | diag(X)         | 生成矩阵X的对角阵           |
  | [v, d] = eig(X) | 计算矩阵X的特征值和特征向量 |
  | X'              | 计算矩阵X的转置             |

  对矩阵进行数学运算与数字运算基本相同，在运算符号前带点“.”表示对每个元素分别运算。

## Python

### NumPy

- import numpy as np：导入numpy库

- np.array([1, 2, 3, ...])：创建并初始化一个数组

  np.zeros((m, n))：创建m行n列的全0数组

  np.ones((m, n))：创建m行n列的全1数组

  np.arange(m, n)：创建一个从m到n的步长为1的递增或递减数列

  np.linspace(m, n, s)：创建一个从m到n的元素个数为s的等间距数组

  np.random.rand(m, n)：创建一个m行n列的随机数组

- a.shape：获取数组尺寸(m, n)

- 数组默认的类型为浮点型

  a = np.zeros((m, n), dtype = np.uint32 )：创建指定类型的数组

  b = a.astype(int)：转换数组数据类型:

- 两个相同尺寸的数组间可直接运算，同位置的数组元素直接运算

  np.dot(a, b)：a、b向量进行点乘运算

  a@b：矩阵的乘法运算

  np.sqrt(a)：对数组元素依次求平方根

  np.sin(a)、np.cos(a)：对数组元素依次进行三角函数运算

  np.log(a)、np.power(a, n)：对数组元素依次进行对数、指数运算

  a*n：数组与数单独运算，数组元素分别对这个数求积

  不同尺寸数组也可以运算。运算前会进行自我行/列复制至尺寸相同。

- a.min()、a.max()：返回数组最小、最大的元素

  a.argmin()、a.argmax() ：返回数组最小、最大元素的索引

- a.sum()：返回所有数组元素的总和

  a.mean()：返回所有数组元素的平均值

  a.median()：返回所有数组元素的中值

  a.var()：返回所有数组元素的方差

  a.std()：返回所有数组元素的标准方差

- 多维数组的数学运算：a.sum(axis = n)： axis代表维度。n=1时为行，逐行运算；n=2时为列，逐列运算...

- a[i, j]：获取数组第i行j列的元素

  a[逻辑运算符表达式]：筛选并获取符合条件的数组元素

  切片：a[0, 0:2]：获取数组第0行，[0, 2)列的数组元素

  ​           a[0, : ] / [0]：获取数组第0行，所有列的数组元素

  ​           a[m: n: s]：获取从m开始到n结束，跨度为s的数组元素（每隔s个取一个）（跨度可以为负）

  ​           a[: : -1]：获取逆序数组

  ​           a[: : -1, : , : ]：获取第一个维度逆序，其他维度不变的数组


### Matplotlib

- import matplotlib.pyplot as plot

  import numpy as np

#### 基本使用

- 定义数据：

  x = np.linspace(m, n, s)：创建一个从m到n的元素个数为s的等间距x数组

  y = f(x)

- 定义窗口：plt.figure(num = n, figsize = (x, y))：定义一个图像窗口，以下在定义新的图像窗口前，均为对当前窗口操作。参数num为窗口编号，参数figsize为窗口大小。

- 画曲线：plt.plot(x, y, color = 'red', linewidth = 1.0, linestyle = '--')：画出(x, y)曲线。参数color为线条颜色，参数linewidth为线宽，参数linestyle为线条样式。

- 显示图像：plt.show()：显示图像

- 设置坐标轴：

  plt.xlim((-1, 2)) / plt.ylim((-1, 2))：设置坐标轴范围

  plt.xlabel('text') / plt.ylabel('text')：设置坐标轴名称

  newticks = np.linspace(m, n, s)
  print(newticks)
  plt.xticks(newticks)：设置坐标轴刻度

  plt.xticks([1, 2, 3, ...] [r'$text1$'...])：设置坐标轴刻度及名称，名称与刻度一一对应，$符号转换字体，转义字符显示空格及其他字符，r为正则表达式。

  plt.xticks(())：隐藏坐标轴。

### PyQt5



## C++

### 基础知识

#### 头文件

- 不以.c结尾：cmath、cstdio、cstring、iostream。

#### 作用域分辨符：::

- 类成员分辨：Class_name :: member
- 全局作用域分辨：:: name

#### 名字空间

- namespace ns{}：名字空间，防止同名冲突。通过::运算符限定属于哪个namespace。

  3种方法使用名字空间X的名字name：using namespace X：引入整个名字空间，使用时不再用::限制；using X::name：使用空间中的单个名字；X::name：程序中需加上名字空间前缀。

  ```c++
  using namespace std;    // 使用标准名字空间std中所有名字
  ```

#### 标准输入输出

- cout：标准输出流（屏幕窗口），

- cin：标准输入流（键盘），

  ```c++
  cout << "text" << endl ;
  cin >> a;
  ```

#### 引用

- ```c++
  double &b = a;    // 通过别名b直接访问某个变量a
  ```

- ```c++
  // 引用常用用于函数传参，表示形参是实参的引用（同一个对象），则在函数中对形参修改等价于直接对实参修改
  void fun(int &x, int &y) {...} 
  fun(a, b);
  void fun(const int &x, int &y);    // 若用const修改符修饰引用的形参，则函数中x不可被修改
  ```

- 引用在定义时必须初始化，初始化后，不能改变引用的指向。

- 如果如int &f(); a = f();返回函数的引用，则实际为返回了return变量的引用。修改a等同于修改return变量的值。

#### try-catch

- try-catch异常情况处理：

  正常代码放在try块，try通过throw抛出异常。catch中定义形参，获取try块抛出的异常（按类型获取）。

  ```c++
  try {
      throw x ;
  } 
  catch (int result) { } 
  catch (char * s){ }
  ```

#### 动态内存分配

- new：自动分配数个一定大小的内存块，返回连续内存的起始地址。如dp = new double[5];。
- delete[]：释放dp指向的多个double元素占据的内存块。若无[]，则只释放第一个元素的内存块。

#### 重载

##### 函数重载

- C++允许函数同名，只要它们的形参不同，则在调用该名称函数时会自动识别形参类型，调用合适的函数。

  ```c++
  // 函数重载的辨析——const重载：主要通过能否对传入的参数进行修改为判断依据，const重载可看做是对函数隐含的参数this指针的参数重载，因此可以视为函数形参不同。
  class Base{
  public:
  	void test(int a);
  	void test(const int a);    //error，不属于重载，函数名冲突。因为传参是传值，所以不会改变实参的值，被视为同一个函数
  };
  
  class Base{
  public:
  	void test(int &a);
  	void test(const int &a);    //right，属于重载。传参是引用，前一个函数可能改变原值，而后一个不行，实质是不同的函数，所以算重载。
  };
      
  class Base{
  public:
  	void test(int a);
  	void test(int a) const;    //right，属于重载。const成员函数不能改变成员变量，实质是不同的函数，所以算重载。
  };
  ```

##### 运算符重载

- 重新定义运算，方便特殊类型的变量进行运算。

- 重载运算符的函数不能有默认的参数，重载的运算符必须和用户定义的自定义类型的对象一起使用，其参数至少应有一个是类对象。

  ```c++
  [返回值类型] operator [原运算符] (参数1, 参数2) {
      [运算]
  	return [返回值];
  }
  
  String& operator = (const String& other) {    // 例子："=" 运算重载
  	strcpy_s(m_data, strlen(other.m_data) + 1, other.m_data);
    	cout << "赋值成功！" << endl;
    	return *this;    // 备注：=可以用this指针来返回
  }; 
  ```

- 重载运算符可以作为类的成员函数和友元函数，其参数表不同。

  ```c++
  Complex operator+(Complex &c2)                      // 成员函数：c1+c2 被替换为 c1.operator+(c2)，只要一个形参
  friend Complex operator+(Complex &c1, Complex &c2)  // 友元函数：c1+c2被替换为operator+（c1，c2），要两个形参。运算符左侧的操作数与函数第一个参数对应，右侧的操作数与函数的第二个参数对应
  ```

  有的运算符必须定义为类的成员函数（赋值运算符、下标运算符、函数调用运算符），有的运算符不能定义为类的成员函数（流插入“<<”和流提取运算符”>>”、类型转换运算符）。一般将单目运算符重载为成员函数，将双目运算符重载为友元函数。

- 重载流运算符：

  只能将重载>>和<<的函数作为友元函数或普通函数，而不能定义为成员函数。

  ```c++
  friend ostream& operator << (ostream&,Complex&);
  friend istream& operator >> (istream&,Complex&);
  
  ostream& operator << (ostream& output, Complex& c) {    // 形参output是ostream类对象引用
      output << "(" << c.real << "+" << c.imag << "i)" << endl;
  	return output;
  }
  istream& operator >> (istream& input, Complex& c) {      // 形参input是istream类对象引用
  	input >> c.real >> c.imag;
  	return input;
  }
  ```

- 类型转换函数：将一个类的对象转换为另一类型的数据。

  函数名前不能指定函数类型，函数没有参数；其返回值类型由函数名中的类型名来确定；只能作为成员函数，不能作为友元函数或普通函数。

  ```c++
  operator double () {return real；}
  ```

##### 函数

#### 函数模板

- 通过模板函数，自动生成一个针对T类型的具体函数。

- ```c++
  template <class T1, class T2>
  T1 minValue(T1 a, T2 b) {
  	if (a < b) return a;
  	else return (T2)b;
  }
  ```

##### 内联函数

- ```c++
  inline double distance(double a, double b) {return ...}
  ```

- 类似宏定义，在编译时将函数直接替换为函数内语句展开，避免函数调用开销。

- 内联函数内不允许用循环语句和开关语句；内联函数的定义必须出现在每一个调用该函数的源文件之中。

##### 函数指针

- 通过指针调用函数：

  ```c++
  void func(int x){           // 函数原型定义
  	cout << x;
  }
  void (*f)(int x) = func;    // 定义函数指针（函数指针+参数表=原型函数名）
  f(1);                       // 使用函数指针调用函数
  ```

##### 函数缺省值

- 若有实际参数值，则缺省值无效；否则参数值采用缺省值。
- 带有缺省值的参数必须全部放置在参数的最后，即在带有缺省值的参数的右边不再出现无缺省值的参数。

##### 占位符参数（哑元）

- 函数声明时，参数可以没有标识符。为了使将来修改函数功能需要增加一个形式参数时，可利用该哑元，保证函数的接口不发生变化，从而不需要修改程序中的函数调用。

- ```c++
  void f(int x, int = 0 , float = 1.1);          // 声明
  void f(int x, int y, float flt) {x, flt, y}    // 定义
  f(1,2,3.0)                                     // 调用时必须为占位符参数提供一个值
  ```

##### 函数自引用

- this指向调用这个函数的类型对象指针，*this为调用这个函数的那个对象，即调用这个函数的对象本身。

  通过返回自引用return *this，可连续调用对象的函数，如day.add(3).add(7);。

#### 类

- struct与class区别：
  1.  class 类中的成员默认 private ，struct 结构体中的成员默认 public；
  2.  class 继承默认 private 继承，struct 继承默认是 public 继承；
  3.  class 可使用模板，struct 不能。

##### struct类

- struct类定义及初始化：

  ```c++
  struct Date {            // Date类定义
  	int d, m, y;
  	void init(int dd, int mm, int yy) {    // 构造函数
  		d = dd; m = mm; y = yy;
  	}
  	void print() {
  		cout << y << "-" << m << "-" << d << endl;
  	}
  };
  Date day;                // 创建Date类型的day对象。
  day.init(4, 6, 1999);    // 通过对象day调用类Date的init方法。
  ```


##### class类

- class类定义：

  ```c++
  class Student {
  private:            // 使用public和private关键字设置访问限制
  	char *name;     // 类的属性
  	int age;
  public:
  	Student(char *n = "no name", int a = 0) {    // 构造函数
  		name = new char[100];
  		strcpy(name, n);
  		age = a;
  	}
  	~Student() {    // 析构函数
  		delete name;
  	}
  }
  ```

##### 访问限制

- public：可以被该类中的函数、派生类的函数、友元函数、该类的对象访问。
- private：只能由该类中的函数、及其友元函数访问，不能被包括该类对象在内的任何其他访问。
- protected：可以被该类中的函数、派生类的函数、友元函数访问，但不能被该类的对象访问。

##### 构造函数与析构函数

- 构造函数：和类名同名，可以带参数，且无返回类型的函数；在定义对象时会自动被调用，用于初始化类对象成用。构造函数可以重载。

  - 构造函数定义及使用：

    ```c++
    Date(int dd =1, int mm =1, int yy =1) {    // 构造函数
    	d = dd; m = mm; y = yy;
    	cout << "构造函数" << endl;
    }
    Date day(1, 1, 1);    // 构造函数使用（在创建对象时可赋初值）
    ```

  - 拷贝构造函数：用一个对象给另一个对象初始化：

    ```c++
    Student(const Student &s) {    // 拷贝构造函数
    	name = new char[100];
    	strcpy(name, s.name);
    	age = s.age;
    }
    Student s;        // 用来拷贝的原对象s
    Student m(s);     // 用原对象s来拷贝构造新对象m
    ```

  - 默认构造函数：如果没有提供任何构造函数，编译器会创建一个默认的构造函数；如果定义了构造函数，但没有无参构造函数，则创建对象时，若不带参数，将会出错。

- 析构函数：在类对象销毁时被自动调用，用于释放该对象占用的资源，如释放占用的内存、关闭打开的文件。

  ```c++
  ~Date() { }    // 析构函数名由~+类名组成，不带参数，无返回类型，不能被重载。
  ```

##### 友元函数

- 友元函数是不属于类成员的函数，但它可以访问该类的private成员。通过将关键字 friend 放置在类定义内的函数原型声明前，即可将函数声明为友元。友元函数定义在类外。

- ```c++
  friend <return type> <function name> (<parameter type list>);
  ```

##### 成员函数的体外定义

- 类的成员函数可在类的定义外定义，但必须在类定义中声明，且体外定义时应有类作用域：void Date::print() {}。

##### 类模板

- 将原类定义中需要模板化的所有类型改为T，并加上模板头。

- ```c++
  template<class T> 
  class Stack { 
  public: 
  	Stack(int); 
  	void push(T ptr); 
  	T pop(); 
  	T top() const; 
  	int size() const; 
  	~Stack(); 
  private: 
  	T *data; 
  	int length; 
  	int ptr; 
  };
  ```

##### 类继承

- 在一个已存在的类的基础上建立一个新的类。可声明一个基类，在基类中只提供某些基本功能，在声明派生类时加入某些具体的功能，形成适用于某一特定应用的派生类。

- ```c++
  // 基类
   class Animal {
     // eat() 函数
     // sleep() 函数
   };
  
   // 派生类：class [新的类名] : [继承方式] [基类名称] {...}
   class Dog : public Animal {
     // bark() 函数
   };
  
  // 多继承：从多个基类继承：class [新的类名] : [继承方式1] [基类名称1] , [继承方式2] [基类名称2] {...}
  class bird : public Animal, protected FlyingThing, …
  {
      // fly() 函数
  };
  ```

- 继承方式：默认继承方式是私有继承

  1. 公有继承（public）：基类的公有成员也是派生类的公有成员，基类的保护成员也是派生类的保护成员，基类的私有成员不能直接被派生类访问，但是可以通过调用基类的公有和保护成员来访问。
  2. 保护继承（protected）：基类的公有和保护成员将成为派生类的保护成员。
  3. 私有继承（private）：基类的公有和保护成员将成为派生类的私有成员。则基类的成员只能由直接派生类访问，不能再继承。

- 派生类不继承基类的：构造函数和拷贝构造函数、析构函数、重载运算符、友元函数。

- 派生类的构造函数：构造函数不能继承，因此派生类的构造函数只对新增的成员进行初始化，所有从基类继承来的成员初始化工作仍由基类的构造函数完成。

  ```c++
  // 派生类的构造函数写法（同时进行基类构造）
  Student(char* str,char* name) : Person(name) {  // 派生类构造函数，将name传给基类构造函数
  	passed = new char[MAXSIZE];
  	strcpy_s(passed, strlen(str) + 1, str);
  }
  ```

- 虚基类：

  类层次结构中虚基类的成员只出现一次，即基类的一个副本被所有派生类对象所共享。最后的派生类负责对虚基类初始化。

  ```c++
  class C : virtual public A {
  public:
      C(int n): A(n){ }
  };
  ```

##### 多态——虚函数

- ```c++
  virtual <类型说明符><函数名>(<参数表>)
      
  // 派生类重写基类中函数方法：虚函数
  virtual void all_info() {        // 重新定义与基类同名的函数，显示全部信息
  	Person::all_info();          // 通过引用调用基类的同名函数
  	cout << "考试通过科目为：" << passed << endl;
  }
  ```

- 同一个类中，不能定义名字参数个数和类型都相同的两个函数；但在类的继承层次结构中，允许在派生类中重新定义与基类一模一样但功能不同的函数，且可通过基类指针或引用来访问基类和派生类中的同名函数。

  ```c++
  // 基类指针访问基类函数
  // 指向基类对象的指针可以指向其派生类对象，但通过基类指针访问的成员是基类的成员，而不是派生类的成员。
  BaseClass *p;    // 基类指针
  ChildClass child;
  p = &child;      // 指向派生类对象
  p->F();          // 仍然调用基类的函数F()
  ```

- 不是通过重载而是通过代码覆盖实现虚函数。重载同名函数只发生在同一个类内的函数之间，不能跨越基类和派生类。当派生类写一个和基类同名的函数时，此时发生代码覆盖。

- 纯虚函数：一个在基类中声明的虚函数，但没有定义具体的操作，要求派生类根据需要定义自己的版本。

  - ```c++
    virtual <类型说明符 ><函数名>(<参数表>) = 0;    // 纯虚函数头后加"=0"
    // 在基类和派生类中都要用virtual定义为虚函数
    ```

  - 声有纯虚函数的类是一个抽象类（不用来定义对象而只作为一种基本类型用作继承的类），且纯虚函数在抽象类中没有定义。用户不能创建抽象类的实例，只能创建它的派生类的实例。

  - 定义纯虚函数的目的在于使派生类仅仅只是继承函数的接口，让所有的派生类对象都可以执行纯虚函数的动作，但类无法为纯虚函数提供一个合理的默认实现。

  - 纯虚函数必须在继承类中重新声明函数（不加后面的=0）。 若纯虚函数没有被重写，则仍是抽象类，会引发错误。

##### this指针

- 编译器给成员函数传递一个隐藏的对象指针参数，该指针总是指向当前要引用的对象，称为this指针。this指针是常量指针，不可修改值。

- 调用当前对象下的变量n：return this->n 或 return  (\this).n

  调用当前对象下的函数f：this.f()

- this指针可实现成员函数的链式调用（通过函数返回this指针）：

  ```c++
  Class X {
  	X &assign(){ …; return (*this)};
  	X &setvalue(){…; return (*this)};
  }
  X objX;
  objX.assign().setvalue().assign();    // 链式调用
  ```

##### 句柄类

- 一个特殊的类，该类中含有一个指向被隐藏的类的指针。

- 实现部分并没有暴露在.h中，保密性好；实现的细节均隐藏在句柄类的背后，因此当实现发生变化时句柄类没有改变，就不必重新编译客户程序，只需重新编译隐藏的实现这一小部分代码并连接即可。

- 公开的部分：

  ```c++
  #ifndef HANDLE_H
  #define HANDLE_H
  class Handle {
  	struct Cheshire;    // 句柄类cheshire的声明
  	Cheshire* smile;    // smile指针指向具体的实现
  public:
  	void initialize();
  	void cleanup(); 
  	int read();
  	void change(int);
  };
  #endif // HANDLE_H
  ```

- 隐藏的部分：

  ```c++
  // 句柄类的实现: cheshire
  struct Handle::Cheshire {
  	int i;
  };
  void Handle::initialize() {
  	smile = new Cheshire;
  	smile->i = 0;
  }
  void Handle::cleanup() {
  	delete smile;
  }
  int Handle::read() {
  	return smile->i;
  }
  void Handle::change(int x) {
  	smile->i = x;
  }
  ```

##### 静态成员变量

- ```c++
  int f() {
  	static int i = 0;    // 静态成员变量
  	return i; 
  }
  ```

- 类的所有对象共用一个静态成员变量；静态成员不可在类体内进行赋值；通过一个对象给它赋值，其他对象里的该成员也会发生变化；静态成员变量不是对象的一部分。

- 静态成员只能在类内先声明，再在类外进行初始化。 ClassName :: StaticName＝初值。

- 可以通过对象进行访问，也可以ClassName :: StaticName访问。

#### const

- C++编译器不为const创建存储空间，而是把它保存在符号表里，即编译时常量。如果想用运行期间产生的值初始化一个变量，并且知道在该变量的生命期内其值不变，则可用const限定该变量。

- const仅在const被定义过的文件里才可见，不用担心名字冲突。

- 当定义一个const时，必须赋一个值给它，除非用extern做出了清楚的说明；当用extern说明了const时，编译器会强制为const分配空间，而不是保存在符号表中。

- const指针：不可改变指向的指针（const修饰符应在最前面）

  非const对象的地址可赋给const指针；const对象的地址不可赋给非const指针。

  ```c++
  const int *u;    // u是一个const指针，它指向int;
  int const *v;    // v是一个指向恰好是const的int对象的普通指针；
  ```

- 返回引用的const：阻止返回值作为左值出现（如防止类中private对象泄露的陷阱出现）。

  ```c++
  const int& f(){return i};    // 本来i被返回引用，可作为左值被修改，但加入const后避免了返回值作为左值出现
  ```

- const用于函数参数传递：兼顾效率与可靠性。

  ```c++
  char *strcpy(const char& src)
  ```

- 类中的const数据成员：

  ```c++
  const int a;         // const常量数据成员：声明时必须被初始化，且初始化后不可被改变。
  int f() const {};    // const成员函数：在函数头后加入const关键字，不能改变成员变量。
  ```

- const对象：声明为const的对象只能调用声明为const的成员函数。

- <img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20220102090111595.png" alt="image-20220102090111595" style="zoom: 67%;" />

#### 类型转换

- static_cast 转换：

  ```c++
  static_cast < new_type > (expression)    // static类型转换
  ```

  把expression转换为new_type类型。用于：①用于类层次结构中基类和派生类间指针或引用的转换，上行转换是安全的，下行转换时由于没有动态类型检查，所以是不安全的；②用于基本数据类型之间的转换，如把int转换成char，把int转换成enum；③把空指针转换成目标类型的空指针；④把任何类型的表达式转换成void类型。注意：static_cast不能转换掉expression的const等属性。

- dynamic_cast 转换：

  ```c++
  dynamic_cast< type* >(e)    // type必须是一个类类型且必须是一个有效的指针
  dynamic_cast< type& >(e)    // type必须是一个类类型且必须是一个左值
  dynamic_cast< type&& >(e)    // type必须是一个类类型且必须是一个右值
  // e的类型必须符合以下三个条件中的任何一个：
  // 1、e的类型是目标类型type的公有派生类
  // 2、e的类型是目标type的共有基类
  // 3、e的类型就是目标type的类型。
  ```

  dynamic_cast主要用于类层次间的上行转换和下行转换，还可以用于类之间的交叉转换。在类层次间进行上行转换时，dynamic_cast和static_cast的效果是一样的；下行转换时，dynamic_cast具有类型检查的功能，比static_cast更安全。

#### 容器Vector

- 容器定义在C++标准模板库中。std::vector是一种常用的容器，功能与数组相似。

- ```c++
  vector<float> v(3);     //类模板
  v[0]=1;v[1]=2;v[2]=3;
  ```

#### 其他

- 用程序块{ }形成内部作用域，可定义域外部作用域同名的变量，在该块里隐藏外部变量。
- string类：string s2("text"); / string s3(s1);：使用构造函数定义字符串变量，可以赋初值。



## ROS

#### ROS相关网站

- ROS wiki - cn：http://wiki.ros.org/cn
- AutoLabor：http://www.autolabor.com.cn/book/ROSTutorials/

#### ROS概述

##### ROS概念

- ROS 是一个适用于机器人的开源的元操作系统。它提供了操作系统应有的服务，包括硬件抽象，底层设备控制，常用函数的实现，进程间消息传递，以及包管理。它也提供用于获取、编译、编写、和跨计算机运行代码所需的工具和库函数。

  ROS 的主要目标是为机器人研究和开发提供代码复用的支持。ROS是一个分布式的进程（也就是节点）框架，这些进程被封装在易于被分享和发布的*程序包*和功能包集中。

- ROS = Plumbing + Tools + Capabilities + Ecosystem

  - Plumbing：通讯机制（ROS不同节点间交互）
  - Tools：工具软件包（ROS中的开发和调试工具）
  - Capabilities：机器人高层技能（ROS中某些功能的集合，比如:导航）
  - Ecosystem：机器人生态系统（跨地域、跨软件与硬件的ROS联盟）

##### ROS的文件系统

![img](C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\文件系统.jpg)

```
WorkSpace --- 自定义的工作空间
    |--- build:编译空间，用于存放CMake和catkin的缓存信息、配置信息和其他中间文件。
    |--- devel:开发空间，用于存放编译后生成的目标文件，包括头文件、动态&静态链接库、可执行文件等。
    |--- src: 源码
        |-- package：功能包(ROS基本单元)包含多个节点、库与配置文件，包名所有字母小写，只能由字母、数字与下划线组成
            |-- CMakeLists.txt 配置编译规则，比如源文件、依赖项、目标文件
            |-- package.xml 包信息，比如:包名、版本、作者、依赖项...(以前版本是 manifest.xml)
            |-- scripts 存储python文件
            |-- src 存储C++源文件
            |-- include 头文件
            |-- msg 消息通信格式文件
            |-- srv 服务通信格式文件
            |-- action 动作格式文件
            |-- launch 可一次性运行多个节点 
            |-- config 配置信息
        |-- CMakeLists.txt: 编译的基本配置
```

- package.xml：该文件定义有关软件包的属性，例如软件包名称，版本号，作者，维护者以及对其他catkin软件包的依赖性。
  - <description>：描述信息
  - <maintainer>：维护者
  - <license>：许可证
  - <buildtool_depend>：依赖的构建工具
  - <build_depend>：构建此软件包需要的软件包
  - <build_export_depend>：根据这个包构建库所需要的包
  - <exec_depend>：运行该程序包中的代码所需的程序包
- CMakelists.txt：是CMake构建系统的输入，用于构建软件包，这些文件描述了如何构建代码以及将代码安装到何处。

##### ROS通信机制

###### 话题通信（发布订阅模式）

- 以发布订阅的方式实现不同节点之间数据交互的通信模式，用于不断更新的、少逻辑处理的数据传输场景。

- 涉及三个角色：

  ROS Master（管理者）：负责保管 Talker 和 Listener 注册的信息，并匹配话题相同的 Talker 与 Listener，帮助 Talker 与 Listener 建立连接。

  Talker（发布者）：发布消息。

  Listener（订阅者）： Talker 发布的消息会被 Listener 订阅。

  <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\01话题通信模型.jpg" alt="img" style="zoom: 40%;" />

  Talker 与 Listener 的启动无先后顺序要求；Talker 与 Listener 都可以有多个；Talker 与 Listener 连接建立后，不再需要 ROS Master。

###### 服务通信（请求响应模式）

- 以请求响应的方式实现不同节点之间数据交互的通信模式，用于偶然的、对时时性有要求、有一定逻辑处理需求的数据传输场景。

- 涉及三个角色：

  ROS master（管理者）：负责保管 Server 和 Client 注册的信息，并匹配话题相同的 Server 与 Client ，帮助 Server 与 Client 建立连接。

  Server（服务端）：Client 发送请求信息。

  Client（客户端）：Server 返回响应信息。

  <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\02_服务通信模型.jpg" alt="img" style="zoom:40%;" />

  客户端请求被处理时，需要保证服务器已经启动；服务端只能存在一个，客户端可以存在多个。

###### 参数服务器

- 以共享的方式实现不同节点之间数据交互的通信模式，存储一些多节点共享的数据，类似于全局变量。

- 涉及三个角色：

  ROS Master（管理者）：作为一个公共容器保存参数。

  Talker（参数设置者）：Talker 可以向容器中设置参数。

  Listener（参数调用者）：Listener 可以获取参数。

  <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\03ROS通信机制03_参数服务器.jpg" alt="img" style="zoom:40%;" />

#### ROS的Ubuntu安装

- 教程：https://www.guyuehome.com/33716
- 解决 rosdep update 命令出现错误：https://zhuanlan.zhihu.com/p/392082731?utm_source=com.tencent.androidqqmail&utm_medium=social&utm_oi=746194479043579904

#### ROS命令

##### 执行相关命令

- roscore：启动ROS。

- rosrun [package_name] [node_name]：运行指定包中的指定节点。

  rosrun rqt_plot rqt_plot：在滚动时间图上显示发布到某个话题上的数据。

  rosrun rqt_graph rqt_graph：用计算图动态显示系统中正在发生的事情。

- roslaunch [package] [filename.launch]：启动定义在指定package功能包中的launch。

  <launch>：根标签

  <node>：启动节点。pkg：节点所在功能包名称；type：节点可执行文件名称；name：节点运行时的名称；output：设置日志的输出目标。

  <param>：在参数服务器上设置参数，为node标签子集。name：参数名；value：参数值；type：参数类型。

  <rosparam>：从 YAML 文件导入参数，或将参数导出到 YAML 文件，也可用来删除参数，为node标签子集。file：文件路径；param：参数名称；command="load | dump | delete" （可选，默认 load）：加载、导出或删除参数。

  <arg>：launch文件内部局部变量。name：命名空间/参数名；default：默认值；value：参数值。调用：value="$(arg arg_name)"。

  <remap>：话题重命名，为node标签子集。from：原命名；to：重命名。

  <include>：包含其他launch文件。file：文件路径。

##### 文件系统相关命令

- catkin_create_pkg [package_name] [depends]：创建新的ROS功能包

- sudo apt install xxx：安装 ROS功能包

- sudo apt purge xxx：删除某个功能包

- rospack list：列出所有功能包

- rospack find [package_name]：获取软件包有关信息，find参数返回软件包所在路径。

  rospack depends1 [package_name]：获取软件包的直接依赖

  rospack depends [package_name]：获取软件包的所有依赖

- roscd [locationname[/subdir]]：切换目录到某个软件包或软件包子目录中。

- rosls [locationname[/subdir]]：显示指定目录内容。

- apt search xxx：搜索某个功能包

- rosed [package_name] [file_name]：编辑功能包文件

##### 通信相关命令

- rosnode list：显示当前正在运行的ROS节点信息。

  rosnode info [node_name]：显示某个指定节点的信息

  rosnode ping：测试到节点的连接状态

  rosnode machine：列出指定设备上节点

  rosnode kill [node_name]：杀死某个节点

  rosnode cleanup：清除不可连接的节点

- rostopic：获取ROS话题信息。

  rostopic echo [topic]：显示在某个话题上发布的数据。

  rostopic list ：列出当前已被订阅和发布的所有话题。

  ​	rostopic list -v : 获取话题详情。

  rostopic type [topic]：查看所发布话题的消息类型

  rostopic find [type]：根据消息类型查找话题

  rostopic pub [-r f] [topic_name] [msg_type] [args]：把数据发布到当前正在广播的话题上，-r f表示以每分钟f次循环。

  rostopic hz [topic]：查看数据发布的频率。

  rostopic bw：显示主题使用的带宽

  rostopic delay：显示带有 header 的主题延迟

- rosmsg：显示有关 ROS消息类型的 信息的命令行工具。

  rosmsg show [msg_name]：显示消息描述

  rosmsg info [msg_name]：显示消息信息

  rosmsg list：列出所有消息

  rosmsg md5：显示 md5 加密后的消息

  rosmsg package [package_name]：显示某个功能包下的所有消息

  rosmsg packages：列出包含消息的功能包

- rosservice：通过服务附加到ROS客户端/服务器框架上。

  rosservice list：输出所有服务的信息。

  rosservice args：打印服务参数

  rosservice call [service] [args]：用给定参数调用指定服务。

  ​		rosservice call /clear：清屏。

  rosservice type [service]：输出指定服务的类型。

  rosservice find：按服务的类型查找服务。

  rosservice uri：输出服务的ROSRPC uri。

- rosrv：显示有关ROS服务类型的信息（类似于rosmsg）

  rossrv show [msg_name]：显示服务消息详情

  rossrv info [msg_name]：显示服务消息相关信息

  rossrv list：列出所有服务信息

  rossrv md5：显示 md5 加密后的服务消息

  rossrv package [package_name]：显示某个包下所有服务消息

  rossrv packages：显示包含服务消息的所有包

- rosparam：使用YAML编码文件在参数服务器上获取和设置ROS参数

  rosparam list：列出节点的参数名。

  rosparam set [param_name]：设置指定参数的参数值。

  rosparam get [param_name]：获取指定参数的参数值。

  rosparam load [file_name] [namespace]：从指定文件中加载参数。

  rosparam dump [file_name] [namespace]：向指定文件中写入参数。

  rosparam delete：删除参数。

##### 其他

- Tab补全：当按Tab键后，命令行会自动补充剩余部分。

- 创建catkin工作空间：

  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  catkin_make
  source devel/setup.bash      # source新生成的setup.*sh
  echo $ROS_PACKAGE_PATH /home/<username>/catkin_ws/src:/opt/ros/<distro>/share  # 确定ROS_PACKAGE_PATH环境变量包含你当前的工作空间目录 
  ```

- 创建软件包：

  ```bash
  cd ~/catkin_ws/src    # 切换到创建的空白catkin工作空间中的源文件空间目录
  catkin_create_pkg beginner_tutorials std_msgs rospy roscpp  # 使用catkin_create_pkg命令创建一个名为beginner_tutorials的新软件包（依赖于std_msgs、roscpp和rospy）
  ```

- catkin_create_pkg <package_name> [depend1] [depend2] [depend3]：在当前目录下创建catkin软件包，需输入软件包名称和依赖的其他软件包。

- 创建发布者publisher代码：

  初始化ROS节点->向ROS Master注册节点信息->创建消息数据->按照一定频率循环发布消息

  ```c++
  #include "ros/ros.h"    //包含头文件
  #include "std_msgs/String.h"
  #include <sstream>
  
  int main(int argc, char **argv){
      ros::init(argc, argv, "file_name");    //初始化ROS
      ros::NodeHandle n;    //为节点创建句柄
      ros::Publisher topic_pub = n.advertise<std_msgs::String>("topic_name", 1000);    //向ROS Master注册节点信息，包括发布的话题名、话题中的消息类型和队列大小
      ros::Rate loop_rate(10);    //指定循环发布的频率
      int count = 0;
      while (ros::ok()){
          std_msgs::String msg;    //创建一个消息
          std::stringstream ss;
          ss << "hello world " << count;
          msg.data = ss.str();
          ROS_INFO("%s", msg.data.c_str());    //输出有关信息
          topic_pub.publish(msg);    //把信息广播给已连接的节点
          
          ros::spinOnce();    //调用回调函数
          loop_rate.sleep();    //使用sleep方法等待一定时间，以达到设定的发布速率
          ++count;
      }
      return 0;
  }
  ```

  配置CMakeList.txt中编译规则：

  add_executable(node_name src/node_name.cpp)

  Target_link_libraries(node_name ${catkin_LIBRARIES})

- 创建订阅者subscriber代码：

  初始化ROS节点->订阅需要的话题->循环等待话题消息，接收到消息后进入回调函数->在回调函数中完成消息处理

  ```c++
  #include "ros/ros.h"
  #include "std_msgs/String.h"
  
  void topicCallback(const std_msgs::String::ConstPtr& msg){    //回调函数，当有新消息到达话题时被调用
      ROS_INFO("I heard: [%s]", msg->data.c_str());
  }
  
  int main(int argc, char **argv){
      ros::init(argc, argv, "listener");
      ros::NodeHandle n;
      ros::Subscriber sub = n.subscribe("topic_name", 1000, topicCallback);    //订阅topic话题，每当有新消息到达时，ROS调用回调函数，第二个参数是队列大小
      ros::spin();    //启动自循环，等待调用回调函数
      return 0;
  }
  ```

- 创建服务端server代码：

  初始化ROS节点->创建server实例->循环等待服务请求，进入回调函数->在回调函数中完成服务功能的处理，并反馈应答数据

  ```c++
  #include "ros/ros.h"
  #include "beginner_tutorials/AddTwoInts.h"
  
  bool pubCommand = false;
  
  bool commandCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res){
      ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");    //显示请求数据
  
    	res.success = true;   	//设置反馈数据
  	  res.message = "Change turtle command state!"
      return true;
  }
  
  int main(int argc, char **argv){
      ros::init(argc, argv, "add_two_ints_server");
      ros::NodeHandle n;
  
      ros::ServiceServer service = n.advertiseService("/spawn", commandCallback);    //创建一个名为service的实例，注册回调函数commandCallback
      ROS_INFO("Ready to add two ints.");
      //循环等待回调函数
      ros::Rate loop_rate(10);
    
      while(ros::ok()){
          ros::spinOnce();
  		    if(pubCommand){    //如果标志为true则发布指令
  			      geometry_msgs::Twist vel_msg;
  			      vel_msg.linear.x = 0.5;
  			      vel_msg.angular.z = 0.2;
  			      turtle_vel_pub.publish(vel_msg);
  		    }
  	      loop_rate.sleep();    //按照循环频率延时
      }
  
      return 0;
  }
  ```

- 创建客户端client代码：

  初始化ROS节点->创建一个client实例->发布服务请求数据->等待server处理后的应答结果

  ```c++
  #include "ros/ros.h"
  #include "beginner_tutorials/AddTwoInts.h"
  #include <cstdlib>
  
  int main(int argc, char **argv){
      ros::init(argc, argv, "add_two_ints_client");
      ros::NodeHandle n;
    
      ros::service::waitForService("/spawn")    //发现/spawn服务
      ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("service_name");    //为服务创建一个客户端，并连接名为service_name的服务
      beginner_tutorials::AddTwoInts srv;    //初始化一个服务类，并赋值
      srv.request.a = atoll(argv[1]);
      srv.request.b = atoll(argv[2]);
      if (client.call(srv)){    //调用服务，直到调用完成后返回
          ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      }
      else{
          ROS_ERROR("Failed to call service add_two_ints");
          return 1;
      }
    
      return 0;
  }
  ```

#### 可视化工具箱

##### rqt工具箱

- 命令：

  rosrun rqt：启动rqt工具箱。

  rosrun rqt_console rqt_console：连接到ROS日志框架，以显示节点输出信息。

  rosrun rqt_logger_level rqt_logger_level：允许我们在节点运行时改变输出信息的详细级别。

  rosrun rqt_graph rqt_graph：计算图可视化工具。

  rosrun rqt_plot rqt_plot：数据绘图工具。

  rosrun rqt_image_view rqt_image_view：图像渲染工具（摄像头）。

- 插件：可以通过 plugins 添加所需的插件

  rosrun rqt_graph、rosrun rqt_console、rqt_plot、rqt_bag。

#### 机器人仿真

如果非仿真环境，使用 URDF 结合 Rviz 直接显示感知的真实环境信息；如果是仿真环境，需要使用 URDF 结合 Gazebo 搭建仿真环境，并结合 Rviz 显示感知的虚拟环境信息。

##### URDF

- 统一机器人描述格式，可以以 XML 的方式描述机器人的部分结构。用于创建机器人模型。

##### Rviz

- ROS的三维可视化工具。主要目的是以三维方式显示ROS消息，可将数据进行可视化表达。用于搭建仿真环境。
- rosrun rviz rviz：运行rviz。

##### Arbotix

- 控制机器人运动的ROS功能包，有多种控制器，通过接受速度控制指令更新机器人的 joint 状态。

##### Gazebo

- 3D动态模拟器，用于显示机器人模型并创建仿真环境,能够在复杂的室内和室外环境中准确有效地模拟机器人。用于图形化的显示机器人各种传感器感知到的环境信息。
- rosrun gazebo_ros gazebo：运行gazebo。

#### ROS编程案例

- ROS编程的步骤：
  1. 先创建一个工作空间；
  2. 再创建一个功能包；
  3. 编辑源文件；
  4. 编辑配置文件；
  5. 编译并执行。

##### 打印HelloWorld：

1. 创建工作空间并初始化

   ```bash
   mkdir -p pkg_name/src
   cd pkg_name
   catkin_make
   ```

2. 进入 src 创建 ros 包并添加依赖

   ```bash
   cd src
   catkin_create_pkg pkg_name roscpp rospy std_msgs
   ```

- **C++版：**

  1. 进入 ros 包的 src 目录编辑源文件

     ```bash
     cd pkg_name
     touch HelloWorld.cpp
     ```

     ```c++
     #include "ros/ros.h"
     
     int main(int argc, char *argv[]) {
         ros::init(argc,argv,"HelloWorld");  // ros 节点初始化
         ros::NodeHandle n;            // 创建 ros 节点句柄(非必须)
         ROS_INFO("Hello World!!!!");  // 控制台输出 hello world
     
         return 0;
     }
     ```

  2. 编辑 ros 包下的 Cmakelist.txt文件

     ```
     add_executable(HelloWorld
       src/HelloWorld.cpp
     )
     target_link_libraries(HelloWorld
       ${catkin_LIBRARIES}
     )
     ```

  3. 进入工作空间目录并编译

     ```bash
     cd pkg_name
     catkin_make
     ```

  4. 执行

     ```bash
     roscore
     cd pkg_name
     source ./devel/setup.bash
     rosrun pkg_name HelloWorld.cpp
     ```

     可以把source ~/pkg_name/devel/setup.bash添加入.bashrc文件，使用上更方便。使用 gedit 或 vi 编辑 .bashrc 文件，在文件最后添加该内容。

- **Python版：**

  1. 进入 ros 包添加 scripts 目录并编辑 python 文件

     ```bash
     cd pkg_name
     mkdir scripts
     touch HelloWorld.py
     ```

     ```python
     #! /usr/bin/env python
     
     import rospy
     
     if __name__ == "__main__":
         rospy.init_node("HelloWorld")
         rospy.loginfo("Hello World!!!!")
     ```

  2. 进入 ros 包添加 scripts 目录并编辑 python 文件

     ```bash
     chmod +x HelloWorld.py
     ```

  3. 编辑 ros 包下的 CamkeList.txt 文件

     ```
     catkin_install_python(PROGRAMS scripts/HelloWorld.py
       DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
     ```

  4. 进入工作空间目录并编译

     ```bash
     cd pkg_name
     catkin_make
     ```

  5. 进入工作空间目录并执行

     ```bash
     roscore
     cd pkg_name
     source ./devel/setup.bash
     rosrun pkg_name HelloWorld.py
     ```



## OpenCV

#### 图像操作

##### 图像读写及基本操作

- cv2.imread('picture.jpg', 0)：加载图片

  参数1：图片的文件名。

  参数2：读入方式，省略即采用默认值。cv2.IMREAD_COLOR：彩色图，默认值(1)；cv2.IMREAD_GRAYSCALE：灰度图(0)；cv2.IMREAD_UNCHANGED：包含透明通道的彩色图(-1)

- cv2.imshow(’window_name‘,img)：显示图片

  参数1：窗口名称；参数2：要显示的图片变量

- cv2.imwrite('picture.jpg', img, save_type)：保存图片。参数1位包含后缀名的文件名，参数2为图片变量，参数3为保存质量。

- cv2.waitKey(n)：程序等待。参数为等待毫秒数，若为传入0则一直等待，可以通过k = cv2.waitKey(0)获取键盘输入。

- cv2.namedWindow(’window_name‘, cv2.WINDOW_NORMAL)：创建一个窗口。参数1是窗口名称，参数2默认是cv2.WINDOW_AUTOSIZE，表示窗口大小自适应图片；也可设置为cv2.WINDOW_NORMAL，表示窗口大小可调整。

- px = img[m,n]：通过行列坐标来获取像素点的值，返回值为BGR的列表，m为行号n为列号。

  img[m,n] = [b, g, r]：通过行列坐标来设置像素点的值。

- img.shape()：获取图像形状，返回包含行数（高度）、列数（宽度）和通道数的元组。

- Img.dtype()：获取图像数据类型。

- Img.size()：获取图像总像素数。

- b, g, r = cv2.split(img)：分开访问图像的BGR三通道。

  img = cv2.merge((b, g, r))：将单独的BGR三通道合一为一个图像。

  b = img[:, :, 0]：提取BGR通道。img列表参数1、2为行列，参数3为通道（0b，1g，2r）。

##### 颜色转换

- cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)：转换颜色。参数1为图片变量，参数2为转换方式。

  ​	cv2.COLOR_BGR2GRAY：彩色图转成灰度图

  ​	cv2.COLOR_BGR2HSV：将彩色图转化为HSV

##### 阈值分割

- cv2.inRange(img, lower, upper)：将图像实现二值化，阈值内像素设置为白色，阈值外像素设置为黑色。参数1为要处理的图像变量，参数2位下边界，参数3位上边界。

- ret, th = cv2.threshold(img, lower, upper, type)：实现阈值分割。

  返回值ret为当前阈值，th为处理后的图像。

  参数1为图像变量（一般为灰度图）

  参数2为下阈值，参数3位上阈值（一般为255）

  参数4为阈值方式：

  ​	cv2.THRESH_BINARY：高于阈值的设为黑色，低于的设为白色；

  ​    cv2.THRESH_BINARY_INV：高于阈值的设为白色，低于的设为黑色；	

  ​	cv2.THRESH_TRUNC：高于阈值部分不变，低于部分设为阈值；

  ​	cv2.THRESH_TOZERO：高于阈值部分设为黑色，低于部分不变；

  ​	cv2.THRESH_TOZERO_INV：高于阈值部分不变，低于部分设为黑色。

  ​	cv2.THRESH_OTSU：使用Otsu算法计算阈值。使用Otsu算法前先进行滤波。

- th2 = cv2.adaptiveThreshold(img, upper, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 4)：自适应阈值，每次取图像小部分计算，适用于明暗分布不均的图片。

  参数1为图像变量，参数2为最大阈值

  参数3为小区域内阈值计算方式：

  ​	cv2.ADAPTIVE_THRESH_MEAN_C：小区域内取均值

  ​	cv2.ADAPTIVE_THRESH_GAUSSIAN_C：小区域内加权求和

  参数4为阈值方法，只能使用THRESH_BINARY、THRESH_BINARY_INV

  参数5为小区域的边长

  参数6：最终阈值等于小区域计算出的阈值减去此值

##### 图像几何变换

- roi = img[m:n , p:q]：截取(m,n)到(p,q)矩形范围部分的图像。

- cv2.resize(img, output, size, fx, fy, interpolation)：图片缩放。

  参数1为图片变量，参数2为输出图片变量

  参数3位输出图片尺寸，参数4为沿x、y轴缩放系数

  参数5为插入方式：

  ​	cv2.INTER_NEAREST：最近邻插值

  ​	cv2.INTER_LINEAR：双线性插值（默认）

  ​	cv2.INTER_AREA：使用像素区域关系进行重采样

  ​	cv2.INTER_CUBIC：4×4像素邻域双三次插值

  ​	cv2.INTER_LANCZOS4：8×8像素邻域的Lanczos插值

- cv2.flip(img, 0)：图片翻转。参数1为图片变量；参数2=0为垂直翻转，>0为水平翻转，<0位中心对称翻转。

- cv2.warpAffine(img, M, (cols, rows))：图片仿射变换。参数1为图片变量，参数2为变换矩阵，参数3为输出图片大小。

  M = np.float32([[1, 0, tx], [0, 1, ty]])：生成平移变换矩阵，tx、ty为x、y轴平移距离。

  M = cv2.getRotationMatrix2D((cols / 2, rows / 2), 45, 0.5)：生成旋转变换矩阵。参数1为图片旋转中心，参数2为图片旋转角度（正为逆时针），参数3为缩放比例。

- cv2.warpPerspective(img, M, (w, h))：图片透视变换。参数1为图像变量，参数2为变换矩阵，参数3为生成目标图像大小。

  M = cv2.getPerspectiveTransform(pts1, pts2)：生成透视变换矩阵。参数1为原图四个顶点坐标的矩阵，参数2为目标图片四个顶点坐标的矩阵。

  ```python
  例：# 原图中卡片的四个角点
  pts1 = np.float32([[148, 80], [437, 114], [94, 247], [423, 288]])
     # 变换后分别在左上、右上、左下、右下四个点
  pts2 = np.float32([[0, 0], [320, 0], [0, 178], [320, 178]])
  ```

##### 图像叠加

- cv2.add(x, y)：叠加两张大小和通道数相同的图像。

  cv2.addWeighted(img1, a, img2, b, c)：按照权重叠加两张大小和通道数相同的图像，dst=a×img1+b×img2+c。

- cv2.bitwise_and(img1, img2, mask)：对图像进行按位与运算。参数1、2为输入图像变量，参数3为图像掩膜。

  创建掩膜，并覆盖在图片的一部分上：

  ```python
  img1 = cv2.imread('lena.jpg')
  img2 = cv2.imread('opencv-logo-white.png')
  
  # 把logo放在左上角，所以我们只关心这一块区域
  rows, cols = img2.shape[:2]    #图片2的长宽
  roi = img1[:rows, :cols]    #剪下图片1中左上角等于图片2大小的部分
  # 创建掩膜
  img2gray = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)    #图像2转化为灰度图
  ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)    #图像2转化为黑白图即掩膜1
  mask_inv = cv2.bitwise_not(mask)    #创造图片2自身的掩膜2（黑白反色）
  # 保留除logo外的背景
  img1_bg = cv2.bitwise_and(roi, roi, mask)    #掩膜1覆盖在图片1剪切部分
  img2_bg = cv2.bitwise_and(img2, img2, mask_inv)    #掩膜2覆盖在图片2上
  dst = cv2.add(img1_bg, img2)    #掩膜覆盖后的图片1与覆盖后的图片2融合
  img1[:rows, :cols] = dst    #融合后将剪切部分恢复到原图上
  ```

  cv2.bitwise_not()，cv2.bitwise_or()，cv2.bitwise_xor()：执行按位或/非/异或运算

##### 图像滤波

- cv2.blur(img, (x, y))：均值滤波，取卷积核区域内元素的均值。参数1为图像变量，参数2为卷积核大小。

- cv2.boxFilter(img, -1, (x, y), normalize=True)：方框滤波，取卷积核区域内的像素和。参数1为图像变量，参数3为卷积核大小，参数4为状态，normalize=True时为均值滤波，normalize=False时为方框滤波。

- cv2.GaussianBlur(img, (x, y), sigmaX)：高斯滤波，取卷积核内部按正态分布的加权平均值。参数1为图像变量，参数2为卷积核大小，参数3为正态分布中的σ。

- cv2.medianBlur(img, d)：中值滤波，将方框内元素比较，取中值为当前值。参数1为图像变量，参数2为方框边长。

- cv2.bilateralFilter(img, d, sigmacolor, sigmaspace)：双边滤波，在高斯滤波过程中考虑不融合、保留线条边缘信息。参数1为图像变量，参数2为方框边长，参数3为空间高斯函数标准差，参数4为灰度值相似性高斯函数标准差。

- 优先高斯滤波cv2.GaussianBlur()，然后均值滤波cv2.blur()。

  斑点和椒盐噪声优先使用中值滤波cv2.medianBlur()。

  要去除噪点的同时尽可能保留更多的边缘信息，使用双边滤波cv2.bilateralFilter()。

  线性滤波方式：均值滤波、方框滤波、高斯滤波（速度相对快）。

  非线性滤波方式：中值滤波、双边滤波（速度相对慢）。

##### 边缘检测

- cv2.Canny(img, lower, upper)：边缘检测。参数1为图像变量，参数2、3为最低、高阈值。

  推荐先进性阈值分割（Otsu）再边沿检测。

##### 轮廓与轮廓特征

- image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)：寻找轮廓。

  返回值1为原图，返回值2为轮廓间的层级关系，参数3为找到的轮廓（数组存储坐标形式）。

  参数1为阈值分割二值化后的图像变量；参数2为轮廓查找方式，一般使用cv2.RETR_TREE，表示提取所有的轮廓并建立轮廓间的层级；参数3为轮廓的近似方法，使用cv2.CHAIN_APPROX_SIMPLE就表示用尽可能少的像素点表示轮廓。

- cv2.drawContours(img, contours, -1, (b, g, r), thickness)：绘制轮廓。参数1为要绘制的图像变量，参数2为轮廓间的层级关系，参数3为-1表示绘制所有轮廓，参数4为轮廓颜色bgr值，参数5为线宽。

- cv2.countNonZero(cnt)：求轮廓面积（像素点个数）。参数1为轮廓坐标数组。

- cv2.arcLength(cnt, True)：求轮廓周长。参数1为轮廓坐标数组，参数2表示轮廓是否封闭。

- x, y, w, h = cv2.boundingRect(cnt)：求轮廓的外接矩形。返回值分别为矩形左上角坐标和矩形宽高。参数1为轮廓坐标数组。

  cv2.rectangle(img_color1, (x, y), (x + w, y + h), (b, g, r), thickness)：绘制出轮廓外接矩形。

- rect = cv2.minAreaRect(cnt)：寻找轮廓外接最小矩形。参数1为轮廓坐标数组。

  box = np.int0(cv2.boxPoints(rect)) ：矩形的四个角点坐标取整。cv2.drawContours(img_color1, [box], -1, (b, g, r), thickness)：绘制轮廓外接最小矩形。参数1为目标图像变量，参数2为矩形轮廓列表，参数3为-1表示绘制列表中所有轮廓。

- (x, y), radius = cv2.minEnclosingCircle(cnt)：寻找轮廓最小外接圆。参数1为轮廓坐标数组。

  (x, y, radius) = np.int0((x, y, radius))： 圆心和半径取整。

  cv2.circle(img_color2, (x, y), radius, (b, g, r), thickness)：绘制轮廓外接最小外接圆。参数1为目标图像变量，参数2、3为圆心位置与半径。

- ellipse = cv2.fitEllipse(cnt)：寻找轮廓拟合椭圆。参数1为轮廓坐标数组。

  cv2.ellipse(img_color2, ellipse, (b, g, r), thickness)：绘制轮廓拟合椭圆。参数1为目标图像变量，参数2为椭圆变量。

- cv2.matchShapes(cnt_a, cnt_b, method, 0.0)：比较两个形状之间的相似度，返回值越小，越相似。参数1、2为两个形状的轮廓坐标数组，参数3为匹配方法。

- cv2.approxPolyDP(cnt, epsilon, close)：多边形轮廓逼近。参数1为轮廓坐标数组；参数2表示多边形轮廓接近实际轮廓的程度，值越小越精确；参数3表示轮廓是否闭合。

- cv2.convexHull(cnt)：寻找轮廓凸包。返回值为凸包顶点坐标列表，参数1为轮廓坐标数组。

- cv2.pointPolygonTest(cnt, (x, y), True)：计算点到轮廓的最短距离。参数1为轮廓坐标数组；参数2为点坐标，参数3为True时表示计算距离（点在轮廓外为负，在内为正），为False时只返回-1/0/1表示相对轮廓位置，不计算距离。

##### 腐蚀与膨胀

- cv2.erode(img, kernel)：对二值化图像白色部分进行腐蚀。参数1为图像变量，参数2为用于腐蚀的结构元素。可使用kernel = np.ones((d, d), np.uint8)创建结构元素。
- cv2.dilate(img, kernel)：对二值化图像白色部分进行膨胀。参数1为图像变量，参数2为用于膨胀的结构元素。
- cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)：开运算（先腐蚀后膨胀），消除物体外的小白点。参数1为图像变量，参数2为开/闭运算状态，参数3为结构元素。
- cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)：闭运算（先膨胀后腐蚀），消除物体内的小白点。参数1为图像变量，参数2为开/闭运算状态，参数3为结构元素。

##### 直方图

- cv2.calcHist([img], channels, mask, histSize, ranges)：计算直方图。参数1为图像变量（以方括号传入），参数2为图像通道数（灰度图为[0]，彩色图B/G/R分别传入[0]/[1]/[2]），参数3为要计算的区域（计算整幅图的为None），参数4为直方图子区段数目，参数5为要计算的像素值范围。

  ```python
  hist = cv2.calcHist([image],[0],None,[256],[0,256])    #灰度图，传入一个通道
  hist= cv2.calcHist([chans[0],chans[2]],[0,1],None,[32,32],[0,256,0,256])    #BG图，传入两个通道
  ```

- Matplotlib绘制直方图：

  ```python
  plt.hist(img.ravel(), 256, [0, 256])    #计算并绘制直方图
  或者 plt.plot(hist)    #用上文的计算结果绘制
  plt.show()
  ```

- cv2.equalizeHist(img)：直方图均衡化。参数1为图片变量。

  cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))：对比度受限的自适应直方图均衡化。参数1为颜色对比度的阈值；参数2为进行像素均衡化的网格大小。

##### 模板匹配

- cv.matchTemplate( img, templ, result, match_method)：模板匹配。

  参数1为原始图像变量，参数2为模板图像变量

  参数3为目标存放矩阵，矩阵每个位置都存有匹配值（最白的地方表示匹配程度最高）。

  参数4为匹配方法：

  ​    CV_TM_SQDIFF：平方差匹配，越小越好

  ​    CV_TM_SQDIFF_NORMED：标准平方差匹配，越小越好

  ​    CV_TM_CCORR：相关匹配（乘法操作），越大越好

  ​    CV_TM_CCORR_NORMED：标准相关匹配，越大越好

  ​    CV_TM_CCOEFF：相关匹配（先减去平均值），越大越好

  ​    CV_TM_CCOEFF_NORMED：标准相关匹配，越大越好

- min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result, minVal, maxVal, minLoc, maxLoc, Mat())：确定矩阵最大值和最小值的位置。参数1为匹配结果矩阵，参数2、3为在矩阵中存储的最小值和最大值，参数4、5为矩阵中最大值和最小值的坐标，参数6为可选的掩膜。

- 画出矩形：

  ```python
  left_top = max_loc  #左上角
  right_bottom = (left_top[0] + w, left_top[1] + h)  #右下角
  cv2.rectangle(img, left_top, right_bottom, 255, 2)  #画出矩形
  ```

- 匹配多个物体：设定一个匹配阈值，输出所有高于阈值的匹配结果。

  ```python
  loc = np.where(res >= threshold)  # 匹配程度大于threshold的坐标y，x
  for pt in zip(*loc[::-1]):  #loc[::-1]逆序矩阵，zip函数转置矩阵
      right_bottom = (pt[0] + w, pt[1] + h)
      cv2.rectangle(img_rgb, pt, right_bottom, (0, 0, 255), 2)
  ```

##### 霍夫变换

- 标准霍夫直线变换（整条直线）：lines = cv2.HoughLines(edges, 0.8, np.pi / 180, 90)。

  返回值为每行2个元素的列表。

  参数1为图像变量（一般为阈值分割或边沿检测后的图）；参数2为距离r的精度，越大考虑的线越多；参数3为角度θ的精度，越小考虑的线越多；参数4为相同点的累加数阈值，值越小，考虑的线越多。

  绘制标准霍夫直线变换的直线：（极坐标）

  ```python
  for line in lines:
      rho, theta = line[0]
      a = np.cos(theta)
      b = np.sin(theta)
      x0 = a * rho
      y0 = b * rho
      x1 = int(x0 + 1000 * (-b))
      y1 = int(y0 + 1000 * (a))
      x2 = int(x0 - 1000 * (-b))
      y2 = int(y0 - 1000 * (a))
      cv2.line(drawing, (x1, y1), (x2, y2), (0, 0, 255))
  ```

- 统计概率霍夫变换（部分直线）：lines = cv2.HoughLinesP(edges, 0.8, np.pi / 180,  90, minLineLength=50, maxLineGap=10)。

  返回值为每行4个元素x1、y1、x2、y2的列表。

  参数1为图像变量；参数2为距离r的精度，越大考虑的线越多；参数3为角度θ的精度，越小考虑的线越多；参数4为相同点的累加数阈值，值越小，考虑的线越多；参数5为最短长度阈值，比该长度短的线会被排除；参数6为同一直线两点之间的最大距离。

  绘制统计概率霍夫直线变换的直线：

  ```python
  for line in lines:
      x1, y1, x2, y2 = line[0]
      cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1, lineType=cv2.LINE_AA)
  ```

- 霍夫圆变换：circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, 1, 20, param2=30)。

  返回每行3个元素x，y，r的列表。

  参数1为图像变量；参数2为变换方法，一般使用霍夫梯度法cv2.HOUGH_GRADIENT；参数3dp=1，表示霍夫梯度法中累加器图像的分辨率与原图一致；参数4为两个不同圆圆心最短距离；参数5为相同点的累加数阈值，值越小，考虑的圆周越多。

  绘制霍夫圆变换的圆：

  ```python
  for i in circles[0, :]:
      cv2.circle(drawing, (i[0], i[1]), i[2], (0, 255, 0), 2)  # 画出外圆
      cv2.circle(drawing, (i[0], i[1]), 2, (0, 0, 255), 3)  # 画出圆心
  ```

##### Example：车道线识别

<img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\cv2_lane_detection_roi_sample.jpg" alt="img" style="zoom:33%;" />

1. 灰度化

   ```python
   gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
   ```

2. 高斯滤波

   ```python
   blur_gray = cv2.GaussianBlur(gray, (blur_ksize, blur_ksize = 5), 1)
   ```

3. Canny边缘检测

   ```python
   edges = cv2.Canny(blur_gray, canny_lth = 50, canny_hth = 150)
   ```

   <img src="C:\Users\user\Desktop\cv2_lane_detection_canny_result.jpg" alt="cv2_lane_detection_canny_result" style="zoom: 33%;" />

4. 不规则ROI区域选取

   创建梯形的mask掩膜，与边缘检测结果图混合运算，掩膜中白色的部分保留，黑色的部分舍弃。

   ```python
   rows, cols = edges.shape
   points = np.array([[(0, rows), (460, 325), (520, 325), (cols, rows)]])    #标记四个坐标点用于ROI截取
   
   mask = np.zeros_like(img)    #创建掩膜
   cv2.fillPoly(mask, corner_points, 255)    #绘制多边形并填充
   masked_img = cv2.bitwise_and(img, mask)    #覆盖掩膜
   roi_edges = masked_img    #赋值给roi图
   ```

   <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\cv2_lane_detection_masked_roi_edges.jpg" alt="只保留关键区域的边缘检测图" style="zoom:33%;" />

5. 霍夫直线变换

   ```python
   #霍夫直线提取
   drawing, lines = hough_lines(roi_edges, rho = 1, theta = np.pi/180, threshold = 15, min_line_len = 40, max_line_gap = 20)
   
   def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
       # 统计概率霍夫直线变换
       lines = cv2.HoughLinesP(img, rho, theta, threshold, minLineLength=min_line_len, maxLineGap=max_line_gap)
       # 新建一副空白画布
       drawing = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
       # draw_lines(drawing, lines)     # 画出直线检测结果
       return drawing, lines
   
   def draw_lines(img, lines, color=[0, 0, 255], thickness=1):
       for line in lines:
           for x1, y1, x2, y2 in line:
               cv2.line(img, (x1, y1), (x2, y2), color, thickness)
   ```

   <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\cv2_lane_detection_hough_lines_direct_result.jpg" alt="霍夫变换结果图" style="zoom:33%;" />

6. 车道线计算：只得到左右两条车道线

   1. 根据斜率正负划分线是左车道还是右车道
   2. 迭代计算各直线斜率与斜率均值的差，排除掉差值过大的异常数据（第一次计算完斜率均值并排除掉异常值后，再在剩余的斜率中取均值，继续排除）
   3. 最小二乘法拟合左右车道线：拟合出一条直线f(xi)=axi+b，使得误差平方和最小E=∑(f(xi)−yi)^2。

   ```python
   draw_lanes(drawing, lines)  #车道拟合计算
   result = cv2.addWeighted(img, 0.9, drawing, 0.2, 0)  #将结果按权重叠加合在原图上
   
   #车道拟合计算
   def draw_lanes(img, lines, color=[255, 0, 0], thickness=8):
       # a.划分左右车道
       left_lines, right_lines = [], []
       for line in lines:
           for x1, y1, x2, y2 in line:
               k = (y2 - y1) / (x2 - x1)
               if k < 0:
                   left_lines.append(line)
               else:
                   right_lines.append(line)
       if (len(left_lines) <= 0 or len(right_lines) <= 0):
           return
   
       # b.清理异常数据
       clean_lines(left_lines, 0.1)
       clean_lines(right_lines, 0.1)
   
       # c.得到左右车道线点的集合，拟合直线(将所有端点表示成元组（x，y）的列表)
       left_points = [(x1, y1) for line in left_lines for x1, y1, x2, y2 in line]
       left_points = left_points + [(x2, y2) for line in left_lines for x1, y1, x2, y2 in line]
       right_points = [(x1, y1) for line in right_lines for x1, y1, x2, y2 in line]
       right_points = right_points + [(x2, y2) for line in right_lines for x1, y1, x2, y2 in line]
   
       left_results = least_squares_fit(left_points, 325, img.shape[0])
       right_results = least_squares_fit(right_points, 325, img.shape[0])
   
       # 注意这里点的顺序
       vtxs = np.array([[left_results[1], left_results[0], right_results[0], right_results[1]]])
       # d.填充车道区域
       cv2.fillPoly(img, vtxs, (0, 255, 0))
       # d.或者只画车道线
       # cv2.line(img, left_results[0], left_results[1], (0, 255, 0), thickness)
       # cv2.line(img, right_results[0], right_results[1], (0, 255, 0), thickness)
   
   # 迭代计算斜率均值，排除掉与差值差异较大的数据
   def clean_lines(lines, threshold):
       slope = [(y2 - y1) / (x2 - x1) for line in lines for x1, y1, x2, y2 in line]
       while len(lines) > 0:
           mean = np.mean(slope)
           diff = [abs(s - mean) for s in slope]
           idx = np.argmax(diff)
           if diff[idx] > threshold:
               slope.pop(idx)
               lines.pop(idx)
           else:
               break
   
   # 最小二乘法拟合
   def least_squares_fit(point_list, ymin, ymax):
       # 得到x、y的点列
       x = [p[0] for p in point_list]
       y = [p[1] for p in point_list]
   
       # np.polyfit：最小二乘多项式拟合。参数1、2为点列坐标，第参数3为拟合多项式的阶数（1代表线性）
       fit = np.polyfit(y, x, 1)  #polyfit返回值为系数列表
       fit_fn = np.poly1d(fit)  # 获取拟合的结果直线函数
   
       xmin = int(fit_fn(ymin))  #获取直线端点坐标
       xmax = int(fit_fn(ymax))
   
       return [(xmin, ymin), (xmax, ymax)]
   ```

   <img src="C:\Users\2022\OneDrive - zju.edu.cn\Robot Learning Note\images\cv2_lane_detection_result_sample.jpg" alt="img" style="zoom:33%;" />

   7. 摄像头/视频处理

   ```python
   capture = cv2.VideoCapture(0) #打开摄像头
   fourcc = cv2.VideoWriter_fourcc(*'MJPG')
   outfile = cv2.VideoWriter('output.avi', fourcc, 25., (640, 480))  #定义编码方式并创建VideoWriter对象
   
   while(True):
       ret, frame = capture.read()  # 获取一帧
       
       # 将这帧进行上述转换
       #......
       
       outfile.write(result)
       cv2.imshow('result', result)
   ```

#### 摄像头/视频

- capture = cv2.VideoCapture(0)：创建VideoCapture对象，参数为摄像头编号，或视频的路径。

  capture.read()：获取一帧，返回两个参数。第一个返回参数为当前帧获取是否正确标志，第二个返回参数为当前帧的图像。

  capture.get(propId)：获取摄像头属性，参数为获取的属性名称。

  capture.set(propId, value)：设置摄像头属性，参数1为是属性名称，参数2为设定值。

- writer = cv2.VideoWriter('file_name.avi', fourcc, fps, (m,n) )：保存视频。参数1为包含后缀名的文件名，参数2位编码方式fourcc，参数3位帧率，参数4位分辨率大小。

  fourcc = cv2.VideoWriter_fourcc(*'MJPG')：指定视频编码方式fourcc。

#### 绘制图形

- 导入模块与通用代码：

  ```
  import cv2
  import numpy as np
  import matplotlib.pyplot as plt
  
  cv2.imshow('img', img)
  cv2.waitKey(0)
  ```

  img = np.zeros/ones((512, 512, 3), np.uint8)：创建一幅黑色/白色的图片。

- cv2.line(img, (x0, y0), (x1, y1), (b, g, r), thickness)：画直线。参数1为图片变量，参数2为起点，参数3为终点，参数4为颜色bgr值，参数5为线宽。

- cv2.rectangle(img, (x0, y0), (x1, y1), (b, g, r), thickness)：画矩形。参数1为图片变量，参数2为左上角坐标，参数3为右下角坐标，参数4为颜色bgr值，参数5为线宽。

- cv2.circle(img, (x0, y0), r, (b, g, r), thickness)：画圆。参数1为图片变量，参数2为圆心坐标，参数3为半径，参数4为颜色bgr值，参数5为线宽（-1表示填充）。

- cv2.ellipse(img, (x0, y0), (a, b), angle, startAngle, endAngle, (b, g, r), thickness)：画椭圆。参数1为图片变量，参数2为椭圆中心，参数3为x、y轴长度，参数4为椭圆旋转角度，参数5、6为椭圆起始角度和结束角度，参数7为颜色bgr值，参数8为线宽（-1表示填充）。

- cv2.polylines(img, [pts], True,  (0, 255, 255))：画多边形或连续的多条直线。参数1为图片变量，参数2为顶点坐标矩阵，参数3设置多边形是否闭合，参数4为颜色bgr值。

  顶点坐标矩阵：为顶点数×1×2维矩阵。pts = np.array([[x0, y0],  [x1, y1], [x2, y2], ......], np.int32)。

- cv2.fillPoly(img, [pts], npt, n, (0, 255, 255))：画多边形并填充。参数1为图片变量，参数2为顶点坐标矩阵，参数3为多边形顶点数目，参数4为绘制多边形数量，参数5为颜色bgr值。

- cv2.putText(img, 'text', (x, y), font_style, font_size, (b, g, r), thickness, lineType)：参数1为图片变量，参数2为要添加的文本，参数3为文字坐标（左下角），参数4为字体，参数5为字体大小，参数6为颜色bgr值，参数7为字体粗细，参数8为线型。

- cv2.createTrackbar('trackbar_name', 'window_name', num, upper, call_back)：创建滑动条。参数1为滑动条名称，参数2为窗口名称，参数3为当前的值，参数4为最大值，参数5为回调函数名称。

  cv2.getTrackbarPos('trackbar_name', 'window_name')：获取滑动条当前的值。参数1为滑动条名称，参数2为窗口名称。 

- cv2.setMouseCallback('window_name', mouse_event_name, param)：创建鼠标回调函数。

  参数1为窗口名称，参数2为鼠标响应回调处理函数，参数4为处理函数ID。

  def mouse_event(event, x, y, flags, param)：定义鼠标回调函数。

  参数1为鼠标事件：

  ​	EVENT_LBUTTONDBLCLK：左键双击

  ​	EVENT_LBUTTONDOWN：左键点击

  ​	EVENT_LBUTTONUP：左键释放

  ​	EVENT_MBUTTONDBLCLK：中间释放

  ​	EVENT_MBUTTONDOWN：中间点击

  ​	EVENT_MOUSEHWHEEL：滚轮事件

  ​	EVENT_MOUSEMOVE：滑动

  ​	EVENT_MOUSEWHEEL：滚轮事件

  ​	EVENT_RBUTTONDBLCLK：右键双击

  ​    EVENT_RBUTTONDOWN：右键点击

  ​	EVENT_RBUTTONUP：右键释放

  参数2、3为鼠标当前位置

  参数4为鼠标事件状态：

  ​    EVENT_FLAG_ALTKEY：按Alt不放事件

  ​    EVENT_FLAG_CTRLKEY：按Ctrl不放事件

  ​    EVENT_FLAG_LBUTTON：左键拖拽

  ​    EVENT_FLAG_MBUTTON：中键拖拽

  ​    EVENT_FLAG_RBUTTON：右键拖拽

  ​    EVENT_FLAG_SHIFTKEY：按Shift不放事件

  参数5为鼠标事件ID。



## Yolo

### 指标分析

- mAP：综合衡量检测效果

- IOU：预测框与真值框的交集除以它们的并集。

- TP：正确划分正例个数；正->正

  FP：错误划分正例个数；负->正

  FN：错误划分负例个数；正->负

  TN：正确划分负例个数；负->负

  一般来说，查准率越高，查全率越低。

- Precision查准率（正确划分正例个数/全部个数）：
  $$
  p=TP/(TP+FP)
  $$

- Recall查全率（预测样本中实际正样本数 / 预测的样本数）：
  $$
  r=TP/(TP+FN)
  $$

### 分类

- one stage：仅使用一个CNN网络，直接预测目标的类别与位置。

- two stage：先使用CNN网络产生目标位置，然后再在目标位置上做分类与回归。

### 实现方法

1. 将一幅图像分为S×S个网格，如果某个object的中心落在网格中，则这个网格负责预测这个object。

2. 每个网格负责预测B个（B为先验框个数）bounding box和C个class的可能度。每个bounding box预测5个值：自身位置（x、y、w、h），边界框confidence（c=Pr(object)×IOU）。因此图片最终输出S×S×(5*B+C)的一组tensor。

   可以得到每个bounding box的class-specific confidence score ：c=Pr(Classi|Object)×Pr(object)×IOU。设置阈值，滤掉低于阈值的bounding box，再进行NMS处理（非极大值抑制），得到最终检测结果。

3. 卷积网络设计：如图，采用24个卷积层和2个全连接层。最终得到S×S×(5*B+C)的一组tensor。

4. 网络训练：预训练分类模型采用20个卷积层，然后添加一个average-pool层和全连接层。预训练之后，在预训练得到的20层卷积层之上加上随机初始化的4个卷积层和2个全连接层。不断采用深度学习算法直到损失函数最小，则训练完成。

   训练损失函数：

   ![image-20211213191546990](C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20211213191546990.png)

   权重：yolo对不同的部分采用了不同的权重值。对于定位误差，即边界框坐标预测误差，采用较大的权重 ![[公式]](https://www.zhihu.com/equation?tex=%5Clambda+_%7Bcoord%7D%3D5) 。然后其区分不包含目标的边界框与含有目标的边界框的置信度，对于前者，采用较小的权重值 ![[公式]](https://www.zhihu.com/equation?tex=%5Clambda+_%7Bnoobj%7D%3D0.5) 。其它权重值均设为1。

   预测函数：Yolo将目标检测看成回归问题，所以采用的是均方差损失函数，其同等对待大小不同的边界框，但是实际上较小的边界框的坐标误差应该要比较大的边界框要更敏感。为了保证这一点，将网络的边界框的宽与高预测改为对其平方根的预测，即预测值变为![[公式]](https://www.zhihu.com/equation?tex=%28x%2Cy%2C%5Csqrt%7Bw%7D%2C+%5Csqrt%7Bh%7D%29) 。

5. 预测：对于每个预测框，根据类别置信度选取置信度最大的那个类别作为其预测标签，得到各个预测框的预测类别及对应的预测框置信度值。然后将置信度小于置信度阈值的box过滤掉，剩余置信度较高的预测框。最后再对这些预测框使用NMS算法，留下的就是检测结果。

### 改进

- yolov2：

  1. 采用先验框（Anchor Boxes），使用K-means聚类提取先验框尺度：对训练集中标注的真实边框进行5类聚类分析，每类边框取中间值，作为先验框大小，以寻找尽可能匹配样本的边框尺寸。

     聚类中的距离：<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20211213194206691.png" alt="image-20211213194206691" style="zoom: 80%;" />

  2. 约束预测边框的调整位置，使预测更稳定：<img src="C:\Users\user\AppData\Roaming\Typora\typora-user-images\image-20211213195534353.png" alt="image-20211213195534353" style="zoom:67%;" />

  3. 引入passthrough层，在最后一个pooling之前，将特征图一拆四直接传递（passthrough）到pooling后（并且又经过一组卷积）的特征图，两者连接到一起作为输出的特征图。防止最后一层感受野过大，以在特征图中保留一些细节信息。

- yolov3：

  1. 设计多scale检测不同大小物体：设置13×13（最大感受野）、26×26、52×52（最小感受野）三种尺寸的特征图，每种尺寸下通过K-means聚类提取3种先验框尺度，共3×3=9类。

     因此，卷积层最后输出的向量为 N×N×[3×(5+C)]。

  2. 

- yolov4：

- yolov5：

### Yolov5+pytroch训练自己的数据集（Ubuntu）

1. 环境搭建：

   ```bash
   // clone yolov5文件夹
   git clone https://github.com/ultralytics/yolov5
   // 安装需要的pip包
   pip install -U -r requirements.txt
   ```

2. 数据准备：在data文件夹下建立Annotations、images、ImageSets、labels四个文件夹。Annotations存放xml文件；images存放原图像；ImageSets文件夹下继续新建Main文件夹，下建立train.txt和test.txt文件；labels存放标签文件。

3. 初始化训练文件

   ```bash
   // 划分训练集与测试集，在ImageSets/Main文件下生成train和test
   python3 temp.py
   // 生成labels标签文件
   python3 voc_labels
   ```

4. 配置训练文件

   在data目录下新建user.yaml，配置训练用的数据，内容如下：

   ```python
   # COCO 2017 dataset http://cocodataset.org
   # Download command: bash yolov5/data/get_coco2017.sh
   # Train command: python train.py --data ./data/coco.yaml
   # Dataset should be placed next to yolov5 folder:
   #   /parent_folder
   #     /coco
   #     /yolov5
    
   # train and val datasets (image directory or *.txt file with image paths)
   train: xx/xx/train2017.txt  # 上面生成的train路径
   val: xx/xx/val2017.txt      # 上面生成的test路径
   #test: ../coco/test-dev2017.txt  # 20k images for submission to https://competitions.codalab.org/competitions/20794
   
   # number of classes
   nc: 2                       # 训练的类别数量
    
   # class names
   names: ['apple','orange']   # 训练的类别名称
    
   # Print classes
   # with open('data/coco.yaml') as f:
   #   d = yaml.load(f, Loader=yaml.FullLoader)  # dict
   #   for i, x in enumerate(d['names']):
   #     print(i, x)
   ```

   从models文件夹中选择一个预训练模型文件（如yolov5m.yaml）复制到data文件夹下，并修改其中头部内容：

   ```python
   # parameters
   nc: 2  # number of classes 训练的类别数量
   ```

5. 训练权重文件

   ```bash
   python3 train.py --data data/user.yaml --cfg yolov5s.yaml --weights '' --batch-size 16 --epochs 100
   ```

6. 测试

   ```bash
    python detect.py -- weights best.pt --source file.jpg  # image (file.mp4  # video)
   ```



## 神经网络与深度学习





## PyTorch

### 基础知识



## **机械设计**