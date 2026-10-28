/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ws2812_port.c
  * @brief   WS2811 输出驱动抽象层实现：SPI / GPIO / PWM 后端，宏选择
  * @note    本文件是灯效引擎与硬件之间的唯一边界：
  *          - 引擎只调用 ws2812_port_init()/ws2812_port_send()
  *          - 后端内部细节（编码、DMA、中断）全部封装在此，切换后端互不影响
  *          - WS_PORT_PWM_SPI 双后端模式：PWM(PB4) + SPI(PE14) 同一帧同步发两路
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "ws2812_port.h"
#include "spi.h"
#include "tim.h"
#include <string.h>

/* USER CODE BEGIN Private defines */
/* 单帧发送超时：DMA 出错时防止忙等死循环。1000 颗灯最坏约 40ms，留足余量 */
#define WS_TX_TIMEOUT_MS                100u
/* ---- SPI4 后端参数（WS2811 时序）-------------------------------------------
 * SPI4 内核时钟 = D2PCLK1 = 120MHz，BaudRatePrescaler = 32 -> 3.75MHz（1 SPI 位 ≈ 267ns）
 * 每个颜色位用变长 SPI 码元编码（MSB 先发），满足 WS2811 时序：
 *   数据 1 -> 111000(6 位)：T1H = 3×267 = 800ns  T1L = 3×267 = 800ns  （580~1000ns）
 *   数据 0 -> 1000  (4 位)：T0H = 1×267 = 267ns  T0L = 3×267 = 800ns  （220~380 / 580~1000ns）
 * 复位码元(RESET)：WS2811 要求 >280us；144 字节 × 8bit / 3.75MHz ≈ 307us，安全锁存
 * ------------------------------------------------------------------------ */
#if (WS_PORT_USE == WS_PORT_SPI) || (WS_PORT_USE == WS_PORT_PWM_SPI)
#define WS2812_SPI_CODE0_BITS              4u    /* '0' 码元占用的 SPI 位数 */
#define WS2812_SPI_CODE1_BITS              6u    /* '1' 码元占用的 SPI 位数 */
#define WS2812_SPI_MAX_BITS_PER_COLOR_BIT  6u    /* 缓冲按最坏(全 1)分配 */
#define WS2812_RESET_BYTES                 144u  /* 尾部复位码元字节数 ≈307us */
#define WS2812_LEAD_BYTES                  1u    /* 帧首哨兵字节 0x00：吸收 SPI 首位预驱动长高电平 */

/* 输出极性：数据线经反相器时置 1（码元与复位自动取反） */
#define WS2812_OUTPUT_INVERT               0u
#endif

/* ---- GPIO 软件模拟后端参数（单线 ~800Kbps，PA5/ws2812_Pin）---------------
 * 延时基于 SYSCLK=480MHz 的 NOP 循环，[saw]* 为估算循环数，需示波器实测微调
 *   数据 1：T1H≈0.70us  T1L≈0.60us   数据 0：T0H≈0.35us  T0L≈0.80us
 *   复位：  >50us 持续低电平
 * ------------------------------------------------------------------------ */
#if (WS_PORT_USE == WS_PORT_GPIO)
#define WS2812_GPIO_PORT         ws2812_GPIO_Port   /* GPIOA */
#define WS2812_GPIO_PIN          ws2812_Pin         /* GPIO_PIN_5 */

/* 以灯珠端电平为准的置高/置低（极性由 WS2812_OUTPUT_INVERT 自动适配） */
#define WS2812_OUTPUT_INVERT     0u

#if WS2812_OUTPUT_INVERT
  /* MCU 输出低 = 灯珠端高 */
  #define WS_LED_HIGH()  WRITE_REG(WS2812_GPIO_PORT->BSRR, ((uint32_t)WS2812_GPIO_PIN) << 16u)
  #define WS_LED_LOW()   WRITE_REG(WS2812_GPIO_PORT->BSRR, WS2812_GPIO_PIN)
#else
  /* MCU 输出高 = 灯珠端高 */
  #define WS_LED_HIGH()  WRITE_REG(WS2812_GPIO_PORT->BSRR, WS2812_GPIO_PIN)
  #define WS_LED_LOW()   WRITE_REG(WS2812_GPIO_PORT->BSRR, ((uint32_t)WS2812_GPIO_PIN) << 16u)
#endif

/* 单次延时循环开销约 5~6 周期(~12ns @480MHz)，实测时优先保证 T1H/T0H */
#define WS_GPIO_T0H_CNT          25u    /* ~0.35us 高 */
#define WS_GPIO_T0L_CNT          100u   /* ~0.80us 低 */
#define WS_GPIO_T1H_CNT          1000u  /* ~0.70us 高 */
#define WS_GPIO_T1L_CNT          25u    /* ~0.60us 低 */
#define WS_GPIO_RESET_CNT        6000u  /* >50us 持续低电平 */
#endif

/* ---- TIM3_CH1(PB4) PWM+DMA 后端参数（WS2811 时序）-------------------------
 * TIM3 内核时钟 = 2xPCLK1 = 240MHz，Period=299 → 码元周期 1.25us(800kHz)
 * 每个颜色位用 CCR1 占空比编码（PWM1 模式，CNT<CCR 输出高）：
 *   数据 1 -> CCR=156 : T1H=650ns  T1L=600ns
 *   数据 0 -> CCR=72  : T0H=300ns  T0L=950ns
 *   复位   -> CCR=0   : 帧尾补 240 个空码元 = 300us（>280us 锁存）
 * ------------------------------------------------------------------------ */
#if (WS_PORT_USE == WS_PORT_PWM) || (WS_PORT_USE == WS_PORT_PWM_SPI)
#define WS_PWM_T1H               156u   /* 数据 1 占空比计数值：156/240MHz = 650ns */
#define WS_PWM_T0H               72u    /* 数据 0 占空比计数值：72/240MHz = 300ns */
#define WS_PWM_RESET_SLOTS       240u   /* 帧尾 CCR=0 码元个数：240 x 1.25us = 300us */
#endif
/* USER CODE END Private defines */

/* USER CODE BEGIN Private_Function_Prototypes */
#if (WS_PORT_USE == WS_PORT_SPI) || (WS_PORT_USE == WS_PORT_PWM_SPI)
static void ws_port_spi_send(const uint8_t *brg, uint16_t count);
#endif
#if (WS_PORT_USE == WS_PORT_GPIO)
static void ws_port_gpio_send(const uint8_t *brg, uint16_t count);
static void ws2812_gpio_delay(uint32_t cnt);
static void ws2812_gpio_bit(uint32_t high_t, uint32_t low_t);
#endif
#if (WS_PORT_USE == WS_PORT_PWM) || (WS_PORT_USE == WS_PORT_PWM_SPI)
static void ws_port_pwm_init(void);
static void ws_port_pwm_send(const uint8_t *brg, uint16_t count);
#endif
/* USER CODE END Private_Function_Prototypes */

/* USER CODE BEGIN Exported_Functions */
/**
  * @brief  初始化所选输出后端（引脚/定时器/DMA），空闲输出复位电平
  */
void ws2812_port_init(void)
{
  /* DWT 周期计数器：延时基准（各后端通用，上电使能一次） */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

#if (WS_PORT_USE == WS_PORT_PWM) || (WS_PORT_USE == WS_PORT_PWM_SPI)
  ws_port_pwm_init();
#endif
  /* SPI/GPIO 后端的外设与引脚由 CubeMX 生成的 MX_SPI4_Init/MX_GPIO_Init 完成 */
}

/**
  * @brief  发送一帧 BRG 颜色数据到全部启用后端（阻塞至整帧 + 复位码元发完）
  */
void ws2812_port_send(const uint8_t *brg, uint16_t count)
{
  if (brg == 0 || count == 0u || count > WS2812_LED_MAX)
  {
    return;
  }
#if (WS_PORT_USE == WS_PORT_PWM) || (WS_PORT_USE == WS_PORT_PWM_SPI)
  ws_port_pwm_send(brg, count);      /* 上层 PB4 */
#endif
#if (WS_PORT_USE == WS_PORT_SPI) || (WS_PORT_USE == WS_PORT_PWM_SPI)
  ws_port_spi_send(brg, count);      /* 下层 PE14 */
#endif
}

/**
  * @brief  微秒级忙等延时（DWT 周期计数，不占用定时器外设）
  */
void ws2812_port_delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}
/* USER CODE END Exported_Functions */

/* USER CODE BEGIN Private_Functions */
#if (WS_PORT_USE == WS_PORT_SPI) || (WS_PORT_USE == WS_PORT_PWM_SPI)
/**
  * @brief  将一个颜色字节按 WS2811 变长码元编码，写入 SPI 发送缓冲
  * @param  buf     发送缓冲（需预先清零）
  * @param  bitpos  当前已占用的 SPI 位位置，函数内自增
  * @param  val     待编码的 1 字节颜色数据（MSB 先发）
  * @note   '1' 位 -> 6 个 SPI 位(111000)，'0' 位 -> 4 个 SPI 位(1000)
  */
static void ws2812_encode_byte(uint8_t *buf, uint32_t *bitpos, uint8_t val)
{
  for (int8_t i = 7; i >= 0; i--)          /* 颜色数据 MSB 先发 */
  {
    uint8_t code;
    uint8_t nbits;
#if WS2812_OUTPUT_INVERT
    /* 硬件反相：低 3 位恒为 1，高位补 0（'1'=000111，'0'=0111） */
    code  = 0x07u;
    nbits = (val & (uint8_t)(1u << i)) ? WS2812_SPI_CODE1_BITS : WS2812_SPI_CODE0_BITS;
#else
    code  = (val & (uint8_t)(1u << i)) ? 0x38u : 0x08u;  /* '1'=111000  '0'=1000 */
    nbits = (val & (uint8_t)(1u << i)) ? WS2812_SPI_CODE1_BITS : WS2812_SPI_CODE0_BITS;
#endif
    for (int8_t b = (int8_t)nbits - 1; b >= 0; b--)        /* 码元 SPI 位，MSB 先发 */
    {
      if (code & (uint8_t)(1u << b))
      {
        buf[*bitpos >> 3] |= (uint8_t)(1u << (7u - (*bitpos & 7u)));
      }
      (*bitpos)++;
    }
  }
}

/**
  * @brief  SPI4 后端：编码整帧并经 DMA 发送（PE14 MOSI）
  * @note   1 字节哨兵(0x00) + count 灯 × 3 字节变长码元 + 144 字节复位码元；
  *         阻塞至 DMA 与 SPI 移位寄存器全部发完，保证复位码元完整输出。
  */
static void ws_port_spi_send(const uint8_t *brg, uint16_t count)
{
  extern SPI_HandleTypeDef hspi4;   /* CubeMX spi.c */

  static uint8_t buf[WS2812_LEAD_BYTES + WS2812_LED_MAX * 3u * WS2812_SPI_MAX_BITS_PER_COLOR_BIT + WS2812_RESET_BYTES];
  uint32_t bitpos = WS2812_LEAD_BYTES * 8u;
  uint16_t led;

  /* 缓冲清零：未写入的位保持低电平（即 RESET / 熄灭） */
  memset(buf, 0, sizeof(buf));

  for (led = 0u; led < count; led++)
  {
    ws2812_encode_byte(buf, &bitpos, brg[led * 3u + 0u]);   /* B */
    ws2812_encode_byte(buf, &bitpos, brg[led * 3u + 1u]);   /* R */
    ws2812_encode_byte(buf, &bitpos, brg[led * 3u + 2u]);   /* G */
  }

  /* 尾部复位码元：正向时保持 0x00；反向输出时填 0xFF，使灯珠端持续低 >280us 锁存 */
#if WS2812_OUTPUT_INVERT
  while (bitpos & 7u)   /* 补齐本字节尾部位 */
  {
    buf[bitpos >> 3] |= (uint8_t)(1u << (7u - (bitpos & 7u)));
    bitpos++;
  }
  memset(buf + (bitpos >> 3), 0xFFu, WS2812_RESET_BYTES);
#endif

  /* 实际帧长 = 数据字节(向上取整) + 复位字节 */
  uint16_t frame_bytes = (uint16_t)((bitpos + 7u) / 8u) + WS2812_RESET_BYTES;

  /* DMA 发送：字节间无 CPU 软件间隙，时序更连续 */
  {
    uint32_t tmo = HAL_GetTick();
    while (HAL_SPI_GetState(&hspi4) != HAL_SPI_STATE_READY)
    {
      if ((HAL_GetTick() - tmo) > WS_TX_TIMEOUT_MS)
      {
        HAL_SPI_Abort(&hspi4);   /* 超时：中止残留传输，恢复 READY，防卡死 */
        return;
      }
    }
  }
  if (HAL_SPI_Transmit_DMA(&hspi4, buf, frame_bytes) != HAL_OK)
  {
    return;   /* 启动失败：放弃本次发送，避免 Error_Handler 死循环 */
  }
  /* 等待整帧(含复位码元)发完再返回 */
  {
    uint32_t tmo = HAL_GetTick();
    while (HAL_SPI_GetState(&hspi4) != HAL_SPI_STATE_READY)
    {
      if ((HAL_GetTick() - tmo) > WS_TX_TIMEOUT_MS)
      {
        HAL_SPI_Abort(&hspi4);   /* 超时：中止残留传输，恢复 READY，防卡死 */
        return;
      }
    }
  }
}

#endif

#if (WS_PORT_USE == WS_PORT_GPIO)
/**
  * @brief  软件延时（volatile 计数值，避免被编译器优化掉，用于相位粗调）
  * @param  cnt  循环次数，每次约 5~6 周期(~12ns @480MHz)
  */
static void ws2812_gpio_delay(uint32_t cnt)
{
  volatile uint32_t i;
  for (i = 0u; i < cnt; i++)
  {
    __NOP();
  }
}

/**
  * @brief  用 GPIO 软件模拟单线时序，输出 1 位数据
  * @param  high_t  高电平延时循环数
  * @param  low_t   低电平延时循环数
  */
static void ws2812_gpio_bit(uint32_t high_t, uint32_t low_t)
{
  /* 灯珠端先高后低（极性由 WS_LED_HIGH/LOW 自动适配） */
  WS_LED_HIGH();
  ws2812_gpio_delay(high_t);
  WS_LED_LOW();
  ws2812_gpio_delay(low_t);
}

/**
  * @brief  GPIO 后端：软件模拟 800Kbps 单线时序发送整帧（PA5）
  * @note   数据顺序 BRG（MSB 先发），末尾自动补 >50us 复位码元锁存
  */
static void ws_port_gpio_send(const uint8_t *brg, uint16_t count)
{
  uint16_t led;
  uint8_t  c;
  int8_t   bit;

  /* 数据线先拉低（灯珠端空闲为低），再按序发送每颗灯珠的 BRG */
  WS_LED_LOW();

  for (led = 0u; led < count; led++)
  {
    for (c = 0u; c < 3u; c++)
    {
      uint8_t byte = brg[led * 3u + c];
      for (bit = 7; bit >= 0; bit--)
      {
        if (byte & (uint8_t)(1u << bit))
        {
          ws2812_gpio_bit(WS_GPIO_T1H_CNT, WS_GPIO_T1L_CNT);
        }
        else
        {
          ws2812_gpio_bit(WS_GPIO_T0H_CNT, WS_GPIO_T0L_CNT);
        }
      }
    }
  }

  /* 复位码元：灯珠端持续低电平 >50us，触发热锁存（极性自动适配） */
  WS_LED_LOW();
  ws2812_gpio_delay(WS_GPIO_RESET_CNT);
}

#endif

#if (WS_PORT_USE == WS_PORT_PWM) || (WS_PORT_USE == WS_PORT_PWM_SPI)
/* ---- PWM+DMA（TIM3_CH1/PB4）后端 ------------------------------------------ */
static DMA_HandleTypeDef hdma_tim3_ws;    /* 周期刷新 CCR1 的 DMA 流(Stream5) */
static volatile uint8_t ws_pwm_busy = 0;  /* 整帧发送中标志(阻塞等待用) */

/**
  * @brief  PWM 后端初始化：DMA1_Stream5 经 DMAMUX 绑定 TIM3_CH1 请求
  * @note   CubeMX 只配置了 TIM3_CH1 的 PWM 输出(800kHz)，DMA 在此手动挂载；
  *         空闲时 CCR1=0 输出常低(复位电平)。
  */
static void ws_port_pwm_init(void)
{
  hdma_tim3_ws.Instance                 = DMA1_Stream5;
  hdma_tim3_ws.Init.Request             = DMA_REQUEST_TIM3_CH1;   /* DMAMUX1 请求号 23 */
  hdma_tim3_ws.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tim3_ws.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tim3_ws.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_tim3_ws.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;  /* CCR1 有效位宽 16bit */
  hdma_tim3_ws.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_tim3_ws.Init.Mode                = DMA_NORMAL;
  hdma_tim3_ws.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_tim3_ws.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_tim3_ws) != HAL_OK)
  {
    Error_Handler();
  }
  __HAL_LINKDMA(&htim3, hdma[TIM_DMA_ID_CC1], hdma_tim3_ws);

  /* DMA 传输完成中断：在 PulseFinished 回调里停 PWM 并清忙标志 */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  /* 空闲即输出低电平（CCR1=0 由 MX_TIM3_Init 配置）：直接使能通道/计数器，
   * 绕过 HAL 通道状态机，保证通道状态 READY，首次 HAL_TIM_PWM_Start_DMA 才不会返回 BUSY */
  TIM3->CCER |= TIM_CCER_CC1E;
  TIM3->CR1  |= TIM_CR1_CEN;
}

/**
  * @brief  PWM 后端：颜色帧展开为 CCR1 码元队列，经 TIM3 DMA 发送
  * @note   阻塞约 count*30us + 300us 复位（200 颗约 6.3ms），期间 DMA 硬件
  *         连续刷新 CCR1 保证时序；帧尾 CCR=0 持续低电平触发热锁存。
  */
static void ws_port_pwm_send(const uint8_t *brg, uint16_t count)
{
  /* count 灯 x 24 位 + 复位码元（uint16_t CCR1 队列，DMA1 可访问） */
  static uint16_t ws_pwm_buf[WS2812_LED_MAX * 24u + WS_PWM_RESET_SLOTS];
  const uint16_t  total = (uint16_t)(count * 24u + WS_PWM_RESET_SLOTS);
  uint16_t n = 0;
  uint16_t led;
  uint8_t  c;
  int8_t   bit;

  {
    uint32_t tmo = HAL_GetTick();
    while (ws_pwm_busy)   /* 保险：等上一帧结束（正常流程函数本身已阻塞到发完） */
    {
      if ((HAL_GetTick() - tmo) > WS_TX_TIMEOUT_MS)
      {
        ws_pwm_busy = 0;   /* 超时：强制清忙标志，防卡死 */
        return;
      }
    }
  }
  ws_pwm_busy = 1;

  for (led = 0u; led < count; led++)
  {
    for (c = 0u; c < 3u; c++)
    {
      uint8_t byte = brg[led * 3u + c];
      for (bit = 7; bit >= 0; bit--)   /* MSB 先发 */
      {
        ws_pwm_buf[n++] = (uint16_t)((byte & (uint8_t)(1u << bit)) ? WS_PWM_T1H : WS_PWM_T0H);
      }
    }
  }
  /* 帧尾复位码元：CCR=0 持续低电平 300us，触发热锁存 */
  for (; n < total; n++)
  {
    ws_pwm_buf[n] = 0u;
  }

  if (HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)ws_pwm_buf, total) != HAL_OK)
  {
    ws_pwm_busy = 0;   /* 启动失败：清标志并放弃本次发送，避免 Error_Handler 死循环 */
    return;
  }

  {
    uint32_t tmo = HAL_GetTick();
    while (ws_pwm_busy)   /* 阻塞至整帧(含复位)发完再返回 */
    {
      if ((HAL_GetTick() - tmo) > WS_TX_TIMEOUT_MS)
      {
        ws_pwm_busy = 0;   /* 超时：强制清忙标志，防卡死 */
        return;
      }
    }
  }
}

/**
  * @brief  TIM3 PWM DMA 发送完成回调：停止输出（引脚回低），解除发送阻塞
  */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim3)
  {
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
    ws_pwm_busy = 0;
  }
}

/**
  * @brief  DMA1_Stream5 中断服务函数（TIM3_CH1 码元队列）
  */
void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_tim3_ws);
}
#endif /* WS_PORT_USE */
/* USER CODE END Private_Functions */
