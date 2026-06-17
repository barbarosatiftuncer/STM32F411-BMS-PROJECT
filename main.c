/**
 * ============================================================================
 * STM32F411RE NUCLEO - ULTIMATE 4S BMS & TELEMETRI
 * ============================================================================
 * PB0 : Acil Durdurma Butonu (Latching/Mühürlemeli)
 * PB4 : Yeşil LED (Sistem Aktif ve Güvenli)
 * PB5 : Kırmızı LED (Sistem Kilitli veya Limit Aşıldı)
 * PA15: Motor PWM Çıkışı (Ters Mantık / 2N2222 Sürücülü)
 * PA7 : Potansiyometre (Hız Ayarı)
 *
 * USART2: PA2 TX, PA3 RX  -> Tera Term 115200 baud
 * I2C1  : PB6 SCL, PB7 SDA -> BME280 + INA226
 */

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

#define I2C_TIMEOUT       100000
#define BME280_ADDR       (0x76 << 1)
#define INA226_ADDR       (0x40 << 1)
#define DIV_RATIO         5.545f

// --- GÜVENLİK REFERANS DEĞERLERİ (LİMİTLER) ---
#define REF_VOLT_MIN      12.0f
#define REF_VOLT_MAX      17.2f
#define REF_TEMP_MAX      30.0f
#define REF_HUM_MAX       68.0f
#define REF_PWM_MAX       75
#define REF_PRESS_MIN     900.0f
#define REF_PRESS_MAX     1100.0f

// --- SENSÖR VE SİSTEM DEĞİŞKENLERİ ---
volatile float bme_temp = 0, bme_press = 0;
volatile float dht_temp = 0, dht_hum = 0;
volatile float ina_volt = 0, ina_curr = 0;

volatile float node[4]  = {0, 0, 0, 0};
volatile float cell[4]  = {0, 0, 0, 0};
volatile int   pct[4]   = {0, 0, 0, 0};

volatile uint8_t sys_ok = 0, dht_ok = 0;

// --- MOTOR VE GÜVENLİK DEĞİŞKENLERİ ---
volatile uint16_t pot_raw = 0;
volatile int pot_percent = 0;
volatile uint8_t system_halted = 0;
volatile uint8_t safety_error = 0;
uint8_t last_button_state = 1;

// --- BME280 KALİBRASYON DEĞİŞKENLERİ ---
uint16_t dig_T1; int16_t dig_T2, dig_T3;
uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
int32_t  t_fine;

/* ================= ZAMANLAMA ================= */

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while((DWT->CYCCNT - start) < ticks);
}

/* ================= UART ================= */

void uart_print(const char *str) {
    while (*str) {
        while (!(USART2->SR & USART_SR_TXE));
        USART2->DR = (*str++ & 0xFF);
    }
}

/* ================= 4S BMS ADC KISMI ================= */

uint16_t read_adc(ADC_TypeDef* ADCx, uint32_t channel) {
    ADCx->SR = 0;
    ADCx->SQR3 = channel & 0x1F;
    ADCx->CR2 |= ADC_CR2_SWSTART;

    while(!(ADCx->SR & ADC_SR_EOC));

    return ADCx->DR;
}

int calc_pct(float v) {
    if(v >= 4.2f) return 100;
    if(v <= 3.2f) return 0;
    return (int)(((v - 3.2f) / 1.0f) * 100.0f);
}

void read_battery_cells(void) {
    node[0] = (read_adc(ADC1, 0) / 4095.0f) * 3.3f * DIV_RATIO; // PA0 / ADC1_IN0
    node[1] = (read_adc(ADC1, 1) / 4095.0f) * 3.3f * DIV_RATIO; // PA1 / ADC1_IN1
    node[2] = (read_adc(ADC1, 4) / 4095.0f) * 3.3f * DIV_RATIO; // PA4 / ADC1_IN4
    node[3] = (read_adc(ADC1, 6) / 4095.0f) * 3.3f * DIV_RATIO; // PA6 / ADC1_IN6

    cell[0] = node[0];
    cell[1] = node[1] - node[0];
    cell[2] = node[2] - node[1];
    cell[3] = node[3] - node[2];

    for(int i = 0; i < 4; i++) {
        if(cell[i] < 0.1f) {
            cell[i] = 0.0f;
        }
        pct[i] = calc_pct(cell[i]);
    }
}

/* ================= DHT22 KISMI ================= */

void read_dht22(void) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint32_t timeout;

    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->MODER |=  (1 << (8 * 2));
    GPIOA->BSRR = (1 << (8 + 16));
    delay_ms(2);
    GPIOA->BSRR = (1 << 8);
    delay_us(30);
    GPIOA->MODER &= ~(3 << (8 * 2));

    timeout = 10000;
    while((GPIOA->IDR & (1 << 8))) {
        if(--timeout == 0) { dht_ok = 0; return; }
    }

    timeout = 10000;
    while(!(GPIOA->IDR & (1 << 8))) {
        if(--timeout == 0) { dht_ok = 0; return; }
    }

    timeout = 10000;
    while((GPIOA->IDR & (1 << 8))) {
        if(--timeout == 0) { dht_ok = 0; return; }
    }

    for(int j = 0; j < 5; j++) {
        for(int i = 0; i < 8; i++) {
            timeout = 10000;
            while(!(GPIOA->IDR & (1 << 8))) {
                if(--timeout == 0) { dht_ok = 0; return; }
            }

            delay_us(40);

            if(GPIOA->IDR & (1 << 8)) {
                data[j] = (data[j] << 1) | 1;

                timeout = 10000;
                while((GPIOA->IDR & (1 << 8))) {
                    if(--timeout == 0) { dht_ok = 0; return; }
                }
            } else {
                data[j] = (data[j] << 1);
            }
        }
    }

    if(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        dht_hum = (float)((data[0] << 8) | data[1]) / 10.0f;
        dht_ok = 1;
    } else {
        dht_ok = 0;
    }
}

/* ================= I2C ÇEKİRDEĞİ - STM32F411RE ================= */

static uint8_t i2c_wait_flag(volatile uint32_t *reg, uint32_t flag, uint8_t set) {
    uint32_t timeout = I2C_TIMEOUT;

    if(set) {
        while(((*reg) & flag) == 0) {
            if(--timeout == 0) return 0;
        }
    } else {
        while(((*reg) & flag) != 0) {
            if(--timeout == 0) return 0;
        }
    }

    return 1;
}

static void i2c_clear_addr(void) {
    volatile uint32_t temp;
    temp = I2C1->SR1;
    temp = I2C1->SR2;
    (void)temp;
}

uint8_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data) {
    uint32_t timeout = I2C_TIMEOUT;

    while((I2C1->SR2 & I2C_SR2_BUSY) && --timeout);
    if(timeout == 0) return 0;

    I2C1->CR1 |= I2C_CR1_START;
    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB, 1)) return 0;

    I2C1->DR = addr & 0xFE;
    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, 1)) return 0;
    i2c_clear_addr();

    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE, 1)) return 0;
    I2C1->DR = reg;

    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE, 1)) return 0;
    I2C1->DR = data;

    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, 1)) return 0;

    I2C1->CR1 |= I2C_CR1_STOP;
    return 1;
}

uint8_t i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
    uint32_t timeout = I2C_TIMEOUT;

    if(len == 0) return 0;

    I2C1->CR1 |= I2C_CR1_ACK;

    while((I2C1->SR2 & I2C_SR2_BUSY) && --timeout);
    if(timeout == 0) return 0;

    I2C1->CR1 |= I2C_CR1_START;
    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB, 1)) return 0;

    I2C1->DR = addr & 0xFE;
    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, 1)) return 0;
    i2c_clear_addr();

    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE, 1)) return 0;
    I2C1->DR = reg;

    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_TXE, 1)) return 0;

    I2C1->CR1 |= I2C_CR1_START;
    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_SB, 1)) return 0;

    I2C1->DR = addr | 0x01;
    if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_ADDR, 1)) return 0;

    if(len == 1) {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        i2c_clear_addr();
        I2C1->CR1 |= I2C_CR1_STOP;

        if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, 1)) return 0;
        data[0] = I2C1->DR;
    } else {
        i2c_clear_addr();

        while(len > 3) {
            if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, 1)) return 0;
            *data++ = I2C1->DR;
            len--;
        }

        if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_BTF, 1)) return 0;
        I2C1->CR1 &= ~I2C_CR1_ACK;

        *data++ = I2C1->DR;
        len--;

        I2C1->CR1 |= I2C_CR1_STOP;

        *data++ = I2C1->DR;
        len--;

        if(len) {
            if(!i2c_wait_flag(&I2C1->SR1, I2C_SR1_RXNE, 1)) return 0;
            *data++ = I2C1->DR;
        }
    }

    I2C1->CR1 |= I2C_CR1_ACK;
    return 1;
}

uint16_t i2c_read_reg16(uint8_t addr, uint8_t reg) {
    uint8_t buf[2];

    if(i2c_read_regs(addr, reg, buf, 2)) {
        return (buf[0] << 8) | buf[1];
    }

    return 0;
}

/* ================= SENSÖRLER ================= */

void sensors_init(void) {
    sys_ok = 1;
    uint8_t c[24];

    if(i2c_read_regs(BME280_ADDR, 0x88, c, 24)) {
        dig_T1 = (c[1] << 8) | c[0];
        dig_T2 = (c[3] << 8) | c[2];
        dig_T3 = (c[5] << 8) | c[4];

        dig_P1 = (c[7] << 8) | c[6];
        dig_P2 = (c[9] << 8) | c[8];
        dig_P3 = (c[11] << 8) | c[10];
        dig_P4 = (c[13] << 8) | c[12];
        dig_P5 = (c[15] << 8) | c[14];
        dig_P6 = (c[17] << 8) | c[16];
        dig_P7 = (c[19] << 8) | c[18];
        dig_P8 = (c[21] << 8) | c[20];
        dig_P9 = (c[23] << 8) | c[22];

        i2c_write_reg(BME280_ADDR, 0xF4, 0x27);
    } else {
        sys_ok = 0;
    }

    i2c_write_reg(INA226_ADDR, 0x00, 0x41);
}

void read_i2c_sensors(void) {
    ina_volt = i2c_read_reg16(INA226_ADDR, 0x02) * 0.00125f;

    int16_t raw_shunt = (int16_t)i2c_read_reg16(INA226_ADDR, 0x01);
    ina_curr = (float)raw_shunt * 0.025f;

    uint8_t data[6];

    if(i2c_read_regs(BME280_ADDR, 0xF7, data, 6)) {
        int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
        int32_t adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

        int32_t var1, var2;

        var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                ((int32_t)dig_T3)) >> 14;

        t_fine = var1 + var2;
        bme_temp = (float)((t_fine * 5 + 128) >> 8) / 100.0f;

        int64_t p, v1p, v2p;

        v1p = ((int64_t)t_fine) - 128000;
        v2p = v1p * v1p * (int64_t)dig_P6 +
              ((v1p * (int64_t)dig_P5) << 17) +
              (((int64_t)dig_P4) << 35);

        v1p = ((v1p * v1p * (int64_t)dig_P3) >> 8) +
              ((v1p * (int64_t)dig_P2) << 12);

        v1p = (((((int64_t)1) << 47) + v1p)) * ((int64_t)dig_P1) >> 33;

        if (v1p != 0) {
            p = 1048576 - adc_P;
            p = (((p << 31) - v2p) * 3125) / v1p;

            v1p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
            v2p = (((int64_t)dig_P8) * p) >> 19;

            p = ((p + v1p + v2p) >> 8) + (((int64_t)dig_P7) << 4);
            bme_press = (float)p / 256.0f;
        }
    }
}

/* ================= DONANIM KURULUMU ================= */

void hardware_init(void) {
    // GPIO, ADC, I2C, USART2, TIM2 clock enable
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN | RCC_APB1ENR_USART2EN | RCC_APB1ENR_TIM2EN;

    volatile uint32_t dummy;
    dummy = RCC->AHB1ENR;
    (void)dummy;

    // DWT cycle counter enable
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // --- PA15: TIM2_CH1 PWM / AF1 ---
    GPIOA->MODER &= ~(3 << (15 * 2));
    GPIOA->MODER |=  (2 << (15 * 2));
    GPIOA->AFR[1] &= ~(0xF << ((15 - 8) * 4));
    GPIOA->AFR[1] |=  (1 << ((15 - 8) * 4));

    // --- PA7: Potansiyometre ADC input ---
    GPIOA->MODER |= (3 << (7 * 2));

    // --- PB0: Acil stop button input pull-up ---
    GPIOB->MODER &= ~(3 << (0 * 2));
    GPIOB->PUPDR &= ~(3 << (0 * 2));
    GPIOB->PUPDR |=  (1 << (0 * 2));

    // --- PB4, PB5: LED outputs ---
    GPIOB->MODER &= ~((3 << (4 * 2)) | (3 << (5 * 2)));
    GPIOB->MODER |=  ((1 << (4 * 2)) | (1 << (5 * 2)));

    // --- USART2: PA2 TX, PA3 RX / AF7 ---
    GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2)));
    GPIOA->MODER |=  ((2 << (2 * 2)) | (2 << (3 * 2)));
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOA->AFR[0] |=  ((7 << (2 * 4)) | (7 << (3 * 4)));

    // --- I2C1: PB6 SCL, PB7 SDA / AF4, open-drain, pull-up ---
    GPIOB->MODER &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->MODER |=  ((2 << (6 * 2)) | (2 << (7 * 2)));
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);
    GPIOB->PUPDR &= ~((3 << (6 * 2)) | (3 << (7 * 2)));
    GPIOB->PUPDR |=  ((1 << (6 * 2)) | (1 << (7 * 2)));
    GPIOB->AFR[0] &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
    GPIOB->AFR[0] |=  ((4 << (6 * 4)) | (4 << (7 * 4)));

    // --- ADC pins: PA0, PA1, PA4, PA6, PA7 analog mode ---
    GPIOA->MODER |= (3 << (0 * 2)) |
                    (3 << (1 * 2)) |
                    (3 << (4 * 2)) |
                    (3 << (6 * 2)) |
                    (3 << (7 * 2));

    // --- DHT22 PA8 initially input ---
    GPIOA->MODER &= ~(3 << (8 * 2));
    GPIOA->PUPDR &= ~(3 << (8 * 2));
    GPIOA->PUPDR |=  (1 << (8 * 2));

    // --- USART2 115200 baud ---
    USART2->BRR = HAL_RCC_GetPCLK1Freq() / 115200;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;

    // --- I2C1 100 kHz standard mode ---
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    uint32_t pclk1_mhz = HAL_RCC_GetPCLK1Freq() / 1000000;
    if(pclk1_mhz < 2) pclk1_mhz = 16;

    I2C1->CR2 = pclk1_mhz;
    I2C1->CCR = HAL_RCC_GetPCLK1Freq() / (2 * 100000);
    I2C1->TRISE = pclk1_mhz + 1;
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_PE;

    // --- TIM2 PWM ayarları ---
    TIM2->PSC = 17 - 1;
    TIM2->ARR = 4095;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
    TIM2->CCER |= TIM_CCER_CC1E;
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
    TIM2->CCR1 = 4095;

    // --- ADC1 ayarları ---
    ADC->CCR = 0;
    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    // Sample time: CH0, CH1, CH4, CH6, CH7
    ADC1->SMPR2 |= (7 << (0 * 3)) |
                   (7 << (1 * 3)) |
                   (7 << (4 * 3)) |
                   (7 << (6 * 3)) |
                   (7 << (7 * 3));

    ADC1->SQR1 = 0;
    ADC1->CR2 |= ADC_CR2_ADON;
    delay_ms(10);
}

/* ================= ANA DÖNGÜ ================= */

int main(void) {
    HAL_Init();
    SystemCoreClockUpdate();

    hardware_init();
    sensors_init();
    delay_ms(100);

    uint32_t timer_1sn = 0;
    char str_buf[128];
    char motor_status_str[64];

    while (1) {
        // --- BUTON KENAR ALGILAMA ---
        uint8_t current_button_state = (GPIOB->IDR & (1 << 0)) ? 1 : 0;

        if (current_button_state == 0 && last_button_state == 1) {
            system_halted = !system_halted;
            delay_ms(50);
        }

        last_button_state = current_button_state;

        if (HAL_GetTick() - timer_1sn >= 1000) {
            timer_1sn = HAL_GetTick();

            read_i2c_sensors();
            read_dht22();
            read_battery_cells();

            pot_raw = read_adc(ADC1, 7);       // PA7 / ADC1_IN7
            pot_percent = (pot_raw * 100) / 4095;

            // --- GÜVENLİK REFERANS KONTROLLERİ ---
            safety_error = 0;

            if (ina_volt < REF_VOLT_MIN || ina_volt > REF_VOLT_MAX) safety_error = 1;
            if (bme_temp > REF_TEMP_MAX) safety_error = 1;
            if (dht_hum > REF_HUM_MAX) safety_error = 1;
            if (pot_percent >= REF_PWM_MAX) safety_error = 1;

            float cur_press = bme_press / 100.0f;
            if (cur_press < REF_PRESS_MIN || cur_press > REF_PRESS_MAX) safety_error = 1;

            // --- MOTOR VE LED MANTIĞI ---
            if (system_halted || safety_error) {
                TIM2->CCR1 = 4095;
                GPIOB->BSRR = (1 << 5);
                GPIOB->BSRR = (1 << (4 + 16));

                if (system_halted) {
                    sprintf(motor_status_str, "!!! ACIL DURDURMA KILITLI !!!");
                } else {
                    sprintf(motor_status_str, "HATA: LIMIT ASILDI (MOTOR DURDU)");
                }
            } else {
                TIM2->CCR1 = 4095 - pot_raw;
                GPIOB->BSRR = (1 << (5 + 16));
                GPIOB->BSRR = (1 << 4);

                if (pot_percent < 2) {
                    sprintf(motor_status_str, "BEKLEMEDE (Pot Sifir)");
                } else {
                    sprintf(motor_status_str, "ACIK (Guc: %% %d)", pot_percent);
                }
            }

            // --- TERA TERM ---
            uart_print("\033[2J\033[H");
            uart_print("=========================================\r\n");
            uart_print("        ULTIMATE 4S BMS & TELEMETRI      \r\n");
            uart_print("=========================================\r\n");

            sprintf(str_buf, " [ATMOSFER] Sicaklik : %.1f C\r\n", bme_temp);
            uart_print(str_buf);

            sprintf(str_buf, " [ATMOSFER] Nem      : %% %.1f\r\n", dht_hum);
            uart_print(str_buf);

            sprintf(str_buf, " [ATMOSFER] Basinc   : %.0f hPa\r\n", bme_press / 100.0f);
            uart_print(str_buf);

            uart_print("-----------------------------------------\r\n");

            sprintf(str_buf, " [INA226] Ana Voltaj : %.2f V\r\n", ina_volt);
            uart_print(str_buf);

            sprintf(str_buf, " [INA226] Ana Akim   : %.0f mA\r\n", ina_curr);
            uart_print(str_buf);

            sprintf(str_buf, " [SISTEM] Motor      : %s\r\n", motor_status_str);
            uart_print(str_buf);

            uart_print("-----------------------------------------\r\n");
            uart_print(" [HATA AYIKLAMA] Direnc Dugum Voltajlari:\r\n");

            sprintf(str_buf, " Node1: %.2fV | Node2: %.2fV\r\n", node[0], node[1]);
            uart_print(str_buf);

            sprintf(str_buf, " Node3: %.2fV | Node4: %.2fV\r\n", node[2], node[3]);
            uart_print(str_buf);

            uart_print("-----------------------------------------\r\n");

            for(int i = 0; i < 4; i++) {
                sprintf(str_buf, " [BMS] Hucre %d: %.2f V  (%% %d)\r\n", i+1, cell[i], pct[i]);
                uart_print(str_buf);
            }

            uart_print("=========================================\r\n");
        }
    }
}
