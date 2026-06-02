#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>
#include <stdint.h>
#include <stdbool.h>
#include <pwm_z42.h>

#define TPM_IRQ_LINE        TPM1_IRQn
#define TPM_IRQ_PRIORITY    1
#define TPM_CLK_HZ          48000000ULL
#define TPM_PRESCALER       128ULL
#define PWM_PERIOD_US       60000u
#define PWM_HIGH_US         10u
#define PWM_MOD_TICKS       (((TPM_CLK_HZ / TPM_PRESCALER) * PWM_PERIOD_US) / 1000000u - 1u)
#define PWM_HIGH_TICKS      ((((TPM_CLK_HZ / TPM_PRESCALER) * PWM_HIGH_US) + 999999u) / 1000000u)
#define TPM_PWM_HIGH_TRUE          (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK)

volatile uint16_t captured = 0;
volatile uint16_t rise_time = 0;
volatile uint16_t fall_time = 0;
volatile uint16_t pulse_ticks = 0;
volatile bool new_measure = false;

static uint32_t ticks_to_us(uint16_t ticks)
{
    return (uint32_t)(((uint64_t)ticks * TPM_PRESCALER * 1000000ULL) / TPM_CLK_HZ);
}

void tpm1_isr(void *arg)
{
    (void)arg;

    if (TPM1->STATUS & TPM_STATUS_CH0F_MASK) {
        captured = TPM1->CONTROLS[0].CnV;

        TPM1->STATUS = TPM_STATUS_CH0F_MASK;

        if (GPIOA->PDIR & (1u << 12)) {
            rise_time = captured;
        } else {
            fall_time = captured;
            pulse_ticks = (uint16_t)(fall_time - rise_time);
            new_measure = true;
        }
    }
}

void main(void)
{
    uint16_t ticks;
    uint32_t pulse_us;
    uint32_t distance_cm_x10;

    printk("Iniciando PWM + Input Capture\n");

    IRQ_CONNECT(TPM_IRQ_LINE, TPM_IRQ_PRIORITY, tpm1_isr, NULL, 0);
    irq_enable(TPM_IRQ_LINE);

    /* PWM de trigger em PTD0 / TPM0_CH0 */
    pwm_tpm_Init(TPM0, TPM_PLLFLL, PWM_MOD_TICKS, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(TPM0, 0, TPM_PWM_HIGH_TRUE, GPIOD, 0);
    TPM0->CONTROLS[0].CnV = PWM_HIGH_TICKS;

    /* Captura em PTA12 / TPM1_CH0 nas bordas de subida e descida */
    pwm_tpm_Init(TPM1, TPM_PLLFLL, 65535, TPM_CLK, PS_128, EDGE_PWM);
    pwm_tpm_Ch_Init(
        TPM1,
        0,
        TPM_INPUT_CAPTURE_RISING | TPM_INPUT_CAPTURE_FALLING | TPM_CHANNEL_INTERRUPT,
        GPIOA,
        12
    );

    printk("PWM gerada em PTD0\n");
    printk("Input Capture lendo em PTA12\n");
    printk("Pulso PWM: aproximadamente 10 us a cada 60 ms\n");

    while (1) {
        if (new_measure) {
            unsigned int key = irq_lock();
            ticks = pulse_ticks;
            new_measure = false;
            irq_unlock(key);

            pulse_us = ticks_to_us(ticks);
            distance_cm_x10 = (pulse_us * 10u) / 58u;

            printk("Pulso alto: %u ticks | %u us | Distancia: %u.%u cm\n",
                   ticks,
                   pulse_us,
                   distance_cm_x10 / 10u,
                   distance_cm_x10 % 10u);
        }

        k_msleep(100);
    }
}
