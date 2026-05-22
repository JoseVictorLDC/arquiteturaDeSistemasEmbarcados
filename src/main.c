#include <zephyr/kernel.h>             // Funções básicas do Zephyr (ex: k_msleep, k_thread, etc.)
#include <zephyr/device.h>             // API para obter e utilizar dispositivos do sistema
#include <zephyr/drivers/gpio.h>       // API para controle de pinos de entrada/saída (GPIO)
#include <pwm_z42.h>                // Biblioteca personalizada com funções de controle do TPM (Timer/PWM Module)
#include <zephyr/sys/printk.h>
#include <zephyr/console/console.h>
#include <stdio.h>
#include <math.h>

// Define o valor do registrador MOD do TPM para configurar o período do PWM
#define TPM_MODULE 1000         // Define a frequência do PWM fpwm = (TPM_CLK / (TPM_MODULE * PS))

// Coeficientes da regressão do tipo green = alpha * ln(red + 1) + beta
#define ALPHA 0.550738058004923
#define BETA 0.550738058004923

#define MAX_INPUT_SIZE 3

float Intensity(const float red);
float GetIntensity();

int main(void)
{
    // Inicializa o módulo TPM2 com:
    // - base do TPMx
    // - fonte de clock PLL/FLL (TPM_CLK)
    // - valor do registrador MOD
    // - tipo de clock (TPM_CLK)
    // - prescaler de 1 a 128 (PS)
    // - modo de operação EDGE_PWM
    pwm_tpm_Init(TPM2, TPM_PLLFLL, TPM_MODULE, TPM_CLK, PS_128, EDGE_PWM);
    console_getline_init();

    // Inicializa o canal 0 do TPM2 para gerar sinal PWM na porta GPIOB_18
    // - modo TPM_PWM_H (nível alto durante o pulso)
    pwm_tpm_Ch_Init(TPM2, 0, TPM_PWM_H, GPIOB, 18); // Red
    pwm_tpm_Ch_Init(TPM2, 1, TPM_PWM_H, GPIOB, 19); // Green

    // Loop infinito
    for (;;)
    {
        const float red = GetIntensity();
        
        if (red == -1)
            continue;
        
        const float green = Intensity(red);

        // Valores de duty cycle correspondentes a diferentes larguras de pulso
        uint16_t duty_red  = (red) * TPM_MODULE;       
        uint16_t duty_green = (green) * TPM_MODULE;

        // Define o valor do duty cycle: nesse caso, duty_100 (LED quase desligado)
        pwm_tpm_CnV(TPM2, 0, duty_red);
        pwm_tpm_CnV(TPM2, 1, duty_green);

        k_msleep(100);
    }

    return 0;
}

float GetIntensity()
{
    printk("Escolha um valor de intensidade do LED em porcentagem:\n");
    char* user_input = console_getline();
    if (user_input == NULL) 
    {
        printk("Erro de leitura de console\n");
        return -1;
    }

    // Loop de parse
    float intensity = 0;
    for(uint8_t i = 0; i < MAX_INPUT_SIZE; i++)
    {
        if(user_input[i] < '0' || user_input[i] > '9')
        {
            break;
        }
        intensity = intensity * 10 + (user_input[i] - '0');
    }

    if(intensity > 100)
    {
        return 0.0;
    }

    return (1.0f - intensity / 100);
}

// o coeficiente BETA acaba gerando valores mais esverdeados para pontos muito próximos a intensidade 0
float Intensity(const float red)
{
    if(red == 1)
        return 1;
    float green = (ALPHA * log(red + 1) + BETA);
    return green;
}