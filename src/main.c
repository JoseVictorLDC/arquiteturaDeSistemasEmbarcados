#include <zephyr/kernel.h>             // Funções básicas do Zephyr (ex: k_msleep, k_thread, etc.)
#include <zephyr/device.h>             // API para obter e utilizar dispositivos do sistema
#include <zephyr/drivers/gpio.h>       // API para controle de pinos de entrada/saída (GPIO)
#include <pwm_z42.h>                // Biblioteca personalizada com funções de controle do TPM (Timer/PWM Module)
#include <zephyr/sys/printk.h>
#include <zephyr/console/console.h>
#include <stdio.h>
#include <math.h>

#define STACK_SIZE 1024
#define PADEIRO_PRIORITY 0
#define CLIENTE_PRIORITY 1

void Padeiro();
void Cliente();

long int saldo_vitrine = 0;

K_SEM_DEFINE(padeiro_sem, 0, 10); // Indica quantos pães podem ser postos na vitrine 
K_SEM_DEFINE(cliente_sem, 0, 10); // Indica quantos pães há na vitrine

int main(void)
{
    printk("Padeiro está vendendo. Saldo inicial da vitrine: %lu\n\n", saldo_vitrine);
    return 0;
}

void Padeiro()
{
    for(;;)
    {
        k_msleep(1000);
        k_sem_take(&padeiro_sem, K_FOREVER);
        saldo_vitrine++;
        k_sem_give(&cliente_sem);
        printk("Padeiro adicionou na vitrine. \nSaldo da vitrine: %ld\n\n", saldo_vitrine);
    }
}

void Cliente()
{
    for(;;)
    {
        k_msleep(1500);
        k_sem_take(&cliente_sem, K_FOREVER);
        saldo_vitrine--;
        k_sem_give(&padeiro_sem);
        printk("Cliente comprou da padaria. \nSaldo da vitrine: %ld\n\n", saldo_vitrine);
    }
}

K_THREAD_DEFINE(padeiro, STACK_SIZE, Padeiro, NULL, NULL, NULL, PADEIRO_PRIORITY, 0, 0);
K_THREAD_DEFINE(cliente, STACK_SIZE, Cliente, NULL, NULL, NULL, CLIENTE_PRIORITY, 0, 0);
