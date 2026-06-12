#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define STACK_SIZE       1024
#define THREAD_PRIORITY  5

/* Recurso compartilhado */
static int saldo_vitrine = 0;

/* Mutex que protege saldo_vitrine */
K_MUTEX_DEFINE(mutex_vitrine);

void thread_padeiro(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sleep(K_SECONDS(1));

        k_mutex_lock(&mutex_vitrine, K_FOREVER);

        saldo_vitrine++;

        int64_t tempo_ms = k_uptime_get();

        printk("[%lld.%03lld s] [PADEIRO] Produziu 1 pao | Saldo: %d\n",
               (long long)(tempo_ms / 1000),
               (long long)(tempo_ms % 1000),
               saldo_vitrine);

        k_mutex_unlock(&mutex_vitrine);
    }
}

void thread_cliente(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sleep(K_MSEC(1500));

        k_mutex_lock(&mutex_vitrine, K_FOREVER);

        saldo_vitrine--;

        int64_t tempo_ms = k_uptime_get();

        printk("[%lld.%03lld s] [CLIENTE] Retirou 1 pao | Saldo: %d\n",
               (long long)(tempo_ms / 1000),
               (long long)(tempo_ms % 1000),
               saldo_vitrine);

        k_mutex_unlock(&mutex_vitrine);
    }
}

K_THREAD_DEFINE(
    padeiro_id,
    STACK_SIZE,
    thread_padeiro,
    NULL,
    NULL,
    NULL,
    THREAD_PRIORITY,
    0,
    0
);

K_THREAD_DEFINE(
    cliente_id,
    STACK_SIZE,
    thread_cliente,
    NULL,
    NULL,
    NULL,
    THREAD_PRIORITY,
    0,
    0
);

int main(void)
{
    printk("\n=== PADARIA: PARTE 2 - UTILIZANDO MUTEX ===\n");
    printk("[0.000 s] Saldo inicial: %d\n\n", saldo_vitrine);

    return 0;
}
