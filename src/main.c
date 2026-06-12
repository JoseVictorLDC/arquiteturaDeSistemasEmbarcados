#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* ============================================================
 * CONFIGURACOES
 * ============================================================ */

#define STACK_SIZE       1024
#define THREAD_PRIORITY  5

/* Recurso compartilhado, propositalmente sem sincronizacao */
volatile int saldo_vitrine = 0;

/* ============================================================
 * THREAD DO PADEIRO
 * ============================================================ */

void thread_padeiro(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sleep(K_SECONDS(1));

        saldo_vitrine++;

        int64_t tempo_ms = k_uptime_get();

        printk("[%lld.%03lld s] [PADEIRO] Produziu 1 pao | Saldo: %d\n",
               (long long)(tempo_ms / 1000),
               (long long)(tempo_ms % 1000),
               saldo_vitrine);
    }
}

/* ============================================================
 * THREAD DO CLIENTE
 * ============================================================ */

void thread_cliente(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sleep(K_MSEC(1500));

        saldo_vitrine--;

        int64_t tempo_ms = k_uptime_get();

        printk("[%lld.%03lld s] [CLIENTE] Retirou 1 pao | Saldo: %d\n",
               (long long)(tempo_ms / 1000),
               (long long)(tempo_ms % 1000),
               saldo_vitrine);
    }
}

/* ============================================================
 * CRIACAO DAS THREADS
 * ============================================================ */

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

/* ============================================================
 * MAIN
 * ============================================================ */

int main(void)
{
    printk("\n=== PADARIA: PARTE 1 - SEM SINCRONIZACAO ===\n");
    printk("[0.000 s] Saldo inicial: %d\n\n", saldo_vitrine);

    return 0;
}
