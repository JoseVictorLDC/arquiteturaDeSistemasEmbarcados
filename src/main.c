#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define STACK_SIZE        1024
#define THREAD_PRIORITY   5
#define CAPACIDADE_MAXIMA 10

/* Recurso compartilhado */
static int saldo_vitrine = 0;

/* Controla o acesso à variável saldo_vitrine */
K_MUTEX_DEFINE(mutex_vitrine);

/*
 * sem_paes:
 *   quantidade de pães disponíveis.
 *
 * sem_vagas:
 *   quantidade de espaços livres na vitrine.
 */
K_SEM_DEFINE(sem_paes, 0, CAPACIDADE_MAXIMA);
K_SEM_DEFINE(sem_vagas, CAPACIDADE_MAXIMA, CAPACIDADE_MAXIMA);

void thread_padeiro(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sleep(K_SECONDS(1));

        /* Espera até existir espaço livre */
        k_sem_take(&sem_vagas, K_FOREVER);

        k_mutex_lock(&mutex_vitrine, K_FOREVER);

        saldo_vitrine++;

        int64_t tempo_ms = k_uptime_get();

        printk("[%lld.%03lld s] [PADEIRO] Produziu 1 pao | Saldo: %d\n",
               (long long)(tempo_ms / 1000),
               (long long)(tempo_ms % 1000),
               saldo_vitrine);

        k_mutex_unlock(&mutex_vitrine);

        /* Informa que existe um novo pão disponível */
        k_sem_give(&sem_paes);
    }
}

void thread_cliente(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    while (1) {
        k_sleep(K_MSEC(1500));

        /* Espera até existir pelo menos um pão */
        k_sem_take(&sem_paes, K_FOREVER);

        k_mutex_lock(&mutex_vitrine, K_FOREVER);

        saldo_vitrine--;

        int64_t tempo_ms = k_uptime_get();

        printk("[%lld.%03lld s] [CLIENTE] Retirou 1 pao | Saldo: %d\n",
               (long long)(tempo_ms / 1000),
               (long long)(tempo_ms % 1000),
               saldo_vitrine);

        k_mutex_unlock(&mutex_vitrine);

        /* Informa que uma nova vaga foi liberada */
        k_sem_give(&sem_vagas);
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
    printk("\n=== PADARIA: PARTE 3 - SEMAFOROS ===\n");
    printk("[0.000 s] Capacidade maxima: %d paes\n",
           CAPACIDADE_MAXIMA);
    printk("[0.000 s] Saldo inicial: %d\n\n",
           saldo_vitrine);

    return 0;
}
