import argparse
import csv
import queue
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any, Dict, Optional

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox
import serial
from serial.tools import list_ports


# ============================================================
# CONFIGURACOES
# ============================================================

MAX_POINTS = 20_000
PLOT_WINDOW_SECONDS = 8.0
PLOT_INTERVAL_SECONDS = 0.02
MAX_ITEMS_PER_FRAME = 2_000
SERIAL_QUEUE_MAXSIZE = 30_000

DEFAULT_ACQ_HZ = 100
MAX_ACQ_HZ = 2_000
DEFAULT_LP_CUTOFF_HZ = 5

RATE_OPTIONS = {
    "100 Hz": 100,
    "500 Hz": 500,
    "1000 Hz": 1000,
    "2000 Hz": 2000,
}


# ============================================================
# SERIAL E PARSE
# ============================================================


def list_available_ports() -> None:
    ports = list(list_ports.comports())

    if not ports:
        print("Nenhuma porta serial encontrada.")
        return

    print("Portas seriais disponiveis:")
    for port in ports:
        print(f"  {port.device} - {port.description}")


def parse_line(line: str) -> Optional[Dict[str, Any]]:
    line = line.strip()

    if not line:
        return None

    parts = line.split(",")

    try:
        if parts[0] == "DATA" and len(parts) == 7:
            return {
                "type": "DATA",
                "seq": int(parts[1]),
                "timestamp_us": int(parts[2]),
                "raw_adc": int(parts[3]),
                "mv": int(parts[4]),
                "value_out_mv": int(parts[5]),
                "queue_drops": int(parts[6]),
            }

        if parts[0] == "STAT" and len(parts) == 8:
            return {
                "type": "STAT",
                "timestamp_us": int(parts[1]),
                "produced": int(parts[2]),
                "transmitted": int(parts[3]),
                "queue_drops": int(parts[4]),
                "adc_errors": int(parts[5]),
                "acq_rate_hz": int(parts[6]),
                "tx_rate_hz": int(parts[7]),
            }

        if parts[0] == "CFG" and len(parts) == 6:
            return {
                "type": "CFG",
                "rate_hz": int(parts[1]),
                "period_us": int(parts[2]),
                "filter_mode": parts[3],
                "cutoff_hz": int(parts[4]),
                "fir_taps": int(parts[5]),
            }

        if parts[0] in ("ACK", "ERR", "READY"):
            return {
                "type": parts[0],
                "raw": line,
            }

    except (ValueError, IndexError):
        return None

    # Linhas do logger do Zephyr e outras mensagens sao ignoradas.
    return None


def open_serial(port: str, baud: int) -> serial.Serial:
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser


def put_latest(
    data_queue: queue.Queue,
    item: Dict[str, Any],
    reader_stats: Dict[str, int],
    reader_stats_lock: threading.Lock,
) -> None:
    """Insere sem bloquear. Se a fila do PC lotar, remove o item mais antigo."""
    try:
        data_queue.put_nowait(item)
        return
    except queue.Full:
        pass

    try:
        data_queue.get_nowait()
    except queue.Empty:
        pass

    with reader_stats_lock:
        reader_stats["pc_queue_drops"] += 1

    try:
        data_queue.put_nowait(item)
    except queue.Full:
        with reader_stats_lock:
            reader_stats["pc_queue_drops"] += 1


def serial_reader(
    ser: serial.Serial,
    data_queue: queue.Queue,
    stop_event: threading.Event,
    reader_stats: Dict[str, int],
    reader_stats_lock: threading.Lock,
) -> None:
    while not stop_event.is_set():
        try:
            raw_line = ser.readline()

            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="ignore").strip()
            parsed = parse_line(line)

            if parsed is not None:
                put_latest(
                    data_queue,
                    parsed,
                    reader_stats,
                    reader_stats_lock,
                )

        except (serial.SerialException, OSError) as exc:
            if not stop_event.is_set():
                print(f"Erro na leitura serial: {exc}")
                time.sleep(0.2)
        except Exception as exc:  # Protege a thread contra uma linha inesperada.
            if not stop_event.is_set():
                print(f"Erro inesperado na leitura serial: {exc}")
                time.sleep(0.2)


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Grafico em tempo real para a Atividade 8 com Zephyr."
    )
    parser.add_argument(
        "--port",
        type=str,
        required=False,
        help="Porta serial, por exemplo COM5.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate da UART.",
    )
    parser.add_argument(
        "--csv",
        type=str,
        default=None,
        help="Arquivo CSV opcional para salvar as amostras.",
    )
    args = parser.parse_args()

    if args.port is None:
        list_available_ports()
        print("\nExecute assim:")
        print(
            "python scripts/plot_serial_realtime.py "
            "--port COM5 --baud 115200"
        )
        return

    csv_file = None
    csv_writer = None
    csv_rows_since_flush = 0

    if args.csv is not None:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "pc_time",
            "seq",
            "timestamp_us",
            "time_s",
            "raw_adc",
            "mv_original",
            "value_out_mv",
            "queue_drops_total_board",
        ])

    print(f"Abrindo porta {args.port} em {args.baud} baud...")

    try:
        ser = open_serial(args.port, args.baud)
    except serial.SerialException as exc:
        print(f"Nao foi possivel abrir {args.port}: {exc}")
        print("Feche o Serial Monitor do PlatformIO e tente novamente.")
        if csv_file is not None:
            csv_file.close()
        return

    print("Serial aberta.")

    data_queue: queue.Queue = queue.Queue(maxsize=SERIAL_QUEUE_MAXSIZE)
    stop_event = threading.Event()
    plot_lock = threading.Lock()
    reader_stats_lock = threading.Lock()
    reader_stats = {"pc_queue_drops": 0}

    state: Dict[str, Any] = {
        "first_timestamp_us": None,
        "last_seq": None,
        "received_samples": 0,
        "lost_by_seq": 0,
        "queue_drop_baseline": None,
        "last_queue_drops_total": 0,
        "session_queue_drops": 0,
        "last_rate_time": time.monotonic(),
        "last_rate_count": 0,
        "python_effective_rate": 0.0,
        "board_acq_rate": 0,
        "board_tx_rate": 0,
        "max_board_acq_rate": 0,
        "max_board_tx_rate": 0,
        "board_produced": 0,
        "board_transmitted": 0,
        "adc_errors": 0,
        "requested_rate_hz": DEFAULT_ACQ_HZ,
        "period_us": 1_000_000 // DEFAULT_ACQ_HZ,
        "filter_label": "Sem filtro",
        "status": "Inicializando...",
        "paused": False,
        "fir_taps": 0,
    }

    x_time = deque(maxlen=MAX_POINTS)
    y_original_mv = deque(maxlen=MAX_POINTS)
    y_filtered_mv = deque(maxlen=MAX_POINTS)

    def clear_serial_queue() -> None:
        while True:
            try:
                data_queue.get_nowait()
            except queue.Empty:
                break

    def clear_plot_data(reset_counters: bool = True) -> None:
        with plot_lock:
            x_time.clear()
            y_original_mv.clear()
            y_filtered_mv.clear()

            state["first_timestamp_us"] = None
            state["last_seq"] = None
            state["queue_drop_baseline"] = None
            state["last_queue_drops_total"] = 0
            state["session_queue_drops"] = 0
            state["last_rate_time"] = time.monotonic()
            state["python_effective_rate"] = 0.0
            state["board_acq_rate"] = 0
            state["board_tx_rate"] = 0
            state["max_board_acq_rate"] = 0
            state["max_board_tx_rate"] = 0

            if reset_counters:
                state["received_samples"] = 0
                state["lost_by_seq"] = 0

            # Nao gera pico falso quando os contadores nao sao zerados.
            state["last_rate_count"] = state["received_samples"]

        clear_serial_queue()

    def send_command(command: str) -> None:
        try:
            line = command.strip() + "\n"
            ser.write(line.encode("utf-8"))
            ser.flush()
            state["status"] = f"Enviado: {command.strip()}"
            print(f"[CMD] {command.strip()}")
        except (serial.SerialException, OSError) as exc:
            state["status"] = "Erro ao enviar comando."
            print(f"Erro ao enviar comando: {exc}")

    def toggle_pause() -> None:
        state["paused"] = not state["paused"]

        if state["paused"]:
            state["status"] = "Pausado; a placa continua adquirindo."
            clear_serial_queue()
            pause_button.label.set_text("Continuar")
            print("Visualizacao pausada.")
        else:
            clear_plot_data(reset_counters=False)
            state["status"] = "Visualizacao retomada."
            pause_button.label.set_text("Pausar")
            print("Visualizacao retomada.")

    def submit_custom_acq_frequency(freq_text: str) -> None:
        try:
            freq_hz = int(float(str(freq_text).replace(",", ".")))

            if not 1 <= freq_hz <= MAX_ACQ_HZ:
                raise ValueError

            clear_plot_data(reset_counters=True)
            state["requested_rate_hz"] = freq_hz
            send_command(f"RATE,{freq_hz}")

        except ValueError:
            state["status"] = (
                f"Frequencia invalida: use de 1 a {MAX_ACQ_HZ} Hz."
            )
            print(
                f"Digite uma frequencia entre 1 e {MAX_ACQ_HZ} Hz."
            )

    def submit_lowpass_frequency(cutoff_text: str) -> None:
        try:
            cutoff_hz = int(float(str(cutoff_text).replace(",", ".")))

            if cutoff_hz <= 0:
                raise ValueError

            clear_plot_data(reset_counters=False)
            state["filter_label"] = f"Passa-baixa {cutoff_hz} Hz"
            send_command(f"FILTER,LOWPASS,{cutoff_hz}")

        except ValueError:
            state["status"] = "Frequencia de corte invalida."
            print("Digite uma frequencia de corte positiva em Hz.")

    def set_filter_none() -> None:
        clear_plot_data(reset_counters=False)
        state["filter_label"] = "Sem filtro"
        send_command("FILTER,NONE")

    def reset_system() -> None:
        try:
            acq_freq_box.set_val(str(DEFAULT_ACQ_HZ))
            lp_cutoff_box.set_val(str(DEFAULT_LP_CUTOFF_HZ))
        except Exception:
            pass

        clear_plot_data(reset_counters=True)

        state["requested_rate_hz"] = DEFAULT_ACQ_HZ
        state["filter_label"] = "Sem filtro"
        state["fir_taps"] = 0
        state["board_produced"] = 0
        state["board_transmitted"] = 0
        state["adc_errors"] = 0

        send_command("RESET")

    reader_thread = threading.Thread(
        target=serial_reader,
        args=(
            ser,
            data_queue,
            stop_event,
            reader_stats,
            reader_stats_lock,
        ),
        daemon=True,
    )
    reader_thread.start()

    # ============================================================
    # INTERFACE GRAFICA
    # ============================================================

    plt.ion()
    fig = plt.figure(figsize=(22, 11))

    try:
        manager = plt.get_current_fig_manager()
        if hasattr(manager, "window"):
            try:
                manager.window.state("zoomed")
            except Exception:
                try:
                    manager.window.showMaximized()
                except Exception:
                    pass
    except Exception:
        pass

    ax = fig.add_axes([0.045, 0.095, 0.615, 0.84])

    line_original, = ax.plot(
        [],
        [],
        linewidth=0.8,
        alpha=0.65,
        label="Sinal original (mV)",
    )

    line_filtered, = ax.plot(
        [],
        [],
        linewidth=1.2,
        label="Saida do filtro na placa (mV)",
    )

    ax.set_title("Aquisicao em tempo real - FRDM-KL25Z")
    ax.set_xlabel("Tempo na janela movel (s)")
    ax.set_ylabel("Tensao (mV)")
    ax.grid(True)
    ax.legend(loc="upper right")

    panel_x = 0.690
    panel_y = 0.045
    panel_w = 0.290
    panel_h = 0.905

    panel = fig.add_axes([panel_x, panel_y, panel_w, panel_h])
    panel.set_facecolor("0.965")
    panel.set_xticks([])
    panel.set_yticks([])

    for spine in panel.spines.values():
        spine.set_visible(False)

    def add_panel_axes(rx: float, ry: float, rw: float, rh: float):
        return fig.add_axes([
            panel_x + rx * panel_w,
            panel_y + ry * panel_h,
            rw * panel_w,
            rh * panel_h,
        ])

    def panel_title(y: float, text: str):
        return panel.text(
            0.06,
            y,
            text,
            fontsize=10.5,
            fontweight="bold",
            ha="left",
            va="top",
            transform=panel.transAxes,
        )

    panel.text(
        0.06,
        0.982,
        "Controles e metricas",
        fontsize=16,
        fontweight="bold",
        ha="left",
        va="top",
        transform=panel.transAxes,
    )

    pause_ax = add_panel_axes(0.62, 0.930, 0.32, 0.047)
    pause_button = Button(
        pause_ax,
        "Pausar",
        color="#f2c94c",
        hovercolor="#f7dc6f",
    )
    pause_button.on_clicked(lambda _event: toggle_pause())

    status_box = panel.text(
        0.06,
        0.915,
        "Status:\nInicializando...",
        fontsize=8.6,
        ha="left",
        va="top",
        transform=panel.transAxes,
        linespacing=1.18,
        bbox=dict(
            boxstyle="round,pad=0.35",
            facecolor="white",
            edgecolor="0.75",
        ),
    )

    metrics_box = panel.text(
        0.06,
        0.805,
        "",
        fontsize=8.2,
        ha="left",
        va="top",
        transform=panel.transAxes,
        linespacing=1.15,
        bbox=dict(
            boxstyle="round,pad=0.35",
            facecolor="white",
            edgecolor="0.75",
        ),
    )

    panel_title(0.580, "Frequencia solicitada (Hz)")
    acq_freq_ax = add_panel_axes(0.06, 0.515, 0.52, 0.046)
    acq_freq_box = TextBox(
        acq_freq_ax,
        "",
        initial=str(DEFAULT_ACQ_HZ),
    )

    acq_send_ax = add_panel_axes(0.64, 0.515, 0.30, 0.046)
    acq_send_button = Button(acq_send_ax, "Enviar")
    acq_send_button.on_clicked(
        lambda _event: submit_custom_acq_frequency(acq_freq_box.text)
    )

    panel_title(0.480, "Atalhos")
    rate_buttons = []
    rate_y = 0.420
    rate_w = 0.205
    rate_gap = 0.020

    for idx, (label, hz) in enumerate(RATE_OPTIONS.items()):
        button_ax = add_panel_axes(
            0.06 + idx * (rate_w + rate_gap),
            rate_y,
            rate_w,
            0.046,
        )
        button = Button(button_ax, label)

        def make_rate_callback(rate_hz: int):
            def callback(_event):
                try:
                    acq_freq_box.set_val(str(rate_hz))
                except Exception:
                    pass
                submit_custom_acq_frequency(str(rate_hz))

            return callback

        button.on_clicked(make_rate_callback(hz))
        rate_buttons.append(button)

    panel_title(0.365, "Filtro FIR: corte nominal (Hz)")
    lp_cutoff_ax = add_panel_axes(0.06, 0.300, 0.52, 0.046)
    lp_cutoff_box = TextBox(
        lp_cutoff_ax,
        "",
        initial=str(DEFAULT_LP_CUTOFF_HZ),
    )

    lp_send_ax = add_panel_axes(0.64, 0.300, 0.30, 0.046)
    lp_send_button = Button(
        lp_send_ax,
        "Ativar PB",
        color="#d62728",
        hovercolor="#ff9896",
    )
    lp_send_button.on_clicked(
        lambda _event: submit_lowpass_frequency(lp_cutoff_box.text)
    )

    panel_title(0.265, "Filtro")
    no_filter_ax = add_panel_axes(0.06, 0.205, 0.88, 0.047)
    no_filter_button = Button(
        no_filter_ax,
        "Sem filtro",
        color="0.85",
        hovercolor="0.75",
    )
    no_filter_button.on_clicked(lambda _event: set_filter_none())

    panel_title(0.165, "Reset")
    reset_ax = add_panel_axes(0.06, 0.105, 0.88, 0.047)
    reset_button = Button(
        reset_ax,
        "Sem filtro + 100 Hz",
        color="#444444",
        hovercolor="#777777",
    )
    reset_button.on_clicked(lambda _event: reset_system())

    # Mantem referencias para impedir coleta de lixo dos widgets.
    widgets = [
        acq_freq_box,
        acq_send_button,
        lp_cutoff_box,
        lp_send_button,
        no_filter_button,
        reset_button,
        pause_button,
    ] + rate_buttons
    _ = widgets

    print("\nLendo dados...")
    print("O grafico mostra o sinal original e a saida FIR da placa.")
    print("Metricas principais:")
    print("  - aquisicao efetiva calculada na placa")
    print("  - transmissao efetiva pela UART")
    print("  - maxima aquisicao observada na configuracao atual")
    print("  - perdas na fila da placa e por salto de sequencia")
    print("Comandos disponiveis: RATE, FILTER, STATUS e RESET.")
    print("Pressione Ctrl+C ou feche a janela para parar.\n")

    send_command("STATUS")

    try:
        while plt.fignum_exists(fig.number):
            updated_plot = False

            if state["paused"]:
                clear_serial_queue()
            else:
                # Limite por quadro: a interface nunca fica presa tentando
                # esvaziar uma fila que continua recebendo dados.
                for _ in range(MAX_ITEMS_PER_FRAME):
                    try:
                        item = data_queue.get_nowait()
                    except queue.Empty:
                        break

                    item_type = item["type"]

                    if item_type == "DATA":
                        seq = item["seq"]
                        timestamp_us = item["timestamp_us"]

                        with plot_lock:
                            if state["first_timestamp_us"] is None:
                                state["first_timestamp_us"] = timestamp_us

                            if state["last_seq"] is not None:
                                expected_seq = state["last_seq"] + 1
                                if seq > expected_seq:
                                    state["lost_by_seq"] += seq - expected_seq

                            state["last_seq"] = seq
                            state["received_samples"] += 1
                            state["last_queue_drops_total"] = item["queue_drops"]

                            if state["queue_drop_baseline"] is None:
                                state["queue_drop_baseline"] = item["queue_drops"]

                            state["session_queue_drops"] = max(
                                0,
                                item["queue_drops"]
                                - state["queue_drop_baseline"],
                            )

                            time_s = (
                                timestamp_us - state["first_timestamp_us"]
                            ) / 1_000_000.0

                            x_time.append(time_s)
                            y_original_mv.append(item["mv"])
                            y_filtered_mv.append(item["value_out_mv"])

                        if csv_writer is not None:
                            csv_writer.writerow([
                                datetime.now().isoformat(timespec="milliseconds"),
                                item["seq"],
                                item["timestamp_us"],
                                time_s,
                                item["raw_adc"],
                                item["mv"],
                                item["value_out_mv"],
                                item["queue_drops"],
                            ])
                            csv_rows_since_flush += 1

                            if csv_rows_since_flush >= 500:
                                csv_file.flush()
                                csv_rows_since_flush = 0

                        updated_plot = True

                    elif item_type == "STAT":
                        state["board_produced"] = item["produced"]
                        state["board_transmitted"] = item["transmitted"]
                        state["last_queue_drops_total"] = item["queue_drops"]
                        state["adc_errors"] = item["adc_errors"]
                        state["board_acq_rate"] = item["acq_rate_hz"]
                        state["board_tx_rate"] = item["tx_rate_hz"]
                        state["max_board_acq_rate"] = max(
                            state["max_board_acq_rate"],
                            item["acq_rate_hz"],
                        )
                        state["max_board_tx_rate"] = max(
                            state["max_board_tx_rate"],
                            item["tx_rate_hz"],
                        )

                    elif item_type == "CFG":
                        state["requested_rate_hz"] = item["rate_hz"]
                        state["period_us"] = item["period_us"]

                        if item["filter_mode"] == "LOWPASS_FIR":
                            state["filter_label"] = (
                                f"Passa-baixa {item['cutoff_hz']} Hz"
                            )
                        else:
                            state["filter_label"] = "Sem filtro"

                        state["fir_taps"] = item["fir_taps"]
                        state["status"] = (
                            f"Config OK: {item['rate_hz']} Hz, "
                            f"{item['filter_mode']}, taps={item['fir_taps']}"
                        )

                    elif item_type in ("ACK", "ERR", "READY"):
                        state["status"] = item["raw"]
                        print(f"[{item_type}] {item['raw']}")

            now = time.monotonic()
            elapsed_rate = now - state["last_rate_time"]

            if not state["paused"] and elapsed_rate >= 1.0:
                delta_count = (
                    state["received_samples"] - state["last_rate_count"]
                )
                state["python_effective_rate"] = delta_count / elapsed_rate

                with reader_stats_lock:
                    pc_queue_drops = reader_stats["pc_queue_drops"]

                print(
                    "[TAXAS] "
                    f"solicitada={state['requested_rate_hz']} Hz | "
                    f"aquisicao_placa={state['board_acq_rate']} Hz | "
                    f"max_aquisicao={state['max_board_acq_rate']} Hz | "
                    f"tx_placa={state['board_tx_rate']} Hz | "
                    f"recebida_python={state['python_effective_rate']:.1f} Hz | "
                    f"drops_fila_sessao={state['session_queue_drops']} | "
                    f"perdas_seq={state['lost_by_seq']} | "
                    f"drops_fila_pc={pc_queue_drops}"
                )

                state["last_rate_time"] = now
                state["last_rate_count"] = state["received_samples"]

            with plot_lock:
                xs = list(x_time)
                ys_original = list(y_original_mv)
                ys_filtered = list(y_filtered_mv)

            if updated_plot and xs:
                latest_t = xs[-1]
                x_display = [x - latest_t for x in xs]

                line_original.set_data(x_display, ys_original)
                line_filtered.set_data(x_display, ys_filtered)

                ax.set_xlim(-PLOT_WINDOW_SECONDS, 0.0)

                combined_values = ys_original + ys_filtered
                ymin = min(combined_values)
                ymax = max(combined_values)

                if ymin == ymax:
                    ymin -= 100
                    ymax += 100

                margin_y = max(30, 0.12 * (ymax - ymin))
                ax.set_ylim(ymin - margin_y, ymax + margin_y)

            with reader_stats_lock:
                pc_queue_drops = reader_stats["pc_queue_drops"]

            status_box.set_text(f"Status:\n{state['status']}")
            metrics_box.set_text(
                f"Filtro: {state['filter_label']}\n"
                f"Taps FIR: {state['fir_taps']}\n"
                f"Solicitada: {state['requested_rate_hz']} Hz\n"
                f"Periodo: {state['period_us']} us\n"
                f"Aquisicao placa: {state['board_acq_rate']} Hz\n"
                f"Max. aquisicao: {state['max_board_acq_rate']} Hz\n"
                f"TX placa: {state['board_tx_rate']} Hz\n"
                f"Max. TX: {state['max_board_tx_rate']} Hz\n"
                f"Recebida Python: {state['python_effective_rate']:.1f} Hz\n"
                f"Produzidas: {state['board_produced']}\n"
                f"Transmitidas: {state['board_transmitted']}\n"
                f"Drops fila (sessao): {state['session_queue_drops']}\n"
                f"Drops fila (total): {state['last_queue_drops_total']}\n"
                f"Perdas por seq.: {state['lost_by_seq']}\n"
                f"Erros ADC: {state['adc_errors']}\n"
                f"Drops fila PC: {pc_queue_drops}"
            )

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(PLOT_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print("\nEncerrando...")

    finally:
        stop_event.set()

        try:
            if ser.is_open:
                ser.close()
        except Exception:
            pass

        reader_thread.join(timeout=1.0)

        if csv_file is not None:
            csv_file.flush()
            csv_file.close()

        with reader_stats_lock:
            pc_queue_drops = reader_stats["pc_queue_drops"]

        print("\nResumo final:")
        print(f"Frequencia solicitada: {state['requested_rate_hz']} Hz")
        print(
            "Maior taxa de aquisicao medida na placa: "
            f"{state['max_board_acq_rate']} Hz"
        )
        print(
            "Maior taxa de transmissao medida na placa: "
            f"{state['max_board_tx_rate']} Hz"
        )
        print(f"Amostras recebidas pelo Python: {state['received_samples']}")
        print(f"Perdas detectadas por sequencia: {state['lost_by_seq']}")
        print(
            "Drops informados pela fila da placa nesta sessao: "
            f"{state['session_queue_drops']}"
        )
        print(f"Drops da fila do PC: {pc_queue_drops}")
        print(f"Filtro final: {state['filter_label']}")
        print("Finalizado.")


if __name__ == "__main__":
    main()
