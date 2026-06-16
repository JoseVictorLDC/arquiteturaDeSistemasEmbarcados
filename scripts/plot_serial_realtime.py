
import argparse
import csv
import queue
import threading
import time
from collections import deque
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, TextBox
import serial
from serial.tools import list_ports


# ============================================================
# CONFIGURAÇÕES
# ============================================================

MAX_POINTS = 8000
PLOT_INTERVAL = 0.02

DEFAULT_ACQ_HZ = 100
DEFAULT_LP_CUTOFF_HZ = 5

RATE_OPTIONS = {
    "100 Hz": 100,
    "500 Hz": 500,
    "1000 Hz": 1000,
}


# ============================================================
# SERIAL / PARSE
# ============================================================

def list_available_ports():
    ports = list(list_ports.comports())

    if not ports:
        print("Nenhuma porta serial encontrada.")
        return

    print("Portas seriais disponíveis:")
    for port in ports:
        print(f"  {port.device} - {port.description}")


def parse_line(line):
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

    except ValueError:
        return None

    return None


def open_serial(port, baud):
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
    time.sleep(2.0)
    ser.reset_input_buffer()
    return ser


def serial_reader(ser, data_queue, stop_event):
    while not stop_event.is_set():
        try:
            raw_line = ser.readline()

            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="ignore").strip()
            parsed = parse_line(line)

            if parsed is not None:
                data_queue.put(parsed)

        except Exception as exc:
            print(f"Erro na leitura serial: {exc}")
            time.sleep(0.2)


# ============================================================
# PROGRAMA PRINCIPAL
# ============================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=str, required=False, help="Porta serial. Ex: COM20")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate da UART")
    parser.add_argument("--csv", type=str, default=None, help="Nome do CSV para salvar os dados")
    args = parser.parse_args()

    if args.port is None:
        list_available_ports()
        print("\nExecute assim:")
        print("python scripts/plot_serial_realtime_serial_cmd.py --port COM20 --baud 115200")
        return

    csv_file = None
    csv_writer = None

    if args.csv is not None:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "pc_time",
            "seq",
            "timestamp_us",
            "time_s",
            "raw_adc",
            "mv",
            "value_out_mv",
            "queue_drops",
        ])

    print(f"Abrindo porta {args.port} em {args.baud} baud...")
    ser = open_serial(args.port, args.baud)
    print("Serial aberta.")

    data_queue = queue.Queue()
    stop_event = threading.Event()
    plot_lock = threading.Lock()

    state = {
        "first_timestamp_us": None,
        "first_seq": None,
        "last_seq": None,
        "received_samples": 0,
        "lost_by_seq": 0,
        "last_queue_drops": 0,
        "last_rate_time": time.time(),
        "last_rate_count": 0,
        "effective_rate": 0.0,
        "board_acq_rate": 0,
        "board_tx_rate": 0,
        "current_rate_label": "não definida",
        "filter_label": "Sem filtro",
        "status": "Inicializando...",
        "paused": False,
        "fir_taps": 0,
    }

    x_time = deque(maxlen=MAX_POINTS)
    y_mv = deque(maxlen=MAX_POINTS)

    def clear_serial_queue():
        while not data_queue.empty():
            try:
                data_queue.get_nowait()
            except queue.Empty:
                break

    def clear_plot_data(reset_counters=True):
        with plot_lock:
            x_time.clear()
            y_mv.clear()
            state["first_timestamp_us"] = None
            state["first_seq"] = None
            state["last_seq"] = None
            state["last_queue_drops"] = 0
            state["last_rate_time"] = time.time()
            state["last_rate_count"] = 0
            state["effective_rate"] = 0.0

            if reset_counters:
                state["received_samples"] = 0
                state["lost_by_seq"] = 0

        clear_serial_queue()

    def send_command(command):
        try:
            line = command.strip() + "\n"
            ser.write(line.encode("utf-8"))
            ser.flush()
            state["status"] = f"Enviado: {command.strip()}"
            print(f"[CMD] {command.strip()}")
        except Exception as exc:
            state["status"] = "Erro ao enviar comando."
            print(f"Erro ao enviar comando: {exc}")

    def toggle_pause():
        state["paused"] = not state["paused"]

        if state["paused"]:
            state["status"] = "Pausado. Modificações ainda podem ser enviadas."
            clear_serial_queue()
            pause_button.label.set_text("Continuar")
            print("Simulação pausada.")
        else:
            clear_plot_data(reset_counters=False)
            state["status"] = "Simulação retomada."
            pause_button.label.set_text("Pausar")
            print("Simulação retomada.")

    def submit_custom_acq_frequency(freq_text):
        try:
            freq_hz = int(float(str(freq_text).replace(",", ".")))
            if freq_hz <= 0:
                raise ValueError

            send_command(f"RATE,{freq_hz}")
            state["current_rate_label"] = f"{freq_hz} Hz"
            clear_plot_data()

        except ValueError:
            state["status"] = "Frequência de aquisição inválida."
            print("Digite uma frequência de aquisição válida em Hz. Exemplo: 250")

    def submit_lowpass_frequency(cutoff_text):
        try:
            cutoff_hz = int(float(str(cutoff_text).replace(",", ".")))
            if cutoff_hz <= 0:
                raise ValueError

            send_command(f"FILTER,LOWPASS,{cutoff_hz}")
            state["filter_label"] = f"Passa-baixa {cutoff_hz} Hz"
            clear_plot_data(reset_counters=False)

        except ValueError:
            state["status"] = "Frequência do passa-baixa inválida."
            print("Digite uma frequência de corte válida em Hz. Exemplo: 5")

    def set_filter_none():
        send_command("FILTER,NONE")
        state["filter_label"] = "Sem filtro"
        clear_plot_data(reset_counters=False)

    def reset_system():
        try:
            acq_freq_box.set_val(str(DEFAULT_ACQ_HZ))
            lp_cutoff_box.set_val(str(DEFAULT_LP_CUTOFF_HZ))
        except Exception:
            pass

        send_command("RESET")

        state["current_rate_label"] = "100 Hz"
        state["filter_label"] = "Sem filtro"
        state["fir_taps"] = 0

        clear_plot_data()

    reader_thread = threading.Thread(
        target=serial_reader,
        args=(ser, data_queue, stop_event),
        daemon=True,
    )
    reader_thread.start()

    # ============================================================
    # INTERFACE GRÁFICA
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

    line_mv, = ax.plot(
        [],
        [],
        linewidth=1.0,
        marker=".",
        markersize=2.0,
        label="Sinal filtrado na placa",
    )

    ax.set_title("Aquisição em Tempo Real - Potenciômetro")
    ax.set_xlabel("Tempo na janela móvel (s)")
    ax.set_ylabel("Tensão / sinal filtrado na placa (mV)")
    ax.grid(True)
    ax.legend(loc="upper right")

    panel_x = 0.690
    panel_y = 0.055
    panel_w = 0.290
    panel_h = 0.885

    panel = fig.add_axes([panel_x, panel_y, panel_w, panel_h])
    panel.set_facecolor("0.965")
    panel.set_xticks([])
    panel.set_yticks([])
    for spine in panel.spines.values():
        spine.set_visible(False)

    def add_panel_axes(rx, ry, rw, rh):
        return fig.add_axes([
            panel_x + rx * panel_w,
            panel_y + ry * panel_h,
            rw * panel_w,
            rh * panel_h,
        ])

    def panel_title(y, text):
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
        0.975,
        "Controles",
        fontsize=17,
        fontweight="bold",
        ha="left",
        va="top",
        transform=panel.transAxes,
    )

    pause_ax = add_panel_axes(0.60, 0.925, 0.34, 0.052)
    pause_button = Button(pause_ax, "Pausar", color="#f2c94c", hovercolor="#f7dc6f")
    pause_button.on_clicked(lambda event: toggle_pause())

    status_box = panel.text(
        0.06,
        0.900,
        "Status:\nInicializando...",
        fontsize=8.8,
        ha="left",
        va="top",
        transform=panel.transAxes,
        linespacing=1.20,
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="0.75"),
    )

    metrics_box = panel.text(
        0.06,
        0.790,
        "",
        fontsize=8.6,
        ha="left",
        va="top",
        transform=panel.transAxes,
        linespacing=1.17,
        bbox=dict(boxstyle="round,pad=0.35", facecolor="white", edgecolor="0.75"),
    )

    panel_title(0.635, "Frequência de aquisição (Hz)")
    acq_freq_ax = add_panel_axes(0.06, 0.555, 0.52, 0.050)
    acq_freq_box = TextBox(acq_freq_ax, "", initial=str(DEFAULT_ACQ_HZ))

    acq_send_ax = add_panel_axes(0.64, 0.555, 0.30, 0.050)
    acq_send_button = Button(acq_send_ax, "Enviar")
    acq_send_button.on_clicked(lambda event: submit_custom_acq_frequency(acq_freq_box.text))

    panel_title(0.515, "Atalhos de frequência")
    rate_buttons = []
    rate_y = 0.435
    rate_w = 0.27
    rate_gap = 0.035

    for idx, (label, hz) in enumerate(RATE_OPTIONS.items()):
        button_ax = add_panel_axes(0.06 + idx * (rate_w + rate_gap), rate_y, rate_w, 0.050)
        button = Button(button_ax, label)

        def make_rate_callback(rate_label, rate_hz):
            def callback(event):
                try:
                    acq_freq_box.set_val(str(rate_hz))
                except Exception:
                    pass

                send_command(f"RATE,{rate_hz}")
                state["current_rate_label"] = rate_label
                clear_plot_data()

            return callback

        button.on_clicked(make_rate_callback(label, hz))
        rate_buttons.append(button)

    panel_title(0.370, "Filtro passa-baixa na placa: corte (Hz)")
    lp_cutoff_ax = add_panel_axes(0.06, 0.290, 0.52, 0.050)
    lp_cutoff_box = TextBox(lp_cutoff_ax, "", initial=str(DEFAULT_LP_CUTOFF_HZ))

    lp_send_ax = add_panel_axes(0.64, 0.290, 0.30, 0.050)
    lp_send_button = Button(lp_send_ax, "Ativar PB", color="#d62728", hovercolor="#ff9896")
    lp_send_button.on_clicked(lambda event: submit_lowpass_frequency(lp_cutoff_box.text))

    panel_title(0.250, "Filtro")
    no_filter_ax = add_panel_axes(0.06, 0.170, 0.88, 0.052)
    no_filter_button = Button(no_filter_ax, "Sem filtro", color="0.85", hovercolor="0.75")
    no_filter_button.on_clicked(lambda event: set_filter_none())

    panel_title(0.145, "Reset")
    reset_ax = add_panel_axes(0.06, 0.065, 0.88, 0.052)
    reset_button = Button(reset_ax, "Sem filtro + 100 Hz", color="#444444", hovercolor="#777777")
    reset_button.on_clicked(lambda event: reset_system())

    widgets = [
        acq_freq_box,
        acq_send_button,
        lp_cutoff_box,
        lp_send_button,
        no_filter_button,
        reset_button,
        pause_button,
    ] + rate_buttons

    print("\nLendo dados...")
    print("Gráfico no domínio do tempo, eixo X em segundos.")
    print("Agora as modificações são enviadas via serial:")
    print("RATE,<Hz>")
    print("FILTER,LOWPASS,<Hz>")
    print("FILTER,NONE")
    print("RESET")
    print("O filtro passa-baixa FIR é aplicado dentro da placa.")
    print("Pressione Ctrl+C para parar.\n")

    send_command("STATUS")

    try:
        while plt.fignum_exists(fig.number):
            updated_plot = False

            if state["paused"]:
                clear_serial_queue()
            else:
                while not data_queue.empty():
                    item = data_queue.get()

                    if item["type"] == "DATA":
                        seq = item["seq"]
                        timestamp_us = item["timestamp_us"]

                        with plot_lock:
                            if state["first_timestamp_us"] is None:
                                state["first_timestamp_us"] = timestamp_us

                            if state["first_seq"] is None:
                                state["first_seq"] = seq

                            if state["last_seq"] is not None:
                                expected_seq = state["last_seq"] + 1
                                if seq > expected_seq:
                                    state["lost_by_seq"] += seq - expected_seq

                            state["last_seq"] = seq
                            state["received_samples"] += 1
                            state["last_queue_drops"] = item["queue_drops"]

                            time_s = (timestamp_us - state["first_timestamp_us"]) / 1_000_000.0

                            x_time.append(time_s)
                            y_mv.append(item["value_out_mv"])

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

                        updated_plot = True

                    elif item["type"] == "STAT":
                        state["board_acq_rate"] = item["acq_rate_hz"]
                        state["board_tx_rate"] = item["tx_rate_hz"]

                    elif item["type"] == "CFG":
                        state["current_rate_label"] = f"{item['rate_hz']} Hz"

                        if item["filter_mode"] == "LOWPASS_FIR":
                            state["filter_label"] = f"Passa-baixa {item['cutoff_hz']} Hz"
                        else:
                            state["filter_label"] = "Sem filtro"

                        state["fir_taps"] = item["fir_taps"]
                        state["status"] = (
                            f"Config OK: {item['rate_hz']} Hz, "
                            f"{item['filter_mode']}, taps={item['fir_taps']}"
                        )

                    elif item["type"] == "ACK":
                        state["status"] = item["raw"]
                        print(f"[{item['type']}] {item['raw']}")

                    elif item["type"] == "ERR":
                        state["status"] = item["raw"]
                        print(f"[{item['type']}] {item['raw']}")

                    elif item["type"] == "READY":
                        state["status"] = item["raw"]
                        print(f"[{item['type']}] {item['raw']}")

            now = time.time()

            if not state["paused"] and now - state["last_rate_time"] >= 1.0:
                delta_count = state["received_samples"] - state["last_rate_count"]
                state["effective_rate"] = delta_count / (now - state["last_rate_time"])

                print(
                    f"[PYTHON] "
                    f"recebidas={state['received_samples']} | "
                    f"taxa_recebida={state['effective_rate']:.1f} Hz | "
                    f"perdas_seq={state['lost_by_seq']} | "
                    f"drops_fila={state['last_queue_drops']} | "
                    f"filtro={state['filter_label']}"
                )

                state["last_rate_time"] = now
                state["last_rate_count"] = state["received_samples"]

            with plot_lock:
                xs = list(x_time)
                ys = list(y_mv)

            if updated_plot and len(xs) > 0:
                latest_t = xs[-1]

                # Eixo X em segundos, usando janela móvel.
                x_display = [x - latest_t for x in xs]
                line_mv.set_data(x_display, ys)

                ax.set_xlim(-8.0, 0.0)

                ymin = min(ys)
                ymax = max(ys)

                if ymin == ymax:
                    ymin -= 100
                    ymax += 100

                margin_y = max(30, 0.12 * (ymax - ymin))
                ax.set_ylim(ymin - margin_y, ymax + margin_y)

            status_box.set_text(f"Status:\n{state['status']}")
            metrics_box.set_text(
                f"Filtro: {state['filter_label']}\n"
                f"Freq.: {state['current_rate_label']}\n"
                f"Taps FIR: {state['fir_taps']}\n"
                f"Python: {state['effective_rate']:.1f} Hz\n"
                f"Placa: {state['board_acq_rate']} Hz\n"
                f"TX: {state['board_tx_rate']} Hz\n"
                f"Drops: {state['last_queue_drops']}\n"
                f"Perdas seq.: {state['lost_by_seq']}"
            )

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(PLOT_INTERVAL)

    except KeyboardInterrupt:
        print("\nEncerrando...")

    finally:
        stop_event.set()

        if ser.is_open:
            ser.close()

        if csv_file is not None:
            csv_file.close()

        print("\nResumo final:")
        print(f"Amostras recebidas pelo Python: {state['received_samples']}")
        print(f"Perdas detectadas por seq: {state['lost_by_seq']}")
        print(f"Drops informados pela fila da placa: {state['last_queue_drops']}")
        print(f"Filtro final: {state['filter_label']}")
        print("Finalizado.")


if __name__ == "__main__":
    main()
