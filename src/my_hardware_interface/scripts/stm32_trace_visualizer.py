#!/usr/bin/env python3

import argparse
import csv
import math
import re
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.widgets import SpanSelector

try:
    from tkinterdnd2 import DND_FILES, TkinterDnD

    DND_AVAILABLE = True
    DND_BACKEND = "tkinterdnd2"
except ImportError:
    DND_FILES = None
    TkinterDnD = None
    DND_AVAILABLE = False
    DND_BACKEND = ""


DEFAULT_DIRECTORY = "/home/li/UAM_ROS/run_logs"
DEFAULT_MAX_POINTS = 6000


def _as_float(value):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return math.nan
    return out if math.isfinite(out) else math.nan


def _newest_csv(directory):
    directory = Path(directory).expanduser()
    files = sorted(directory.glob("*.csv"), key=lambda path: path.stat().st_mtime, reverse=True)
    return files[0] if files else None


def _load_csv(path):
    path = Path(path).expanduser()
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fields = list(reader.fieldnames or [])
    if not rows:
        raise RuntimeError(f"{path} has no data rows")

    numeric = {}
    for field in fields:
        values = np.array([_as_float(row.get(field, "")) for row in rows], dtype=float)
        if np.isfinite(values).any():
            numeric[field] = values
    if not numeric:
        raise RuntimeError(f"{path} has no numeric columns")
    return path, rows, fields, numeric


def _default_columns(numeric, x_column):
    candidates = [name for name in numeric if name != x_column]
    preferred = [
        "cmd_j2",
        "fb_pos_j2",
        "err_pos_j2",
        "fb_vel_j2",
        "fb_tau_j2",
        "tau_ros_j2",
        "tau_hw_j2",
        "cmd_j3",
        "fb_pos_j3",
        "err_pos_j3",
        "fb_vel_j3",
        "fb_tau_j3",
        "tau_ros_j3",
        "tau_hw_j3",
        "mode",
        "sys_state",
        "trigger_active",
    ]
    chosen = [name for name in preferred if name in candidates]
    return chosen if chosen else candidates[:8]


def _split_dnd_paths(data, root):
    if not data:
        return []
    try:
        return [Path(item) for item in root.tk.splitlist(data)]
    except tk.TclError:
        return [Path(data.strip("{}"))]


class TraceVisualizerApp:
    def __init__(self, root, initial_file=None, initial_directory=DEFAULT_DIRECTORY):
        self.root = root
        self.root.title("STM32 trace visualizer")
        self.root.geometry("1320x820")

        self.path = None
        self.rows = []
        self.fields = []
        self.numeric = {}
        self.x_column = "time_sec"
        self.x = np.array([], dtype=float)
        self.x_label = "time [s]"
        self.visible_column_names = []
        self.lines = {}
        self.event_artists = []
        self.span = None

        self.path_var = tk.StringVar(value=str(initial_file or ""))
        self.filter_var = tk.StringVar()
        self.max_points_var = tk.IntVar(value=DEFAULT_MAX_POINTS)
        self.show_events_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Open or drag a CSV trace file.")

        self._build_ui(initial_directory)
        if initial_file:
            self.load_file(initial_file)

    def _build_ui(self, initial_directory):
        top = ttk.Frame(self.root, padding=(10, 8))
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="CSV").pack(side=tk.LEFT)
        path_entry = ttk.Entry(top, textvariable=self.path_var)
        path_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 6))
        path_entry.bind("<Return>", lambda _event: self.load_file(self.path_var.get()))

        ttk.Button(top, text="Open", command=self.open_dialog).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="Newest", command=lambda: self.open_newest(initial_directory)).pack(side=tk.LEFT, padx=3)
        ttk.Button(top, text="Load", command=lambda: self.load_file(self.path_var.get())).pack(side=tk.LEFT, padx=3)

        body = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, padding=(10, 8))
        body.add(left, weight=0)

        plot_frame = ttk.Frame(body, padding=(4, 4))
        body.add(plot_frame, weight=1)

        self.drop_label = tk.Label(
            left,
            text="Drop CSV file here",
            relief=tk.GROOVE,
            height=3,
            bg="#eef2f6",
        )
        self.drop_label.pack(fill=tk.X, pady=(0, 8))
        if not self.enable_drop_target(self.drop_label):
            self.drop_label.configure(text="Open CSV file\n(drag/drop backend not available)")

        filter_row = ttk.Frame(left)
        filter_row.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(filter_row, text="Filter").pack(side=tk.LEFT)
        filter_entry = ttk.Entry(filter_row, textvariable=self.filter_var, width=18)
        filter_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(6, 4))
        filter_entry.bind("<Return>", lambda _event: self.refresh_column_list())
        ttk.Button(filter_row, text="Apply", command=self.refresh_column_list).pack(side=tk.LEFT)

        list_frame = ttk.Frame(left)
        list_frame.pack(fill=tk.BOTH, expand=True)
        self.column_list = tk.Listbox(
            list_frame,
            selectmode=tk.EXTENDED,
            exportselection=False,
            height=24,
            width=30,
        )
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.column_list.yview)
        self.column_list.configure(yscrollcommand=scrollbar.set)
        self.column_list.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.LEFT, fill=tk.Y)
        self.column_list.bind("<Double-Button-1>", lambda _event: self.plot_selected())

        quick = ttk.LabelFrame(left, text="Quick Select", padding=(8, 6))
        quick.pack(fill=tk.X, pady=(8, 0))
        for text, pattern in [
            ("joint 2", r"j2|mode|trigger"),
            ("joint 3", r"j3|mode|trigger"),
            ("position", r"cmd_|fb_pos|err_pos|mode|trigger"),
            ("velocity", r"fb_vel|mode|trigger"),
            ("torque", r"tau|tor|mode|trigger"),
            ("gains", r"kp_|kd_|mode|trigger"),
        ]:
            ttk.Button(quick, text=text, command=lambda pat=pattern: self.select_pattern(pat)).pack(
                side=tk.LEFT, padx=2, pady=2
            )

        options = ttk.LabelFrame(left, text="Plot Options", padding=(8, 6))
        options.pack(fill=tk.X, pady=(8, 0))
        ttk.Label(options, text="max points").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(options, from_=500, to=200000, increment=500, textvariable=self.max_points_var, width=9).grid(
            row=0, column=1, sticky="w", padx=(6, 0)
        )
        ttk.Checkbutton(options, text="event marks", variable=self.show_events_var).grid(
            row=1, column=0, columnspan=2, sticky="w", pady=(5, 0)
        )

        button_row = ttk.Frame(left)
        button_row.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(button_row, text="Plot Selected", command=self.plot_selected).pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(button_row, text="Clear", command=self.clear_plot).pack(side=tk.LEFT, padx=(6, 0))

        self.summary = tk.Text(left, height=8, width=34, wrap=tk.WORD)
        self.summary.pack(fill=tk.X, pady=(10, 0))

        self.figure = Figure(figsize=(10, 7), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.ax.grid(True)
        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.toolbar = NavigationToolbar2Tk(self.canvas, plot_frame)
        self.toolbar.update()

        ttk.Label(self.root, textvariable=self.status_var, anchor="w", padding=(10, 4)).pack(side=tk.BOTTOM, fill=tk.X)

    def on_drop(self, event):
        paths = _split_dnd_paths(event.data, self.root)
        if paths:
            self.load_file(paths[0])

    def on_drop_data(self, data):
        paths = _split_dnd_paths(data, self.root)
        if paths:
            self.load_file(paths[0])

    def enable_drop_target(self, widget):
        global DND_AVAILABLE, DND_BACKEND
        if DND_AVAILABLE and DND_BACKEND == "tkinterdnd2":
            widget.drop_target_register(DND_FILES)
            widget.dnd_bind("<<Drop>>", self.on_drop)
            return True

        try:
            self.root.tk.call("package", "require", "tkdnd")
            self.root.tk.call("tkdnd::drop_target", "register", widget, "DND_Files")
            command = widget.register(self.on_drop_data)
            self.root.tk.call("bind", widget, "<<Drop:DND_Files>>", command + " %D")
            DND_AVAILABLE = True
            DND_BACKEND = "tkdnd"
            return True
        except tk.TclError:
            return False

    def open_dialog(self):
        path = filedialog.askopenfilename(
            title="Open STM32 trace CSV",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if path:
            self.load_file(path)

    def open_newest(self, directory):
        path = _newest_csv(directory)
        if path is None:
            messagebox.showwarning("STM32 trace visualizer", f"No CSV files found in {directory}")
            return
        self.load_file(path)

    def load_file(self, path):
        try:
            path = Path(str(path)).expanduser()
            loaded_path, rows, fields, numeric = _load_csv(path)
        except Exception as exc:
            messagebox.showerror("STM32 trace visualizer", f"Failed to load CSV:\n{exc}")
            return

        self.path = loaded_path
        self.rows = rows
        self.fields = fields
        self.numeric = numeric
        self.x_column = "time_sec" if "time_sec" in numeric else next(iter(numeric))
        self.x = np.array(numeric[self.x_column], dtype=float)
        if self.x_column == "time_sec" and np.isfinite(self.x).any():
            self.x = self.x - np.nanmin(self.x)
            self.x_label = "time since trace start [s]"
        else:
            self.x_label = self.x_column

        self.path_var.set(str(loaded_path))
        self.refresh_column_list()
        self.select_columns(_default_columns(numeric, self.x_column))
        self.plot_selected()
        self.status_var.set(
            f"Loaded {loaded_path.name}: {len(rows)} rows, {len(numeric)} numeric columns. "
            f"Plotting is downsampled; selection stats use full data."
        )

    def candidate_columns(self):
        pattern = self.filter_var.get().strip()
        columns = [name for name in self.numeric if name != self.x_column]
        if not pattern:
            return columns
        try:
            rx = re.compile(pattern)
        except re.error:
            return columns
        return [name for name in columns if rx.search(name)]

    def refresh_column_list(self):
        selected = set(self.selected_columns())
        self.column_list.delete(0, tk.END)
        self.visible_column_names = self.candidate_columns()
        for name in self.visible_column_names:
            self.column_list.insert(tk.END, name)
            if name in selected:
                self.column_list.selection_set(tk.END)

    def selected_columns(self):
        return [self.visible_column_names[i] for i in self.column_list.curselection()]

    def select_columns(self, columns):
        wanted = set(columns)
        self.column_list.selection_clear(0, tk.END)
        for idx, name in enumerate(self.visible_column_names):
            if name in wanted:
                self.column_list.selection_set(idx)

    def select_pattern(self, pattern):
        self.filter_var.set(pattern)
        self.refresh_column_list()
        try:
            rx = re.compile(pattern)
        except re.error:
            return
        self.select_columns([name for name in self.visible_column_names if rx.search(name)])
        self.plot_selected()

    def display_indices(self):
        count = len(self.x)
        max_points = max(100, int(self.max_points_var.get() or DEFAULT_MAX_POINTS))
        stride = max(1, int(math.ceil(count / max_points)))
        return np.arange(0, count, stride, dtype=int), stride

    def plot_selected(self):
        if not self.numeric:
            return
        columns = self.selected_columns()
        if not columns:
            messagebox.showinfo("STM32 trace visualizer", "Select one or more columns to plot.")
            return

        self.clear_plot(keep_summary=True)
        idx, stride = self.display_indices()
        x_plot = self.x[idx]
        for name in columns:
            values = self.numeric.get(name)
            if values is None:
                continue
            line, = self.ax.plot(x_plot, values[idx], linewidth=1.2, label=name)
            self.lines[name] = line

        self.draw_event_marks()
        self.ax.set_xlabel(self.x_label)
        self.ax.grid(True)
        self.ax.legend(loc="upper right", fontsize="small")
        self.ax.relim()
        self.ax.autoscale_view()
        self.install_span_selector()
        self.canvas.draw_idle()
        self.status_var.set(f"Plotted {len(columns)} columns with stride {stride} from {self.path.name}.")

    def clear_plot(self, keep_summary=False):
        self.ax.clear()
        self.lines.clear()
        self.event_artists.clear()
        self.ax.grid(True)
        if not keep_summary:
            self.summary.delete("1.0", tk.END)
        self.canvas.draw_idle()

    def draw_event_marks(self):
        if not self.show_events_var.get() or "event" not in self.fields or not self.rows:
            return
        last_event = None
        marker_count = 0
        max_markers = 200
        for idx, row in enumerate(self.rows):
            event = row.get("event", "")
            if event == last_event:
                continue
            last_event = event
            if not event or idx >= len(self.x) or not math.isfinite(self.x[idx]):
                continue
            self.ax.axvline(self.x[idx], color="0.45", alpha=0.18, linewidth=1)
            marker_count += 1
            if marker_count >= max_markers:
                break

    def install_span_selector(self):
        self.span = SpanSelector(
            self.ax,
            self.on_span,
            "horizontal",
            useblit=True,
            props={"alpha": 0.18, "facecolor": "tab:blue"},
            interactive=True,
            drag_from_anywhere=True,
        )

    def on_span(self, xmin, xmax):
        if xmin > xmax:
            xmin, xmax = xmax, xmin
        mask = np.isfinite(self.x) & (self.x >= xmin) & (self.x <= xmax)
        columns = list(self.lines)
        lines = [f"selection {xmin:.4f} .. {xmax:.4f} s, samples={int(mask.sum())}"]
        for name in columns[:14]:
            values = self.numeric[name][mask]
            values = values[np.isfinite(values)]
            if values.size:
                lines.append(
                    f"{name}: min={np.min(values):.5g}, mean={np.mean(values):.5g}, max={np.max(values):.5g}"
                )
        text = "\n".join(lines)
        self.summary.delete("1.0", tk.END)
        self.summary.insert(tk.END, text)
        print(text)


def make_root():
    if DND_AVAILABLE:
        return TkinterDnD.Tk()
    return tk.Tk()


def main():
    parser = argparse.ArgumentParser(description="Standalone STM32 trace visualizer.")
    parser.add_argument("--file", help="CSV trace file to open.")
    parser.add_argument(
        "--directory",
        default=DEFAULT_DIRECTORY,
        help="Directory used by the Newest button and startup auto-load.",
    )
    parser.add_argument(
        "--no-auto-load",
        action="store_true",
        help="Start empty instead of opening --file or the newest CSV in --directory.",
    )
    args = parser.parse_args()

    initial_file = None
    if not args.no_auto_load:
        initial_file = Path(args.file).expanduser() if args.file else _newest_csv(args.directory)

    root = make_root()
    app = TraceVisualizerApp(root, initial_file=initial_file, initial_directory=args.directory)
    if not DND_AVAILABLE:
        app.status_var.set(
            "Standalone mode ready. Drag/drop needs tkinterdnd2 or tkdnd; Open/Newest works without ROS."
        )
    else:
        app.status_var.set(f"Standalone mode ready with drag/drop backend: {DND_BACKEND}.")
    root.mainloop()


if __name__ == "__main__":
    main()
