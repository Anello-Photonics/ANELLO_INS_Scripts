#!/usr/bin/env python3
import asyncio
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from mavsdk import System

MAX_LOGS_TO_SHOW = 20

class LogDownloaderGUI:
    def __init__(self, master):
        self.master = master
        master.title("PX4 Log Downloader (MAVSDK)")

        self.drone = System()
        self.logs = []
        self.downloading = False

        # --- GUI ---
        self.status_label = tk.Label(master, text="Not connected")
        self.status_label.pack(pady=5)

        self.log_listbox = tk.Listbox(master, width=80, height=20)
        self.log_listbox.pack(padx=10, pady=10)

        self.download_button = tk.Button(master, text="Download Selected Log", command=self.start_download, state="disabled")
        self.download_button.pack(pady=5)

        self.progress = ttk.Progressbar(master, orient="horizontal", length=400, mode="determinate")
        self.progress.pack(pady=5)

        self.connect_button = tk.Button(master, text="Connect to Drone", command=self.connect)
        self.connect_button.pack(pady=5)

    # ---------------- Connect ----------------
    def connect(self):
        self.status_label.config(text="Connecting to drone...")
        threading.Thread(target=self._connect_task, daemon=True).start()

    def _connect_task(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._async_connect())

    async def _async_connect(self):
        await self.drone.connect(system_address="udpin://:14550")
        self.status_label.config(text="Waiting for drone...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                break
        self.status_label.config(text="Drone connected")
        await self.request_recent_logs()

    # ---------------- Request Recent Logs ----------------
    async def request_recent_logs(self):
        self.logs.clear()
        logs_list = await self.drone.log_files.get_entries()

        # Sort newest first and take MAX_LOGS_TO_SHOW
        logs_list.sort(key=lambda x: x.id, reverse=True)
        logs_list = logs_list[:MAX_LOGS_TO_SHOW]

        self.logs = logs_list

        self.log_listbox.delete(0, tk.END)
        for log in logs_list:
            line = f"Log {log.id} | Size: {log.size_bytes} bytes | UTC: {log.date_utc}"
            self.log_listbox.insert(tk.END, line)

        self.download_button.config(state="normal")
        self.status_label.config(text=f"Showing {len(logs_list)} recent logs")

    # ---------------- Start Download ----------------
    def start_download(self):
        if self.downloading:
            return

        sel = self.log_listbox.curselection()
        if not sel:
            messagebox.showerror("Error", "No log selected")
            return

        index = sel[0]
        log_entry = self.logs[index]

        save_file = filedialog.asksaveasfilename(defaultextension=".ulg",
                                                 initialfile=f"log_{log_entry.id}.ulg",
                                                 filetypes=[("ULog files", "*.ulg"), ("All files", "*.*")])
        if not save_file:
            return

        self.progress["value"] = 0
        self.progress["maximum"] = log_entry.size_bytes
        self.downloading = True

        threading.Thread(target=self._download_task, args=(log_entry, save_file), daemon=True).start()

    def _download_task(self, log_entry, save_path):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self._async_download(log_entry, save_path))

    async def _async_download(self, log_entry, save_path):
        self._update_status(f"Downloading log {log_entry.id}...")
        try:
            # Progress callback updates the Tkinter progress bar
            def progress_callback(downloaded_bytes):
                self.progress["value"] = downloaded_bytes

            await self.drone.log_files.download_log_file(log_entry, save_path, progress_callback)

            self._update_status(f"Download complete: {save_path}")
            messagebox.showinfo("Download Complete", f"Log saved to:\n{save_path}")

        except Exception as e:
            self._update_status("Download failed")
            messagebox.showerror("Download Failed", str(e))
        finally:
            self.downloading = False

    def _update_status(self, text):
        self.master.after(0, lambda: self.status_label.config(text=text))


# ------------------- Main -------------------
if __name__ == "__main__":
    root = tk.Tk()
    app = LogDownloaderGUI(root)
    root.mainloop()
