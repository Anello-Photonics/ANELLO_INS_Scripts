import tkinter as tk
from tkinter import filedialog, messagebox
import subprocess
import os

selected_files = []

def run_decoder():
    if not selected_files:
        messagebox.showwarning("No Files", "Please select one or more files first.")
        return

    for file_path in selected_files:
        if not os.path.exists(file_path):
            messagebox.showerror("File Not Found", f"File does not exist:\n{file_path}")
            continue

        try:
            if file_path.endswith(".ulg"):
                subprocess.run(["python", "ulog2csv.py", file_path], check=True)
            else:
                messagebox.showwarning("Unknown Extension", f"Unsupported file type:\n{file_path}")
                continue

        except subprocess.CalledProcessError:
            messagebox.showerror("Error", f"Failed to process:\n{file_path}")
        except FileNotFoundError:
            messagebox.showerror("Script Not Found", f"Decoder script or tool not found for:\n{file_path}")

    messagebox.showinfo("Done", "Processing completed.")

def open_files():
    global selected_files
    file_paths = filedialog.askopenfilenames()
    if file_paths:
        selected_files = file_paths
        file_list_box.config(state='normal')
        file_list_box.delete(1.0, tk.END)
        for path in selected_files:
            file_list_box.insert(tk.END, path + "\n")
        file_list_box.config(state='disabled')

# GUI setup
root = tk.Tk()
root.title("Log File Auto Decoder")

open_files_button = tk.Button(root, text="Select Files", command=open_files)
open_files_button.pack(pady=10)

file_list_box = tk.Text(root, height=10, width=70, state='disabled', wrap='none')
file_list_box.pack(pady=5)

decode_button = tk.Button(root, text="Decode Files", command=run_decoder)
decode_button.pack(pady=10)

root.mainloop()
