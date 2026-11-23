import socket
import threading
import queue
import time
import math
import re
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

IP_DEFECTO = "192.168.137.67"   # IP del ESP32
PUERTO_DEFECTO = 4210

L1 = 13.0
L2 = 15.0
M1_ARRIBA = 16.0 


def cinematica_directa(m1, m2, m3):
    t1 = math.radians(m3)
    t2 = math.radians(m2)
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    z = m1 - M1_ARRIBA
    return x, y, z

PUNTOS = {
    "HOME":       (-15, 10, 0),
    "PLL":        (0, 26, 0),
    "AGARRE PLL": (0, 26, -8),
    "PS":         (0, -26, 0),
    "SOLTAR PS":  (0, -26, -8),
}

COLOR_ACTIVO = "#2e7d32" 
COLOR_INACTIVO = "#9e9e9e"   

MAX_PUNTOS_RASTRO = 3000
DIST_MIN_RASTRO = 0.2   

class App:
    def __init__(self, root):
        self.root = root
        root.title("Control Robotino (UDP)")
        root.minsize(1200, 620)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", 0))
        self.sock.settimeout(0.5)
        self.cola = queue.Queue()
        self.activo = True
        threading.Thread(target=self.escuchar, daemon=True).start()

        self.rastro = []  
        self.actual = None
        self.redibujar = False
        self.t_ultimo_dato = 0.0

        self.izq = ttk.Frame(root)
        self.izq.pack(side="left", fill="both", expand=True)
        self.der = ttk.Frame(root)
        self.der.pack(side="right", fill="both", expand=True)

        self.construir_controles()
        self.construir_grafica()

        root.bind("<Escape>", lambda e: self.enviar("STOP"))   # Esc = STOP
        root.protocol("WM_DELETE_WINDOW", self.cerrar)
        root.after(50, self.revisar_cola)
        root.after(150, self.dibujar)
        root.after(500, self.revisar_conexion)

    # ---------------- Controles ----------------
    def construir_controles(self):
        pad = {"padx": 6, "pady": 4}
        p = self.izq

        # Conexion
        f_con = ttk.LabelFrame(p, text="Conexion")
        f_con.pack(fill="x", **pad)
        ttk.Label(f_con, text="IP del robot:").pack(side="left", padx=4)
        self.ip = tk.StringVar(value=IP_DEFECTO)
        ttk.Entry(f_con, textvariable=self.ip, width=16).pack(side="left")
        ttk.Label(f_con, text="Puerto:").pack(side="left", padx=(12, 4))
        self.puerto = tk.StringVar(value=str(PUERTO_DEFECTO))
        ttk.Entry(f_con, textvariable=self.puerto, width=6).pack(side="left")
        ttk.Button(f_con, text="Probar (ESTADO)",
                   command=lambda: self.enviar("ESTADO")).pack(side="left", padx=12)
        self.lbl_con = tk.Label(f_con, text="Sin datos", fg="white", bg="#d32f2f", width=12)
        self.lbl_con.pack(side="left", padx=4)

        # Sensores: finales de carrera y hall
        f_sen = ttk.LabelFrame(p, text="Sensores (verde = activo)")
        f_sen.pack(fill="x", **pad)
        self.sensores = {}
        for clave, texto in [("FC1", "FC1 (M1 tope)"), ("FC2", "FC2 (M1 cero)"),
                             ("FC3", "FC3 (M2 home)"), ("HALL", "HALL")]:
            lbl = tk.Label(f_sen, text=texto, width=15, fg="white", bg=COLOR_INACTIVO,
                           font=("Segoe UI", 9, "bold"), relief="ridge", bd=2)
            lbl.pack(side="left", padx=4, pady=4)
            self.sensores[clave] = lbl

        cuerpo = ttk.Frame(p)
        cuerpo.pack(fill="x", **pad)

        # Ciclo
        f_ciclo = ttk.LabelFrame(cuerpo, text="Ciclo")
        f_ciclo.pack(side="left", fill="y", padx=(0, 6))
        for cmd in ["PLL", "AGARRE PLL", "PS", "SOLTAR PS"]:
            ttk.Button(f_ciclo, text=cmd, width=14,
                       command=lambda c=cmd: self.enviar(c)).pack(padx=6, pady=3)

        # General
        f_gen = ttk.LabelFrame(cuerpo, text="General")
        f_gen.pack(side="left", fill="y", padx=(0, 6))
        for cmd in ["HOME", "HOME2", "CAL", "ESTADO", "IP"]:
            ttk.Button(f_gen, text=cmd, width=10,
                       command=lambda c=cmd: self.enviar(c)).pack(padx=6, pady=3)

        # Manual
        f_man = ttk.LabelFrame(cuerpo, text="Manual")
        f_man.pack(side="left", fill="both", expand=True)
        filas = [
            ("M1", "cm  (-17 a 17)", "16"),
            ("M2", "grados", "0"),
            ("M3", "grados", "0"),
            ("S",  "gripper (0 a 180)", "90"),
            ("V",  "velocidad % (1 a 100)", "70"),
        ]
        for fila, (clave, unidad, defecto) in enumerate(filas):
            ttk.Label(f_man, text=clave, width=3).grid(row=fila, column=0, padx=4, pady=2)
            var = tk.StringVar(value=defecto)
            entrada = ttk.Entry(f_man, textvariable=var, width=8)
            entrada.grid(row=fila, column=1, pady=2)
            accion = lambda e=None, k=clave, v=var: self.enviar(f"{k} {v.get()}")
            entrada.bind("<Return>", accion)
            ttk.Button(f_man, text="Enviar", command=accion).grid(row=fila, column=2, padx=4)
            ttk.Label(f_man, text=unidad, foreground="gray").grid(row=fila, column=3, sticky="w")

        # STOP grande
        tk.Button(p, text="STOP  (Esc)", bg="#d32f2f", fg="white",
                  activebackground="#b71c1c", activeforeground="white",
                  font=("Segoe UI", 14, "bold"), height=1,
                  command=lambda: self.enviar("STOP")).pack(fill="x", **pad)

        # Comando libre
        f_cmd = ttk.Frame(p)
        f_cmd.pack(fill="x", **pad)
        ttk.Label(f_cmd, text="Comando:").pack(side="left")
        self.cmd_libre = tk.StringVar()
        e = ttk.Entry(f_cmd, textvariable=self.cmd_libre)
        e.pack(side="left", fill="x", expand=True, padx=4)
        e.bind("<Return>", lambda ev: self.enviar_libre())
        ttk.Button(f_cmd, text="Enviar", command=self.enviar_libre).pack(side="left")

        # Ultima linea de reporte del robot
        self.reporte = tk.StringVar(value="(sin datos del robot todavia)")
        ttk.Label(p, textvariable=self.reporte, font=("Consolas", 9),
                  relief="sunken", anchor="w").pack(fill="x", **pad)

        # Registro
        f_log = ttk.Frame(p)
        f_log.pack(fill="both", expand=True, **pad)
        self.log_txt = ScrolledText(f_log, height=10, font=("Consolas", 9), state="disabled")
        self.log_txt.pack(fill="both", expand=True)
        self.log_txt.tag_config("enviado", foreground="#1565c0")
        self.log_txt.tag_config("error", foreground="#d32f2f")
        ttk.Button(p, text="Limpiar registro",
                   command=self.limpiar_log).pack(anchor="e", **pad)

    def construir_grafica(self):
        self.fig = Figure(figsize=(5.5, 5.5), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.der)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        f_bot = ttk.Frame(self.der)
        f_bot.pack(fill="x", padx=6, pady=4)
        self.lbl_xyz = ttk.Label(f_bot, text="x = -  y = -  z = -", font=("Consolas", 10))
        self.lbl_xyz.pack(side="left")
        ttk.Button(f_bot, text="Borrar trayectoria",
                   command=self.borrar_rastro).pack(side="right")

        self.preparar_ejes()
        self.canvas.draw()

    def preparar_ejes(self):
        ax = self.ax
        ax.clear()
        ax.set_title("Trayectoria del efector final")
        ax.set_xlabel("X (cm)")
        ax.set_ylabel("Y (cm)")
        ax.set_zlabel("Z (cm)")
        ax.set_xlim(-35, 35)
        ax.set_ylim(-35, 35)
        ax.set_zlim(-12, 4)

        # Puntos de referencia de la grafica
        for nombre, (x, y, z) in PUNTOS.items():
            ax.scatter([x], [y], [z], color="#1f3b57", s=25)
            ax.text(x, y, z + 0.8, nombre, fontsize=7)

        self.linea, = ax.plot([], [], [], color="#e65100", linewidth=2)
        self.punto, = ax.plot([], [], [], "o", color="#e65100", markersize=7)

    def borrar_rastro(self):
        self.rastro = []
        self.redibujar = True

    def agregar_posicion(self, m1, m2, m3):
        x, y, z = cinematica_directa(m1, m2, m3)
        self.actual = (x, y, z)
        if not self.rastro or math.dist(self.rastro[-1], self.actual) >= DIST_MIN_RASTRO:
            self.rastro.append(self.actual)
            if len(self.rastro) > MAX_PUNTOS_RASTRO:
                self.rastro.pop(0)
        self.lbl_xyz.config(text=f"x = {x:6.1f}  y = {y:6.1f}  z = {z:6.1f} cm")
        self.redibujar = True

    def dibujar(self):
        if self.redibujar:
            self.redibujar = False
            if self.rastro:
                xs, ys, zs = zip(*self.rastro)
                self.linea.set_data_3d(xs, ys, zs)
            else:
                self.linea.set_data_3d([], [], [])
            if self.actual:
                x, y, z = self.actual
                self.punto.set_data_3d([x], [y], [z])
            self.canvas.draw_idle()
        self.root.after(150, self.dibujar)

    def enviar(self, cmd):
        cmd = cmd.strip()
        if not cmd:
            return
        try:
            destino = (self.ip.get().strip(), int(self.puerto.get()))
            self.sock.sendto(cmd.encode(), destino)
            self.log(f">> {cmd}\n", "enviado")
        except Exception as e:
            self.log(f"Error al enviar: {e}\n", "error")

    def enviar_libre(self):
        self.enviar(self.cmd_libre.get())
        self.cmd_libre.set("")

    def escuchar(self):
        while self.activo:
            try:
                datos, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            self.cola.put(datos.decode(errors="ignore"))

    def revisar_cola(self):
        while not self.cola.empty():
            for linea in self.cola.get().splitlines():
                self.t_ultimo_dato = time.time()
                if linea.startswith("POS:"):
                    try:
                        m1, m2, m3 = (float(v) for v in linea[4:].split(","))
                        self.agregar_posicion(m1, m2, m3)
                    except ValueError:
                        pass
                elif linea.startswith("HALL:"):
                    self.reporte.set(linea)     
                    self.actualizar_sensores(linea)
                elif linea.strip():
                    self.log(linea + "\n")
        self.root.after(50, self.revisar_cola)

    def actualizar_sensores(self, linea):
        for clave, valor in re.findall(r"(FC\d|HALL):([YN])", linea):
            if clave in self.sensores:
                self.sensores[clave].config(bg=COLOR_ACTIVO if valor == "Y" else COLOR_INACTIVO)

    def revisar_conexion(self):
        if time.time() - self.t_ultimo_dato < 1.0:
            self.lbl_con.config(text="Conectado", bg="#2e7d32")
        else:
            self.lbl_con.config(text="Sin datos", bg="#d32f2f")
        self.root.after(500, self.revisar_conexion)

    def log(self, texto, tag=None):
        self.log_txt.configure(state="normal")
        self.log_txt.insert("end", texto, tag)
        self.log_txt.see("end")
        self.log_txt.configure(state="disabled")

    def limpiar_log(self):
        self.log_txt.configure(state="normal")
        self.log_txt.delete("1.0", "end")
        self.log_txt.configure(state="disabled")

    def cerrar(self):
        self.activo = False
        self.sock.close()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    App(root)
    root.mainloop()