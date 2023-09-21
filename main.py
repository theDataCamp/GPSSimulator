import random
import tkinter as tk
from tkinter import ttk, messagebox
import time
import serial
import threading


class NMEASimulator(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("NMEA GPS Serial Data Simulator")
        self.geometry("550x350")
        self.row_num = 0

        # Time offset selector
        self.time_offset_var = tk.StringVar(value=0)
        ttk.Label(self, text="Time Offset (seconds):").grid(row=self.row_num, column=0, padx=10, pady=5, sticky=tk.W)
        self.time_offset = ttk.Spinbox(self, from_=-12 * 3600, to=12 * 3600, width=10,
                                       textvariable=self.time_offset_var)
        self.time_offset.grid(row=self.row_num, column=1, padx=10, pady=5, sticky=(tk.W + tk.E))
        self.row_num += 1

        # COM Port selector
        ttk.Label(self, text="COM Port:").grid(row=self.row_num, column=0, padx=10, pady=5, sticky=tk.W)
        self.com_port = ttk.Entry(self)
        self.com_port.grid(row=self.row_num, column=1, padx=10, pady=5, sticky=(tk.W + tk.E))
        self.com_port.insert(0, "COM17")
        self.row_num += 1

        # Baudrate selector
        ttk.Label(self, text="Baudrate:").grid(row=self.row_num, column=0, padx=10, pady=5, sticky=tk.W)
        self.baudrate = ttk.Combobox(self, values=["4800", "9600", "14400", "19200", "38400", "57600", "115200"],
                                     state="readonly")
        self.baudrate.grid(row=self.row_num, column=1, padx=10, pady=5, sticky=(tk.W + tk.E))
        self.baudrate.set("4800")
        self.row_num += 1

        # Satellite count entry
        self.satellite_count_var = tk.StringVar(value=12)
        ttk.Label(self, text="Satellites in view:").grid(row=self.row_num, column=0, padx=10, pady=5, sticky=tk.W)
        self.sat_count = ttk.Entry(self, textvariable=self.satellite_count_var)
        self.sat_count.grid(row=self.row_num, column=1, padx=10, pady=5, sticky=(tk.W + tk.E))
        self.row_num += 1

        # Interval between sending
        self.sleep_interval_var = tk.StringVar(value=1)
        ttk.Label(self, text="Sleep Interval (seconds):").grid(row=self.row_num, column=0, padx=10, pady=5, sticky=tk.W)
        self.sleep_interval = ttk.Entry(self, textvariable=self.sleep_interval_var)
        self.sleep_interval.grid(row=self.row_num, column=1, padx=10, pady=5, sticky=(tk.W + tk.E))
        self.row_num += 1

        # Start simulation button
        self.start_btn = ttk.Button(self, text="Start Simulation", command=self.start_simulation)
        self.start_btn.grid(row=self.row_num, column=0, columnspan=2, pady=20)
        self.row_num += 1

        self.stop_btn = ttk.Button(self, text="Stop Simulation", command=self.stop_simulation)
        self.stop_btn.grid(row=self.row_num, column=0, columnspan=2, pady=20)
        self.row_num += 1

        self.simulation_thread = None  # Hold reference to the simulation thread
        self.simulation_running = threading.Event()  # Event to control the simulation loop

    def calculate_checksum(self, sentence):
        """Calculate the NMEA checksum for a sentence."""
        check = 0
        for char in sentence:
            check ^= ord(char)
        return format(check, '02X')

    def start_simulation(self):
        if not self.simulation_running.is_set():
            self.simulation_running.set()  # Mark simulation as running
            print("Starting simulation...")
            # Start the simulation in a new thread
            self.simulation_thread = threading.Thread(target=self.simulate_nmea_data)
            self.simulation_thread.start()

    def stop_simulation(self):
        if self.simulation_running.is_set():
            self.simulation_running.clear()  # Mark simulation as stopped
            print("Stopping simulation")
            if self.simulation_thread:
                self.simulation_thread.join()  # Wait for the simulation thread to finish
            self.simulation_thread = None
            print("Simulation stopped.")

    def random_lat_lon(self):
        lat = random.uniform(0, 90)
        lon = random.uniform(-180, 180)
        lat = "{:09.4f}".format(lat)
        lon = "{:010.4f}".format(lon)
        lat_dir = random.choice(['N', 'S'])
        lon_dir = random.choice(['E', 'W'])
        return lat, lon, lat_dir, lon_dir

    def generate_gsv_sentences(self, sats_in_view):
        sentences = []  # List to hold all the GSV sentences
        total_sentences = (sats_in_view + 3) // 4  # Determine how many GSV sentences will be needed

        for sentence_num in range(1, total_sentences + 1):
            gsv_data = "$GPGSV,{},{},{},,".format(total_sentences, sentence_num, sats_in_view)

            # Determine how many satellites to process in this sentence (maximum of 4)
            satellites_in_this_sentence = min(4, sats_in_view - (sentence_num - 1) * 4)
            for i in range(0, satellites_in_this_sentence):  # Add satellite data for each satellite
                sat_data = "{:02},{:03},{:03},{:02}".format((sentence_num - 1) * 4 + i + 1,
                                                            random.randint(0, 359),
                                                            random.randint(0, 90),
                                                            random.randint(0, 99))
                gsv_data += sat_data + ','

            gsv_data = gsv_data.rstrip(',')  # Remove the trailing comma
            gsv_data += "*" + self.calculate_checksum(gsv_data[1:])
            sentences.append(gsv_data)

        return sentences

    def generate_vtg_data(self):
        # Creating a random example. You can adjust as necessary.
        base_vtg = "$GPVTG,{track},T,,M,{speed},N,,K,A"
        speed = "{:.1f}".format(random.uniform(0, 100))  # Speed over ground in knots
        track = "{:.1f}".format(random.uniform(0, 360))  # Track angle in degrees
        formatted_vtg = base_vtg.format(track=track, speed=speed)
        nmea_vtg = formatted_vtg + "*" + self.calculate_checksum(formatted_vtg[1:])
        return nmea_vtg

    def generate_gsa_data(self):
        # Creating a random example. You can adjust as necessary.
        base_gsa = "$GPGSA,A,3,{satellites},1.0,1.0,1.0"
        # Just picking 12 random satellite numbers as an example
        satellites = ",".join(["{:02}".format(random.randint(1, 30)) for _ in range(12)])
        formatted_gsa = base_gsa.format(satellites=satellites)
        nmea_gsa = formatted_gsa + "*" + self.calculate_checksum(formatted_gsa[1:])
        nmea_gsa = "$GPGSA,A,3,01,05,,,,18,,22,30,31,48,51,2.5,1.1,1.9*39"
        return nmea_gsa

    def generate_random_nmea_data(self, current_time, current_date):
        base_gga = "$GPGGA,{time},{lat},{lat_dir},{lon},{lon_dir},1,08,1.01,{alt}M,{geo_sep}M,,"
        base_rmc = "$GPRMC,{time},A,{lat},{lat_dir},{lon},{lon_dir},{speed},{track},{date},,,"

        lat, lon, lat_dir, lon_dir = self.random_lat_lon()
        alt = "{:.1f}".format(random.uniform(0, 1000))  # Altitude
        geo_sep = "{:.1f}".format(random.uniform(0, 100))  # Geoidal separation
        sat = "{:02}".format(random.randint(4, int(self.satellite_count_var.get())))  # Satellites used
        speed = "{:.1f}".format(random.uniform(0, 100))  # Speed over ground in knots
        track = "{:.1f}".format(random.uniform(0, 360))  # Track angle in degrees

        formatted_gga = base_gga.format(time=current_time, lat=lat, lat_dir=lat_dir, lon=lon, lon_dir=lon_dir, sat=sat,
                                        alt=alt, geo_sep=geo_sep)
        nmea_gga = formatted_gga + "*" + self.calculate_checksum(formatted_gga[1:])

        formatted_rmc = base_rmc.format(time=current_time, lat=lat, lat_dir=lat_dir, lon=lon, lon_dir=lon_dir,
                                        speed=speed,
                                        track=track, date=current_date)
        nmea_rmc = formatted_rmc + "*" + self.calculate_checksum(formatted_rmc[1:])

        return nmea_gga, nmea_rmc

    def simulate_nmea_data(self):
        offset = self.time_offset_var.get()
        baudrate = int(self.baudrate.get())
        com_port = self.com_port.get()
        sat_count = int(self.satellite_count_var.get())

        print(f"Simulating NMEA data on {com_port} at {baudrate} baudrate with offset {offset} seconds...")

        try:
            with serial.Serial(com_port, baudrate=baudrate) as ser:
                print(f"Opened {com_port} successfully.")
                while self.simulation_running.is_set():
                    current_time = time.strftime('%H%M%S', time.gmtime(time.time() + int(offset)))
                    current_date = time.strftime('%d%m%y', time.gmtime(time.time() + int(offset)))

                    nmea_gga, nmea_rmc = self.generate_random_nmea_data(current_time, current_date)
                    nmea_gsv = self.generate_gsv_sentences(sat_count)
                    nmea_vtg = self.generate_vtg_data()
                    nmea_gsa = self.generate_gsa_data()

                    # Send the sentences
                    ser.write(nmea_gga.encode('utf-8'))
                    ser.write(b'\r\n')
                    print(f"Sent gga {nmea_gga}")
                    for gsv_line in nmea_gsv:
                        ser.write(gsv_line.encode('utf-8'))  # Add GSV data to the serial output
                        ser.write(b'\r\n')
                    print(f"Sent gsv {nmea_gsv}")
                    ser.write(nmea_rmc.encode('utf-8'))
                    ser.write(b'\r\n')
                    ser.write(nmea_vtg.encode('utf-8'))
                    ser.write(b'\r\n')
                    print(f"Sent vtg {nmea_vtg}")
                    ser.write(nmea_gsa.encode('utf-8'))
                    ser.write(b'\r\n')
                    print(f"Sent gsa {nmea_gsa}")
                    print(f"Sent rmc {nmea_rmc}")


                    sleep_interval = float(self.sleep_interval_var.get())
                    print(f"Sent NMEA data for time: {current_time} and date: {current_date} interval {sleep_interval}")
                    time.sleep(sleep_interval)  # Send every second
        except serial.SerialException as e:
            # Ensure the GUI-related actions are done in the main thread
            print(f"Error: Failed to open {com_port}: {e}")
            self.after(0, lambda: messagebox.showerror("Error", f"Failed to open {com_port}: {e}"))

    def on_close(self):
        self.stop_simulation()  # Stop the simulation when the window is closed
        print("Application closed.")
        self.destroy()


if __name__ == "__main__":
    app = NMEASimulator()
    app.protocol("WM_DELETE_WINDOW", app.on_close)  # Bind the window close event
    app.mainloop()
