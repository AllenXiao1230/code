import csv, time

class CSVLogger:
    def __init__(self):
        self.f = open(f"telemetry_{int(time.time())}.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "time","lat","lon","alt","gps_alt","baro_alt","speed","heading",
            "sat","roll","pitch","accx","accy","accz","gyro_x","gyro_y","gyro_z",
            "battery","pressure_kpa","status","water"
        ])

    def log(self, d):
        self.w.writerow([
            d["time"], d["lat"], d["lon"], d["alt"], d["gps_alt"], d["baro_alt"],
            d["speed"], d["heading"], d["sat"], d["roll"], d["pitch"],
            d["accx"], d["accy"], d["accz"], d["gyro_x"], d["gyro_y"], d["gyro_z"],
            d["battery"], d["pressure"], d["status"], d.get("water", 0)
        ])

    def close(self):
        self.f.close()
