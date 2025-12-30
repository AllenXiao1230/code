import csv, time

class CSVLogger:
    def __init__(self):
        self.f = open(f"telemetry_{int(time.time())}.csv", "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "time","lat","lon","alt","speed","heading",
            "sat","accz","pitch","gyro",
            "battery","temp","hum","pressure","status"
        ])

    def log(self, d):
        self.w.writerow([
            d["time"], d["lat"], d["lon"], d["alt"], d["speed"],
            d["heading"], d["sat"], d["accz"], d["pitch"], d["gyro"],
            d["battery"], d["temp"], d["hum"], d["pressure"], d["status"]
        ])

    def close(self):
        self.f.close()
