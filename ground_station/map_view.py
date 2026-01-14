# map_view.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl
import tempfile
import os


HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8" />
<title>Rocket GPS</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  html, body, #map { height: 100%; margin: 0; }
</style>
</head>
<body>
<div id="map"></div>
<script>
  var map = L.map('map').setView([25.0330, 121.5654], 12);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19
  }).addTo(map);

  var marker = L.marker([25.0330, 121.5654]).addTo(map);
  var path = [];
  var polyline = L.polyline(path, {color: 'deepskyblue', weight: 3}).addTo(map);

  function updateGPS(lat, lon) {
    marker.setLatLng([lat, lon]);
    path.push([lat, lon]);
    polyline.setLatLngs(path);
    map.setView([lat, lon], 16);
  }
</script>
</body>
</html>
"""


class MapView(QWidget):
    """
    GPS Map View using OpenStreetMap (Leaflet).
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        self.web = QWebEngineView()
        layout.addWidget(self.web)
        self._page_ready = False
        self._pending_updates = []

        # Write temp HTML
        self._html_path = os.path.join(
            tempfile.gettempdir(), "rocket_map.html"
        )
        with open(self._html_path, "w", encoding="utf-8") as f:
            f.write(HTML_TEMPLATE)

        self.web.load(QUrl.fromLocalFile(self._html_path))
        self.web.loadFinished.connect(self._on_load_finished)

    # ---------- Public API ----------

    def update_gps(self, lat: float, lon: float, alt: float):
        """
        Update marker position. If the page isn't ready yet, queue the update.
        """
        if not self._page_ready:
            self._pending_updates.append((lat, lon))
            return
        js = f"updateGPS({lat}, {lon});"
        self.web.page().runJavaScript(js)

    # ---------- Internal ----------

    def _on_load_finished(self, ok: bool):
        self._page_ready = ok
        if not ok:
            return
        # Flush queued updates
        for lat, lon in self._pending_updates:
            js = f"updateGPS({lat}, {lon});"
            self.web.page().runJavaScript(js)
        self._pending_updates.clear()
