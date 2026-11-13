#!/usr/bin/env python3
"""Opens a browser map for picking a site's scan area or lap route.

The map page loads Leaflet and map tiles in the browser, while its localhost endpoint plans
the scan rows with the same SPathPlanner, altitude and overlap the mission uses.

    uv run python scripts/Scan_Picker.py
    uv run python scripts/Scan_Picker.py --site SIM --platform sim
"""

import argparse
import json
import math
import os
import re
import secrets
import tempfile
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from bv_autonomy.components.path_planner import SPathPlanner
from bv_autonomy.components.platform import CompDronePlatform, SimPlatform, X650Platform
from bv_autonomy.missions.suas2026 import sites
from bv_autonomy.missions.suas2026.suas2026 import LAP_COUNT, SCAN_ALT, SCAN_OVERLAP
from bv_autonomy.types import WaypointSequence

PLATFORMS = {"comp_drone": CompDronePlatform, "x650": X650Platform, "sim": SimPlatform}
SITES_PATH = Path(sites.__file__)
SITE_NAME = re.compile(r"[A-Z][A-Z0-9_]*")
EARTH_R = 6378137.0


def _validated_boundary(boundary):
    if not isinstance(boundary, list) or len(boundary) != 4:
        raise ValueError("region must contain exactly four corners")
    result = []
    for point in boundary:
        if not isinstance(point, list) or len(point) != 2:
            raise ValueError("each corner must contain latitude and longitude")
        lat, lon = float(point[0]), float(point[1])
        if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
            raise ValueError("region contains an invalid latitude or longitude")
        result.append([lat, lon])
    if len({tuple(point) for point in result}) != 4:
        raise ValueError("region corners must be distinct")
    return result


def update_site_source(text, site, kind, boundary, *, lap_direction="counterclockwise"):
    """Returns sites.py with one site's scan area or lap route replaced by the picked corners.

    The browser sends the corners as northwest, southwest, southeast, northeast, which is
    counterclockwise; a clockwise lap keeps northwest first and reverses the rest. The mission
    closes the lap itself, so the first point isn't repeated at the end.
    """
    corners = _validated_boundary(boundary)
    if kind == "scan":
        field, points = "scan_area", corners
    elif kind == "lap":
        if lap_direction not in ("clockwise", "counterclockwise"):
            raise ValueError("invalid lap direction")
        field = "laps"
        points = corners if lap_direction == "counterclockwise" else [corners[0], corners[3], corners[2], corners[1]]
    else:
        raise ValueError("region kind must be 'scan' or 'lap'")

    block = _site_block(text, site)
    if block is None:
        raise ValueError(f"no site {site} in {SITES_PATH.name}")
    opening = re.compile(rf"(?m)^(\s*){field}=WaypointSequence\.at_alt\([^\n]*\[\n").search(text, block[0])
    if opening is None or opening.start() > block[1]:
        raise ValueError(f"{site} has no {field}=WaypointSequence.at_alt(..., [ ... ]) to replace")
    closing = text.index("])", opening.end())
    closing_line = text.rfind("\n", 0, closing) + 1
    indent = opening.group(1) + "    "
    body = "".join(f"{indent}({lat:.8f}, {lon:.8f}),\n" for lat, lon in points)
    return text[:opening.end()] + body + text[closing_line:]


def _site_block(text, site):
    """Returns where a site's `NAME = Site(...)` block starts and ends in sites.py, or None."""
    start = re.search(rf"(?m)^{re.escape(site)} = Site\(\n", text)
    if start is None:
        return None
    end = re.compile(r"(?m)^\)\n?").search(text, start.end())
    return (start.start(), end.end()) if end else None


def add_site_source(text, site, source):
    """Returns sites.py with a new site appended, copied from the source site."""
    if not SITE_NAME.fullmatch(site):
        raise ValueError(f"{site!r} isn't a site name; use capitals, digits and underscores, like FIELD_2")
    if hasattr(sites, site) or re.search(rf"(?m)^{re.escape(site)}\b", text):
        raise ValueError(f"{site} is already defined in {SITES_PATH.name} and isn't a Site")
    block = _site_block(text, source)
    if block is None:
        raise ValueError(f"no site {source} in {SITES_PATH.name} to copy")
    copy = f"{site}{text[block[0] + len(source):block[1]]}".rstrip("\n") + "\n"
    return text.rstrip("\n") + "\n\n" + copy


def save_region(site, kind, boundary, source, **options):
    """Atomically saves a picked region into a site in sites.py, adding the site copied from the source site
    if it doesn't exist. Returns whether the site was added.
    """
    original = SITES_PATH.read_text(encoding="utf-8")
    original_mode = SITES_PATH.stat().st_mode
    created = _site_block(original, site) is None
    text = add_site_source(original, site, source) if created else original
    updated = update_site_source(text, site, kind, boundary, **options)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=SITES_PATH.parent,
        prefix=f".{SITES_PATH.name}.", suffix=".tmp", delete=False,
    ) as stream:
        stream.write(updated)
        temporary_path = Path(stream.name)
    try:
        os.chmod(temporary_path, original_mode)
        os.replace(temporary_path, SITES_PATH)
    finally:
        if temporary_path.exists():
            temporary_path.unlink()
    return created


def _distance_m(a, b):
    dx = EARTH_R * math.radians(a.lon - b.lon) * math.cos(math.radians(b.lat))
    dy = EARTH_R * math.radians(a.lat - b.lat)
    return math.hypot(dx, dy)


PAGE = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Scan Region Picker</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <style>
    * { box-sizing: border-box; }
    html, body { height: 100%; margin: 0; font: 15px/1.4 system-ui, sans-serif; }
    body { display: grid; grid-template-columns: minmax(0, 1fr) 390px; color: #172033; }
    #map { height: 100%; cursor: crosshair; }
    aside { padding: 22px; overflow: auto; background: #f7f8fb; box-shadow: -2px 0 12px #0002; z-index: 500; }
    h1 { margin: 0 0 8px; font-size: 23px; }
    p { margin: 8px 0 18px; }
    #status { padding: 11px 12px; border-radius: 8px; background: #e8eefc; }
    textarea { width: 100%; height: 170px; margin: 14px 0 10px; padding: 12px; resize: vertical; border: 1px solid #b8c0d0; border-radius: 7px; font: 14px/1.55 ui-monospace, monospace; }
    button { margin: 0 7px 8px 0; padding: 9px 13px; border: 0; border-radius: 6px; background: #295bd6; color: white; font-weight: 650; cursor: pointer; }
    button.secondary { background: #596174; }
    button.choice { background: #dfe4ef; color: #273044; }
    button.choice.active { background: #295bd6; color: white; }
    button:disabled { opacity: .45; cursor: default; }
    .location-search { display: flex; gap: 7px; margin: 14px 0 8px; }
    .location-search input { min-width: 0; flex: 1; padding: 9px 10px; border: 1px solid #b8c0d0; border-radius: 6px; font: inherit; }
    .location-search button { margin: 0; white-space: nowrap; }
    .search-note { margin: -3px 0 9px; color: #596174; font-size: 11px; }
    .tabs { display: grid; grid-template-columns: 1fr 1fr; gap: 6px; margin-bottom: 16px; }
    .tabs button { margin: 0; background: #dfe4ef; color: #273044; }
    .tabs button.active { background: #172033; color: white; }
    .hidden { display: none; }
    .controls { margin: 15px 0 8px; padding: 12px; border: 1px solid #d2d7e2; border-radius: 8px; background: white; }
    .controls label { display: block; margin-bottom: 7px; font-size: 13px; font-weight: 700; }
    #plan-summary { min-height: 24px; margin: 8px 0; font-weight: 700; color: #24533c; }
    #plan-summary.warn { color: #9c2a1c; }
    .direction-arrow { color: #123caa; font-size: 20px; font-weight: 900; line-height: 20px; text-shadow: 0 0 3px white, 0 0 3px white; }
    .waypoint-number { width: 25px; height: 25px; border-radius: 50%; background: #7b2fc4; color: white; border: 2px solid white; text-align: center; font: 700 13px/21px system-ui, sans-serif; box-shadow: 0 1px 4px #0008; }
    .hint { margin-top: 14px; color: #596174; font-size: 13px; }
    .leaflet-control-attribution { font-size: 10px; }
    @media (max-width: 760px) {
      body { grid-template: minmax(55vh, 1fr) auto / 1fr; }
      aside { box-shadow: 0 -2px 12px #0002; }
    }
  </style>
</head>
<body>
  <main id="map" aria-label="Map for selecting scan region"></main>
  <aside>
    <div class="tabs">
      <button id="scan-tab" class="active">Scan region</button>
      <button id="lap-tab">Lap route</button>
    </div>
    <h1 id="mode-title">Scan region</h1>
    <p id="instructions">Click one corner of the 1–10 acre search area, then click its opposite corner.</p>
    <div id="status">Finding your location…</div>
    <div id="lap-controls" class="controls hidden">
      <label>Lap direction</label>
      <button id="counterclockwise" class="choice active">Counterclockwise</button>
      <button id="clockwise" class="choice">Clockwise</button>
    </div>
    <div id="plan-summary">Select a region to preview the rows.</div>
    <textarea id="yaml" readonly aria-label="Generated site code"># Select two corners on the map</textarea>
    <div class="location-search">
      <input id="site-name" aria-label="Site to save to" spellcheck="false" autocomplete="off">
      <button id="use-region" disabled>Save to site</button>
    </div>
    <div class="search-note">An existing site is overwritten; a new name adds a site
      to sites.py, copied from the one shown.</div>
    <button id="copy" disabled>Copy</button>
    <button id="reset" class="secondary">Reset</button>
    <form id="location-search" class="location-search">
      <input id="location-query" type="search" aria-label="Address or coordinates"
        placeholder="Address or lat, lon" autocomplete="off">
      <button id="search-location" type="submit" class="secondary">Find</button>
    </form>
    <div class="search-note">Submit-only address search by
      <a href="https://www.openstreetmap.org/copyright" target="_blank">OpenStreetMap</a> ·
      <a href="https://operations.osmfoundation.org/policies/nominatim/"
        target="_blank">usage policy</a>
    </div>
    <button id="locate" class="secondary">My location</button>
    <button id="configured" class="secondary">Show configured region</button>
    <button id="quit" class="secondary">Stop server</button>
    <div class="hint">Satellite imagery is shown by default; use the layer control
      to switch to streets. The generated corners are ordered around the perimeter as
      northwest, southwest, southeast, northeast. Paste the block into the site in
      sites.py. Blue arrows are scan rows; orange arrows are turns between rows. Lap
      markers are numbered in flight order. <b>Save to site</b> writes the displayed
      route into the named site in sites.py. Always verify the
      boundary against surveyed or official coordinates before flight.</div>
  </aside>

  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script>
    const streets = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 20,
      attribution: '&copy; OpenStreetMap contributors'
    });
    const satellite = L.tileLayer(
      'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      {
        maxNativeZoom: 19,
        maxZoom: 21,
        attribution: 'Tiles &copy; Esri &mdash; Sources: Esri, Vantor, Earthstar Geographics, and the GIS User Community'
      }
    );
    const map = L.map('map', {layers: [satellite]}).setView([39.8283, -98.5795], 4);
    L.control.layers({Satellite: satellite, Streets: streets}, null, {
      collapsed: false
    }).addTo(map);
    L.control.scale({imperial: true, metric: true}).addTo(map);

    const statusBox = document.getElementById('status');
    const yamlBox = document.getElementById('yaml');
    const copyButton = document.getElementById('copy');
    const useRegionButton = document.getElementById('use-region');
    const planSummary = document.getElementById('plan-summary');
    const configuredButton = document.getElementById('configured');
    const locateButton = document.getElementById('locate');
    const locationQuery = document.getElementById('location-query');
    const searchLocationButton = document.getElementById('search-location');
    const configuredRegion = __CONFIGURED_REGION__;
    const writeToken = __WRITE_TOKEN__;
    let firstCorner = null;
    let rectangle = null;
    let cornerMarkers = [];
    let configuredLayer = null;
    let planLayer = null;
    let lapDirection = 'counterclockwise';
    let mode = 'scan';
    let planRequest = 0;
    let activeBounds = null;
    let activeScanBoundary = null;
    let activeLapRoute = null;
    let locationLayer = null;
    let locationRequest = 0;
    let searchLayer = null;

    function setStatus(message) { statusBox.textContent = message; }

    function codeFor(bounds) {
      if (mode === 'lap') {
        return `laps=WaypointSequence.at_alt(LAP_ALT, [\\n` +
          lapRouteFor(bounds).slice(0, -1).map(
            point => `    (${point.lat.toFixed(8)}, ${point.lng.toFixed(8)}),`
          ).join('\\n') + `\\n]),`;
      }
      return `scan_area=WaypointSequence.at_alt(0.0, [\\n` +
        boundaryFor(bounds).map(
          point => `    (${point[0].toFixed(8)}, ${point[1].toFixed(8)}),`
        ).join('\\n') + `\\n]),`;
    }

    function regionSize(bounds) {
      const widthM = map.distance(bounds.getNorthWest(), bounds.getNorthEast());
      const heightM = map.distance(bounds.getNorthWest(), bounds.getSouthWest());
      const acres = widthM * heightM / 4046.8564224;
      const shownAcres = acres < 10 ? acres.toFixed(2) : acres.toFixed(1);
      return `${shownAcres} acres (${Math.round(widthM)} × ${Math.round(heightM)} m)`;
    }

    function clearSelection() {
      firstCorner = null;
      if (rectangle) map.removeLayer(rectangle);
      rectangle = null;
      activeBounds = null;
      activeScanBoundary = null;
      activeLapRoute = null;
      if (planLayer) map.removeLayer(planLayer);
      planLayer = null;
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [];
      yamlBox.value = '# Select two corners on the map';
      copyButton.disabled = true;
      useRegionButton.disabled = true;
      planSummary.textContent = mode === 'scan' ?
        'Select a region to preview the rows.' :
        'Select a region to preview the lap route.';
      setStatus('Click the first corner.');
    }

    function boundaryFor(bounds) {
      if (mode === 'scan' && activeScanBoundary) {
        return activeScanBoundary.map(point => [...point]);
      }
      return [bounds.getNorthWest(), bounds.getSouthWest(),
        bounds.getSouthEast(), bounds.getNorthEast()].map(
          point => [point.lat, point.lng]);
    }

    function bearingDegrees(from, to) {
      const lat1 = from.lat * Math.PI / 180;
      const lat2 = to.lat * Math.PI / 180;
      const deltaLon = (to.lng - from.lng) * Math.PI / 180;
      const y = Math.sin(deltaLon) * Math.cos(lat2);
      const x = Math.cos(lat1) * Math.sin(lat2) -
        Math.sin(lat1) * Math.cos(lat2) * Math.cos(deltaLon);
      return Math.atan2(y, x) * 180 / Math.PI;
    }

    function arrowFor(from, to, color) {
      const midpoint = L.latLng(
        (from.lat + to.lat) / 2, (from.lng + to.lng) / 2);
      const angle = bearingDegrees(from, to);
      return L.marker(midpoint, {
        interactive: false,
        icon: L.divIcon({
          className: '', iconSize: [22, 22], iconAnchor: [11, 11],
          html: `<div class="direction-arrow" style="color:${color};transform:rotate(${angle}deg)">↑</div>`
        })
      });
    }

    function altitudeSummary(plan) {
      const height = `${plan.altitude_m.toFixed(1)} m above home`;
      if (plan.terrain_status === 'ok') {
        const low = Math.min(...plan.ground_msl);
        const high = Math.max(...plan.ground_msl);
        return `${height} · ground ${low.toFixed(0)}–${high.toFixed(0)} m MSL`;
      }
      if (plan.terrain_status === 'uncovered') {
        return `${height} · ⚠ region reaches past ${configuredRegion.terrain_map}`;
      }
      return `${height} (no terrain map)`;
    }

    function altitudeLabel(plan, index) {
      return plan.terrain_status === 'ok' ?
        ` · ground ${plan.ground_msl[index].toFixed(0)} m MSL` : '';
    }

    function drawPlan(plan) {
      if (planLayer) map.removeLayer(planLayer);
      const layers = [];
      const points = plan.waypoints.map(point => L.latLng(point[0], point[1]));
      for (let index = 0; index + 1 < points.length; index++) {
        const isRow = index % 2 === 0;
        const color = isRow ? '#245bd6' : '#e47720';
        layers.push(L.polyline([points[index], points[index + 1]], {
          color, weight: isRow ? 5 : 3,
          opacity: 0.9, dashArray: isRow ? null : '7 7'
        }));
        layers.push(arrowFor(points[index], points[index + 1], color));
        if (isRow) {
          layers.push(L.circleMarker(points[index], {
            radius: 6, color: '#fff', weight: 2,
            fillColor: color, fillOpacity: 1
          }).bindTooltip(
            `Row ${index / 2 + 1} start` +
            altitudeLabel(plan, index)));
        }
      }
      if (points.length) {
        layers.push(L.circleMarker(points[points.length - 1], {
          radius: 6, color: '#fff', weight: 2,
          fillColor: '#172033', fillOpacity: 1
        }).bindTooltip(
          'Scan finish' +
          altitudeLabel(plan, plan.waypoints.length - 1)));
      }
      planLayer = L.featureGroup(layers).addTo(map);
      planSummary.classList.toggle('warn', plan.terrain_status === 'uncovered');
      planSummary.textContent =
        `${plan.row_count} row${plan.row_count === 1 ? '' : 's'} · ` +
        `${plan.row_spacing_m.toFixed(1)} m apart · ` +
        `${plan.waypoints.length} waypoints · ${altitudeSummary(plan)}`;
    }

    function lapRouteFor(bounds) {
      if (activeLapRoute) {
        const route = activeLapRoute.map(point => L.latLng(point[0], point[1]));
        if (!route[0].equals(route[route.length - 1])) route.push(route[0]);
        return route;
      }
      const corners = [bounds.getNorthWest(), bounds.getSouthWest(),
        bounds.getSouthEast(), bounds.getNorthEast()];
      const route = lapDirection === 'counterclockwise' ? corners :
        [corners[0], corners[3], corners[2], corners[1]];
      return [...route, route[0]];
    }

    function drawLapPlan(route) {
      if (planLayer) map.removeLayer(planLayer);
      const layers = [];
      for (let index = 0; index + 1 < route.length; index++) {
        layers.push(L.polyline([route[index], route[index + 1]], {
          color: '#7b2fc4', weight: 5, opacity: 0.9
        }));
        layers.push(arrowFor(route[index], route[index + 1], '#7b2fc4'));
        layers.push(L.marker(route[index], {
          icon: L.divIcon({
            className: '', iconSize: [25, 25], iconAnchor: [12, 12],
            html: `<div class="waypoint-number">${index + 1}</div>`
          })
        }).bindTooltip(`Lap waypoint ${index + 1}`));
      }
      planLayer = L.featureGroup(layers).addTo(map);
      const laps = configuredRegion.lap_count;
      const waypointCount = route[0].equals(route[route.length - 1]) ?
        route.length - 1 : route.length;
      planSummary.textContent = `${waypointCount} lap waypoints · ${lapDirection} · ` +
        `${laps} configured lap${laps === 1 ? '' : 's'} · closes at waypoint 1`;
    }

    function refreshPreview(bounds) {
      if (mode === 'scan') refreshPlan(bounds);
      else drawLapPlan(lapRouteFor(bounds));
    }

    async function refreshPlan(bounds) {
      if (!bounds) return;
      const request = ++planRequest;
      planSummary.textContent = 'Generating mission rows…';
      try {
        const response = await fetch('/plan', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({boundary: boundaryFor(bounds)})
        });
        const result = await response.json();
        if (request !== planRequest) return;
        if (!response.ok) throw new Error(result.error || 'planner failed');
        drawPlan(result);
      } catch (error) {
        if (request !== planRequest) return;
        planSummary.textContent = `Could not generate rows: ${error.message}`;
      }
    }

    function updateFinishedRectangle() {
      activeScanBoundary = null;
      const bounds = L.latLngBounds(
        cornerMarkers[0].getLatLng(), cornerMarkers[1].getLatLng());
      activeBounds = bounds;
      rectangle.setBounds(bounds);
      yamlBox.value = codeFor(bounds);
      copyButton.disabled = false;
      useRegionButton.disabled = false;
      setStatus(`Rectangle ready — ${regionSize(bounds)}. Drag either corner to adjust it.`);
      refreshPreview(bounds);
    }

    function finishRectangle(secondCorner) {
      const bounds = L.latLngBounds(firstCorner, secondCorner);
      rectangle.setBounds(bounds);
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [
        L.marker(bounds.getNorthWest(), {draggable: true}).addTo(map),
        L.marker(bounds.getSouthEast(), {draggable: true}).addTo(map)
      ];
      cornerMarkers.forEach(marker => marker.on('drag', updateFinishedRectangle));
      firstCorner = null;
      updateFinishedRectangle();
    }

    function showConfiguredRegion() {
      if (configuredLayer) map.removeLayer(configuredLayer);
      configuredLayer = null;
      if (planLayer) map.removeLayer(planLayer);
      planLayer = null;
      const configuredPoints = mode === 'scan' ?
        configuredRegion.boundary : configuredRegion.lap_points;
      if (configuredPoints.length < 3) {
        setStatus(`No configured ${mode} region found in ${configuredRegion.label}.`);
        return;
      }
      activeBounds = L.latLngBounds(configuredPoints);
      if (mode === 'scan') {
        configuredLayer = L.featureGroup([
          L.polygon(configuredPoints, {
            color: '#20a464', weight: 4, fillOpacity: 0.12
          }).bindTooltip(`${configuredRegion.label}: scan region`)
        ]).addTo(map);
        activeScanBoundary = configuredPoints.map(point => [...point]);
      } else {
        activeLapRoute = configuredPoints.map(point => [...point]);
        lapDirection = configuredRegion.lap_direction;
      }
      map.fitBounds(activeBounds, {padding: [45, 45], maxZoom: 19});
      updateChoiceButtons();
      yamlBox.value = codeFor(activeBounds);
      copyButton.disabled = false;
      useRegionButton.disabled = mode === 'lap';
      refreshPreview(activeBounds);
      setStatus(`Showing the configured ${mode} region from ${configuredRegion.label}.`);
    }

    function updateChoiceButtons() {
      document.getElementById('clockwise').classList.toggle(
        'active', lapDirection === 'clockwise');
      document.getElementById('counterclockwise').classList.toggle(
        'active', lapDirection === 'counterclockwise');
      if (activeBounds && !firstCorner) {
        yamlBox.value = codeFor(activeBounds);
        refreshPreview(activeBounds);
      }
    }

    document.getElementById('clockwise').addEventListener('click', () => {
      lapDirection = 'clockwise'; updateChoiceButtons();
    });
    document.getElementById('counterclockwise').addEventListener('click', () => {
      lapDirection = 'counterclockwise'; updateChoiceButtons();
    });

    function switchMode(nextMode) {
      if (nextMode === mode) return;
      mode = nextMode;
      firstCorner = null;
      if (rectangle) map.removeLayer(rectangle);
      rectangle = null;
      activeBounds = null;
      activeLapRoute = null;
      cornerMarkers.forEach(marker => map.removeLayer(marker));
      cornerMarkers = [];
      if (configuredLayer) map.removeLayer(configuredLayer);
      configuredLayer = null;
      if (planLayer) map.removeLayer(planLayer);
      planLayer = null;
      document.getElementById('scan-tab').classList.toggle('active', mode === 'scan');
      document.getElementById('lap-tab').classList.toggle('active', mode === 'lap');
      document.getElementById('lap-controls').classList.toggle('hidden', mode !== 'lap');
      document.getElementById('mode-title').textContent =
        mode === 'scan' ? 'Scan region' : 'Lap route';
      document.getElementById('instructions').textContent = mode === 'scan' ?
        'Click one corner of the 1–10 acre search area, then click its opposite corner.' :
        'Click two opposite corners. The rectangle becomes a numbered, closed lap route.';
      configuredButton.textContent = `Show ${configuredRegion.label} ${mode}`;
      yamlBox.value = '# Select two corners on the map';
      copyButton.disabled = true;
      useRegionButton.disabled = true;
      planSummary.textContent = mode === 'scan' ?
        'Select a region to preview the rows.' :
        'Select a region to preview the lap route.';
      showConfiguredRegion();
    }

    document.getElementById('scan-tab').addEventListener('click', () => switchMode('scan'));
    document.getElementById('lap-tab').addEventListener('click', () => switchMode('lap'));

    map.on('click', event => {
      if (!firstCorner && activeBounds && copyButton.disabled === false) clearSelection();
      if (!firstCorner) {
        firstCorner = event.latlng;
        cornerMarkers = [L.marker(firstCorner).addTo(map)];
        rectangle = L.rectangle([firstCorner, firstCorner], {
          color: '#e34133', weight: 3, fillOpacity: 0.16
        }).addTo(map);
        setStatus('Now click the opposite corner.');
      } else {
        finishRectangle(event.latlng);
      }
    });

    map.on('mousemove', event => {
      if (firstCorner && rectangle) rectangle.setBounds([firstCorner, event.latlng]);
    });

    function finishLocate(request) {
      if (request !== locationRequest) return false;
      locateButton.disabled = false;
      locateButton.textContent = 'My location';
      return true;
    }

    function locationErrorMessage(error) {
      if (error && error.code === 1) {
        return 'Location permission is blocked. Allow location for this page in your browser settings, then try again.';
      }
      if (error && error.code === 2) {
        return 'Your location could not be determined. Check Location Services or pan/zoom the map manually.';
      }
      if (error && error.code === 3) {
        return 'Finding your location timed out. Try again or pan/zoom the map manually.';
      }
      return 'Location is unavailable. Pan/zoom the map, then click the first corner.';
    }

    function coordinatesFor(query) {
      const match = query.match(/^\\s*([-+]?\\d+(?:\\.\\d+)?)\\s*[, ]\\s*([-+]?\\d+(?:\\.\\d+)?)\\s*$/);
      if (!match) return null;
      const latitude = Number(match[1]);
      const longitude = Number(match[2]);
      if (latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180) {
        return null;
      }
      return [latitude, longitude];
    }

    function showSearchLocation(latitude, longitude, label) {
      const point = [latitude, longitude];
      map.setView(point, 18);
      if (searchLayer) map.removeLayer(searchLayer);
      searchLayer = L.marker(point, {title: label}).addTo(map);
      setStatus(`Showing ${label}. Click the first corner.`);
    }

    async function searchLocation(event) {
      event.preventDefault();
      const query = locationQuery.value.trim();
      if (!query) {
        setStatus('Enter an address or latitude, longitude.');
        locationQuery.focus();
        return;
      }
      searchLocationButton.disabled = true;
      searchLocationButton.textContent = 'Finding…';
      try {
        const coordinates = coordinatesFor(query);
        if (coordinates) {
          showSearchLocation(coordinates[0], coordinates[1], query);
          return;
        }
        const parameters = new URLSearchParams({
          q: query, format: 'jsonv2', limit: '1'
        });
        const response = await fetch(
          `https://nominatim.openstreetmap.org/search?${parameters}`,
          {headers: {'Accept': 'application/json'}});
        if (!response.ok) throw new Error(`search failed (${response.status})`);
        const results = await response.json();
        if (!results.length) throw new Error('address not found');
        const latitude = Number(results[0].lat);
        const longitude = Number(results[0].lon);
        if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) {
          throw new Error('search returned invalid coordinates');
        }
        showSearchLocation(latitude, longitude, results[0].display_name);
      } catch (error) {
        setStatus(`Could not find that location: ${error.message}.`);
      } finally {
        searchLocationButton.disabled = false;
        searchLocationButton.textContent = 'Find';
      }
    }

    function locate(forceRecenter = false) {
      const request = ++locationRequest;
      setStatus('Finding your location…');
      locateButton.disabled = forceRecenter;
      locateButton.textContent = 'Locating…';
      if (!navigator.geolocation) {
        finishLocate(request);
        setStatus(locationErrorMessage());
        return;
      }
      navigator.geolocation.getCurrentPosition(position => {
        if (!finishLocate(request)) return;
        if (!forceRecenter && (firstCorner || rectangle || configuredLayer)) return;
        const here = [position.coords.latitude, position.coords.longitude];
        map.setView(here, 19);
        if (locationLayer) map.removeLayer(locationLayer);
        locationLayer = L.circle(here, {
          radius: Math.max(position.coords.accuracy, 2),
          color: '#295bd6', fillOpacity: 0.10, weight: 2
        }).addTo(map).bindPopup(`Approximate location (±${Math.round(position.coords.accuracy)} m)`);
        setStatus(rectangle ? 'Centered on your location; the rectangle is unchanged.' :
          'Click the first corner.');
      }, error => {
        if (!finishLocate(request)) return;
        if (!forceRecenter && (firstCorner || rectangle || configuredLayer)) return;
        setStatus(locationErrorMessage(error));
      }, {enableHighAccuracy: false, timeout: 15000, maximumAge: 300000});
    }

    copyButton.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(yamlBox.value);
      } catch (_) {
        yamlBox.select();
        document.execCommand('copy');
      }
      setStatus('Copied to the clipboard.');
    });
    const siteNameInput = document.getElementById('site-name');
    siteNameInput.value = configuredRegion.label;
    useRegionButton.addEventListener('click', async () => {
      if (!activeBounds) return;
      const site = siteNameInput.value.trim();
      const region = mode === 'scan' ? 'scan area' : 'lap route';
      useRegionButton.disabled = true;
      setStatus(`Saving the ${region} to ${site}…`);
      try {
        const response = await fetch('/use-region', {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({
            token: writeToken,
            site,
            kind: mode,
            boundary: boundaryFor(activeBounds),
            lap_direction: lapDirection
          })
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || 'save failed');
        if (site === configuredRegion.label && mode === 'scan') {
          configuredRegion.boundary = boundaryFor(activeBounds);
        } else if (site === configuredRegion.label) {
          configuredRegion.lap_points = lapRouteFor(activeBounds).slice(0, -1).map(
            point => [point.lat, point.lng]);
          configuredRegion.lap_direction = lapDirection;
        }
        setStatus(result.created ?
          `Added site ${site} to sites.py with this ${region}.` :
          `Saved the ${region} to ${site} in sites.py.`);
      } catch (error) {
        setStatus(error instanceof TypeError ?
          'Could not save: the picker is no longer running. Start it again and reload this page.' :
          `Could not save: ${error.message}`);
      } finally {
        useRegionButton.disabled = false;
      }
    });
    document.getElementById('reset').addEventListener('click', clearSelection);
    document.getElementById('location-search').addEventListener(
      'submit', searchLocation);
    locateButton.addEventListener('click', () => locate(true));
    const hasConfiguredRegion = configuredRegion.boundary.length >= 3 ||
      configuredRegion.lap_points.length >= 3;
    configuredButton.disabled = !hasConfiguredRegion;
    configuredButton.textContent = hasConfiguredRegion ?
      `Show ${configuredRegion.label} scan` : 'No configured region found';
    configuredButton.addEventListener('click', showConfiguredRegion);
    document.getElementById('quit').addEventListener('click', async () => {
      await fetch('/shutdown', {method: 'POST'});
      setStatus('Server stopped. You can close this tab.');
    });
    locate();
  </script>
</body>
</html>
"""


class MapHandler(BaseHTTPRequestHandler):
    """Serves the picker page, its scan preview and its save and shutdown endpoints."""

    page = PAGE
    site = ""
    planner = None
    terrain = None
    write_token = ""
    write_lock = threading.Lock()

    def do_GET(self):
        if self.path not in ("/", "/index.html"):
            self.send_error(404)
            return
        body = self.page.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if self.path == "/plan":
            self._serve_plan()
            return
        if self.path == "/use-region":
            self._use_region()
            return
        if self.path != "/shutdown":
            self.send_error(404)
            return
        self.send_response(204)
        self.end_headers()
        threading.Thread(target=self.server.shutdown, daemon=True).start()

    def _read_request(self):
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0 or content_length > 16_384:
            raise ValueError("invalid request size")
        return json.loads(self.rfile.read(content_length))

    def _serve_plan(self):
        """Plans the scan rows the mission would fly over the picked corners."""
        try:
            boundary = _validated_boundary(self._read_request()["boundary"])
            waypoints = self.planner.plan(WaypointSequence.at_alt(0.0, [tuple(point) for point in boundary]))
            ground_msl, terrain_status = self._ground(waypoints)
            self._send_json(200, {
                "waypoints": [[point.lat, point.lon] for point in waypoints],
                "row_count": len(waypoints) // 2,
                "row_spacing_m": _distance_m(waypoints[1], waypoints[2]) if len(waypoints) > 2 else 0.0,
                "altitude_m": SCAN_ALT,
                "ground_msl": ground_msl,
                "terrain_status": terrain_status,
            })
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._send_json(400, {"error": str(error)})

    def _ground(self, waypoints):
        """Returns the ground height under each waypoint and whether the terrain map covers them all."""
        if self.terrain is None:
            return [], "no_dem"
        try:
            return [self.terrain.ground_msl(point.lat, point.lon) for point in waypoints], "ok"
        except ValueError:
            return [], "uncovered"

    def _use_region(self):
        """Saves a picked scan area or lap route into the site in sites.py."""
        try:
            request = self._read_request()
            if not secrets.compare_digest(str(request.get("token", "")), self.write_token):
                print("Could not save: the page is from an earlier run of the picker; reload it", flush=True)
                self._send_json(403, {"error": "this page is from an earlier run of the picker; reload it"})
                return
            kind = request["kind"]
            site = str(request.get("site") or self.site)
            options = {"lap_direction": request["lap_direction"]} if kind == "lap" else {}
            with self.write_lock:
                created = save_region(site, kind, request["boundary"], self.site, **options)
            print(f"{'Added' if created else 'Saved'} {site}'s {'scan area' if kind == 'scan' else 'lap route'} "
                  f"in {SITES_PATH}", flush=True)
            self._send_json(200, {"saved": True, "created": created, "site": site, "kind": kind,
                                  "path": str(SITES_PATH)})
        except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            print(f"Could not save: {error}", flush=True)
            self._send_json(400, {"error": str(error)})

    def _send_json(self, status, value):
        body = json.dumps(value, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format_, *args):
        """Keeps the terminal to the start and stop messages."""


def parse_args():
    parser = argparse.ArgumentParser(description="Open a map to pick a site's scan area or lap route.")
    parser.add_argument("--site", default="SIM", help="the site in sites.py to show and update (default: SIM)")
    parser.add_argument("--platform", choices=list(PLATFORMS), default="comp_drone",
                        help="whose camera the scan rows are spaced for (default: comp_drone)")
    parser.add_argument("--port", type=int, default=0, help="localhost port (default: any free port)")
    parser.add_argument("--no-browser", action="store_true", help="serve the page without opening a browser")
    return parser.parse_args()


def configured_region(name, site):
    """Returns a site's scan area and lap route for the page to show."""
    lap_points = [[point.lat, point.lon] for point in site.laps]
    signed_area = sum(
        lap_points[index][1] * lap_points[(index + 1) % len(lap_points)][0]
        - lap_points[(index + 1) % len(lap_points)][1] * lap_points[index][0]
        for index in range(len(lap_points))
    ) if len(lap_points) >= 3 else 0.0
    return {
        "label": name,
        "boundary": [[point.lat, point.lon] for point in site.scan_area],
        "lap_points": lap_points,
        "lap_direction": "counterclockwise" if signed_area >= 0 else "clockwise",
        "lap_count": LAP_COUNT,
        "terrain_map": Path(site.terrain_map).name,
    }


def load_terrain(path):
    """Loads the site's terrain map, or returns None if it's missing or rasterio isn't installed."""
    if not Path(path).is_file():
        print(f"No terrain map at {path}; previews show no ground heights.")
        return None
    try:
        from bv_autonomy.components.terrain import DemTerrain
    except ImportError:
        print("rasterio isn't installed; previews show no ground heights.")
        return None
    print(f"Terrain map: {path}")
    return DemTerrain(path)


def main():
    args = parse_args()
    site = getattr(sites, args.site, None)
    if site is None:
        raise SystemExit(f"No site {args.site} in {SITES_PATH}")
    intrinsics = PLATFORMS[args.platform]().intrinsics
    write_token = secrets.token_urlsafe(32)
    MapHandler.page = PAGE.replace(
        "__CONFIGURED_REGION__",
        json.dumps(configured_region(args.site, site), separators=(",", ":")).replace("</", "<\\/"),
    ).replace("__WRITE_TOKEN__", json.dumps(write_token))
    MapHandler.site = args.site
    MapHandler.planner = SPathPlanner(intrinsics, SCAN_ALT, SCAN_OVERLAP)
    MapHandler.terrain = load_terrain(site.terrain_map)
    MapHandler.write_token = write_token
    try:
        server = ThreadingHTTPServer(("127.0.0.1", args.port), MapHandler)
    except OSError as error:
        raise SystemExit(f"Could not start the map server: {error}") from error

    url = f"http://127.0.0.1:{server.server_port}/"
    print(f"Scan picker for {args.site}: {url}")
    print('Press Ctrl+C here, or click "Stop server" in the page, when done.')
    if not args.no_browser and not webbrowser.open(url):
        print("The browser did not open automatically; open the URL above.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping the scan picker.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
