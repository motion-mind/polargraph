#pragma once
/**
 * WiFiManager.h — WiFi, web dashboard, G-code upload & job control
 *
 * Endpoints:
 *   GET  /               Dashboard (monitoring + job control)
 *   GET  /status         JSON: position, pen, job state, progress, SG
 *   GET  /files          JSON: list of LittleFS files
 *   POST /upload         Chunked multipart upload → LittleFS /job.gcode
 *   POST /job/start      Start the stored job
 *   POST /job/pause      Pause running job
 *   POST /job/resume     Resume paused job
 *   POST /job/stop       Stop & flush queue
 *   POST /gcode          Execute a single G-code line (manual cmd)
 *   POST /home           Trigger sensorless homing
 *   GET  /sg             JSON: StallGuard live values
 *   POST /sgconfig       Update homing config
 *   POST /config         Update machine config
 *   GET  /image          Image processor app (from LittleFS)
 *   GET  /wifi           WiFi setup page
 *   POST /wifi           Save WiFi credentials & reboot
 *   GET  /update         ElegantOTA firmware update
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ElegantOTA.h>
#include <LittleFS.h>
#include "Config.h"
#include "StallHoming.h"
#include "JobExecutor.h"

extern class MotionPlanner  planner;
extern class PenServo       penServo;
extern class GCodeParser    gcodeParser;
extern class StallHoming    stallHoming;
extern class JobExecutor    jobExec;
extern class BinJobExecutor binExec;

// Shared job state (defined in main.cpp)
extern volatile bool     gJobRunning;
extern volatile bool     gJobPaused;
extern volatile bool     gJobStop;
extern volatile uint32_t gJobLinesDone;
extern volatile uint32_t gJobLinesTotal;
extern volatile char     gJobFile[64];
extern volatile char     gJobType[8];

class WiFiManager {
public:
    void begin() {
        if (!LittleFS.begin(true)) {  // true = format if mount fails
            Serial.println(F("  WiFi: LittleFS mount failed"));
        } else {
            Serial.println(F("  LittleFS mounted"));
        }

        _prefs.begin("wifi", true);
        String ssid = _prefs.getString("ssid", "");
        String pass = _prefs.getString("pass", "");
        _prefs.end();

        if (ssid.length() > 0) {
            Serial.printf("  WiFi: connecting to '%s'", ssid.c_str());
            WiFi.begin(ssid.c_str(), pass.c_str());
            uint32_t t0 = millis();
            while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) {
                delay(250); Serial.print('.');
            }
            if (WiFi.status() == WL_CONNECTED) {
                Serial.printf("\n  WiFi: http://%s\n", WiFi.localIP().toString().c_str());
                _connected = true;
            } else {
                Serial.println(F("\n  WiFi: timeout — AP mode"));
                startAP();
            }
        } else {
            Serial.println(F("  WiFi: no creds — AP mode"));
            startAP();
        }

        setupRoutes();
        ElegantOTA.begin(&_server);
        _server.begin();
        Serial.println(F("  Web server started"));
    }

    void service() {
        _server.handleClient();
        ElegantOTA.loop();
    }

private:
    WebServer   _server{80};
    Preferences _prefs;
    bool        _connected  = false;
    bool        _apMode     = false;
    uint32_t    _jobStartMs = 0;   // for ETA calculation

    // ── Upload state ──────────────────────────────────────────
    File        _uploadFile;
    size_t      _uploadBytes = 0;
    char        _uploadExt[8] = "";

    void startAP() {
        WiFi.softAP("Polargraph-Setup", "polargraph");
        Serial.printf("  AP: SSID=Polargraph-Setup  IP=%s\n",
                      WiFi.softAPIP().toString().c_str());
        _apMode = true;
    }

    // ═══════════════════════════════════════════════════════════
    void setupRoutes() {

        // ── WiFi setup ─────────────────────────────────────────
        _server.on("/wifi", HTTP_GET,  [this](){ _server.send(200,"text/html",wifiPage()); });
        _server.on("/wifi", HTTP_POST, [this](){
            String s = _server.arg("ssid"), p = _server.arg("pass");
            _prefs.begin("wifi", false);
            _prefs.putString("ssid", s); _prefs.putString("pass", p);
            _prefs.end();
            _server.send(200,"text/html","<h2>Saved — rebooting...</h2>");
            delay(1500); ESP.restart();
        });

        // ── Dashboard ──────────────────────────────────────────
        _server.on("/", HTTP_GET, [this](){ _server.send(200,"text/html",dashboard()); });

        // ── Rich status JSON ───────────────────────────────────
        _server.on("/status", HTTP_GET, [this](){
            // Compute ETA
            uint32_t etaSec = 0;
            if (gJobRunning && gJobLinesDone > 0) {
                uint32_t elapsed = (millis() - _jobStartMs) / 1000;
                if (elapsed > 0 && gJobLinesTotal > 0) {
                    uint32_t total = (elapsed * gJobLinesTotal) / gJobLinesDone;
                    etaSec = total > elapsed ? total - elapsed : 0;
                }
            }
            const char* state = !gJobRunning  ? "idle"     :
                                 gJobStop     ? "stopping" :
                                 gJobPaused   ? "paused"   : "running";
            char buf[512];
            snprintf(buf, sizeof(buf),
                "{"
                "\"x\":%.2f,\"y\":%.2f,"
                "\"penUp\":%s,\"busy\":%s,"
                "\"job\":{"
                  "\"state\":\"%s\","
                  "\"type\":\"%s\","
                  "\"file\":\"%s\","
                  "\"linesDone\":%lu,"
                  "\"linesTotal\":%lu,"
                  "\"pct\":%u,"
                  "\"etaSec\":%lu"
                "},"
                "\"queue\":%u"
                "}",
                planner.posX(), planner.posY(),
                penServo.isUp() ? "true":"false",
                planner.isBusy()? "true":"false",
                state,
                (const char*)gJobType,
                (const char*)gJobFile,
                (unsigned long)gJobLinesDone,
                (unsigned long)gJobLinesTotal,
                gJobLinesTotal ? (uint8_t)((gJobLinesDone*100UL)/gJobLinesTotal) : 0,
                (unsigned long)etaSec,
                (unsigned)planner.queueDepth()
            );
            _server.send(200, "application/json", buf);
        });

        // ── StallGuard status ──────────────────────────────────
        _server.on("/sg", HTTP_GET, [this](){
            char buf[128];
            snprintf(buf, sizeof(buf),
                "{\"sgL\":%u,\"sgR\":%u,\"stallL\":%s,\"stallR\":%s}",
                stallHoming.sgLeft(), stallHoming.sgRight(),
                stallHoming.isStallL()?"true":"false",
                stallHoming.isStallR()?"true":"false");
            _server.send(200, "application/json", buf);
        });

        // ── File list ──────────────────────────────────────────
        _server.on("/files", HTTP_GET, [this](){
            _server.send(200, "application/json", JobExecutor::listFiles());
        });

        // ── G-code file upload (multipart/form-data) ──────────
        // The WebServer upload handler fires for each chunk;
        // we stream it straight to LittleFS, never buffering in RAM.
        _server.on("/upload", HTTP_POST,
            // completion handler — fires after all chunks received
            [this](){
                _uploadFile.close();
                Serial.printf("[Upload] Complete — %u bytes → %s\n",
                              _uploadBytes, "/job.gcode");
                _server.send(200, "application/json",
                    "{\"ok\":true,\"bytes\":" + String(_uploadBytes) + "}");
                _uploadBytes = 0;
            },
            // upload chunk handler
            [this](){
                HTTPUpload& upload = _server.upload();
                if (upload.status == UPLOAD_FILE_START) {
                    Serial.printf("[Upload] Start: %s\n", upload.filename.c_str());
                    // Detect binary vs gcode from filename extension
                    String fn = upload.filename;
                    fn.toLowerCase();
                    bool isBin = fn.endsWith(".plg");
                    const char* dest = isBin ? "/job.plg" : "/job.gcode";
                    // Clean up the other type
                    LittleFS.remove(isBin ? "/job.gcode" : "/job.plg");
                    LittleFS.remove(dest);
                    _uploadFile  = LittleFS.open(dest, "w");
                    _uploadBytes = 0;
                    if (!_uploadFile) Serial.println(F("[Upload] open failed"));
                } else if (upload.status == UPLOAD_FILE_WRITE) {
                    if (_uploadFile) {
                        _uploadFile.write(upload.buf, upload.currentSize);
                        _uploadBytes += upload.currentSize;
                    }
                } else if (upload.status == UPLOAD_FILE_END) {
                    // Final chunk — completion handler fires next
                }
            }
        );

        // ── Job control ────────────────────────────────────────
        _server.on("/job/start", HTTP_POST, [this](){
            if (gJobRunning) { _server.send(409,"text/plain","job already running"); return; }

            // Detect file type: check for .plg extension or PLG1 magic
            bool isBinary = false;
            if (LittleFS.exists("/job.plg")) {
                isBinary = true;
            } else if (LittleFS.exists("/job.gcode")) {
                // Peek at magic bytes
                File peek = LittleFS.open("/job.gcode","r");
                if (peek) {
                    char magic[4]{}; peek.read((uint8_t*)magic,4); peek.close();
                    if (magic[0]=='P'&&magic[1]=='L'&&magic[2]=='G') isBinary = true;
                }
            }

            bool ok;
            if (isBinary) {
                const char* path = LittleFS.exists("/job.plg") ? "/job.plg" : "/job.gcode";
                ok = binExec.start(path);
            } else {
                ok = jobExec.start("/job.gcode");
            }
            if (!ok) { _server.send(500,"text/plain","start failed"); return; }
            _jobStartMs = millis();
            _server.send(200,"text/plain", isBinary ? "ok (binary)" : "ok (gcode)");
        });

        _server.on("/job/pause",  HTTP_POST, [this](){
            gJobPaused = true;
            _server.send(200,"text/plain","ok");
        });
        _server.on("/job/resume", HTTP_POST, [this](){
            gJobPaused = false;
            _server.send(200,"text/plain","ok");
        });
        _server.on("/job/stop",   HTTP_POST, [this](){
            gJobStop = true;
            planner.eStop();
            _server.send(200,"text/plain","ok");
        });

        // ── Manual single G-code line (manual cmd panel) ──────
        _server.on("/gcode", HTTP_POST, [this](){
            String cmd = _server.arg("cmd");
            if (cmd.length() > 0) gcodeParser.process(cmd.c_str());
            _server.send(200, "text/plain", "ok");
        });

        // ── Sensorless homing ──────────────────────────────────
        _server.on("/home", HTTP_POST, [this](){
            if (planner.isBusy() || !jobExec.isIdle()) {
                _server.send(409, "text/plain", "busy"); return;
            }
            int32_t oL=0, oR=0;
            HomingResult res = stallHoming.home(oL, oR);
            _server.send(res==HomingResult::SUCCESS ? 200 : 500, "text/plain",
                res==HomingResult::SUCCESS            ? "ok"                        :
                res==HomingResult::STALL_TIMEOUT_L    ? "error: left stall timeout" :
                res==HomingResult::STALL_TIMEOUT_R    ? "error: right stall timeout":
                                                        "error: driver fault");
        });

        // ── StallGuard config ──────────────────────────────────
        _server.on("/sgconfig", HTTP_POST, [this](){
            if (_server.hasArg("sgt")) stallHoming.homingCfg.sgThreshold=(uint8_t)_server.arg("sgt").toInt();
            if (_server.hasArg("spd")) stallHoming.homingCfg.homingSpeed=_server.arg("spd").toFloat();
            if (_server.hasArg("bof")) stallHoming.homingCfg.backoffMm=_server.arg("bof").toFloat();
            if (_server.hasArg("ofl")) stallHoming.homingCfg.homeCordOffsetL=_server.arg("ofl").toFloat();
            if (_server.hasArg("ofr")) stallHoming.homingCfg.homeCordOffsetR=_server.arg("ofr").toFloat();
            stallHoming.saveConfig();
            _server.send(200, "text/plain", "ok");
        });

        // ── Machine config ─────────────────────────────────────
        _server.on("/config", HTTP_POST, [this](){
            if (_server.hasArg("width"))   Config::cfg.machineWidth=_server.arg("width").toFloat();
            if (_server.hasArg("height"))  Config::cfg.machineHeight=_server.arg("height").toFloat();
            if (_server.hasArg("spm"))     Config::cfg.stepsPerMm=_server.arg("spm").toFloat();
            if (_server.hasArg("maxfeed")) Config::cfg.maxFeedrate=_server.arg("maxfeed").toFloat();
            if (_server.hasArg("accel"))   Config::cfg.acceleration=_server.arg("accel").toFloat();
            Config::save();
            _server.send(200, "text/plain", "ok");
        });

        // ── Image processor (LittleFS) ─────────────────────────
        _server.on("/image", HTTP_GET, [this](){
            if (LittleFS.exists("/image_processor.html")) {
                File f = LittleFS.open("/image_processor.html", "r");
                _server.streamFile(f, "text/html"); f.close();
            } else {
                _server.send(404, "text/plain",
                    "Not found. Run: pio run -t uploadfs");
            }
        });

        // ── Captive portal / 404 ───────────────────────────────
        _server.onNotFound([this](){
            if (_apMode) {
                _server.sendHeader("Location","http://192.168.4.1/wifi");
                _server.send(302);
            } else {
                _server.send(404,"text/plain","Not found");
            }
        });
    }

    // ═══════════════════════════════════════════════════════════
    // Dashboard HTML
    // ═══════════════════════════════════════════════════════════
    static String dashboard() { return R"HTML(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Polargraph</title>
<style>
:root{--bg:#0c0e14;--panel:#13151f;--card:#1a1d2b;--border:#252838;
      --accent:#7c6af7;--green:#4ade80;--red:#f87171;--amber:#fbbf24;
      --text:#e2e8f0;--muted:#64748b;--r:10px}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;
     font-size:14px;min-height:100vh;display:flex;flex-direction:column}
header{background:var(--panel);border-bottom:1px solid var(--border);
       padding:.7rem 1.2rem;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:.5rem}
header h1{font-size:1.05rem;color:#a78bfa;font-weight:700}
header nav a{color:var(--muted);text-decoration:none;font-size:.82rem;margin-left:1rem;
              padding:.3rem .6rem;border-radius:5px;border:1px solid var(--border)}
header nav a:hover{border-color:var(--accent);color:var(--text)}
.workspace{display:flex;flex:1;overflow:hidden;flex-wrap:wrap}
.sidebar{width:280px;min-width:240px;background:var(--panel);border-right:1px solid var(--border);
         overflow-y:auto;padding:1rem;display:flex;flex-direction:column;gap:.8rem}
.main{flex:1;padding:1rem;overflow-y:auto;display:flex;flex-direction:column;gap:1rem;min-width:280px}
.card{background:var(--card);border:1px solid var(--border);border-radius:var(--r);padding:.9rem}
.card h3{font-size:.7rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted);margin-bottom:.6rem}
.stat{font-size:1.5rem;font-weight:700}
.stat-unit{font-size:.8rem;color:var(--muted);margin-left:.2rem}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:.8rem}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:.6rem}
.badge{display:inline-block;padding:.2rem .6rem;border-radius:999px;font-size:.72rem;font-weight:700}
.b-idle   {background:#1c1917;color:var(--muted)}
.b-running{background:#1e3a2f;color:var(--green)}
.b-paused {background:#422006;color:var(--amber)}
.b-stop   {background:#450a0a;color:var(--red)}
.b-up  {background:#14532d;color:var(--green)}
.b-down{background:#450a0a;color:var(--red)}
.progress-track{height:8px;background:#1a1a2e;border-radius:4px;overflow:hidden;margin:.5rem 0}
.progress-fill {height:100%;background:var(--accent);border-radius:4px;
                transition:width .4s;width:0%}
button{background:var(--accent);color:#fff;border:none;border-radius:7px;
       padding:.45rem .9rem;cursor:pointer;font-size:.82rem;font-weight:600;transition:opacity .15s}
button:hover{opacity:.85} button:disabled{opacity:.35;cursor:not-allowed}
button.sec{background:var(--card);border:1px solid var(--border);color:var(--text)}
button.danger{background:#7f1d1d;color:var(--red)}
button.warn  {background:#422006;color:var(--amber)}
button.go    {background:#14532d;color:var(--green)}
.btn-row{display:flex;gap:.4rem;flex-wrap:wrap}
label{display:block;font-size:.75rem;color:var(--muted);margin-bottom:.2rem}
input[type=text],input[type=number],select{
  width:100%;background:var(--bg);border:1px solid var(--border);
  color:var(--text);border-radius:6px;padding:.38rem .6rem;font-size:.82rem}
.sg-bar{height:6px;background:#111;border-radius:3px;margin:.25rem 0;overflow:hidden}
.sg-fill{height:100%;border-radius:3px;transition:width .3s,background .3s;background:var(--accent)}
.log{background:var(--bg);border:1px solid var(--border);border-radius:8px;
     padding:.6rem;font-family:monospace;font-size:.74rem;color:#86efac;
     height:140px;overflow-y:auto;white-space:pre-wrap;word-break:break-all}
canvas{display:block;width:100%;border-radius:8px;background:#0a0a0f}
.eta{font-size:.75rem;color:var(--muted)}
/* upload drop zone */
#dropzone{border:2px dashed var(--border);border-radius:var(--r);
           padding:1.5rem;text-align:center;cursor:pointer;color:var(--muted);
           transition:border-color .2s,background .2s}
#dropzone:hover,#dropzone.over{border-color:var(--accent);background:rgba(124,106,247,.06);color:var(--text)}
#dropzone svg{display:block;margin:0 auto .5rem;opacity:.4}
.upload-progress{display:none}
.upload-progress .progress-fill{background:var(--green)}
</style>
</head>
<body>
<header>
  <h1>⬡ Polargraph</h1>
  <nav>
    <a href="/image">🖼 Image Processor</a>
    <a href="/update">⬆ OTA Update</a>
    <a href="/wifi">📶 WiFi</a>
  </nav>
</header>

<div class="workspace">

<!-- ── SIDEBAR ───────────────────────────────────────────── -->
<div class="sidebar">

  <!-- Upload -->
  <div class="card">
    <h3>Upload G-code</h3>
    <div id="dropzone" onclick="document.getElementById('fileinput').click()">
      <svg width="28" height="28" viewBox="0 0 24 24" fill="none"
           stroke="currentColor" stroke-width="1.5">
        <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
        <polyline points="17 8 12 3 7 8"/>
        <line x1="12" y1="3" x2="12" y2="15"/>
      </svg>
      <div style="font-size:.85rem">Drop .gcode / click to browse</div>
    </div>
    <input type="file" id="fileinput" accept=".gcode,.nc,.txt" style="display:none"
           onchange="uploadFile(this.files[0])">
    <div class="upload-progress" id="uprog">
      <div class="progress-track"><div class="progress-fill" id="uprog-fill"></div></div>
      <div id="uprog-label" class="eta"></div>
    </div>
    <div id="upload-status" style="font-size:.75rem;color:var(--muted);margin-top:.4rem"></div>
  </div>

  <!-- Job control -->
  <div class="card">
    <h3>Job Control</h3>
    <div style="margin-bottom:.6rem">
      <span id="job-badge" class="badge b-idle">IDLE</span>
      <span id="job-file" style="font-size:.72rem;color:var(--muted);margin-left:.4rem"></span>
    </div>
    <div class="progress-track"><div class="progress-fill" id="job-prog"></div></div>
    <div class="btn-row" style="margin-top:.5rem">
      <button class="go"   id="btn-start"  onclick="jobCmd('start')">▶ Start</button>
      <button class="warn" id="btn-pause"  onclick="jobCmd('pause')"  disabled>⏸ Pause</button>
      <button class="sec"  id="btn-resume" onclick="jobCmd('resume')" disabled>▷ Resume</button>
      <button class="danger" id="btn-stop" onclick="jobCmd('stop')"   disabled>■ Stop</button>
    </div>
    <div id="job-eta" class="eta" style="margin-top:.4rem"></div>
  </div>

  <!-- StallGuard -->
  <div class="card">
    <h3>StallGuard — Cord Tension</h3>
    <div style="font-size:.72rem;color:var(--muted)">Left</div>
    <div class="sg-bar"><div class="sg-fill" id="sg-l" style="width:0%"></div></div>
    <div style="font-size:.72rem;color:var(--muted)">Right</div>
    <div class="sg-bar"><div class="sg-fill" id="sg-r" style="width:0%"></div></div>
    <div id="sg-vals" style="font-size:.7rem;color:var(--muted);margin-top:.2rem">L:— R:—</div>
    <div class="btn-row" style="margin-top:.6rem">
      <button class="sec" onclick="triggerHome()">⌂ Home</button>
    </div>
  </div>

  <!-- Manual G-code -->
  <div class="card">
    <h3>Manual Command</h3>
    <input type="text" id="gcmd" placeholder="G1 X350 Y400 F3000"
           onkeydown="if(event.key==='Enter')sendCmd()">
    <div class="btn-row" style="margin-top:.4rem">
      <button onclick="sendCmd()">Send</button>
      <button class="sec" onclick="sendCmd('M5')">Pen ↑</button>
      <button class="sec" onclick="sendCmd('M3')">Pen ↓</button>
      <button class="sec" onclick="sendCmd('G28')">Home</button>
    </div>
    <div class="log" id="cmdlog" style="margin-top:.5rem"></div>
  </div>

</div><!-- sidebar -->

<!-- ── MAIN ──────────────────────────────────────────────── -->
<div class="main">

  <!-- Status row -->
  <div class="grid3">
    <div class="card" style="text-align:center">
      <h3>X Position</h3>
      <span class="stat" id="stat-x">—</span><span class="stat-unit">mm</span>
    </div>
    <div class="card" style="text-align:center">
      <h3>Y Position</h3>
      <span class="stat" id="stat-y">—</span><span class="stat-unit">mm</span>
    </div>
    <div class="card" style="text-align:center">
      <h3>Pen</h3>
      <span class="badge b-up" id="pen-badge">UP</span>
    </div>
  </div>

  <!-- Job progress detail -->
  <div class="card" id="progress-card">
    <h3>Job Progress</h3>
    <div class="progress-track" style="height:12px">
      <div class="progress-fill" id="main-prog" style="height:100%"></div>
    </div>
    <div style="display:flex;justify-content:space-between;margin-top:.4rem">
      <span id="prog-lines" class="eta">— / — lines</span>
      <span id="prog-pct"   class="eta">0%</span>
      <span id="prog-eta"   class="eta">ETA —</span>
    </div>
  </div>

  <!-- Canvas -->
  <div class="card">
    <h3>Canvas Preview <span id="canvas-info" style="color:var(--muted);font-weight:400"></span></h3>
    <canvas id="cv" height="420"></canvas>
  </div>

  <!-- Config (collapsed by default) -->
  <details class="card">
    <summary style="cursor:pointer;color:var(--muted);font-size:.8rem">⚙ Machine Config</summary>
    <div style="display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:.6rem;margin-top:.8rem">
      <div><label>Width (mm)</label><input type="number" id="cfg-w" value="700"></div>
      <div><label>Height (mm)</label><input type="number" id="cfg-h" value="900"></div>
      <div><label>Steps/mm</label><input type="number" id="cfg-spm" value="80" step=".01"></div>
      <div><label>Max feed (mm/s)</label><input type="number" id="cfg-mf" value="150"></div>
      <div><label>Accel (mm/s²)</label><input type="number" id="cfg-acc" value="400"></div>
    </div>
    <div style="margin-top:.6rem">
      <button onclick="saveConfig()">Save Config</button>
    </div>
  </details>

</div><!-- main -->
</div><!-- workspace -->

<script>
// ── Canvas setup ──────────────────────────────────────────────
const cv = document.getElementById('cv');
const ctx = cv.getContext('2d');
let mW = 700, mH = 900;
let trail = [];
let prevPenDown = false;

function resize() {
  const w = cv.parentElement.clientWidth - 32;
  cv.width = w;
  cv.height = Math.round(w * mH / mW);
}
resize(); new ResizeObserver(resize).observe(cv.parentElement);

function drawCanvas(x, y, penUp) {
  const W = cv.width, H = cv.height;
  const sx = px => px / mW * W;
  const sy = py => py / mH * H;

  ctx.fillStyle = '#0a0a0f'; ctx.fillRect(0, 0, W, H);

  // Canvas border
  ctx.strokeStyle = '#252838'; ctx.lineWidth = 1;
  ctx.strokeRect(1, 1, W-2, H-2);

  // Cords
  ctx.strokeStyle = '#1e2237'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(sx(x), sy(y)); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(W, 0); ctx.lineTo(sx(x), sy(y)); ctx.stroke();

  // Trail
  if (!penUp) {
    if (!prevPenDown) { trail.push({x, y, lift: true}); }
    else              { trail.push({x, y, lift: false}); }
  }
  prevPenDown = !penUp;

  if (trail.length > 1) {
    ctx.strokeStyle = '#7c6af7'; ctx.lineWidth = 1.2;
    ctx.beginPath();
    for (let i = 0; i < trail.length; i++) {
      const p = trail[i];
      if (i === 0 || p.lift) ctx.moveTo(sx(p.x), sy(p.y));
      else                   ctx.lineTo(sx(p.x), sy(p.y));
    }
    ctx.stroke();
    // Trim trail if very long
    if (trail.length > 8000) trail.splice(0, 2000);
  }

  // Gondola dot
  ctx.fillStyle = penUp ? '#64748b' : '#f59e0b';
  ctx.beginPath(); ctx.arc(sx(x), sy(y), 5, 0, 2*Math.PI); ctx.fill();

  // Anchors
  ctx.fillStyle = '#7c6af7';
  ctx.beginPath(); ctx.arc(0, 0, 4, 0, 2*Math.PI); ctx.fill();
  ctx.beginPath(); ctx.arc(W, 0, 4, 0, 2*Math.PI); ctx.fill();
}

// ── Polling ───────────────────────────────────────────────────
async function poll() {
  try {
    const r = await fetch('/status');
    const d = await r.json();

    document.getElementById('stat-x').textContent = d.x.toFixed(1);
    document.getElementById('stat-y').textContent = d.y.toFixed(1);

    const pb = document.getElementById('pen-badge');
    pb.textContent = d.penUp ? 'UP' : 'DOWN';
    pb.className = 'badge ' + (d.penUp ? 'b-up' : 'b-down');

    // Job badge & buttons
    const jb = document.getElementById('job-badge');
    const s = d.job.state;
    jb.textContent = s.toUpperCase();
    jb.className = 'badge ' + ({idle:'b-idle',running:'b-running',
                                  paused:'b-paused',stopping:'b-stop'}[s]||'b-idle');

    document.getElementById('job-file').textContent =
      d.job.file ? d.job.file.replace('/','') : '';

    // Button states
    const running = s==='running', paused=s==='paused', active=running||paused;
    document.getElementById('btn-start') .disabled = active;
    document.getElementById('btn-pause') .disabled = !running;
    document.getElementById('btn-resume').disabled = !paused;
    document.getElementById('btn-stop')  .disabled = !active;

    // Progress
    const pct = d.job.pct || 0;
    document.getElementById('job-prog').style.width  = pct + '%';
    document.getElementById('main-prog').style.width = pct + '%';
    document.getElementById('prog-pct').textContent  = pct + '%';
    document.getElementById('prog-lines').textContent =
      d.job.linesDone.toLocaleString() + ' / ' + d.job.linesTotal.toLocaleString() + ' lines';
    const eta = d.job.etaSec;
    document.getElementById('prog-eta').textContent =
      eta > 0 ? 'ETA ' + fmtTime(eta) : '';
    document.getElementById('job-eta').textContent =
      active && eta > 0 ? 'Remaining: ' + fmtTime(eta) : '';

    document.getElementById('canvas-info').textContent =
      `${mW}×${mH} mm`;

    drawCanvas(d.x, d.y, d.penUp);
  } catch(e){}

  // SG (lower priority — separate fetch)
  try {
    const sg = await (await fetch('/sg')).json();
    const max = 511;
    document.getElementById('sg-l').style.width = (sg.sgL/max*100)+'%';
    document.getElementById('sg-r').style.width = (sg.sgR/max*100)+'%';
    document.getElementById('sg-l').style.background = sg.stallL?'#ef4444':'#7c6af7';
    document.getElementById('sg-r').style.background = sg.stallR?'#ef4444':'#7c6af7';
    document.getElementById('sg-vals').textContent =
      `L:${sg.sgL}${sg.stallL?' ⚡':''} R:${sg.sgR}${sg.stallR?' ⚡':''}`;
  } catch(e){}
}
setInterval(poll, 600);
poll();

function fmtTime(s) {
  if (s < 60) return s + 's';
  const m = Math.floor(s/60), r = s%60;
  if (m < 60) return m + 'm ' + r + 's';
  return Math.floor(m/60) + 'h ' + (m%60) + 'm';
}

// ── Upload ────────────────────────────────────────────────────
const dz = document.getElementById('dropzone');
dz.addEventListener('dragover', e=>{e.preventDefault();dz.classList.add('over')});
dz.addEventListener('dragleave', ()=>dz.classList.remove('over'));
dz.addEventListener('drop', e=>{
  e.preventDefault(); dz.classList.remove('over');
  if (e.dataTransfer.files[0]) uploadFile(e.dataTransfer.files[0]);
});

async function uploadFile(file) {
  if (!file) return;
  const prog = document.getElementById('uprog');
  const fill = document.getElementById('uprog-fill');
  const label = document.getElementById('uprog-label');
  const status = document.getElementById('upload-status');

  prog.style.display = 'block';
  fill.style.width = '0%';
  label.textContent = 'Uploading…';
  status.textContent = '';

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/upload');

  xhr.upload.onprogress = e => {
    if (e.lengthComputable) {
      const pct = Math.round(e.loaded / e.total * 100);
      fill.style.width = pct + '%';
      label.textContent = pct + '% — ' + fmtBytes(e.loaded) + ' / ' + fmtBytes(e.total);
    }
  };
  xhr.onload = () => {
    fill.style.width = '100%';
    fill.style.background = '#4ade80';
    label.textContent = 'Complete';
    status.textContent = `✓ ${file.name} (${fmtBytes(file.size)}) ready`;
    status.style.color = '#4ade80';
    setTimeout(()=>{ prog.style.display='none'; fill.style.background=''; }, 2500);
  };
  xhr.onerror = () => {
    status.textContent = '✗ Upload failed';
    status.style.color = '#f87171';
  };

  const fd = new FormData();
  fd.append('file', file, file.name);
  xhr.send(fd);
}

function fmtBytes(b) {
  if (b < 1024) return b + ' B';
  if (b < 1048576) return (b/1024).toFixed(1) + ' KB';
  return (b/1048576).toFixed(2) + ' MB';
}

// ── Job control ───────────────────────────────────────────────
async function jobCmd(cmd) {
  const r = await fetch('/job/' + cmd, {method:'POST'});
  const t = await r.text();
  log('[job/' + cmd + '] ' + t, r.ok);
}

// ── Home ──────────────────────────────────────────────────────
async function triggerHome() {
  if (!confirm('Start sensorless homing? Cords will retract to stall.')) return;
  log('[home] starting…', true);
  const r = await fetch('/home', {method:'POST'});
  const t = await r.text();
  log('[home] ' + t, r.ok);
}

// ── Manual command ────────────────────────────────────────────
async function sendCmd(cmd) {
  const txt = cmd || document.getElementById('gcmd').value.trim();
  if (!txt) return;
  const fd = new FormData(); fd.append('cmd', txt);
  const r = await fetch('/gcode', {method:'POST', body:fd});
  log('> ' + txt, r.ok);
  if (!cmd) document.getElementById('gcmd').value = '';
}

function log(msg, ok=true) {
  const el = document.getElementById('cmdlog');
  const d = document.createElement('div');
  d.style.color = ok ? '#86efac' : '#f87171';
  d.textContent = msg;
  el.appendChild(d);
  el.scrollTop = el.scrollHeight;
}

// ── Config save ───────────────────────────────────────────────
async function saveConfig() {
  const fd = new FormData();
  fd.append('width',   document.getElementById('cfg-w').value);
  fd.append('height',  document.getElementById('cfg-h').value);
  fd.append('spm',     document.getElementById('cfg-spm').value);
  fd.append('maxfeed', document.getElementById('cfg-mf').value);
  fd.append('accel',   document.getElementById('cfg-acc').value);
  const r = await fetch('/config', {method:'POST', body:fd});
  log('[config] ' + (r.ok ? 'saved' : 'error'), r.ok);
}
</script>
</body></html>)HTML"; }

    // ═══════════════════════════════════════════════════════════
    // WiFi setup page
    // ═══════════════════════════════════════════════════════════
    static String wifiPage() { return R"HTML(<!DOCTYPE html>
<html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi Setup</title>
<style>
body{background:#0c0e14;color:#e2e8f0;font-family:sans-serif;
     display:flex;align-items:center;justify-content:center;min-height:100vh}
.box{background:#1a1d2b;border:1px solid #252838;padding:2rem;border-radius:12px;width:320px}
h2{color:#a78bfa;margin-bottom:1.5rem}
label{display:block;font-size:.82rem;color:#64748b;margin:.6rem 0 .2rem}
input{width:100%;background:#0c0e14;border:1px solid #252838;color:#e2e8f0;
      border-radius:7px;padding:.5rem .7rem}
button{width:100%;background:#7c6af7;color:#fff;border:none;border-radius:7px;
       padding:.65rem;margin-top:1.2rem;cursor:pointer;font-size:.95rem;font-weight:600}
</style></head>
<body><div class="box">
<h2>📶 WiFi Setup</h2>
<form method="POST" action="/wifi">
  <label>Network (SSID)</label><input name="ssid" type="text" required autocomplete="off">
  <label>Password</label><input name="pass" type="password" autocomplete="off">
  <button type="submit">Connect &amp; Save</button>
</form>
</div></body></html>)HTML"; }
};
