/*
============================================================================
 esp_V8  --  ESP8266 bridge + browser UI (3-wheel kiwi/holonomic drive)
 Modes: Free Drive | Survey | Waypoint Patrol | Calibration | Log History
============================================================================
 SERIAL PROTOCOL (mirror of comm.h -- COMM_MAX_PAYLOAD = 64)
 Frame: [0xAA][len][cmd][payload...][crc8], len = 1 + payload_len

   0x01 CMD_SET_SPEED       ESP->STM32  12B [f32 vx_mps][f32 vy_mps][f32 omega_radps]
                                         body-frame velocity, +X=forward, +Y=left,
                                         omega positive=CCW -- holonomic, not
                                         rpm_left/right.
   0x04 CMD_STATUS          STM32->ESP  27B [f32x3 rpm][i32x3 enc][u8x3 duty_pct]
                                         (no battery field -- this chassis
                                         doesn't report one)
   0x0E CMD_TELEMETRY_LOG   STM32->ESP  61B (see main.c's SendTelemetryLog for the
                                         exact field layout)
   0x0B CMD_VERTEX_DATA     STM32->ESP  1B [status] or 2+36B [status][index][Vertex_t]
                                         (Vertex_t: 3 wheel encoders)
 Phase-3 (patrol) additions:
   0x11 CMD_NAV_CLEAR      ESP->STM32  empty
   0x12 CMD_NAV_ADD_WP     ESP->STM32  8B [f32 x][f32 y]
   0x13 CMD_NAV_START      ESP->STM32  1B [u8 speed_pct]
   0x14 CMD_NAV_STOP       ESP->STM32  empty
   0x15 CMD_NAV_STATUS     STM32->ESP  8B [state][target][count][fault][f32 dist]
   0x16 CMD_NAV_DEBUG      STM32->ESP  44B [state][target][wp_count][flags][fault]
                                        [settle_ticks][i16x3 d][f32 dist][f32 rate_dps]
                                        [f32 theta_deg][f32 bearing_deg][f32 err_deg]
                                        [f32x3 cmd_rpm]
 Field calibration (see the Calib tab):
   0x19 CMD_CAL_SPIN        ESP->STM32  12B [f32 deg_ccw_target][f32 deg_cw_target][f32 power_pct]
   0x1A CMD_CAL_ROLL        ESP->STM32  8B  [f32 dist_m][f32 power_pct]
   0x1B CMD_CAL_STATUS      STM32->ESP  14B [u8 type][u8 leg][u8 done][u8 reserved][f32 val1][f32 val2]
                                         type: 0=spin,1=roll. reserved: 1=aborted/timed out.
                                         spin: val1=gyro_deg_this_leg, val2=kin_deg_this_leg
                                         roll: val1=dist_estimated_m, val2=0
   0x1C CMD_CAL_APPLY_SPIN  ESP->STM32  8B  [f32 measured_ccw_deg][f32 measured_cw_deg]
   0x1D CMD_CAL_APPLY_ROLL  ESP->STM32  4B  [f32 d_true_m]
   0x1F CMD_CAL_STOP        ESP->STM32  empty
 Live waypoint capture (mirrors CMD_MARK_VERTEX for Patrol):
   0x20 CMD_NAV_MARK_WP    ESP->STM32  empty (capture LIVE odometry position as next waypoint)
   0x21 CMD_NAV_WP_DATA    STM32->ESP  10B [u8 ok][u8 index][f32 x][f32 y]

 WEBSOCKET PROTOCOL
 Browser -> ESP:
   0x01 [f32 vx][f32 vy][f32 omega] drive command (holonomic -- see CMD_SET_SPEED above)
   0x03 reset odom | 0x04 get odom | 0x05 [u8 mode] motion mode
   0x06 mark vertex | 0x07 clear vertices | 0x08 close survey | 0x09 get all vertices
   0x0A nav clear | 0x0B [f32 x][f32 y] | 0x0C [u8 pct] | 0x0D nav stop
   0x10 [f32 measured_ccw][f32 measured_cw] apply spin | 0x11 [f32 d_true] apply roll
   0x14 mark waypoint here (live odometry capture)
   0x15 [f32 deg_ccw][f32 deg_cw][f32 power_pct] start spin test
   0x16 [f32 dist_m][f32 power_pct] start roll test | 0x17 stop calibration
 ESP -> Browser:
   0x02 telemetry (status) | 0x03 odom | 0x04 vertex | 0x06 nav status | 0x07 nav debug
   0x08 cal status | 0x09 telem log | 0x0A nav wp
============================================================================
*/
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>

const char* ssid     = "RobotCar";
const char* password = "12345678";

ESP8266WebServer   server(80);
WebSocketsServer   webSocket(81);

/* ---------------- Serial protocol constants (mirror of comm.h) --------- */
#define SYNC_BYTE              0xAA
#define CMD_SET_SPEED          0x01
#define CMD_REQUEST_TELEMETRY  0x02
#define CMD_SET_PID            0x03
#define CMD_STATUS             0x04
#define CMD_HEARTBEAT          0x05
#define CMD_RESET_ODOM         0x07
#define CMD_GET_ODOM           0x08
#define CMD_ODOMETRY_DATA      0x09
#define CMD_MARK_VERTEX        0x0A
#define CMD_VERTEX_DATA        0x0B
#define CMD_CLEAR_VERTICES     0x0C
#define CMD_SET_MOTION_MODE    0x0D
#define CMD_TELEMETRY_LOG      0x0E
#define CMD_GET_ALL_VERTICES   0x0F
#define CMD_CLOSE_SURVEY       0x10

#define CMD_NAV_CLEAR          0x11
#define CMD_NAV_ADD_WP         0x12
#define CMD_NAV_START          0x13
#define CMD_NAV_STOP           0x14
#define CMD_NAV_STATUS         0x15
#define CMD_NAV_DEBUG          0x16

#define CMD_CAL_SPIN           0x19
#define CMD_CAL_ROLL            0x1A
#define CMD_CAL_STATUS          0x1B
#define CMD_CAL_APPLY_SPIN     0x1C
#define CMD_CAL_APPLY_ROLL     0x1D
#define CMD_CAL_STOP           0x1F

#define CMD_NAV_MARK_WP        0x20
#define CMD_NAV_WP_DATA        0x21

/* ESP -> browser WebSocket message types */
#define WS_TELEMETRY_TYPE      0x02
#define WS_ODOM_TYPE           0x03
#define WS_VERTEX_TYPE         0x04
#define WS_NAV_STATUS_TYPE     0x06
#define WS_NAV_DEBUG_TYPE      0x07
#define WS_CAL_TYPE            0x08
#define WS_TELEM_LOG_TYPE      0x09
#define WS_NAV_WP_TYPE         0x0A

/* Must match COMM_MAX_PAYLOAD in comm.h (verified: 64). */
#define MAX_PAYLOAD            64

/* ---------------- State -------------------------------------------------- */
static float odom_x = 0.0f, odom_y = 0.0f, odom_theta = 0.0f;
static bool  odom_report_ready = false;

uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

typedef enum { RX_SYNC = 0, RX_LENGTH, RX_CMD, RX_PAYLOAD, RX_CRC } RxState_t;
static RxState_t rx_state       = RX_SYNC;
static uint8_t   rx_payload_len = 0;
static uint8_t   rx_cmd         = 0;
static uint8_t   rx_payload[MAX_PAYLOAD];
static uint8_t   rx_payload_idx = 0;

static float    telem_rpm1      = 0.0f;
static float    telem_rpm2      = 0.0f;
static float    telem_rpm3      = 0.0f;
static int32_t  telem_enc1      = 0;
static int32_t  telem_enc2      = 0;
static int32_t  telem_enc3      = 0;
static uint8_t  telem_duty1     = 0;
static uint8_t  telem_duty2     = 0;
static uint8_t  telem_duty3     = 0;
static bool     telem_updated   = false;

static uint8_t  vert_last_payload[38];
static uint8_t  vert_last_len = 0;
static bool     vert_updated  = false;

static uint8_t  nav_status_payload[8];
static bool     nav_status_updated = false;

static uint8_t  nav_debug_payload[44];
static bool     nav_debug_updated = false;

static uint8_t  cal_status_payload[14];
static bool     cal_status_updated = false;

static uint8_t  nav_wp_payload[10];
static bool     nav_wp_updated = false;

/* ---------------- STM32 -> ESP handlers ---------------------------------- */
static void onVertexDataReceived(const uint8_t *payload, uint8_t len) {
  if (len > sizeof(vert_last_payload)) len = sizeof(vert_last_payload);
  memcpy(vert_last_payload, payload, len);
  vert_last_len = len;
  vert_updated  = true;
}
static void broadcastVertexData() {
  uint8_t buf[1 + 38];
  buf[0] = WS_VERTEX_TYPE;
  memcpy(buf + 1, vert_last_payload, vert_last_len);
  webSocket.broadcastBIN(buf, 1 + vert_last_len);
}
static void broadcastNavStatus() {
  uint8_t buf[1 + 8];
  buf[0] = WS_NAV_STATUS_TYPE;
  memcpy(buf + 1, nav_status_payload, 8);
  webSocket.broadcastBIN(buf, sizeof(buf));
}
static void broadcastNavDebug() {
  uint8_t buf[1 + 44];
  buf[0] = WS_NAV_DEBUG_TYPE;
  memcpy(buf + 1, nav_debug_payload, 44);
  webSocket.broadcastBIN(buf, sizeof(buf));
}
static void broadcastCalStatus() {
  uint8_t buf[1 + 14];
  buf[0] = WS_CAL_TYPE;
  memcpy(buf + 1, cal_status_payload, 14);
  webSocket.broadcastBIN(buf, sizeof(buf));
}
static void broadcastNavWp() {
  uint8_t buf[1 + 10];
  buf[0] = WS_NAV_WP_TYPE;
  memcpy(buf + 1, nav_wp_payload, 10);
  webSocket.broadcastBIN(buf, sizeof(buf));
}
static void onStatusReceived(const uint8_t *payload, uint8_t len) {
  if (len < 12) return;
  memcpy(&telem_rpm1, payload,      4);
  memcpy(&telem_rpm2, payload + 4,  4);
  memcpy(&telem_rpm3, payload + 8,  4);
  if (len >= 24) {
    memcpy(&telem_enc1, payload + 12, 4);
    memcpy(&telem_enc2, payload + 16, 4);
    memcpy(&telem_enc3, payload + 20, 4);
  }
  if (len >= 27) {
    telem_duty1 = payload[24];
    telem_duty2 = payload[25];
    telem_duty3 = payload[26];
  }
  telem_updated = true;
}

static void processSerial() {
  while (Serial.available()) {
    uint8_t byte = (uint8_t)Serial.read();
    switch (rx_state) {
      case RX_SYNC:
        if (byte == SYNC_BYTE) rx_state = RX_LENGTH;
        break;
      case RX_LENGTH:
        if (byte >= 1 && byte <= (MAX_PAYLOAD + 1)) {
          rx_payload_len = byte - 1;
          rx_state = RX_CMD;
        } else { rx_state = RX_SYNC; }
        break;
      case RX_CMD:
        rx_cmd = byte; rx_payload_idx = 0;
        rx_state = (rx_payload_len > 0) ? RX_PAYLOAD : RX_CRC;
        break;
      case RX_PAYLOAD:
        rx_payload[rx_payload_idx++] = byte;
        if (rx_payload_idx >= rx_payload_len) rx_state = RX_CRC;
        break;
      case RX_CRC: {
        uint8_t crc_buf[1 + 1 + MAX_PAYLOAD];
        crc_buf[0] = rx_payload_len + 1;
        crc_buf[1] = rx_cmd;
        if (rx_payload_len > 0) memcpy(&crc_buf[2], rx_payload, rx_payload_len);
        uint8_t calc = crc8(crc_buf, 2 + rx_payload_len);
        if (calc == byte && rx_cmd == CMD_STATUS)
          onStatusReceived(rx_payload, rx_payload_len);
        if (calc == byte && rx_cmd == CMD_ODOMETRY_DATA) {
          if (rx_payload_len >= 12) {
            memcpy(&odom_x,     rx_payload,     4);
            memcpy(&odom_y,     rx_payload + 4, 4);
            memcpy(&odom_theta, rx_payload + 8, 4);
            odom_report_ready = true;
          }
        }
        if (calc == byte && rx_cmd == CMD_NAV_DEBUG && rx_payload_len == 44) {
          memcpy(nav_debug_payload, rx_payload, 44);
          nav_debug_updated = true;
        }
        if (calc == byte && rx_cmd == CMD_CAL_STATUS && rx_payload_len == 14) {
          memcpy(cal_status_payload, rx_payload, 14);
          cal_status_updated = true;
        }
        if (calc == byte && rx_cmd == CMD_VERTEX_DATA) {
          onVertexDataReceived(rx_payload, rx_payload_len);
        }
        if (calc == byte && rx_cmd == CMD_NAV_STATUS) {
          if (rx_payload_len >= 8) {
            memcpy(nav_status_payload, rx_payload, 8);
            nav_status_updated = true;
          }
        }
        if (calc == byte && rx_cmd == CMD_NAV_WP_DATA && rx_payload_len == 10) {
          memcpy(nav_wp_payload, rx_payload, 10);
          nav_wp_updated = true;
        }
        if (calc == byte && rx_cmd == CMD_TELEMETRY_LOG && rx_payload_len == 61) {
          uint8_t ws_buf[62];
          ws_buf[0] = WS_TELEM_LOG_TYPE; // 0x09
          memcpy(ws_buf + 1, rx_payload, 61);
          webSocket.broadcastBIN(ws_buf, 62);
        }
        rx_state = RX_SYNC;
        break;
      }
      default: rx_state = RX_SYNC; break;
    }
  }
}

/* ---------------- ESP -> browser broadcasters ---------------------------- */
static void broadcastTelemetry() {
  uint8_t buf[28];
  buf[0] = WS_TELEMETRY_TYPE;
  memcpy(buf +  1, &telem_rpm1, 4);
  memcpy(buf +  5, &telem_rpm2, 4);
  memcpy(buf +  9, &telem_rpm3, 4);
  memcpy(buf + 13, &telem_enc1, 4);
  memcpy(buf + 17, &telem_enc2, 4);
  memcpy(buf + 21, &telem_enc3, 4);
  buf[25] = telem_duty1;
  buf[26] = telem_duty2;
  buf[27] = telem_duty3;
  webSocket.broadcastBIN(buf, sizeof(buf));
}
static void broadcastOdometry() {
  uint8_t buf[13];
  buf[0] = WS_ODOM_TYPE;
  memcpy(buf + 1, &odom_x,     4);
  memcpy(buf + 5, &odom_y,     4);
  memcpy(buf + 9, &odom_theta, 4);
  webSocket.broadcastBIN(buf, sizeof(buf));
}

/* ---------------- ESP -> STM32 senders ----------------------------------- */
void sendPacket(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  if (len > MAX_PAYLOAD) return;
  uint8_t buf[1 + 1 + 1 + MAX_PAYLOAD + 1];
  uint8_t idx = 0;
  buf[idx++] = SYNC_BYTE;
  buf[idx++] = 1 + len;
  buf[idx++] = cmd;
  if (len) { memcpy(&buf[idx], payload, len); idx += len; }
  buf[idx++] = crc8(&buf[1], idx - 1);
  Serial.write(buf, idx);
}
void sendSetVelocity(float vx, float vy, float omega) {
  uint8_t data[12];
  memcpy(data,     &vx,    4);
  memcpy(data + 4, &vy,    4);
  memcpy(data + 8, &omega, 4);
  sendPacket(CMD_SET_SPEED, data, 12);
}
/* Full-stick-deflection speed/turn-rate. Tuning constants for how the
 * joystick FEELS, not a precision calibration -- same role maxRPM
 * played for the old tank-drive mixing. Keep comfortably below the
 * chassis' real top speed (PID/duty clamps on the STM32 side handle
 * any excess gracefully either way) and adjust to taste once the
 * real robot is on the bench. */
static const float maxSpeedMps   = 0.4f;
static const float maxOmegaRadps = 3.0f;
static float cmdVx = 0.0f, cmdVy = 0.0f, cmdOmega = 0.0f;
void updateMotors() {
  sendSetVelocity(cmdVx, cmdVy, cmdOmega);
}

/* ---------------- Browser -> ESP dispatch -------------------------------- */
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_BIN: {
      if (length < 1) break;
      uint8_t msgType = payload[0];
      if (msgType == 0x01 && length >= 13) {
        memcpy(&cmdVx,    payload + 1, 4);
        memcpy(&cmdVy,    payload + 5, 4);
        memcpy(&cmdOmega, payload + 9, 4);
        updateMotors();
      }
      else if (msgType == 0x03) { sendPacket(CMD_RESET_ODOM, NULL, 0); }
      else if (msgType == 0x04) { sendPacket(CMD_GET_ODOM, NULL, 0); }
      else if (msgType == 0x05 && length >= 2) { sendPacket(CMD_SET_MOTION_MODE, payload + 1, 1); }
      else if (msgType == 0x06) { sendPacket(CMD_MARK_VERTEX, NULL, 0); }
      else if (msgType == 0x07) { sendPacket(CMD_CLEAR_VERTICES, NULL, 0); }
      else if (msgType == 0x08) { sendPacket(CMD_CLOSE_SURVEY, NULL, 0); }
      else if (msgType == 0x09) { sendPacket(CMD_GET_ALL_VERTICES, NULL, 0); }
      else if (msgType == 0x0A) { sendPacket(CMD_NAV_CLEAR, NULL, 0); }
      else if (msgType == 0x0B && length >= 9) { sendPacket(CMD_NAV_ADD_WP, payload + 1, 8); }
      else if (msgType == 0x0C && length >= 2) { sendPacket(CMD_NAV_START, payload + 1, 1); }
      else if (msgType == 0x0D) { sendPacket(CMD_NAV_STOP, NULL, 0); }
      else if (msgType == 0x10 && length >= 9)  { sendPacket(CMD_CAL_APPLY_SPIN, payload + 1, 8); }
      else if (msgType == 0x11 && length >= 5)  { sendPacket(CMD_CAL_APPLY_ROLL, payload + 1, 4); }
      else if (msgType == 0x14 ) { sendPacket(CMD_NAV_MARK_WP, NULL, 0); }
      else if (msgType == 0x15 && length >= 13) { sendPacket(CMD_CAL_SPIN, payload + 1, 12); }
      else if (msgType == 0x16 && length >= 9)  { sendPacket(CMD_CAL_ROLL, payload + 1, 8); }
      else if (msgType == 0x17) { sendPacket(CMD_CAL_STOP, NULL, 0); }
      break;
    }
    case WStype_DISCONNECTED:
      cmdVx = 0.0f; cmdVy = 0.0f; cmdOmega = 0.0f;
      updateMotors();
      break;
    default: break;
  }
}

/* ================= Browser UI (5 modes) ================================== */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no,viewport-fit=cover">
<title>RC Controller</title>
<style>
:root{
--bg:#080c10; --surface:#0d1117; --panel:#111820; --border:#1e2d3d;
--accent:#00c8ff; --accentG:#00ff9d; --danger:#ff3b5c; --warn:#ffb020;
--text:#c9d1d9; --muted:#4a5568;
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;touch-action:none;user-select:none;-webkit-user-select:none;}
html,body{height:100%;width:100%;overflow:hidden;background:var(--bg);color:var(--text);font-family:'SF Pro Display','Segoe UI',system-ui,sans-serif;}
body{display:flex;flex-direction:column;}
.hidden{display:none !important;}
.page{flex:1;display:flex;flex-direction:column;min-height:0;}
#surveyMode,#patrolMode,#calibMode,#logMode{overflow-y:auto;touch-action:pan-y;}
#calibMode, #calibMode *{touch-action:pan-y;}
#calibMode .sv-card{margin:6px 10px 0;padding:8px;}
#calibMode .cal-note{margin-bottom:6px;}
#calibMode .cal-row{margin-bottom:6px;}
#calibMode .pv-start,#calibMode .cal-apply{height:40px;margin-bottom:6px;}
.header{display:flex;justify-content:space-between;align-items:center;padding:10px 12px;background:var(--surface);border-bottom:1px solid var(--border);flex-shrink:0;gap:8px;}
.brand{display:flex;align-items:center;gap:8px;}
.brand-dot{width:8px;height:8px;border-radius:50%;background:var(--accent);box-shadow:0 0 8px var(--accent);animation:pulse 2s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:0.4}}
.brand-name{font-size:12px;font-weight:700;letter-spacing:2px;text-transform:uppercase;color:var(--accent);}
.mode-tabs{display:flex;gap:3px;background:var(--panel);border:1px solid var(--border);border-radius:10px;padding:3px;}
.mode-tab{border:none;background:transparent;color:var(--muted);font-size:10px;font-weight:800;letter-spacing:1px;text-transform:uppercase;padding:6px 9px;border-radius:7px;cursor:pointer;transition:all .2s;}
.mode-tab.active{background:var(--accent);color:#00131a;box-shadow:0 0 10px rgba(0,200,255,0.35);}
.hdr-right{display:flex;align-items:center;gap:6px;flex-shrink:0;}
.auto-badge{font-size:9px;font-weight:900;letter-spacing:1px;padding:4px 8px;border-radius:10px;background:#2a1a05;color:var(--warn);border:1px solid #3d2a10;animation:pulse 1.2s infinite;}
.conn-badge{font-size:11px;font-weight:600;padding:4px 10px;border-radius:12px;background:#1a0a0f;color:var(--danger);border:1px solid #3d1020;transition:all .3s;flex-shrink:0;}
.conn-badge.ok{background:#0a1a12;color:var(--accentG);border-color:#103d20;}
.telem{display:grid;grid-template-columns:repeat(4,1fr);gap:6px;padding:8px 10px;background:var(--surface);border-bottom:1px solid var(--border);flex-shrink:0;}
.tc{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:8px 6px;text-align:center;}
.tc-label{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1.5px;font-weight:600;}
.tc-val{font-size:18px;font-weight:800;margin-top:2px;font-variant-numeric:tabular-nums;}
.tc-unit{font-size:9px;color:var(--muted);}
.cL{color:var(--accentG);}.cR{color:var(--accent);}.cS{color:#aa80ff;}.cD{color:var(--danger);}
.odom-panel{background:var(--panel);border:1px solid var(--border);border-radius:12px;margin:8px 10px;padding:12px;flex-shrink:0;}
.odom-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;}
.odom-title{font-size:12px;font-weight:700;color:var(--accent);letter-spacing:1px;text-transform:uppercase;}
.odom-status{font-size:10px;padding:2px 8px;border-radius:10px;background:#1a1a1a;color:var(--muted);}
.odom-status.active{background:#0a2a1a;color:var(--accentG);}
.odom-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:12px;}
.odom-cell{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:8px;text-align:center;}
.odom-cell.highlight{border-color:var(--accent);background:rgba(0,200,255,0.05);}
.odom-label{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;}
.odom-val{font-size:16px;font-weight:800;margin-top:2px;font-variant-numeric:tabular-nums;}
.odom-unit{font-size:9px;color:var(--muted);}
.odom-actions{display:flex;gap:8px;}
.odom-btn{flex:1;height:40px;border-radius:8px;border:none;font-size:11px;font-weight:700;letter-spacing:0.5px;cursor:pointer;transition:all .15s;}
.odom-btn.origin{background:#0a2a1a;color:var(--accentG);border:1px solid #103d20;}
.odom-btn.report{background:#2a0a0a;color:var(--danger);border:1px solid #3d1020;}
.odom-btn:active{transform:scale(0.96);}
.main{flex:1;display:flex;flex-direction:column;align-items:center;justify-content:center;padding:12px;gap:16px;min-height:0;}
.joystick-wrap{position:relative;display:flex;align-items:center;justify-content:center;}
.joystick-zone{position:relative;width:min(280px,70vw);height:min(280px,70vw);border-radius:50%;background:radial-gradient(circle at center,#0d1a26 0%,#080c10 70%);border:2px solid var(--border);box-shadow:0 0 0 1px #1e2d3d,inset 0 0 40px rgba(0,0,0,0.6);cursor:crosshair;}
.joystick-zone::before,.joystick-zone::after{content:'';position:absolute;pointer-events:none;background:rgba(255,255,255,0.05);}
.joystick-zone::before{left:50%;top:10%;bottom:10%;width:1px;transform:translateX(-50%);}
.joystick-zone::after{top:50%;left:10%;right:10%;height:1px;transform:translateY(-50%);}
.ring{position:absolute;border-radius:50%;border:1px solid rgba(255,255,255,0.04);top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;}
.ring1{width:40%;height:40%;}.ring2{width:70%;height:70%;}
.knob{position:absolute;width:72px;height:72px;border-radius:50%;background:radial-gradient(circle at 38% 35%,#2a3a4a,#0d1117);border:2px solid var(--accent);box-shadow:0 0 20px rgba(0,200,255,0.2),0 4px 12px rgba(0,0,0,0.5);pointer-events:none;top:50%;left:50%;transform:translate(-50%,-50%);transition:border-color .2s,box-shadow .2s;display:flex;align-items:center;justify-content:center;}
.knob.active{border-color:#fff;box-shadow:0 0 28px rgba(0,200,255,0.4),0 4px 16px rgba(0,0,0,0.6);}
.knob-inner{width:24px;height:24px;border-radius:50%;background:var(--accent);opacity:0.6;box-shadow:0 0 10px var(--accent);}
.dir-ring{position:absolute;width:calc(min(280px,70vw) + 32px);height:calc(min(280px,70vw) + 32px);top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;}
.dir-arr{position:absolute;font-size:14px;opacity:0.15;transition:opacity .15s,color .15s;width:20px;height:20px;display:flex;align-items:center;justify-content:center;}
.dir-arr.N{top:0;left:50%;transform:translateX(-50%);}
.dir-arr.S{bottom:0;left:50%;transform:translateX(-50%);}
.dir-arr.W{left:0;top:50%;transform:translateY(-50%);}
.dir-arr.E{right:0;top:50%;transform:translateY(-50%);}
.dir-arr.lit{opacity:1;}
.dir-arr.fwd{color:var(--accentG);}.dir-arr.rev{color:var(--danger);}.dir-arr.str{color:var(--accent);}
.motor-bars{display:flex;gap:10px;align-items:center;}
.qrow{display:flex;gap:8px;flex-shrink:0;}
.qbtn{flex:1;height:44px;border-radius:10px;border:1px solid var(--border);background:var(--panel);color:var(--muted);font-size:11px;font-weight:700;letter-spacing:1px;text-transform:uppercase;cursor:pointer;transition:all .1s;}
.qbtn:active,.qbtn.held{transform:scale(0.94);filter:brightness(1.3);}
.qbtn.stop{border-color:#3d1020;color:var(--danger);background:#120508;}
.qbtn.fwd{border-color:#103d20;color:var(--accentG);background:#050f08;}
.qbtn.rev{border-color:#3d2010;color:var(--warn);}
.qbtn.pvt{border-color:#1e1a3d;color:#aa80ff;}
.sv-status{display:flex;align-items:center;gap:8px;padding:8px 12px;flex-shrink:0;}
.sv-state{font-size:10px;font-weight:800;letter-spacing:1.5px;padding:4px 10px;border-radius:10px;flex-shrink:0;}
.sv-state.rec{background:#0a2a1a;color:var(--accentG);border:1px solid #103d20;}
.sv-state.closed{background:#2a0a0a;color:var(--danger);border:1px solid #3d1020;}
.sv-state.idle{background:#111820;color:var(--muted);border:1px solid var(--border);}
.sv-state.work{background:#050a12;color:var(--accent);border:1px solid #10303d;}
.sv-state.fault{background:#2a0a0a;color:var(--danger);border:1px solid #3d1020;}
.sv-count{font-size:11px;font-weight:700;color:var(--text);font-variant-numeric:tabular-nums;flex-shrink:0;}
.sv-robot{margin-left:auto;font-size:10px;color:var(--muted);font-variant-numeric:tabular-nums;text-align:right;}
.map-wrap{position:relative;flex:1;margin:0 10px;min-height:160px;background:radial-gradient(circle at center,#0d1a26 0%,#080c10 75%);border:1px solid var(--border);border-radius:12px;overflow:hidden;}
.map-wrap canvas{position:absolute;left:0;top:0;}
.map-empty{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;text-align:center;font-size:11px;line-height:1.8;color:var(--muted);pointer-events:none;padding:20px;}
.map-legend{position:absolute;left:8px;bottom:6px;font-size:9px;color:var(--muted);background:rgba(8,12,16,0.72);padding:3px 8px;border-radius:6px;pointer-events:none;}
.sv-card{background:var(--panel);border:1px solid var(--border);border-radius:12px;margin:8px 10px 0;padding:10px;flex-shrink:0;}
.sv-mode-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px;}
.sv-mode{height:42px;border-radius:9px;border:1px solid var(--border);background:var(--surface);color:var(--muted);font-size:11px;font-weight:800;letter-spacing:1px;text-transform:uppercase;cursor:pointer;transition:all .15s;}
.sv-mode.active{border-color:var(--accent);color:var(--accent);background:rgba(0,200,255,0.07);box-shadow:0 0 12px rgba(0,200,255,0.15);}
.sv-mode:disabled{opacity:0.35;}
.sv-drive-row{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:8px;}
.sv-drive{height:48px;border-radius:9px;font-size:12px;font-weight:800;letter-spacing:1px;cursor:pointer;transition:all .1s;text-transform:uppercase;}
.sv-drive:active,.sv-drive.held{transform:scale(0.96);filter:brightness(1.35);}
.sv-drive:disabled{opacity:0.35;}
.sv-drive.fwd{background:#050f08;color:var(--accentG);border:1px solid #103d20;}
.sv-drive.rev{background:#120508;color:var(--danger);border:1px solid #3d1020;}
.sv-drive.ccw{background:#050a12;color:var(--accent);border:1px solid #10303d;}
.sv-drive.cw{background:#0d0512;color:#aa80ff;border:1px solid #2a1a3d;}
.sv-drive-hint{height:36px;display:flex;align-items:center;justify-content:center;font-size:9px;color:var(--muted);border:1px dashed var(--border);border-radius:9px;margin-bottom:8px;letter-spacing:1.5px;text-transform:uppercase;text-align:center;}
.sv-speed-row{display:flex;align-items:center;gap:10px;}
.sv-speed-label{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1.5px;font-weight:700;flex-shrink:0;}
.sv-speed-val{font-size:12px;font-weight:800;color:var(--accent);width:42px;text-align:right;font-variant-numeric:tabular-nums;flex-shrink:0;}
input[type=range]{flex:1;touch-action:auto;-webkit-appearance:none;appearance:none;height:4px;border-radius:2px;background:var(--border);outline:none;min-width:40px;}
input[type=range]:disabled{opacity:0.35;}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;width:22px;height:22px;border-radius:50%;background:var(--accent);border:2px solid #00384a;box-shadow:0 0 8px rgba(0,200,255,0.5);cursor:pointer;}
.sv-mark,.pv-start{width:100%;height:52px;border-radius:10px;border:1px solid #103d20;background:linear-gradient(180deg,#0c2a1c,#07130c);color:var(--accentG);font-size:14px;font-weight:900;letter-spacing:2px;text-transform:uppercase;cursor:pointer;transition:all .12s;margin-bottom:8px;}
.sv-mark:active,.pv-start:active{transform:scale(0.97);filter:brightness(1.4);}
.sv-mark:disabled,.pv-start:disabled{opacity:0.35;}
.sv-actions{display:grid;grid-template-columns:1fr 1.3fr 1fr;gap:8px;}
.sv-act{height:40px;border-radius:9px;font-size:10px;font-weight:800;letter-spacing:1px;text-transform:uppercase;cursor:pointer;transition:all .1s;}
.sv-act:active{transform:scale(0.95);}
.sv-act:disabled{opacity:0.35;}
.sv-act.clear{background:var(--surface);color:var(--warn);border:1px solid #3d2010;}
.sv-act.close{background:#050a12;color:var(--accent);border:1px solid #10303d;}
.sv-act.new{background:#0d0512;color:#aa80ff;border:1px solid #2a1a3d;}
.sv-act.undo{background:#0d0512;color:#aa80ff;border:1px solid #2a1a3d;}
.sv-act.stop{background:#120508;color:var(--danger);border:1px solid #3d1020;}
.sv-act.origin{background:#0a2a1a;color:var(--accentG);border:1px solid #103d20;}
/* Calib page */
.cal-title{font-size:11px;font-weight:800;letter-spacing:1.5px;color:var(--accent);text-transform:uppercase;margin-bottom:8px;}
.cal-note{font-size:9px;color:var(--muted);line-height:1.6;margin-bottom:10px;}
.cal-row{display:flex;align-items:center;gap:8px;margin-bottom:8px;}
.cal-row label{font-size:9px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;flex-shrink:0;width:110px;}
.cal-row input[type=number]{flex:1;background:var(--surface);border:1px solid var(--border);color:var(--text);border-radius:6px;padding:8px;font-size:13px;touch-action:auto;user-select:text;-webkit-user-select:text;}
.cal-apply{width:100%;height:42px;border-radius:9px;border:1px solid #10303d;background:#050a12;color:var(--accent);font-size:11px;font-weight:800;letter-spacing:1px;text-transform:uppercase;cursor:pointer;margin-bottom:8px;}
.cal-apply:disabled{opacity:0.35;}
.cal-result{font-size:10px;color:var(--accentG);min-height:14px;margin-bottom:4px;font-variant-numeric:tabular-nums;}
/* Results overlay */
.results-overlay{position:fixed;inset:0;z-index:50;background:rgba(4,6,9,0.72);display:flex;align-items:center;justify-content:center;padding:18px;}
.results-card{width:100%;max-width:420px;max-height:86vh;overflow-y:auto;touch-action:pan-y;background:var(--surface);border:1px solid var(--border);border-radius:14px;padding:16px;}
.results-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;}
.results-title{font-size:13px;font-weight:800;letter-spacing:2px;text-transform:uppercase;color:var(--accent);}
.results-close{background:var(--panel);border:1px solid var(--border);color:var(--muted);width:30px;height:30px;border-radius:8px;font-size:13px;cursor:pointer;}
.res-areas{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px;}
.res-area-box{border-radius:10px;padding:10px;text-align:center;border:1px solid var(--border);}
.res-area-box.raw{background:rgba(0,200,255,0.06);border-color:#10303d;}
.res-area-box.adj{background:rgba(0,255,157,0.06);border-color:#103d20;}
.res-lab{font-size:8.5px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;font-weight:700;line-height:1.4;}
.res-big{font-size:22px;font-weight:900;margin-top:3px;font-variant-numeric:tabular-nums;}
.res-area-box.raw .res-big{color:var(--accent);}
.res-area-box.adj .res-big{color:var(--accentG);}
.res-unit{font-size:9px;color:var(--muted);}
.res-row{display:flex;justify-content:space-between;align-items:center;padding:7px 2px;border-bottom:1px solid rgba(30,45,61,0.5);font-size:11px;color:var(--muted);gap:8px;}
.res-row b{color:var(--text);font-variant-numeric:tabular-nums;font-weight:700;text-align:right;}
.res-warn{font-size:12px;color:var(--warn);text-align:center;padding:14px 4px;line-height:1.6;}
.results-note{margin-top:10px;font-size:9px;color:var(--muted);line-height:1.5;text-align:center;}
/* Toast */
.toast{position:fixed;top:52px;left:50%;transform:translateX(-50%) translateY(-8px);background:var(--surface);border:1px solid var(--border);color:var(--text);font-size:11px;font-weight:700;padding:8px 16px;border-radius:10px;opacity:0;pointer-events:none;transition:all .25s;z-index:60;max-width:86vw;text-align:center;}
.toast.show{opacity:1;transform:translateX(-50%) translateY(0);}
.toast.warn{border-color:#3d2010;color:var(--warn);}
.toast.err{border-color:#3d1020;color:var(--danger);}
.toast.ok{border-color:#103d20;color:var(--accentG);}

/* --- LIVE LOG OVERLAY & HISTORY TAB --- */
.log-toggle { position: fixed; bottom: 14px; right: 14px; z-index: 99; width: 52px; height: 52px; border-radius: 50%;background: #0a2a1a; color: #00ff9d; border: 2px solid #103d20;font-size: 12px; font-weight: 800; letter-spacing: 1px; cursor: pointer;
    box-shadow: 0 4px 15px rgba(0,255,157,0.2); display: flex; align-items: center; justify-content: center; }
.log-overlay {
    position: fixed; top: 42px; left: 0; right: 0; height: 55%; 
    background: rgba(13, 17, 23, 0.95); backdrop-filter: blur(16px); -webkit-backdrop-filter: blur(16px);
    border-bottom: 1px solid var(--border); z-index: 45; display: flex; flex-direction: column;
    padding: 12px; box-shadow: 0 15px 40px rgba(0,0,0,0.6);
    transform: translateY(0); opacity: 1; transition: transform 0.35s cubic-bezier(0.4, 0, 0.2, 1), opacity 0.3s;
}
.log-overlay.hidden { transform: translateY(-110%); opacity: 0; pointer-events: none; }
.log-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
.log-title { font-size: 11px; font-weight: 800; letter-spacing: 2px; color: var(--accent); text-transform: uppercase; }
.log-controls { display: flex; gap: 8px; }
.log-btn { background: var(--panel); border: 1px solid var(--border); color: var(--text); font-size: 9px; font-weight: 700; padding: 4px 10px; border-radius: 6px; cursor: pointer; letter-spacing: 1px; }
.log-btn.close { color: var(--danger); border-color: #3d1020; }
.log-btn.active { background: var(--accent); color: #00131a; border-color: var(--accent); }
.log-metrics { display: grid; grid-template-columns: repeat(4, 1fr); gap: 8px; margin-bottom: 12px; }
.metric { background: rgba(255,255,255,0.03); border: 1px solid var(--border); border-radius: 8px; padding: 8px 4px; text-align: center; }
.m-label { display: block; font-size: 8px; color: var(--muted); letter-spacing: 1px; margin-bottom: 4px; font-weight: 700; }
.m-val { display: block; font-size: 18px; font-weight: 800; font-variant-numeric: tabular-nums; }

.log-stream-wrap { flex: 1; background: rgba(0,0,0,0.2); border: 1px solid var(--border); border-radius: 8px; overflow: hidden; min-height: 100px; display: flex; flex-direction: column; }
.log-stream, #fullLog { flex: 1; overflow-y: auto; padding: 8px; font-family: monospace; font-size: 11px; line-height: 1.4; color: #c9d1d9; }
#fullLog { background: var(--surface); margin: 0; border-radius: 0; }

.log-row { display: grid; grid-template-columns: 80px 1fr 1fr 1fr 1fr; gap: 8px; border-bottom: 1px solid rgba(255,255,255,0.05); padding: 2px 4px; }
.log-row.header { font-weight: bold; color: var(--muted); border-bottom: 1px solid var(--border); margin-bottom: 4px; position: sticky; top: 0; background: rgba(13, 17, 23, 0.95); z-index: 1; }
.log-page-header { display: flex; justify-content: space-between; align-items: center; padding: 10px 12px; border-bottom: 1px solid var(--border); }

@media (max-width:400px){.brand-name{display:none;}.mode-tab{padding:6px 7px;font-size:9px;}}
</style>
</head>
<body>
<div class="header">
  <div class="brand"><div class="brand-dot"></div><div class="brand-name">RC Control</div></div>
  <div class="mode-tabs">
    <button class="mode-tab active" id="tabFree">Free Drive</button>
    <button class="mode-tab" id="tabSurvey">Survey</button>
    <button class="mode-tab" id="tabPatrol">Patrol</button>
    <button class="mode-tab" id="tabCalib">Calib</button>
    <button class="mode-tab" id="tabLog">Log</button>
  </div>
  <div class="hdr-right">
    <span class="auto-badge hidden" id="autoBadge">AUTO</span>
    <div class="conn-badge" id="connBadge">Offline</div>
  </div>
</div>
<div class="telem" id="telemPanel">
  <div class="tc"><div class="tc-label">RPM 1</div><div class="tc-val cL" id="rpm1">&mdash;</div><div class="tc-unit">rpm</div></div>
  <div class="tc"><div class="tc-label">RPM 2</div><div class="tc-val cR" id="rpm2">&mdash;</div><div class="tc-unit">rpm</div></div>
  <div class="tc"><div class="tc-label">RPM 3</div><div class="tc-val cS" id="rpm3">&mdash;</div><div class="tc-unit">rpm</div></div>
  <div class="tc"><div class="tc-label">Heading</div><div class="tc-val cS" id="headingVal">0</div><div class="tc-unit">deg</div></div>
  <div class="tc" style="border-color:#2a1a3d;"><div class="tc-label">Enc 1</div><div class="tc-val" style="color:#aa80ff;font-size:14px;" id="enc1">0</div><div class="tc-unit">cnt</div></div>
  <div class="tc" style="border-color:#2a1a3d;"><div class="tc-label">Enc 2</div><div class="tc-val" style="color:#aa80ff;font-size:14px;" id="enc2">0</div><div class="tc-unit">cnt</div></div>
  <div class="tc" style="border-color:#2a1a3d;"><div class="tc-label">Enc 3</div><div class="tc-val" style="color:#aa80ff;font-size:14px;" id="enc3">0</div><div class="tc-unit">cnt</div></div>
  <div class="tc" style="border-color:#3d1a2a;"><div class="tc-label">Duty 1</div><div class="tc-val cD" id="duty1">0</div><div class="tc-unit">%</div></div>
  <div class="tc" style="border-color:#3d1a2a;"><div class="tc-label">Duty 2</div><div class="tc-val cD" id="duty2">0</div><div class="tc-unit">%</div></div>
  <div class="tc" style="border-color:#3d1a2a;"><div class="tc-label">Duty 3</div><div class="tc-val cD" id="duty3">0</div><div class="tc-unit">%</div></div>
</div>
<!-- ============ FREE DRIVE ============ -->
<div id="freeMode" class="page">
  <div class="odom-panel" id="odomPanel">
    <div class="odom-header"><span class="odom-title">Odometry</span><span class="odom-status" id="odomStatus">Idle</span></div>
    <div class="odom-grid">
      <div class="odom-cell"><div class="odom-label">Delta X</div><div class="odom-val" id="valX">0.00</div><div class="odom-unit">m</div></div>
      <div class="odom-cell"><div class="odom-label">Delta Y</div><div class="odom-val" id="valY">0.00</div><div class="odom-unit">m</div></div>
      <div class="odom-cell"><div class="odom-label">Heading</div><div class="odom-val" id="valTheta">0&deg;</div><div class="odom-unit">deg</div></div>
      <div class="odom-cell highlight"><div class="odom-label">Displacement</div><div class="odom-val" id="valDisp">0.00</div><div class="odom-unit">m</div></div>
    </div>
    <div class="odom-actions">
      <button class="odom-btn origin" id="btnSetOrigin">Set Origin</button>
      <button class="odom-btn report" id="btnEndReport">End &amp; Report</button>
    </div>
  </div>
  <div class="main">
    <div class="motor-bars">
      <div class="joystick-wrap">
        <div class="joystick-zone" id="jZone"><div class="ring ring1"></div><div class="ring ring2"></div><div class="knob" id="knob"><div class="knob-inner"></div></div></div>
        <div class="dir-ring"><div class="dir-arr N" id="dN">&#9650;</div><div class="dir-arr S" id="dS">&#9660;</div><div class="dir-arr W" id="dW">&#9664;</div><div class="dir-arr E" id="dE">&#9654;</div></div>
      </div>
    </div>
    <div class="qrow">
      <button class="qbtn stop" id="btnStop">Stop</button>
      <button class="qbtn fwd"  id="btnFwd">Fwd</button>
      <button class="qbtn rev"  id="btnRev">Rev</button>
      <button class="qbtn pvt"  id="btnCcw">CCW</button>
      <button class="qbtn pvt"  id="btnCw">CW</button>
    </div>
  </div>
</div>
<!-- ============ SURVEY ============ -->
<div id="surveyMode" class="page hidden">
  <div class="sv-status"><span class="sv-state rec" id="svState">RECORDING</span><span class="sv-count" id="svCount">0 / 64</span><span class="sv-robot" id="svRobot">x &mdash;, y &mdash;, &theta; &mdash;</span></div>
  <div class="map-wrap" id="mapWrap"><canvas id="mapCanvas"></canvas>
    <div class="map-empty" id="mapEmpty">No vertices marked yet.<br>Select a motion mode, drive to a corner,<br>then tap MARK VERTEX.</div>
    <div class="map-legend hidden" id="mapLegend">solid: measured &middot; green dashed: Bowditch-adjusted</div>
  </div>
  <div class="sv-card">
    <div class="sv-mode-row"><button class="sv-mode" id="btnModeStraight">&#8593; Straight</button><button class="sv-mode" id="btnModeRotate">&#10227; Rotate</button></div>
    <div class="sv-drive-row hidden" id="driveStraight"><button class="sv-drive fwd" id="btnSvFwd">&#9650; Forward</button><button class="sv-drive rev" id="btnSvRev">&#9660; Reverse</button></div>
    <div class="sv-drive-row hidden" id="driveRotate"><button class="sv-drive ccw" id="btnSvCcw">&#10226; CCW</button><button class="sv-drive cw" id="btnSvCw">&#10227; CW</button></div>
    <div class="sv-drive-hint" id="driveHint">Select a motion mode to drive</div>
    <div class="sv-speed-row"><span class="sv-speed-label">Speed</span><input type="range" id="speedSlider" min="10" max="100" value="80"><span class="sv-speed-val" id="speedVal">80%</span></div>
  </div>
  <div class="sv-card" style="margin-bottom:10px">
    <button class="sv-mark" id="btnMarkVertex">&#9670; Mark Vertex</button>
    <div class="sv-actions"><button class="sv-act clear" id="btnSvClear">Clear</button><button class="sv-act close" id="btnSvClose">Close Survey</button><button class="sv-act new" id="btnSvNew">New Survey</button></div>
  </div>
</div>
<!-- ============ PATROL ============ -->
<div id="patrolMode" class="page hidden">
  <div class="sv-status">
    <span class="sv-state idle" id="pvState">IDLE</span>
    <span class="sv-count" id="pvCount">0 / 16</span>
    <span class="sv-robot" id="pvInfo">target — · dist —</span>
  </div>
  <div class="map-wrap" id="patrolMapWrap">
    <canvas id="patrolCanvas"></canvas>
    <div class="map-empty" id="patrolEmpty">Tap map to add waypoints (max 16).<br>Drive from FREE DRIVE, then MARK CURRENT POSITION.</div>
    <div class="map-legend" id="patrolLegend">tap map: add waypoint · loop 1→2→…→1</div>
  </div>
  <div class="sv-card" style="padding:8px 10px;">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:6px;">
      <span style="font-size:9px;font-weight:800;letter-spacing:1px;color:var(--muted);text-transform:uppercase;">Live Nav Debug</span>
      <span style="font-size:9px;font-weight:700;color:var(--accent);" id="navDbgState">--</span>
    </div>
    <div style="display:grid;grid-template-columns:repeat(4,1fr);gap:6px;">
      <div style="text-align:center;"><div style="font-size:8px;color:var(--muted);">RATE</div><div style="font-size:12px;font-weight:700;" id="navDbgRate">--</div></div>
      <div style="text-align:center;"><div style="font-size:8px;color:var(--muted);">ERR</div><div style="font-size:12px;font-weight:700;" id="navDbgErr">--</div></div>
      <div style="text-align:center;"><div style="font-size:8px;color:var(--muted);">DIST</div><div style="font-size:12px;font-weight:700;" id="navDbgDist">--</div></div>
      <div style="text-align:center;"><div style="font-size:8px;color:var(--muted);">SETTLE</div><div style="font-size:12px;font-weight:700;" id="navDbgSettle">--</div></div>
    </div>
    <div style="text-align:center;margin-top:4px;font-size:9px;color:var(--muted);">Coasting: <span id="navDbgCoast" style="font-weight:700;">--</span></div>
  </div>
  <div class="sv-card"><div class="sv-speed-row"><span class="sv-speed-label">Speed</span><input type="range" id="patrolSpeed" min="10" max="100" value="95"><span class="sv-speed-val" id="patrolSpeedVal">95%</span></div></div>
  <div class="sv-card" style="margin-bottom:10px">
    <button class="pv-start" id="btnPatrolStart">▶ Start Patrol</button>
    <div class="sv-actions">
      <button class="sv-act undo" id="btnPatrolUndo">Undo</button>
      <button class="sv-act stop" id="btnPatrolStop">Stop</button>
      <button class="sv-act clear" id="btnPatrolClear">Clear</button>
    </div>
    <div class="sv-actions" style="margin-top:8px;grid-template-columns:1fr;">
      <button class="sv-act origin" id="btnPvMarkHere">◆ Mark Current Position</button>
    </div>
  </div>
</div>
<!-- ============ CALIB ============ -->
<div id="calibMode" class="page hidden">
  <div class="sv-status">
    <span class="sv-state idle" id="calState">IDLE</span>
    <span class="sv-robot" id="calInfo">ready</span>
    <button class="sv-act stop" id="btnCalStop" style="flex:0 0 70px;">Stop</button>
  </div>

  <div class="sv-card">
    <div class="cal-row"><label>Test power (%)</label><input type="number" id="calPowerPct" min="10" max="100" step="5" value="60"></div>
  </div>

  <div class="sv-card">
    <div class="cal-title">Gyro Scale &amp; Robot Radius &mdash; Spin Test</div>
    <div class="cal-note">1) Mark a chassis reference point and its floor position. 2) SPIN NOW (drives CCW then CW automatically, each leg to the target below, then stops itself). 3) Measure the ACTUAL degrees rotated each leg (protractor/floor marks). 4) APPLY.</div>
    <div class="cal-row"><label>CCW turns</label><input type="number" id="calTurnsCCW" min="1" max="10" step="1" value="3"></div>
    <div class="cal-row"><label>CW turns</label><input type="number" id="calTurnsCW" min="1" max="10" step="1" value="3"></div>
    <button class="pv-start" id="btnCalSpin">Spin Now</button>
    <div class="cal-row"><label>Measured CCW (deg)</label><input type="number" id="calMeasuredCcw" step="0.5" value="0"></div>
    <div class="cal-row"><label>Measured CW (deg)</label><input type="number" id="calMeasuredCw" step="0.5" value="0"></div>
    <button class="cal-apply" id="btnCalApplySpin" disabled>Apply Gyro Scale + Robot Radius</button>
    <div class="cal-result" id="calSpinResult"></div>
  </div>

  <div class="sv-card" style="margin-bottom:10px">
    <div class="cal-title">Wheel Radius &mdash; Roll Test</div>
    <div class="cal-note">1) Mark a straight line on the OPERATING surface. 2) Robot at start. 3) ROLL NOW (stops itself at the target distance). 4) Measure the TRUE travelled distance. 5) APPLY.</div>
    <div class="cal-row"><label>Target dist (m)</label><input type="number" id="calRollTarget" min="0.1" step="0.1" value="2.0"></div>
    <button class="pv-start" id="btnCalRoll">Roll Now</button>
    <div class="cal-row"><label>True dist (m)</label><input type="number" id="calRollTrue" step="0.005" value="2.0"></div>
    <button class="cal-apply" id="btnCalApplyRoll" disabled>Apply Wheel Radius</button>
    <div class="cal-result" id="calRollResult"></div>
  </div>
</div>

<!-- ============ LOG HISTORY TAB ============ -->
<div id="logMode" class="page hidden">
  <div class="log-page-header">
      <span class="log-title" style="color:var(--accent);">LOG HISTORY</span>
      <button id="btnFullLogClear" class="log-btn">CLEAR HISTORY</button>
  </div>
  <div id="fullLog"></div>
</div>

<!-- Survey results overlay -->
<div class="results-overlay hidden" id="resultsOverlay">
  <div class="results-card">
    <div class="results-head"><span class="results-title">Survey Results</span><button class="results-close" id="btnResultsClose">&#10005;</button></div>
    <div id="resultsBody"></div>
    <div class="results-note">Both values are always reported: RAW reflects raw sensor quality; Bowditch distributes the closure error proportionally along the traverse.</div>
  </div>
</div>
<div class="toast" id="toast"></div>

<!-- LIVE LOG OVERLAY -->
<button class="log-toggle" id="btnLogToggle">LOG</button>
<div id="logOverlay" class="log-overlay hidden">
    <div class="log-header">
        <span class="log-title">LIVE SENSOR FUSION</span>
        <div class="log-controls">
            <button id="btnLogPause" class="log-btn">PAUSE</button>
            <button id="btnLogClear" class="log-btn">CLEAR</button>
            <button id="btnLogClose" class="log-btn close">&#10005;</button>
        </div>
    </div>
    <div class="log-metrics">
        <div class="metric"><span class="m-label">Δ LEFT</span><span class="m-val cL" id="logDL">0</span></div>
        <div class="metric"><span class="m-label">Δ RIGHT</span><span class="m-val cR" id="logDR">0</span></div>
        <div class="metric"><span class="m-label">GYRO RAW</span><span class="m-val cB" id="logGyro">0.0</span></div>
        <div class="metric"><span class="m-label">GYRO CORR</span><span class="m-val cS" id="logGyroCorr">0.0</span></div>
        <div class="metric"><span class="m-label">HD (SLIP)</span><span class="m-val cD" id="logHD">0.0</span></div>
        <div class="metric"><span class="m-label">BIAS</span><span class="m-val" id="logBias">0.0</span></div>
        <div class="metric"><span class="m-label">CALIBRATED</span><span class="m-val" id="logCal">--</span></div>
        <div class="metric"><span class="m-label">MAX dt</span><span class="m-val" id="logMaxDt">--</span></div>
    </div>
    <div class="log-stream-wrap">
        <div class="log-stream" id="logStream">
            <div class="log-row header">
                <span>TIME</span><span>Δ L</span><span>Δ R</span><span>GYRO</span><span>SLIP</span>
            </div>
        </div>
    </div>
</div>

<script>
/* ============ WebSocket ============ */
let ws, connected = false;
const connBadge = document.getElementById('connBadge');
const rpm1El = document.getElementById('rpm1');
const rpm2El = document.getElementById('rpm2');
const rpm3El = document.getElementById('rpm3');
const headingEl = document.getElementById('headingVal');
let odomX = 0, odomY = 0, odomTheta = 0;
let odomActive = false;
const elOdomStatus = document.getElementById('odomStatus');
const elValX = document.getElementById('valX');
const elValY = document.getElementById('valY');
const elValTheta = document.getElementById('valTheta');
const elValDisp = document.getElementById('valDisp');
const SURVEY_MAX = 64;
let appMode = 'free';
let vertices = [];
let surveyClosed = false;
let surveyDriveMode = 0;
let surveyHolding = false;
let lastResults = null;
let robotPos = { x:0, y:0, theta:0, valid:false };
let posPollTimer = null;
const PATROL_MAX = 16;
let navWP = [];
let navActive = false;
let navState = 0;
let navTarget = 0;
let navCountSTM = 0;
let navFault = 0;
let navDist = 0;
let patrolPollTimer = null;
let calActive = false;
let calSpinLegGyroDeg = [0, 0];   /* gyro-measured deg per leg (0=CCW,1=CW), for operator reference only */
const WS_NAV_DEBUG_TYPE = 0x07;
const WS_CAL_TYPE = 0x08;
const WS_NAV_WP_TYPE = 0x0A;
let pvDriveMode = 0;

function updateOdomUI(){
  elValX.textContent = odomX.toFixed(2);
  elValY.textContent = odomY.toFixed(2);
  elValTheta.textContent = (odomTheta*180/Math.PI).toFixed(0)+'\u00B0';
  headingEl.textContent = (odomTheta*180/Math.PI).toFixed(0);   
  const disp = Math.sqrt(odomX*odomX+odomY*odomY);
  elValDisp.textContent = disp.toFixed(2);
}
function connect(){
  ws = new WebSocket('ws://'+location.hostname+':81');
  ws.binaryType = 'arraybuffer';
  ws.onopen = ()=>{ connected=true; connBadge.textContent='Online'; connBadge.className='conn-badge ok'; };
  ws.onclose = ()=>{ connected=false; connBadge.textContent='Offline'; connBadge.className='conn-badge'; setJoy(0,0); sendCmd(); setTimeout(connect,2000); };
  ws.onmessage = (evt)=>{
    if(!(evt.data instanceof ArrayBuffer)) return;
    const dv = new DataView(evt.data);
    const mt = dv.getUint8(0);
    if(mt===0x02 && dv.byteLength>=28){
      rpm1El.textContent = dv.getFloat32(1,true).toFixed(0);
      rpm2El.textContent = dv.getFloat32(5,true).toFixed(0);
      rpm3El.textContent = dv.getFloat32(9,true).toFixed(0);
      document.getElementById('enc1').textContent=dv.getInt32(13,true);
      document.getElementById('enc2').textContent=dv.getInt32(17,true);
      document.getElementById('enc3').textContent=dv.getInt32(21,true);
      document.getElementById('duty1').textContent=dv.getUint8(25);
      document.getElementById('duty2').textContent=dv.getUint8(26);
      document.getElementById('duty3').textContent=dv.getUint8(27);
    }
    else if(mt===0x03 && dv.byteLength>=13){
      odomX=dv.getFloat32(1,true); odomY=dv.getFloat32(5,true); odomTheta=dv.getFloat32(9,true);
      updateOdomUI(); odomActive=false;
      elOdomStatus.textContent='Reported'; elOdomStatus.className='odom-status';
      robotPos.x=odomX; robotPos.y=odomY; robotPos.theta=odomTheta; robotPos.valid=true;
      if(appMode==='survey'){ updateRobotReadout(); drawMap(); }
      if(appMode==='patrol'){ drawPatrolMap(); }
    }
    else if(mt===0x04){ handleVertexMessage(dv); }
    else if(mt===0x06 && dv.byteLength>=9){ handleNavStatus(dv); }
    else if(mt===WS_NAV_WP_TYPE && dv.byteLength>=11){ handleNavWpData(dv); }
    else if (mt === 0x09 && dv.byteLength >= 62) { // WS_TELEM_LOG_TYPE -- see main.c's SendTelemetryLog for the 61-byte STM32 layout this mirrors (browser offset = STM32 offset + 1)
        const dL = dv.getInt16(17, true);
        const dR = dv.getInt16(19, true);
        const gyroRaw = dv.getFloat32(23, true);
        const gyroBias = dv.getFloat32(27, true);
        const hdSigned = dv.getFloat32(47, true);
        const isCal = dv.getUint8(56);
        const gyroScale = dv.getFloat32(57, true);
        const maxDtMs = dv.getUint8(61);
        pushLogData(dL, dR, gyroRaw, hdSigned, gyroBias, isCal, gyroScale, maxDtMs);
    }
    else if(mt===WS_NAV_DEBUG_TYPE){ handleNavDebug(dv); }
    else if(mt===WS_CAL_TYPE && dv.byteLength>=15){ handleCalStatus(dv); }
  };
}
/* ============ Free drive (holonomic) ============ */
/* Full-stick-deflection speed / full-hold turn-rate. Mirrors the
 * ESP firmware's own maxSpeedMps/maxOmegaRadps (see sendSetVelocity
 * above) -- an operator-feel tuning value, not a precision
 * calibration; the two can be re-tuned independently by feel. */
const maxSpeedMps = 0.4, maxOmegaRadps = 3.0;
let joyThr=0, joyStr=0;   /* joystick axes, -100..100: Y=fwd/back, X=strafe */
let cmdOmega=0;           /* rad/s, from the CCW/CW hold buttons only -- independent of the joystick */
function setJoy(thr,str){
  joyThr=Math.max(-100,Math.min(100,thr));
  joyStr=Math.max(-100,Math.min(100,str));
  updateKnobUI(); updateDirArrows();
}
function updateKnobUI(){
  const zone=document.getElementById('jZone'), knob=document.getElementById('knob');
  const r=zone.offsetWidth/2-2, maxR=r-36;
  knob.style.left=(r+(joyStr/100)*maxR)+'px';
  knob.style.top =(r-(joyThr/100)*maxR)+'px';
  knob.style.transform='translate(-50%,-50%)';
  knob.classList.toggle('active', joyThr!==0||joyStr!==0);
}
function updateDirArrows(){
  document.getElementById('dN').className='dir-arr N'+(joyThr> 5?' lit fwd':'');
  document.getElementById('dS').className='dir-arr S'+(joyThr<-5?' lit rev':'');
  document.getElementById('dE').className='dir-arr E'+(joyStr> 5?' lit str':'');
  document.getElementById('dW').className='dir-arr W'+(joyStr<-5?' lit str':'');
}
/* Shared low-level sender for every drive command in the app (free
 * drive, survey manual drive) -- one place building the one correct
 * 13-byte [0x01][f32 vx][f32 vy][f32 omega] frame. */
function sendVelocity(vx,vy,omega){
  if(!connected||ws.readyState!==WebSocket.OPEN) return;
  const b=new ArrayBuffer(13), dv=new DataView(b);
  dv.setUint8(0,0x01);
  dv.setFloat32(1,vx,true); dv.setFloat32(5,vy,true); dv.setFloat32(9,omega,true);
  ws.send(b);
}
function sendCmd(){
  /* Body frame: +X=forward, +Y=left (kiwi_kinematics.h convention).
   * joyThr (stick pushed away from operator) -> forward -> +X, no
   * flip needed. joyStr (stick pushed right, screen +X) should
   * strafe the robot to ITS OWN right, i.e. body -Y -- hence the
   * sign flip below. */
  sendVelocity((joyThr/100)*maxSpeedMps, -(joyStr/100)*maxSpeedMps, cmdOmega);
}
function stopAll(){ joyThr=0; joyStr=0; cmdOmega=0; updateKnobUI(); updateDirArrows(); sendCmd(); }
const jZone=document.getElementById('jZone'); let jTouchId=null, jMouseDown=false;
function getJoyVal(cx,cy){
  const r=jZone.getBoundingClientRect();
  const mx=r.left+r.width/2, my=r.top+r.height/2, maxR=r.width/2-10;
  let dx=cx-mx, dy=cy-my; const d=Math.sqrt(dx*dx+dy*dy);
  if(d>maxR){dx=dx/d*maxR; dy=dy/d*maxR;}
  return {thr:Math.round((-dy/maxR)*100), str:Math.round((dx/maxR)*100)};
}
jZone.addEventListener('touchstart',(e)=>{e.preventDefault(); if(jTouchId!==null)return; const t=e.changedTouches[0]; jTouchId=t.identifier; const v=getJoyVal(t.clientX,t.clientY); setJoy(v.thr,v.str); sendCmd(); if(navigator.vibrate)navigator.vibrate(8);},{passive:false});
document.addEventListener('touchmove',(e)=>{e.preventDefault(); for(const t of e.changedTouches){ if(t.identifier===jTouchId){ const v=getJoyVal(t.clientX,t.clientY); setJoy(v.thr,v.str); sendCmd(); }}},{passive:false});
document.addEventListener('touchend',(e)=>{ for(const t of e.changedTouches){ if(t.identifier===jTouchId){ jTouchId=null; setJoy(0,0); sendCmd(); }}});
document.addEventListener('touchcancel',()=>{ jTouchId=null; setJoy(0,0); sendCmd(); });
jZone.addEventListener('mousedown',(e)=>{ jMouseDown=true; const v=getJoyVal(e.clientX,e.clientY); setJoy(v.thr,v.str); sendCmd(); });
document.addEventListener('mousemove',(e)=>{ if(!jMouseDown)return; const v=getJoyVal(e.clientX,e.clientY); setJoy(v.thr,v.str); sendCmd(); });
document.addEventListener('mouseup',()=>{ if(!jMouseDown)return; jMouseDown=false; setJoy(0,0); sendCmd(); });
function bindHold(id,onDown,onUp){
  const el=document.getElementById(id);
  const dn=(e)=>{e.preventDefault(); el.classList.add('held'); onDown();};
  const up=(e)=>{e.preventDefault(); el.classList.remove('held'); onUp();};
  el.addEventListener('touchstart',dn,{passive:false}); el.addEventListener('touchend',up,{passive:false});
  el.addEventListener('mousedown',dn); el.addEventListener('mouseup',up);
  el.addEventListener('mouseleave',()=>{el.classList.remove('held'); onUp();});
}
const zero=()=>{ setJoy(0,0); sendCmd(); };
bindHold('btnStop', stopAll, ()=>{});
bindHold('btnFwd',  ()=>{setJoy(80,0);sendCmd();}, zero);
bindHold('btnRev',  ()=>{setJoy(-60,0);sendCmd();}, zero);
bindHold('btnCcw',  ()=>{cmdOmega= maxOmegaRadps;sendCmd();}, ()=>{cmdOmega=0;sendCmd();});
bindHold('btnCw',   ()=>{cmdOmega=-maxOmegaRadps;sendCmd();}, ()=>{cmdOmega=0;sendCmd();});
document.getElementById('btnSetOrigin').addEventListener('click',()=>{
  if(!connected) return;
  const b=new ArrayBuffer(1); new DataView(b).setUint8(0,0x03); ws.send(b);
  resetAllFramesLocal(); odomActive=true; odomX=0;odomY=0;odomTheta=0; updateOdomUI();
  elOdomStatus.textContent='Recording'; elOdomStatus.className='odom-status active';
  if(navigator.vibrate)navigator.vibrate(20);
});
document.getElementById('btnEndReport').addEventListener('click',()=>{
  if(!connected) return;
  const b=new ArrayBuffer(1); new DataView(b).setUint8(0,0x04); ws.send(b);
  if(navigator.vibrate)navigator.vibrate([30,50,30]);
});
/* ============ shared helpers ============ */
function sendSimple(type,bytes){
  if(!connected||ws.readyState!==WebSocket.OPEN) return;
  const b=new ArrayBuffer(1+(bytes?bytes.length:0)), dv=new DataView(b);
  dv.setUint8(0,type); if(bytes) for(let i=0;i<bytes.length;i++) dv.setUint8(1+i,bytes[i]);
  ws.send(b);
}
function f32buf(type,a,b){
  const buf=new ArrayBuffer(9), dv=new DataView(buf);
  dv.setUint8(0,type); dv.setFloat32(1,a,true); dv.setFloat32(5,b,true);
  return buf;
}
function f32buf3(type,a,b,c){
  const buf=new ArrayBuffer(13), dv=new DataView(buf);
  dv.setUint8(0,type);
  dv.setFloat32(1,a,true); dv.setFloat32(5,b,true); dv.setFloat32(9,c,true);
  return buf;
}
function getSpeedPct(){ return parseInt(document.getElementById('speedSlider').value,10); }
let toastTimer=null;
function showToast(msg,kind){
  const t=document.getElementById('toast'); t.textContent=msg; t.className='toast show '+(kind||'');
  if(toastTimer)clearTimeout(toastTimer); toastTimer=setTimeout(()=>{t.className='toast';},2600);
}
function resetAllFramesLocal(){
  vertices=[]; surveyClosed=false; lastResults=null; navWP=[];
  robotPos={x:0,y:0,theta:0,valid:true};
  updateSurveyUI(); updatePatrolUI();
}
function showPage(mode){
  appMode=mode;
  document.getElementById('telemPanel').classList.toggle('hidden', mode === 'calib' || mode === 'log');
  document.getElementById('freeMode').classList.toggle('hidden',mode!=='free');
  document.getElementById('surveyMode').classList.toggle('hidden',mode!=='survey');
  document.getElementById('patrolMode').classList.toggle('hidden',mode!=='patrol');
  document.getElementById('calibMode').classList.toggle('hidden',mode!=='calib');
  document.getElementById('logMode').classList.toggle('hidden',mode!=='log');
  
  document.getElementById('tabFree').classList.toggle('active',mode==='free');
  document.getElementById('tabSurvey').classList.toggle('active',mode==='survey');
  document.getElementById('tabPatrol').classList.toggle('active',mode==='patrol');
  document.getElementById('tabCalib').classList.toggle('active',mode==='calib');
  document.getElementById('tabLog').classList.toggle('active',mode==='log');
  
  stopPosPoll(); stopPatrolPoll(); sendVelocity(0,0,0);
  if(mode==='survey'){
    sendSimple(0x05,[0]); surveyDriveMode=0; surveyHolding=false; updateSurveyUI();
    sendSimple(0x09); sendSimple(0x04); startPosPoll();
    requestAnimationFrame(()=>{resizeMapCanvas(); drawMap();});
  } else if(mode==='patrol'){
    updatePatrolUI(); sendSimple(0x04); startPatrolPoll();
    requestAnimationFrame(()=>{resizePatrolCanvas(); drawPatrolMap();});
  } else if(mode==='calib'){
    document.getElementById('calState').textContent = calActive?'BUSY':'IDLE';
  } else if(mode==='log'){
    renderFullLog();
  }
}
function startPosPoll(){ stopPosPoll(); posPollTimer=setInterval(()=>{ if(appMode==='survey'&&connected) sendSimple(0x04); },500); }
function stopPosPoll(){ if(posPollTimer){clearInterval(posPollTimer); posPollTimer=null;} }
function startPatrolPoll(){ stopPatrolPoll(); patrolPollTimer=setInterval(()=>{ if(appMode==='patrol'&&connected) sendSimple(0x04); },200); }
function stopPatrolPoll(){ if(patrolPollTimer){clearInterval(patrolPollTimer); patrolPollTimer=null;} }
/* ============ Survey ============ */
function handleVertexMessage(dv){
  if(dv.byteLength>=39 && dv.getUint8(1)===0){
    const idx=dv.getUint8(2);
    vertices[idx]={x:dv.getFloat32(3,true),y:dv.getFloat32(7,true),theta:dv.getFloat32(11,true),
      ts:dv.getUint32(15,true),enc1:dv.getInt32(19,true),enc2:dv.getInt32(23,true),enc3:dv.getInt32(27,true),
      hdS:dv.getFloat32(31,true),hdA:dv.getFloat32(35,true)};
    if(navigator.vibrate)navigator.vibrate(15);
    updateSurveyUI(); drawMap();
  } else if(dv.byteLength>=2){
    const st=dv.getUint8(1);
    if(st===1)showToast('Vertex buffer full (64 max)','warn');
    else if(st===2)showToast('IMU faulted - vertex marking disabled','err');
    else showToast('Vertex rejected (status '+st+')','warn');
    if(navigator.vibrate)navigator.vibrate([40,40,40]);
  }
}
function computeSurveyResults(){
  const n=vertices.length; const r={n:n,valid:n>=3};
  if(n<2) return r;
  const x0=vertices[0].x,y0=vertices[0].y; const pts=[];
  for(let i=0;i<n;i++) pts.push({x:vertices[i].x-x0,y:vertices[i].y-y0});
  const legs=[]; let P=0;
  for(let i=0;i<n;i++){ const a=pts[i],b=pts[(i+1)%n]; const L=Math.hypot(b.x-a.x,b.y-a.y); legs.push(L); P+=L; }
  r.perimeter=P; let s=0;
  for(let i=0;i<n;i++){ const a=pts[i],b=pts[(i+1)%n]; s+=a.x*b.y-b.x*a.y; }
  r.areaRaw=Math.abs(s)/2;
  if(!r.valid) return r;
  const cX=pts[n-1].x-pts[0].x, cY=pts[n-1].y-pts[0].y;
  r.closureErr=Math.hypot(cX,cY);
  r.precision=(r.closureErr>1e-9&&P>0)?(P/r.closureErr):Infinity;
  r.adjusted=[]; let cum=0;
  for(let i=0;i<n;i++){ const f=(P>0)?(cum/P):0; r.adjusted.push({x:pts[i].x-cX*f,y:pts[i].y-cY*f}); cum+=legs[i]; }
  let sa=0;
  for(let i=0;i<n;i++){ const a=r.adjusted[i],b=r.adjusted[(i+1)%n]; sa+=a.x*b.y-b.x*a.y; }
  r.areaAdj=Math.abs(sa)/2;
  let hd=0; for(let i=0;i<n;i++) hd+=Math.abs(vertices[i].hdS);
  r.hdMean=hd/n; r.hdIntegral=vertices[n-1].hdA;
  return r;
}
function showResultsOverlay(){
  const body=document.getElementById('resultsBody'); const r=lastResults;
  if(!r||!r.valid){
    body.innerHTML='<div class="res-warn">Survey closed with '+(r?r.n:0)+' vertex/vertices.<br>At least 3 vertices are required to compute area.</div>';
  } else {
    const precStr=isFinite(r.precision)?('1 : '+Math.round(r.precision)):'&mdash; (no closure error)';
    body.innerHTML='<div class="res-areas">'+
      '<div class="res-area-box raw"><div class="res-lab">RAW Area<br>(Shoelace)</div><div class="res-big">'+r.areaRaw.toFixed(2)+'</div><div class="res-unit">m&sup2;</div></div>'+
      '<div class="res-area-box adj"><div class="res-lab">Adjusted Area<br>(Bowditch)</div><div class="res-big">'+r.areaAdj.toFixed(2)+'</div><div class="res-unit">m&sup2;</div></div></div>'+
      '<div class="res-row"><span>Vertices</span><b>'+r.n+'</b></div>'+
      '<div class="res-row"><span>Perimeter</span><b>'+r.perimeter.toFixed(2)+' m</b></div>'+
      '<div class="res-row"><span>Closure error</span><b>'+r.closureErr.toFixed(3)+' m</b></div>'+
      '<div class="res-row"><span>Precision ratio</span><b>'+precStr+'</b></div>'+
      '<div class="res-row"><span>Heading disagree. (mean |signed|)</span><b>'+r.hdMean.toFixed(2)+'&deg;</b></div>'+
      '<div class="res-row"><span>Heading disagree. (abs integral)</span><b>'+r.hdIntegral.toFixed(2)+'&deg;</b></div>';
  }
  document.getElementById('resultsOverlay').classList.remove('hidden');
}
function updateSurveyUI(){
  const n=vertices.length;
  document.getElementById('svCount').textContent=n+' / '+SURVEY_MAX;
  const st=document.getElementById('svState');
  if(surveyClosed){st.textContent='CLOSED'; st.className='sv-state closed';}
  else{st.textContent='RECORDING'; st.className='sv-state rec';}
  document.getElementById('btnMarkVertex').disabled=surveyClosed;
  document.getElementById('btnSvClose').disabled=surveyClosed;
  document.getElementById('mapEmpty').classList.toggle('hidden',n>0);
  document.getElementById('mapLegend').classList.toggle('hidden',!surveyClosed);
  updateModeButtons(); updateDriveRows(); updateRobotReadout();
}
function updateModeButtons(){
  const bs=document.getElementById('btnModeStraight'), br=document.getElementById('btnModeRotate');
  bs.classList.toggle('active',surveyDriveMode===2); br.classList.toggle('active',surveyDriveMode===1);
  const dis=surveyClosed||surveyHolding; bs.disabled=dis; br.disabled=dis;
}
function updateDriveRows(){
  document.getElementById('driveStraight').classList.toggle('hidden',surveyDriveMode!==2);
  document.getElementById('driveRotate').classList.toggle('hidden',surveyDriveMode!==1);
  document.getElementById('driveHint').classList.toggle('hidden',surveyDriveMode!==0);
  ['btnSvFwd','btnSvRev','btnSvCcw','btnSvCw'].forEach(id=>{document.getElementById(id).disabled=surveyClosed;});
}
function updateRobotReadout(){
  const el=document.getElementById('svRobot');
  if(!robotPos.valid){ el.innerHTML='x &mdash;, y &mdash;, &theta; &mdash;'; return; }
  el.textContent='x '+robotPos.x.toFixed(2)+', y '+robotPos.y.toFixed(2)+' m, \u03B8 '+(robotPos.theta*180/Math.PI).toFixed(0)+'\u00B0';
}
function setSurveyMode(m){
  if(surveyHolding||surveyClosed||appMode!=='survey') return;
  surveyDriveMode=m; sendSimple(0x05,[m]); updateModeButtons(); updateDriveRows();
  if(navigator.vibrate)navigator.vibrate(8);
}
function bindSurveyDrive(id,fn){
  bindHold(id,()=>{ if(surveyClosed||appMode!=='survey')return; surveyHolding=true; updateModeButtons(); fn(); },
           ()=>{ if(surveyHolding){surveyHolding=false; sendVelocity(0,0,0); updateModeButtons();} });
}
bindSurveyDrive('btnSvFwd',()=>{const p=getSpeedPct(); sendVelocity((p/100)*maxSpeedMps,0,0);});
bindSurveyDrive('btnSvRev',()=>{const p=getSpeedPct(); sendVelocity(-(p/100)*maxSpeedMps,0,0);});
bindSurveyDrive('btnSvCcw',()=>{const p=getSpeedPct(); sendVelocity(0,0,(p/100)*maxOmegaRadps);});
bindSurveyDrive('btnSvCw', ()=>{const p=getSpeedPct(); sendVelocity(0,0,-(p/100)*maxOmegaRadps);});
document.getElementById('btnModeStraight').addEventListener('click',()=>setSurveyMode(2));
document.getElementById('btnModeRotate').addEventListener('click',()=>setSurveyMode(1));
document.getElementById('btnMarkVertex').addEventListener('click',()=>{ if(appMode!=='survey'||surveyClosed)return; sendSimple(0x06); });
document.getElementById('btnSvClear').addEventListener('click',()=>{
  if(appMode!=='survey')return;
  if(vertices.length>0&&!confirm('Clear all marked vertices?'))return;
  sendVelocity(0,0,0); sendSimple(0x05,[0]); sendSimple(0x07);
  vertices=[]; surveyClosed=false; lastResults=null; updateSurveyUI(); drawMap(); showToast('Vertices cleared','ok');
});
document.getElementById('btnSvClose').addEventListener('click',()=>{
  if(appMode!=='survey'||surveyClosed)return;
  sendVelocity(0,0,0); sendSimple(0x08); surveyClosed=true; surveyHolding=false;
  lastResults=computeSurveyResults(); updateSurveyUI(); drawMap(); showResultsOverlay();
  if(navigator.vibrate)navigator.vibrate([30,50,30]);
});
document.getElementById('btnSvNew').addEventListener('click',()=>{
  if(appMode!=='survey')return;
  if(vertices.length>0&&!confirm('Start a new survey? Vertices and origin will be reset.'))return;
  sendVelocity(0,0,0); sendSimple(0x03); sendSimple(0x05,[0]);
  surveyDriveMode=0; surveyHolding=false; resetAllFramesLocal(); drawMap();
  showToast('New survey started - origin reset','ok');
});
document.getElementById('btnResultsClose').addEventListener('click',()=>{document.getElementById('resultsOverlay').classList.add('hidden');});
document.getElementById('speedSlider').addEventListener('input',()=>{document.getElementById('speedVal').textContent=document.getElementById('speedSlider').value+'%';});
/* ============ Patrol ============ */
const mapCanvas=document.getElementById('mapCanvas'); const mapCtx=mapCanvas.getContext('2d');
const patrolCanvas=document.getElementById('patrolCanvas'); const patrolCtx=patrolCanvas.getContext('2d');
let pScale=60,pCx=0,pCy=0,pW=10,pH=10;
function resizeMapCanvas(){ const w=document.getElementById('mapWrap'); const d=window.devicePixelRatio||1;
  if(w.clientWidth<2||w.clientHeight<2)return; mapCanvas.width=Math.round(w.clientWidth*d); mapCanvas.height=Math.round(w.clientHeight*d);
  mapCanvas.style.width=w.clientWidth+'px'; mapCanvas.style.height=w.clientHeight+'px'; }
function resizePatrolCanvas(){ const w=document.getElementById('patrolMapWrap'); const d=window.devicePixelRatio||1;
  if(w.clientWidth<2||w.clientHeight<2)return; patrolCanvas.width=Math.round(w.clientWidth*d); patrolCanvas.height=Math.round(w.clientHeight*d);
  patrolCanvas.style.width=w.clientWidth+'px'; patrolCanvas.style.height=w.clientHeight+'px'; }
function drawGrid(ctx,W,H,toSX,toSY,cx,cy,scale){
  const steps=[0.1,0.2,0.5,1,2,5,10,20,50]; let step=50;
  for(const s of steps){ if(s*scale>=34){step=s;break;} }
  const wx0=cx-W/2/scale, wx1=cx+W/2/scale, wy0=cy-H/2/scale, wy1=cy+H/2/scale;
  ctx.lineWidth=1; ctx.strokeStyle='rgba(255,255,255,0.05)'; ctx.beginPath();
  for(let x=Math.ceil(wx0/step)*step;x<=wx1;x+=step){ctx.moveTo(toSX(x),0);ctx.lineTo(toSX(x),H);}
  for(let y=Math.ceil(wy0/step)*step;y<=wy1;y+=step){ctx.moveTo(0,toSY(y));ctx.lineTo(W,toSY(y));}
  ctx.stroke();
  ctx.strokeStyle='rgba(255,255,255,0.13)'; ctx.beginPath();
  ctx.moveTo(toSX(0),0);ctx.lineTo(toSX(0),H); ctx.moveTo(0,toSY(0));ctx.lineTo(W,toSY(0)); ctx.stroke();
}
function drawRobot(ctx,toSX,toSY){
  if(!robotPos.valid)return;
  const rx=toSX(robotPos.x), ry=toSY(robotPos.y);
  const hx=Math.cos(robotPos.theta), hy=Math.sin(robotPos.theta);
  ctx.beginPath(); ctx.arc(rx,ry,6,0,Math.PI*2); ctx.fillStyle='#ffb020'; ctx.fill();
  ctx.strokeStyle='rgba(255,176,32,0.9)'; ctx.lineWidth=2; ctx.beginPath();
  ctx.moveTo(rx,ry); ctx.lineTo(rx+hx*16,ry-hy*16); ctx.stroke();
}
function drawMap(){
  const d=window.devicePixelRatio||1, W=mapCanvas.width/d, H=mapCanvas.height/d;
  if(W<10||H<10)return; mapCtx.setTransform(d,0,0,d,0,0); mapCtx.clearRect(0,0,W,H);
  const pts=[[0,0]]; if(robotPos.valid)pts.push([robotPos.x,robotPos.y]);
  for(const v of vertices) if(v) pts.push([v.x,v.y]);
  const v0=vertices.length?vertices[0]:null;
  if(surveyClosed&&lastResults&&lastResults.adjusted&&v0) for(const a of lastResults.adjusted) pts.push([a.x+v0.x,a.y+v0.y]);
  let mnX=1/0,mxX=-1/0,mnY=1/0,mxY=-1/0;
  for(const p of pts){mnX=Math.min(mnX,p[0]);mxX=Math.max(mxX,p[0]);mnY=Math.min(mnY,p[1]);mxY=Math.max(mxY,p[1]);}
  let sx=mxX-mnX, sy=mxY-mnY; const cx=(mnX+mxX)/2, cy=(mnY+mxY)/2;
  if(sx<2)sx=2; if(sy<2)sy=2;
  const pad=34, scale=Math.min((W-pad*2)/sx,(H-pad*2)/sy);
  const toSX=x=>W/2+(x-cx)*scale, toSY=y=>H/2-(y-cy)*scale;
  drawGrid(mapCtx,W,H,toSX,toSY,cx,cy,scale);
  const n=vertices.length;
  if(n>=1){
    mapCtx.strokeStyle='#00c8ff'; mapCtx.lineWidth=2; mapCtx.beginPath();
    for(let i=0;i<n-1;i++){const a=vertices[i],b=vertices[i+1]; if(!a||!b)continue; mapCtx.moveTo(toSX(a.x),toSY(a.y)); mapCtx.lineTo(toSX(b.x),toSY(b.y));}
    mapCtx.stroke();
    if(n>=2){const a=vertices[n-1],b=vertices[0]; mapCtx.strokeStyle='rgba(0,200,255,0.45)'; mapCtx.setLineDash([6,5]); mapCtx.beginPath(); mapCtx.moveTo(toSX(a.x),toSY(a.y)); mapCtx.lineTo(toSX(b.x),toSY(b.y)); mapCtx.stroke(); mapCtx.setLineDash([]);}
    if(surveyClosed&&lastResults&&lastResults.adjusted&&n>=3&&v0){
      const adj=lastResults.adjusted; mapCtx.strokeStyle='rgba(0,255,157,0.6)'; mapCtx.lineWidth=1.5; mapCtx.setLineDash([3,4]); mapCtx.beginPath();
      for(let i=0;i<adj.length;i++){const a=adj[i],b=adj[(i+1)%adj.length]; mapCtx.moveTo(toSX(a.x+v0.x),toSY(a.y+v0.y)); mapCtx.lineTo(toSX(b.x+v0.x),toSY(b.y+v0.y));}
      mapCtx.stroke(); mapCtx.setLineDash([]);
    }
    for(let i=0;i<n;i++){const v=vertices[i]; if(!v)continue; const X=toSX(v.x),Y=toSY(v.y);
      mapCtx.beginPath(); mapCtx.arc(X,Y,9,0,Math.PI*2); mapCtx.fillStyle=(i===0)?'#0a2a1a':'#0d1117'; mapCtx.fill();
      mapCtx.strokeStyle=(i===0)?'#00ff9d':'#00c8ff'; mapCtx.lineWidth=1.5; mapCtx.stroke();
      mapCtx.fillStyle='#c9d1d9'; mapCtx.font='9px sans-serif'; mapCtx.textAlign='center'; mapCtx.textBaseline='middle'; mapCtx.fillText(String(i+1),X,Y);}
  }
  drawRobot(mapCtx,toSX,toSY);
}
function drawPatrolMap(){
  const d=window.devicePixelRatio||1, W=patrolCanvas.width/d, H=patrolCanvas.height/d;
  if(W<10||H<10)return; patrolCtx.setTransform(d,0,0,d,0,0); patrolCtx.clearRect(0,0,W,H);
  const pts=[[0,0]]; if(robotPos.valid)pts.push([robotPos.x,robotPos.y]);
  for(const w of navWP) pts.push([w.x,w.y]);
  let mnX=1/0,mxX=-1/0,mnY=1/0,mxY=-1/0;
  for(const p of pts){mnX=Math.min(mnX,p[0]);mxX=Math.max(mxX,p[0]);mnY=Math.min(mnY,p[1]);mxY=Math.max(mxY,p[1]);}
  let sx=mxX-mnX, sy=mxY-mnY; const cx=(mnX+mxX)/2, cy=(mnY+mxY)/2;
  if(sx<2)sx=2; if(sy<2)sy=2;
  const pad=34, scale=Math.min((W-pad*2)/sx,(H-pad*2)/sy);
  const toSX=x=>W/2+(x-cx)*scale, toSY=y=>H/2-(y-cy)*scale;
  pScale=scale; pCx=cx; pCy=cy; pW=W; pH=H;
  drawGrid(patrolCtx,W,H,toSX,toSY,cx,cy,scale);
  const n=navWP.length;
  if(n>=1){
    patrolCtx.strokeStyle='#00ff9d'; patrolCtx.lineWidth=2; patrolCtx.beginPath();
    for(let i=0;i<n-1;i++){patrolCtx.moveTo(toSX(navWP[i].x),toSY(navWP[i].y)); patrolCtx.lineTo(toSX(navWP[i+1].x),toSY(navWP[i+1].y));}
    patrolCtx.stroke();
    if(n>=2){patrolCtx.strokeStyle='rgba(0,255,157,0.45)'; patrolCtx.setLineDash([6,5]); patrolCtx.beginPath();
      patrolCtx.moveTo(toSX(navWP[n-1].x),toSY(navWP[n-1].y)); patrolCtx.lineTo(toSX(navWP[0].x),toSY(navWP[0].y)); patrolCtx.stroke(); patrolCtx.setLineDash([]);}
    for(let i=0;i<n;i++){const w=navWP[i]; const X=toSX(w.x),Y=toSY(w.y);
      const isT=navActive&&(navState===1||navState===2||navState===3)&&i===navTarget;
      patrolCtx.beginPath(); patrolCtx.arc(X,Y,isT?11:9,0,Math.PI*2); patrolCtx.fillStyle=(i===0)?'#0a2a1a':'#0d1117'; patrolCtx.fill();
      patrolCtx.strokeStyle=isT?'#ffb020':((i===0)?'#00ff9d':'rgba(0,255,157,0.75)'); patrolCtx.lineWidth=isT?2.5:1.5; patrolCtx.stroke();
      patrolCtx.fillStyle='#c9d1d9'; patrolCtx.font='9px sans-serif'; patrolCtx.textAlign='center'; patrolCtx.textBaseline='middle'; patrolCtx.fillText(String(i+1),X,Y);}
  }
  drawRobot(patrolCtx,toSX,toSY);
}
function patrolTap(cx,cy){
  if(appMode!=='patrol')return;
  if(navActive){showToast('Stop the patrol before editing waypoints','warn');return;}
  if(navWP.length>=PATROL_MAX){showToast('Waypoint limit reached (16)','warn');return;}
  const r=patrolCanvas.getBoundingClientRect();
  const sx=cx-r.left, sy=cy-r.top;
  const wx=pCx+(sx-pW/2)/pScale, wy=pCy-(sy-pH/2)/pScale;
  navWP.push({x:wx,y:wy}); sendAddWp(wx,wy); updatePatrolUI(); drawPatrolMap();
  if(navigator.vibrate)navigator.vibrate(10);
}
patrolCanvas.addEventListener('touchstart',(e)=>{e.preventDefault(); const t=e.changedTouches[0]; patrolTap(t.clientX,t.clientY);},{passive:false});
patrolCanvas.addEventListener('mousedown',(e)=>{patrolTap(e.clientX,e.clientY);});
function sendAddWp(x,y){
  if(!connected||ws.readyState!==WebSocket.OPEN)return;
  const b=new ArrayBuffer(9), dv=new DataView(b);
  dv.setUint8(0,0x0B); dv.setFloat32(1,x,true); dv.setFloat32(5,y,true); ws.send(b);
}
function uploadRoute(){ sendSimple(0x0A); for(const w of navWP) sendAddWp(w.x,w.y); }
function getPatrolSpeedPct(){ return parseInt(document.getElementById('patrolSpeed').value,10); }
function updatePatrolUI(){
  document.getElementById('pvCount').textContent=navWP.length+' / '+PATROL_MAX;
  const st=document.getElementById('pvState');
  if(navFault===1&&!navActive){st.textContent='ENC FAULT'; st.className='sv-state fault';}
  else if(!navActive){st.textContent='IDLE'; st.className='sv-state idle';}
  else if(navState===1){st.textContent='ALIGN WP'+(navTarget+1); st.className='sv-state work';}
  else if(navState===2){st.textContent='TO WP'+(navTarget+1); st.className='sv-state work';}
  else if(navState===3){st.textContent='AT WP'+(navTarget+1); st.className='sv-state rec';}
  const info=document.getElementById('pvInfo');
  if(navActive) info.textContent='target WP'+(navTarget+1)+' \u00B7 dist '+navDist.toFixed(2)+' m';
  else if(robotPos.valid) info.textContent='x '+robotPos.x.toFixed(2)+', y '+robotPos.y.toFixed(2)+' m';
  else info.textContent='target \u2014 \u00B7 dist \u2014';
  document.getElementById('patrolEmpty').classList.toggle('hidden',navWP.length>0);
  document.getElementById('btnPatrolStart').disabled=navActive||navWP.length<2;
  document.getElementById('btnPatrolStop').disabled=!navActive;
  document.getElementById('btnPatrolUndo').disabled=navActive||navWP.length===0;
  document.getElementById('btnPatrolClear').disabled=navActive||navWP.length===0;
  document.getElementById('patrolSpeed').disabled=navActive;
  document.getElementById('autoBadge').classList.toggle('hidden',!navActive);
}
function handleNavStatus(dv){
  const prevActive=navActive;
  navState=dv.getUint8(1); navTarget=dv.getUint8(2); navCountSTM=dv.getUint8(3);
  navFault=dv.getUint8(4); navDist=dv.getFloat32(5,true);
  navActive=navState!==0;
  if(prevActive&&!navActive){
    if(navFault===1)showToast('Encoder connection fault - patrol aborted','err');
    else showToast('Patrol stopped','ok');
  }
  updatePatrolUI(); if(appMode==='patrol')drawPatrolMap();
}

/* Confirmation for CMD_NAV_MARK_WP: the MCU captured ITS OWN live
 * odometry position atomically (same pattern as Survey vertex marking)
 * rather than trusting whatever position the browser last happened to
 * poll. Apply the authoritative (index, x, y) directly. */
function handleNavWpData(dv){
  const ok = dv.getUint8(1);
  if(!ok){
    showToast('Could not mark waypoint (limit reached or patrol active)','warn');
    return;
  }
  const idx = dv.getUint8(2);
  const wx = dv.getFloat32(3,true), wy = dv.getFloat32(7,true);
  navWP[idx] = {x:wx, y:wy};
  updatePatrolUI(); drawPatrolMap();
  if(navigator.vibrate)navigator.vibrate(10);
}

document.getElementById('btnPvMarkHere').addEventListener('click', ()=>{
  if (navActive) { showToast('Stop patrol first','warn'); return; }
  if (navWP.length >= PATROL_MAX) { showToast('Waypoint limit (16)','warn'); return; }
  sendSimple(0x14);
});

document.getElementById('btnPatrolStart').addEventListener('click',()=>{
  if(navActive||navWP.length<2||appMode!=='patrol')return;
  uploadRoute();
  const expectedCount = navWP.length;
  setTimeout(()=>{
    if(navCountSTM === expectedCount){
      sendSimple(0x0C,[getPatrolSpeedPct()]);
      if(navigator.vibrate)navigator.vibrate(20);
      return;
    }
    showToast('Waypoint upload mismatch (robot has '+navCountSTM+' of '+expectedCount+') - retrying','warn');
    uploadRoute();
    setTimeout(()=>{
      if(navCountSTM !== expectedCount){
        showToast('Still mismatched after retry - check connection before starting','err');
        return;
      }
      sendSimple(0x0C,[getPatrolSpeedPct()]);
      if(navigator.vibrate)navigator.vibrate(20);
    }, 500);
  }, 400);
});
document.getElementById('btnPatrolStop').addEventListener('click',()=>{ if(navActive) sendSimple(0x0D); });
document.getElementById('btnPatrolUndo').addEventListener('click',()=>{
  if(navActive||navWP.length===0)return;
  navWP.pop(); uploadRoute(); updatePatrolUI(); drawPatrolMap();
});
document.getElementById('btnPatrolClear').addEventListener('click',()=>{
  if(navActive||navWP.length===0)return;
  if(!confirm('Clear all waypoints?'))return;
  navWP=[]; sendSimple(0x0A); updatePatrolUI(); drawPatrolMap(); showToast('Waypoints cleared','ok');
});
document.getElementById('patrolSpeed').addEventListener('input',()=>{document.getElementById('patrolSpeedVal').textContent=document.getElementById('patrolSpeed').value+'%';});
/* ============ Calib ============ */
function handleCalStatus(dv){
  const type=dv.getUint8(1), leg=dv.getUint8(2), done=dv.getUint8(3), aborted=dv.getUint8(4);
  const val1=dv.getFloat32(5,true), val2=dv.getFloat32(9,true);
  const st=document.getElementById('calState');

  if(type===0){ /* SPIN -- val1=gyro_deg_this_leg, val2=kin_deg_this_leg */
    if(!done){
      st.textContent = leg===0 ? 'SPIN CCW' : 'SPIN CW'; st.className='sv-state work';
      document.getElementById('calInfo').textContent = val1.toFixed(0)+'\u00B0 gyro so far...';
      return;
    }
    calSpinLegGyroDeg[leg] = val1;
    if(leg===0 && !aborted){
      st.textContent='SPIN CW'; st.className='sv-state work';
      document.getElementById('calInfo').textContent='CCW done: '+val1.toFixed(0)+'\u00B0 gyro \u2014 now CW...';
      return;
    }
    calActive=false;
    st.textContent='IDLE'; st.className='sv-state idle';
    document.getElementById('btnCalSpin').disabled=false;
    document.getElementById('btnCalRoll').disabled=false;
    document.getElementById('btnCalApplySpin').disabled=false;
    document.getElementById('calInfo').textContent=(aborted?'Spin stopped':'Spin done')+' \u2014 CCW '+calSpinLegGyroDeg[0].toFixed(0)+'\u00B0, CW '+calSpinLegGyroDeg[1].toFixed(0)+'\u00B0 (gyro)';
    document.getElementById('calSpinResult').textContent='Enter measured CCW/CW degrees, then Apply.';
    if(navigator.vibrate)navigator.vibrate([30,50,30]);
  } else { /* ROLL -- val1=dist_estimated_m, val2 unused */
    if(!done){
      st.textContent='ROLLING'; st.className='sv-state work';
      document.getElementById('calInfo').textContent = val1.toFixed(3)+'m so far...';
      return;
    }
    calActive=false;
    st.textContent='IDLE'; st.className='sv-state idle';
    document.getElementById('btnCalSpin').disabled=false;
    document.getElementById('btnCalRoll').disabled=false;
    document.getElementById('btnCalApplyRoll').disabled=false;
    document.getElementById('calInfo').textContent=(aborted?'Roll stopped: ':'Roll done: ')+val1.toFixed(3)+'m estimated';
    document.getElementById('calRollResult').textContent='Enter true distance, then Apply.';
    if(navigator.vibrate)navigator.vibrate([30,50,30]);
  }
}
document.getElementById('btnCalSpin').addEventListener('click',()=>{
  if(calActive||!connected)return;
  const ccwDeg=(parseInt(document.getElementById('calTurnsCCW').value,10)||3)*360;
  const cwDeg =(parseInt(document.getElementById('calTurnsCW').value,10)||3)*360;
  const pwr=parseInt(document.getElementById('calPowerPct').value,10)||60;
  calActive=true; calSpinLegGyroDeg=[0,0];
  document.getElementById('btnCalSpin').disabled=true;
  document.getElementById('btnCalRoll').disabled=true;
  document.getElementById('btnCalApplySpin').disabled=true;
  const st=document.getElementById('calState'); st.textContent='SPIN CCW'; st.className='sv-state work';
  document.getElementById('calInfo').textContent='keep clear...';
  ws.send(f32buf3(0x15,ccwDeg,cwDeg,pwr));
});
document.getElementById('btnCalApplySpin').addEventListener('click',()=>{
  if(!connected)return;
  const mCcw=parseFloat(document.getElementById('calMeasuredCcw').value)||0;
  const mCw =parseFloat(document.getElementById('calMeasuredCw').value)||0;
  ws.send(f32buf(0x10,mCcw,mCw));
  document.getElementById('calSpinResult').textContent='Applied: measured '+mCcw.toFixed(0)+'\u00B0/'+mCw.toFixed(0)+'\u00B0 vs gyro '+calSpinLegGyroDeg[0].toFixed(0)+'\u00B0/'+calSpinLegGyroDeg[1].toFixed(0)+'\u00B0';
});
document.getElementById('btnCalRoll').addEventListener('click',()=>{
  if(calActive||!connected)return;
  const dist=parseFloat(document.getElementById('calRollTarget').value)||2.0;
  const pwr=parseInt(document.getElementById('calPowerPct').value,10)||60;
  calActive=true;
  document.getElementById('btnCalSpin').disabled=true;
  document.getElementById('btnCalRoll').disabled=true;
  document.getElementById('btnCalApplyRoll').disabled=true;
  const st=document.getElementById('calState'); st.textContent='ROLLING'; st.className='sv-state work';
  document.getElementById('calInfo').textContent='keep clear...';
  ws.send(f32buf(0x16,dist,pwr));
});
document.getElementById('btnCalApplyRoll').addEventListener('click',()=>{
  if(!connected)return;
  const dTrue=parseFloat(document.getElementById('calRollTrue').value)||0;
  const b=new ArrayBuffer(5), dv=new DataView(b);
  dv.setUint8(0,0x11); dv.setFloat32(1,dTrue,true);
  ws.send(b);
  document.getElementById('calRollResult').textContent='Applied: true distance '+dTrue.toFixed(3)+'m';
});
function resetCalUI(msg){
  calActive = false;
  const st = document.getElementById('calState');
  st.textContent = 'IDLE'; st.className = 'sv-state idle';
  document.getElementById('btnCalSpin').disabled = false;
  document.getElementById('btnCalRoll').disabled = false;
  document.getElementById('calInfo').textContent = msg || 'ready';
}
document.getElementById('btnCalStop').addEventListener('click', () => {
  if (!connected) return;
  sendSimple(0x17);
  resetCalUI('stopped');
});
function handleNavDebug(dv){
  if(dv.byteLength<45)return;
  const state=dv.getUint8(1), settle=dv.getUint8(6);
  const dist=dv.getFloat32(13,true), rate=dv.getFloat32(17,true);
  const err=dv.getFloat32(29,true);
  const flags=dv.getUint8(4);
  const coast=(flags&1)?1:0;
  /* theta_deg/bearing_deg (dv 21/25) and cmd1/2/3_rpm (dv 33/37/41)
   * are also available in this packet but not currently displayed --
   * parsed here for any future diagnostic extension. */

  const elRate=document.getElementById('navDbgRate');
  if(!elRate) return; /* debug strip not present on this page (not in Patrol tab's DOM yet) */
  elRate.textContent = rate.toFixed(0)+'\u00B0/s';
  document.getElementById('navDbgErr').textContent  = err.toFixed(1)+'\u00B0';
  document.getElementById('navDbgDist').textContent = dist.toFixed(2)+'m';
  document.getElementById('navDbgCoast').textContent = coast ? 'YES' : 'no';
  document.getElementById('navDbgCoast').style.color = coast ? 'var(--warn)' : 'var(--muted)';
  document.getElementById('navDbgSettle').textContent = settle+'/5';
  document.getElementById('navDbgState').textContent = ['IDLE','DRIVING','SETTLING'][state] || '--';
}


/* ============ tabs / resize / init ============ */
document.getElementById('tabFree').addEventListener('click',()=>{ if(appMode!=='free')showPage('free'); });
document.getElementById('tabSurvey').addEventListener('click',()=>{ if(appMode!=='survey')showPage('survey'); });
document.getElementById('tabPatrol').addEventListener('click',()=>{ if(appMode!=='patrol')showPage('patrol'); });
document.getElementById('tabCalib').addEventListener('click',()=>{ if(appMode!=='calib')showPage('calib'); });
document.getElementById('tabLog').addEventListener('click',()=>{ if(appMode!=='log')showPage('log'); });
window.addEventListener('resize',()=>{ if(appMode==='survey'){resizeMapCanvas();drawMap();} if(appMode==='patrol'){resizePatrolCanvas();drawPatrolMap();} });
setJoy(0,0); updateOdomUI(); updateSurveyUI(); updatePatrolUI();

/* ============ LIVE LOG ENGINE ============ */
const MAX_LOGS = 1000;
let logHistory = [];
let logPaused = false;
const logStreamEl = document.getElementById('logStream');
const fullLogEl = document.getElementById('fullLog');

function pushLogData(dL, dR, gyro, hd, gyroBias, isCal, gyroScale, maxDtMs) {
    if (logPaused) return;

    const gyroCorr = (gyro - gyroBias) * gyroScale;

    document.getElementById('logDL').textContent = dL;
    document.getElementById('logDR').textContent = dR;
    document.getElementById('logGyro').textContent = gyro.toFixed(2);
    document.getElementById('logGyroCorr').textContent = gyroCorr.toFixed(2);
    document.getElementById('logHD').textContent = hd.toFixed(2);
    document.getElementById('logBias').textContent = gyroBias.toFixed(2);
    const calEl = document.getElementById('logCal');
    calEl.textContent = isCal ? 'YES' : 'NO';
    calEl.style.color = isCal ? 'var(--accentG)' : 'var(--warn)';
    const dtEl = document.getElementById('logMaxDt');
    dtEl.textContent = maxDtMs + ' ms';
    dtEl.style.color = maxDtMs > 30 ? 'var(--danger)' : (maxDtMs > 12 ? 'var(--warn)' : 'var(--accentG)');

    const now = new Date();
    const time = now.toLocaleTimeString('en-US', { hour12: false }) + '.' + String(now.getMilliseconds()).padStart(3, '0');
    const entry = { time, dL, dR, gyro: gyro.toFixed(1), hd: hd.toFixed(2) };
    
    logHistory.push(entry);
    if (logHistory.length > MAX_LOGS) logHistory.shift();
    
    if (logStreamEl) {
        const row = document.createElement('div');
        row.className = 'log-row';
        row.innerHTML = `<span>${entry.time}</span><span class="cL">${entry.dL}</span><span class="cR">${entry.dR}</span><span class="cB">${entry.gyro}</span><span class="cD">${entry.hd}</span>`;
        logStreamEl.appendChild(row);
        
        // Cap DOM nodes for performance
        while (logStreamEl.children.length > 150) {
            logStreamEl.removeChild(logStreamEl.firstChild);
        }
        logStreamEl.scrollTop = logStreamEl.scrollHeight;
    }
}

function renderFullLog() {
    if (!fullLogEl) return;
    fullLogEl.innerHTML = '';
    const header = document.createElement('div');
    header.className = 'log-row header';
    header.innerHTML = `<span>TIME</span><span>Δ L</span><span>Δ R</span><span>GYRO</span><span>SLIP</span>`;
    fullLogEl.appendChild(header);
    
    const fragment = document.createDocumentFragment();
    for (const entry of logHistory) {
        const row = document.createElement('div');
        row.className = 'log-row';
        row.innerHTML = `<span>${entry.time}</span><span class="cL">${entry.dL}</span><span class="cR">${entry.dR}</span><span class="cB">${entry.gyro}</span><span class="cD">${entry.hd}</span>`;
        fragment.appendChild(row);
    }
    fullLogEl.appendChild(fragment);
    fullLogEl.scrollTop = fullLogEl.scrollHeight;
}

document.getElementById('btnLogToggle').addEventListener('click', () => {
    document.getElementById('logOverlay').classList.toggle('hidden');
});
document.getElementById('btnLogClose').addEventListener('click', () => {
    document.getElementById('logOverlay').classList.add('hidden');
});
document.getElementById('btnLogPause').addEventListener('click', (e) => {
    logPaused = !logPaused;
    e.target.textContent = logPaused ? 'RESUME' : 'PAUSE';
    e.target.classList.toggle('active', logPaused);
});

function clearAllLogs() {
    logHistory = [];
    if (logStreamEl) {
        logStreamEl.innerHTML = '<div class="log-row header"><span>TIME</span><span>Δ L</span><span>Δ R</span><span>GYRO</span><span>SLIP</span></div>';
    }
    if (appMode === 'log') renderFullLog();
}

document.getElementById('btnLogClear').addEventListener('click', clearAllLogs);
document.getElementById('btnFullLogClear').addEventListener('click', clearAllLogs);

connect();
</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", index_html); }

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  server.on("/", handleRoot);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

static unsigned long lastHeartbeat = 0;
void loop() {
  webSocket.loop();
  server.handleClient();
  processSerial();
  if (odom_report_ready)   { odom_report_ready   = false; broadcastOdometry(); }
  if (telem_updated)       { telem_updated       = false; broadcastTelemetry(); }
  if (vert_updated)        { vert_updated        = false; broadcastVertexData(); }
  if (nav_status_updated)  { nav_status_updated  = false; broadcastNavStatus(); }
  if (nav_debug_updated)   { nav_debug_updated   = false; broadcastNavDebug(); }
  if (cal_status_updated)  { cal_status_updated  = false; broadcastCalStatus(); }
  if (nav_wp_updated)      { nav_wp_updated      = false; broadcastNavWp(); }
  if (millis() - lastHeartbeat >= 100) {
    lastHeartbeat = millis();
    sendPacket(CMD_HEARTBEAT, NULL, 0);
  }
}
