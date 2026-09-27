// SRS ΩΞ 0.3 — based on SRS ΩΞ 0.1 / SRS ΩΨ 9.9.3
// Compact diagnostic build: preserves 9.9.3 radar/tracking/map/BLE behavior
// while reducing Serial load and replacing fixed 2-of-4 temporal confirmation
// with a sliding weighted evidence window.
// RAW capture is ALWAYS enabled in ΩΞ 0.2 so a field test cannot lose
// hours of RAW data because RAW_ON was forgotten. RAW_ON/RAW_OFF from
// Android control only the BLE log mirror. BLE LOG: ON therefore starts
// receiving the existing RAW + compact stream immediately, without any
// Android application change. Long BLE lines are split/reassembled as
// before; USB Serial remains the authoritative complete log.
// v0.9.12: BLE log mirror actually works now. serviceBleDatasetQueue() was
// a stub that silently dropped everything queued by datasetLog()/
// datasetLogf() -- the dataset/diagnostic stream never reached BLE despite
// looking wired up. Added a dedicated NOTIFY-only log characteristic
// (separate from T1/T2/T3:V telemetry) and a working queue sender.
// printRadarData()'s [RADAR] summary line now also goes through
// datasetLog(), so it too is mirrored to BLE (previously USB-only).
// Removed the dead DEBUG_ON/OFF and BLE_LOG_ON/OFF BLE commands; log
// mirroring is now automatic while a BLE client is connected.
// SRS ΩΨ 9.9.2
// v0.9.11: temperature label shortened to TP:NNC (degree symbol removed,
// it didn't render correctly on this display's font).
// v0.9.10: OLED target marker mirrored (sign flip on targetX only) to
// match the app, confirmed correct via a matched screenshot/photo pair.
// SRS ΩΨ 0.9.9
// v0.9.9: added chip temperature readout (ESP32-C3 internal sensor),
// displayed under BZ:ON/OFF as TEMP:NN C, refreshed every 5s.
// SRS ΩΨ 0.9.8
// v0.9.8: grid back to 9 spokes (+-80,+-60,+-40,+-20,0), matching the
// original v0.6-0.9 look; the two outer ones were dropped by mistake in
// v0.9.5 and not restored in v0.9.6/0.9.7.
// v0.9.7: reverted v0.9.6's ring clipping -- rings are full circles again,
// flaring past the +-60 deg spokes at the bottom corners on purpose.
// v0.9.6: BLE angle mirror fix (app was mirrored left/right vs the correct
// OLED).
// v0.9.5: back to a true (undistorted) circular grid, sized for the RD-03D's
// real +-60 deg azimuth FOV (RADAR_R=138px vs the old +-80 deg guess's
// ~121px). Grid and target marker share one scale again, so no egg shape
// and no mismatch between the two -- matches the reference phone app's look.
// v0.9.2: display range 8 m -> 6 m, grid rings derived from the same scale (1.5 m/ring).
// v0.9.1: OLED target marker drawn with sin/cos matching the grid (was swapped).
// Decision-layer redesign:
// - temporal evidence is corroboration, never an unconditional HUMAN override;
// - chaoticScore remains an AI feature/diagnostic, not a manual vegetation gate;
// - vegetation evidence has a cold-start component independent of maps;
// - map learning is separated from vegetation veto to avoid score→map→score feedback;
// - adaptive background is learned only from independently confirmed non-human behavior;
// - full [DIAG] and compact [DECISION] logs are both retained;
// - target/distance UI uses the same font size as the voltage readout.
// Surgical pre-classification guard: delay OBSERVE/IGNORE -> HUMAN when
// another HUMAN GHOST predicts the same candidate position.
// No radar protocol, display, buzzer, or general tracking logic changed.
// v0.9 fixes: [DECISION] varargs alignment (S=radarSpeed + DIRF), duplicate
// track-debug invocation, ghost veto uses vegetationPatternScore(), cold-start
// vegetation scaling, and keeps the v0.8 map-learning rate changes (/40, /50).

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <U8g2_for_Adafruit_GFX.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include <stdarg.h>
#include <RD03D.h>

// =========================================================
// SPARTAK RADAR SRS
// ESP32-C3 Super Mini
//
// Версія 0.3.4 + DISP_PRIORITY + RANGE_MAX_RUN + SCENE MAP + ENTRY VECTOR + AI + LOCK + GHOST:
// 0.3.4: fix long radial walk treated as vegetation. Remove blind REP>=50→G=0.55
//        floor unless netDisp small AND range_max_run low. Large netDisplacement
//        (>=1.5m) blocks vegetation evidence -1/-2 so evidence window cannot stay
//        poisoned at W=-8 while AI says H=1.0. Stronger progression for multi-metre
//        tracks. Goal: keep good human recall (like ΩΨ era) without reopening easy FP.
// - 60 секунд початкового навчання всієї сцени
// - 60 секунд стартового навчання всієї сцени
// - перманентна RAM-мапа до перезапуску
// - 60 с = стартовий prior, далі адаптивне розширення non-HUMAN фону
// - 48x32 карта з кроком 250 мм
// - окрема карта динамічного фону
// - карта характеру/хаотичності коливань
// - адаптивна карта амплітуди руху фону після стартового навчання
// - локальний 3x3 аналіз
// - аналіз напрямку руху
// - 3 одночасні треки
// - Alpha-фільтр
// - захист від різких "перехоплень" треків
// - захист від фантомів на краях
// - RD-03D Doppler-speed evidence filter проти гілок/вітру
// - BLE / OLED / buzzer / battery збережені
// - TinyML-style AI fusion layer (10->6->3) для A/B тесту
// - AI 75% final gate for OLED / BLE / buzzer
// - AI release after 2s of clearly non-human confidence
// - HUMAN_LOCK винесений з класифікації: lock армується ТІЛЬКИ після
//   фактичного відображення HUMAN на OLED та BLE notify (якщо BLE підключений)
// - HUMAN_LOCK більше НЕ блокує AI reclassification: HUMAN може бути
//   відкликаний AI та перейти назад у OBSERVE/не-HUMAN
// - lock використовується тільки для стабілізації ID/reassociation/ghost
// - HUMAN GHOST після повної втрати radar track
// - cross-slot / same-slot HUMAN reacquisition
//
// ЗМІНИ v5 (виправлення хибних спрацювань на гілках/вітрі):
//
// 1) Buzzer, BLE presence-біт і тон сигналу тепер прив'язані
//    до targets[i].isHuman, а НЕ до "active && !isClutter".
//    Раніше S/E/Doppler-фільтр коректно НЕ підтверджував ціль
//    як людину (isHuman лишався false), але сигналізація все
//    одно спрацьовувала, бо перевіряла лише isClutter — а
//    ціль ставала isClutter тільки якщо її встигала розпізнати
//    карта фону. Тепер висновок фільтра isHuman нарешті
//    впливає на сигнал/BLE, а не тільки на текст у логах.
//
// 2) Навчання dynamicMap/oscillationMap більше не вимагає,
//    щоб клітинка вже була isKnownStaticBackground. Раніше
//    це створювало замкнене коло: гілка, що вже гойдалась на
//    момент старту, ніколи не потрапляла в bgMap (бо
//    updateTraining() свідомо виключає рухомі об'єкти з
//    навчання фону), а без запису в bgMap динамічна карта для
//    цієї ж клітинки теж ніколи не навчалась (бо це залежало
//    від bgMap) — тому "dynamic phantom"-фільтр НІКОЛИ не міг
//    спрацювати для такої гілки, і вона лишалась active
//    і не-clutter завжди. Тепер dynamicMap навчається
//    незалежно від bgMap, на основі власного chaoticScore
//    цілі.
//
// Що НЕ змінилось: OLED, як і раніше, показує всі
// active && !isClutter треки (щоб ви бачили сирі дані радара
// для налагодження) — просто тепер такі "гілкові" треки з
// часом самі стають isClutter завдяки пункту (2), а сигналізація
// на них взагалі не реагує завдяки пункту (1).
//
// ВАЖЛИВО:
// Карта зберігається тільки в RAM. Після RESET/POWER OFF
// навчання виконується заново.
// =========================================================

// =========================================================
// ПІНИ ТА ДИСПЛЕЙ
// =========================================================
#define BATTERY_PIN 1
#define BUZZER_PIN 4

// Startup sound sequence:
// 1 short beep immediately after power-on
// 60 s training
// 2 short beeps when training finishes
#define STARTUP_BEEP_FREQ 1800
#define STARTUP_BEEP_DURATION 80
#define STARTUP_BEEP_GAP 100

// Firmware identity is printed to Serial and exposed over BLE so every
// saved Android debug log can be tied to the exact firmware build.
#define SRS_FIRMWARE_NAME "SRS_ΩΞ_0.3.5"
#define SRS_BUILD_DATE "2026-09-26"

#define RADAR_RX_PIN 5
#define RADAR_TX_PIN 6
#define TFT_SCK 9
#define TFT_SDA 8
#define TFT_RES 20
#define TFT_DC 10
#define TFT_BLK 7
#define TOUCH_BUTTON_PIN 21

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240

// GMT130-V1.0 requires a real SPI MODE3 transaction on this module.
// Use ESP32-C3 hardware SPI with the existing wiring remapped to GPIO9/8.
// The display has no CS pin, so CS remains unused (-1).
Adafruit_ST7789 display(&SPI, -1, TFT_DC, TFT_RES);
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;
// Full-frame backbuffer prevents the visible black flash caused by clearing
// the physical ST7789 on every radar refresh.
GFXcanvas16 radarCanvas(SCREEN_WIDTH, SCREEN_HEIGHT);

const int CX = SCREEN_WIDTH / 2;
const int CY = SCREEN_HEIGHT - 1;
const int R = 100;  // legacy circular radius (kept for reference)

// v0.9.5: RD-03D datasheet gives a real azimuth FOV of +-60 deg (not +-80).
// A true (undistorted) semicircle that stays on-screen at that FOV can be
// as large as ~138 px (limited by screen width at the +-60 deg edges), vs
// ~121 px at the old, wider +-80 deg assumption. No more anisotropic
// stretch: RADAR_R is used for both axes, so rings stay circular. At max
// range the target sits at ~3/4 of this radius rather than at the screen
// edge -- the same look the reference phone app has, just on a smaller
// screen.
const float RADAR_R = 138.0f;

const float RADAR_FOV = 120.0;
const float HALF_FOV = RADAR_FOV / 2.0;
// v0.9.2: display range matches the radar's practical range (6 m) so targets
// are not squeezed toward the origin. Grid rings are derived from the same
// scale: every GRID_RING_MM = 25 px with R = 100.
const float MAX_DISPLAY_DISTANCE = 6000.0;
const float GRID_RING_MM = 1500.0;

// =========================================================
// ВИСОКОДЕТАЛЬНА ПЕРМАНЕНТНА КАРТА ФОНУ
//
// Фізична область:
//   X = -6000 ... +6000 мм  (12 м ширина)
//   Y = 0 ... 8000 мм       (8 м дальність)
//
// Роздільна здатність:
//   250 x 250 мм.
//
// 48x32 = 1536 клітинок.
// Одна uint8_t-карта = 1536 байт.
//
// Дві додаткові карти:
//   dynamicMap  = наскільки зона характерно рухома;
//   oscillationMap = наскільки рух хаотичний/коливальний.
//
// Разом приблизно 4.5 KB RAM.
// Усі карти скидаються тільки після RESET/POWER OFF.
// =========================================================
#define GRID_COLS 48
#define GRID_ROWS 32
#define CELL_SIZE 250

#define MAP_X_MIN (-6000)
#define MAP_Y_MIN 0

uint8_t bgMap[GRID_COLS][GRID_ROWS] = {0};
uint8_t dynamicMap[GRID_COLS][GRID_ROWS] = {0};
uint8_t oscillationMap[GRID_COLS][GRID_ROWS] = {0};
// Адаптивна карта амплітуди рухомого фону: розширюється після стартового навчання
// тільки з non-HUMAN / IGNORE спостережень. Це не замінює bgMap.
uint8_t adaptiveBackgroundMap[GRID_COLS][GRID_ROWS] = {0};

// Максимальна сила комірки.
const uint8_t MAP_MAX = 100;

// Статичний фон.
const uint8_t CLUTTER_THRESHOLD = 65;

// 3x3 локальне вікно навколо цілі.
const int MAP_NEIGHBOR_RADIUS = 1;

// Поріг середнього локального статичного фону.
const uint8_t LOCAL_CLUTTER_THRESHOLD = 55;

// ENTRY VECTOR: a NEW target that starts near the field boundary and
// progresses inward is a strong human-entry signal.
const float ENTRY_EDGE_ANGLE = 55.0f;
const float ENTRY_FAR_Y = 6000.0f;
const float ENTRY_NEAR_Y = 700.0f;
const float ENTRY_EDGE_X = 4500.0f;
const uint8_t ENTRY_REQUIRED_INWARD_FRAMES = 3;
const float ENTRY_MIN_SCORE = 0.55f;
const float ENTRY_INWARD_COS_MIN = 0.35f;

// Динамічний фон: достатньо сильний профіль
// хаотичного/коливального руху.
const uint8_t DYNAMIC_CLUTTER_THRESHOLD = 55;

// Додатковий поріг хаотичності.
const uint8_t OSCILLATION_THRESHOLD = 45;

// Адаптивне розширення карти фону. Значення зберігаються тільки в RAM.
const uint8_t ADAPTIVE_BG_THRESHOLD = 35;
const uint8_t ADAPTIVE_BG_STRONG_THRESHOLD = 55;
const float ADAPTIVE_CONTINUITY_MAX_JUMP = 1800.0f;
const float ADAPTIVE_CONTINUITY_MAX_RANGE_DELTA = 1100.0f;
const float ADAPTIVE_CONTINUITY_MAX_ANGLE_DELTA = 14.0f;
const uint8_t ADAPTIVE_CONTINUITY_MIN_HISTORY = 6;
const uint8_t ADAPTIVE_LEARN_ADD = 2;
const uint8_t ADAPTIVE_LEARN_NEIGHBOR_ADD = 1;

// =========================================================
// НАВЧАННЯ
// =========================================================
const unsigned long TRAINING_TIME = 60000;

unsigned long bootTime = 0;
bool trainingFinished = false;

// Після навчання статична карта не змінюється.
// Динамічна карта може повільно навчатися характеру
// фонового руху (вітер, листя, гілки, трава).
unsigned long lastMapUpdate = 0;
const unsigned long MAP_UPDATE_INTERVAL = 250;

// Не дозволяємо динамічному навчанню початися
// одразу після завершення калібрування.
const unsigned long DYNAMIC_LEARNING_DELAY = 1500;

// =========================================================
// СТРУКТУРА ТРЕКУ
// =========================================================
enum TrackState { TRACK_OBSERVE=0, TRACK_HUMAN=1, TRACK_IGNORE=2 };

const uint8_t HISTORY_LEN = 20;
struct TrackPoint { float x,y; int16_t speed; unsigned long t; };

struct RadarTarget {
  bool active, isHuman, isClutter;
  TrackState state;
  float x,y,startX,startY,distance,angleOled,angleBle,prevX,prevY,dirX,dirY;
  float movementAccum,recentMovement;
  int directionalFrames;
  unsigned long birthTime,lastSeen,lastMovementTime,lastHumanTime,observeSince,lastRepeatTime;
  unsigned long aiLastPassTime, aiLowSince;
  bool aiConfirmedHuman;
  // TRACK STABILIZATION LOCK:
  // This is NOT a classifier latch. It is armed only AFTER the HUMAN
  // target has already been rendered to the user and (when BLE is
  // connected) notified. It protects target identity/re-association
  // so the same visible target does not keep being born as T1/T2/T3.
  bool humanLock;
  bool humanDisplayed;
  bool humanNotified;
  // HUMAN GHOST: keep a confirmed HUMAN briefly after full track dropout
  // so a short RD-03D disappearance does not destroy the classification.
  bool humanGhost;
  unsigned long humanGhostSince;
  // Effective ghost hold duration for THIS disappearance, decided once at
  // enterHumanGhost() based on the distance from the radar at that moment.
  // See HUMAN_GHOST_FULL_HOLD_MIN_DIST_MM / MAX_DIST_MM below. This is a
  // secondary, distance-based safeguard layered ON TOP OF the sole-candidate
  // handoff (soleActiveHumanGhostSlot/isSoleVegetationClearedCandidate) --
  // it does not replace that logic, it just shortens the window during
  // which a stale duplicate slot can exist at the edges of radar range.
  unsigned long humanGhostEffectiveTime;
  float humanGhostX, humanGhostY;
  float humanGhostVX, humanGhostVY;
  int ghostCandidateSlot;
  uint8_t ghostCandidateFrames;
  unsigned long ghostCandidateSince;
  float ghostCandidateX, ghostCandidateY;
  // Locked-HUMAN track re-association state.
  // A large radar slot jump is held briefly instead of immediately
  // destroying a confirmed human track.
  bool switchPending;
  uint8_t switchPendingFrames;
  unsigned long switchPendingSince;
  float switchPendingX, switchPendingY;
  int16_t switchPendingSpeed;
  // Pre-lock HUMAN continuity across a suspicious RD-03D slot jump/dropout.
  bool humanContinuityPending;
  uint8_t humanContinuityFrames;
  unsigned long humanContinuitySince;
  float humanContinuityX, humanContinuityY;
  int16_t humanContinuitySpeed;
  // Temporal classifier state. The compact score itself uses a 2 s
  // raw-history window. Final fresh-human confirmation uses a causal
  // 2-of-4 sample ring (one sample per second), which is intentionally
  // separate from HUMAN_LOCK.
  float temporalScore;
  uint8_t temporalPositiveCount;
  bool temporalReady;
  uint8_t temporalSampleFlags[4];
  float temporalSampleScores[4];
  uint8_t temporalSampleHead;
  uint8_t temporalSampleCount;
  unsigned long temporalLastSampleTime;
  // Sliding weighted HUMAN evidence window. +2 strong HUMAN, +1 weak HUMAN,
  // 0 neutral, -1 weak vegetation, -2 strong vegetation.
  int8_t humanEvidenceWindow[8];
  int8_t humanEvidenceSum;
  uint8_t humanEvidenceHead;
  uint8_t humanEvidenceCount;
  unsigned long humanEvidenceLastSample;
  uint16_t framesSeen,directionChanges;
  uint8_t chaoticScore,speedEvidenceFrames,historyCount,historyHead,directionReversals,repeatHits,orbitScore;
  int16_t radarSpeed;
  TrackPoint history[HISTORY_LEN];
  float pathLength,netDisplacement,pathRatio;
  bool entryFromEdge;
  uint8_t entryZone; // 0=none, 1=far, 2=left, 3=right, 4=near
  uint8_t entryFrames;
  uint8_t entryInwardFrames;
  float entryScore;
  float entryStartX, entryStartY;
};
RadarTarget targets[3];

// Per-frame strong human-candidate cache used by HUMAN GHOST handoff.
// It is refreshed once per radar frame before cross-slot reacquisition.
bool vegetationClearedThisFrame[3] = {false, false, false};

bool anyTargetDetected = false;
bool buzzerEnabled = true;

// =========================================================
// TTP223 TOUCH BUTTON (GPIO21) -- cosmetic/UX layer only.
// Does not touch classification, tracking, or any detection logic.
//   1 click            -> toggle buzzer on/off
//   hold >= 5s          -> reset scene background training
//   10 clicks in a row  -> play a short anthem excerpt on the buzzer
// =========================================================
const unsigned long TOUCH_DEBOUNCE_MS = 40;
const unsigned long TOUCH_CLICK_GAP_MS = 500;   // max gap between clicks to count as a click train
const unsigned long TOUCH_HOLD_MS = 5000;       // hold duration for training reset
const uint8_t TOUCH_ANTHEM_CLICK_COUNT = 10;

bool touchPinState = false;
bool touchPinStableState = false;
unsigned long touchLastChangeTime = 0;
unsigned long touchPressStartTime = 0;
bool touchHoldActionFired = false;
uint8_t touchClickCount = 0;
unsigned long touchLastClickTime = 0;
bool anthemPlaying = false;

// =========================================================
// ПАРАМЕТРИ ТРЕКІНГУ / ФІЛЬТРА
// =========================================================

// Alpha-фільтр.
const float TRACK_ALPHA = 0.30;

// Якщо сирий радар стрибнув сильніше цього,
// не вважаємо це продовженням старого треку.
const float TARGET_SWITCH_DISTANCE = 1300.0;

// Re-association for an already locked HUMAN.  The RD-03D target slot
// can occasionally jump to another reported position without the person
// actually disappearing.  We keep the locked track briefly and accept the
// new position only after several consecutive, spatially coherent frames.
const float HUMAN_REASSOC_MAX_JUMP = 2200.0;
const float HUMAN_REASSOC_CLUSTER_RADIUS = 350.0;
const uint8_t HUMAN_REASSOC_REQUIRED_FRAMES = 3;
const unsigned long HUMAN_REASSOC_TIMEOUT = 450;

// Pre-lock HUMAN continuity across a suspicious RD-03D slot jump.
const float HUMAN_CONTINUITY_MAX_JUMP = 2200.0;
const float HUMAN_CONTINUITY_CLUSTER_RADIUS = 400.0;
const uint8_t HUMAN_CONTINUITY_REQUIRED_FRAMES = 3;
const unsigned long HUMAN_CONTINUITY_TIMEOUT = 650;

// HUMAN GHOST: total additional grace after the normal 2 s HUMAN hold.
// During this period the old HUMAN position is retained and the firmware
// looks for a spatially compatible reacquisition in any radar slot.
const unsigned long HUMAN_GHOST_TIME = 3000;
// Variant B addition: at very close (<0.5m) or at/beyond range-edge (>=5m)
// distances, skip the full 3s ghost hold -- a wide-radius re-entry rarely
// lands near the linear-extrapolation prediction there anyway, and the
// sole-candidate handoff (below) already covers reacquisition when the
// person reappears as the only convincing candidate, regardless of distance.
// This constant only controls how long a STALE, unmatched ghost lingers
// before being force-cleared at the range edges.
const float HUMAN_GHOST_FULL_HOLD_MIN_DIST_MM = 500.0f;
const float HUMAN_GHOST_FULL_HOLD_MAX_DIST_MM = 5000.0f;
const unsigned long HUMAN_GHOST_TIME_EDGE_DISTANCE = 0;
const float HUMAN_GHOST_MAX_REACQUIRE_DISTANCE = 1800.0;
const float HUMAN_GHOST_MIN_GATE = 450.0;
const float HUMAN_GHOST_GATE_SPEED_GAIN = 0.90;
const float HUMAN_GHOST_GATE_TIME_GROWTH = 140.0;
const float HUMAN_GHOST_DIRECTION_COS_MIN = -0.35;
const float HUMAN_GHOST_CLUSTER_RADIUS = 400.0;
const uint8_t HUMAN_GHOST_REQUIRED_FRAMES = 3;
const unsigned long HUMAN_GHOST_CLUSTER_TIMEOUT = 650;
// Prevent a fresh slot from being promoted to HUMAN while it is
// plausibly the same person currently retained by another HUMAN GHOST.
// This is intentionally tighter than the full reacquisition gate: it only
// delays HUMAN classification for one frame/cycle and leaves the slot in
// TRACK_OBSERVE so cross-slot reacquisition can claim it first.
const float HUMAN_GHOST_PRECLASS_GATE = 550.0;

// Anti-switch safety: a reappearing candidate must remain close to the
// last confirmed HUMAN position. No AI reclassification is performed
// during reacquisition; the old HUMAN lock is preserved only on a coherent
// spatial return.

// Дуже маленький рух ігноруємо як шум.
const float MIN_MOVEMENT_STEP = 45.0;

// RD-03D reports radial speed in cm/s. Ignore sentinel/no-motion values.
const int16_t RADAR_SPEED_MIN = 12;
const int16_t RADAR_SPEED_MAX = 180;
const uint8_t SPEED_EVIDENCE_REQUIRED = 3;
const unsigned long OBSERVE_MIN_TIME = 1600;
const unsigned long OBSERVE_MAX_TIME = 5000;
const unsigned long IGNORE_CONFIRM_TIME = 2600;
const uint8_t HUMAN_SPEED_EVIDENCE_REQUIRED = 4;
const float HUMAN_MIN_DISPLACEMENT_V6 = 700.0;
const float HUMAN_MIN_PATH_V6 = 1000.0;
const float REPEAT_RADIUS = 325.0;
const unsigned long REPEAT_MIN_TIME = 650;
const uint8_t ORBIT_REPEAT_REQUIRED = 3;
const uint8_t ORBIT_REVERSAL_REQUIRED = 2;
const float ORBIT_PATH_RATIO = 2.8;
const float ORBIT_MAX_NET_DISPLACEMENT = 850.0;
const int16_t ORBIT_MAX_RADAR_SPEED = 10;


// Скільки послідовних напрямлених кадрів
// достатньо для сильного підтвердження.
const int HUMAN_DIRECTION_FRAMES = 5;
const uint16_t HUMAN_NO_DOPPLER_MIN_FRAMES = 12;
const float HUMAN_NO_DOPPLER_MIN_RECENT_STEP = 20.0;

// Кут між напрямками, який ще вважаємо
// одним напрямком руху.
const float DIRECTION_COS_THRESHOLD = 0.55;

// Для фонового руху (вітер/листя) напрямок часто змінюється.
const float CHAOTIC_COS_THRESHOLD = 0.15;
const uint8_t CHAOTIC_SCORE_STEP = 8;
const uint8_t CHAOTIC_SCORE_DECAY = 2;

// Динамічний фон навчається тільки за наявності
// ознак хаотичного руху в уже відомій області.
const float DYNAMIC_MIN_STEP = 35.0;
const float DYNAMIC_MAX_STEP = 500.0;

// Час, після якого повністю нерухомий непідтверджений
// об'єкт може стати clutter.
const unsigned long STATIC_CONFIRM_TIMEOUT = 5000;

// Після втрати руху вже підтверджена ціль
// не втрачає статус миттєво.
const unsigned long HUMAN_HOLD_TIME = 2000;

// Втрата кадрів треку для звичайних non-HUMAN цілей.
// HUMAN використовує окремий 2s hold + GHOST.
const unsigned long TARGET_LOST_TIMEOUT = 1200; // tolerate a short real frame dropout

// =========================================================
// ФІЛЬТР КРАЇВ
// =========================================================
static float edgeLastX[3] = {0};
static float edgeLastY[3] = {0};
static unsigned long edgeStaticStart[3] = {0};

// =========================================================
// БАТАРЕЯ
// =========================================================
float cachedVoltage = 0.0;
unsigned long lastBatteryCheck = 0;
const unsigned long BATTERY_UPDATE_INTERVAL = 600000;

// v0.9.9: ESP32-C3's built-in die temperature sensor, shown under BZ:ON/OFF.
// temperatureRead() is the Arduino-ESP32 core's own function (works on
// C3/S2/S3/C6/H2) -- no extra include needed.
float cachedTempC = 0.0;
unsigned long lastTempCheck = 0;
const unsigned long TEMP_UPDATE_INTERVAL = 5000;

// =========================================================
// OLED
// =========================================================
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 25;

// =========================================================
// BLE
// =========================================================
unsigned long lastBleUpdate = 0;
const unsigned long BLE_UPDATE_INTERVAL = 200;
unsigned long lastTrainingBleUpdate = 0;
bool trainingCompleteStatusSent = false;

// BLE log stream: mirrors the same datasetLog() stream used by USB Serial
// to a dedicated log characteristic. The Android app's BLE LOG: ON sends
// RAW_ON, which enables this mirror immediately. The stream contains both
// [RAW] frames and the compact [R]/[TR]/[AI]/[RS]/[CL]/[HG] diagnostics.
// RAW capture itself is independent and remains enabled all the time.
// Non-blocking BLE dataset queue: the radar loop never waits for BLE TX.
// If BLE falls behind, the queue drops the oldest queued chunk rather than
// blocking radar processing. USB Serial remains the authoritative complete
// log.
static const uint8_t BLE_DATASET_QUEUE_SIZE = 32;
// Max length of one logical log line (matches the largest snprintf buffer
// used for datasetLog(), e.g. [DECISION]). This is independent of the BLE
// packet payload limit -- long lines are split into multiple BLE packets
// by enqueueBleDataset() below, then reassembled by the Android app.
static const uint16_t BLE_DATASET_LINE_SIZE = 480;
// Max bytes of line text per BLE notification packet. MTU is negotiated to
// 517 (247 ATT payload), leaving headroom for the chunk-header prefix below.
static const uint16_t BLE_CHUNK_PAYLOAD_SIZE = 220;
char bleDatasetQueue[BLE_DATASET_QUEUE_SIZE][BLE_DATASET_LINE_SIZE];
volatile uint8_t bleDatasetHead = 0;
volatile uint8_t bleDatasetTail = 0;
volatile uint32_t bleDatasetDropped = 0;
unsigned long lastBleDatasetTx = 0;
const unsigned long BLE_DATASET_TX_INTERVAL = 10;

BLEServer* pServer = NULL;
// Telemetry characteristic: T1/T2/T3:V packets, parsed by the Android app.
BLECharacteristic* pCharacteristic = NULL;
// Log characteristic: a 1:1 mirror of every USB Serial log line. Kept
// separate from telemetry so the Android telemetry parser is never exposed
// to log text.
BLECharacteristic* pLogCharacteristic = NULL;
bool deviceConnected = false;
// Log stream is off until Android BLE LOG: ON sends RAW_ON. RAW capture
// itself is always enabled, independently of BLE state.
volatile bool bleLogStreamEnabled = false;
// RAW capture is deliberately permanent for this firmware build.
// BLE RAW_ON/RAW_OFF controls only the BLE mirror; it must never disable
// USB RAW capture. This prevents losing hours of field data by forgetting
// to send RAW_ON before a test.
bool rawCaptureEnabled = true;
// Last reconstructed RAW frame number emitted by logRawRD03DFrame().
uint32_t lastRawFrameNo = 0;

// Forward declarations for BLE-triggered scene-training reset.
void resetTarget(int i);
void resetSceneTraining();

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26aa"
// Dedicated log-mirror characteristic (separate from telemetry above).
#define LOG_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26ab"

void datasetLog(const char* line);
void datasetLogf(const char* fmt, ...);
void enqueueBleDataset(const char* line);
void serviceBleDatasetQueue();
void logRawRD03DFrame(uint32_t frameNo, TargetData* radarTargets[3]);

// =========================================================
// BUZZER
// =========================================================
unsigned long lastBeep = 0;
bool buzzerOn = false;
const unsigned long BEEP_DURATION = 45;

// =========================================================
// RD-03D LIBRARY
// =========================================================
// RD03D_Arduino 1.0.1 / javier-fg
// The library handles the 256000-baud UART protocol and
// provides up to 3 targets in MULTI_TARGET mode.
// =========================================================
RD03D radar(
  RADAR_RX_PIN,
  RADAR_TX_PIN,
  256000,
  &Serial1,
  1000
);

unsigned long lastRadarFrame = 0;
uint32_t radarFrameCount = 0;

// =========================================================
// BLE CALLBACKS
// =========================================================
class MyServerCallbacks : public BLEServerCallbacks {

  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    bleDatasetHead = bleDatasetTail = 0;
    Serial.println("BLE connected");

    // Always expose firmware identity once per BLE connection.
    if (pCharacteristic != nullptr) {
      char fwMsg[128];
      snprintf(fwMsg, sizeof(fwMsg),
               "FW:%s|BUILD:%s",
               SRS_FIRMWARE_NAME, SRS_BUILD_DATE);
      pCharacteristic->setValue(fwMsg);
      pCharacteristic->notify();
    }
  }

  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    bleLogStreamEnabled = false;
    bleDatasetHead = bleDatasetTail = 0;
    Serial.println("BLE disconnected");
    server->startAdvertising();
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pChar) override {

    String rxValue = pChar->getValue();

    if (rxValue.length() == 0)
      return;

    String value = rxValue.c_str();
    value.trim();

    datasetLogf("[B] CMD=%s", value.c_str());

    if (value == "BUZZER_OFF" || value == "B0") {

      buzzerEnabled = false;
      noTone(BUZZER_PIN);
      buzzerOn = false;

      datasetLog("[B] OFF");
    }

    else if (value == "BUZZER_ON" || value == "B1") {

      buzzerEnabled = true;

      datasetLog("[B] ON");
    }

    // Ручний reset 60-секундного навчання сцени через BLE.
    // Підтримуємо обидві команди для сумісності:
    // TRAINING_RESET — нова команда Android,
    // MAP_RESET      — стара команда попередніх версій.
    else if (value == "TRAINING_RESET" || value == "MAP_RESET") {

      resetSceneTraining();
    }

    // RAW_ON / RAW_OFF are kept for Android compatibility.
    // IMPORTANT: RAW capture itself is ALWAYS ON in ΩΞ 0.2. These commands
    // only start/stop the BLE log mirror. Therefore tapping BLE LOG: ON in
    // the Android app immediately starts receiving the RAW + compact stream,
    // while USB RAW capture continues regardless of BLE state.
    else if (value == "RAW_ON") {
      rawCaptureEnabled = true; // defensive; remains true permanently
      bleLogStreamEnabled = true;
      bleDatasetHead = bleDatasetTail = 0;
      datasetLog("[TM] RAW=ON BLE=ON");
    }

    else if (value == "RAW_OFF") {
      // Do NOT disable rawCaptureEnabled here. RAW capture must continue.
      bleLogStreamEnabled = false;
      bleDatasetHead = bleDatasetTail = 0;
      Serial.println("[TM] RAW=OFF BLE=OFF (RAW capture remains ON)");
    }

    else if (value == "GET_FW") {

      char fwMsg[128];
      snprintf(fwMsg, sizeof(fwMsg),
               "FW:%s|BUILD:%s",
               SRS_FIRMWARE_NAME, SRS_BUILD_DATE);

      if (deviceConnected && pCharacteristic != nullptr) {
        pCharacteristic->setValue(fwMsg);
        pCharacteristic->notify();
      }
    }
  }
};

// =========================================================
// GRID
// =========================================================
int getGridX(float x) {

  int gx =
    (int)floorf(
      (x - MAP_X_MIN) / CELL_SIZE
    );

  return constrain(
    gx,
    0,
    GRID_COLS - 1
  );
}

int getGridY(float y) {

  int gy =
    (int)floorf(
      (y - MAP_Y_MIN) / CELL_SIZE
    );

  return constrain(
    gy,
    0,
    GRID_ROWS - 1
  );
}

// =========================================================
// ВІДСТАНЬ
// =========================================================
float distance2D(
  float x1,
  float y1,
  float x2,
  float y2
) {

  float dx = x1 - x2;
  float dy = y1 - y2;

  return sqrtf(
    dx * dx +
    dy * dy
  );
}

// =========================================================
// НАПРЯМОК
// =========================================================
bool updateMovementDirection(
  int i,
  float dx,
  float dy
) {

  float step =
    sqrtf(
      dx * dx +
      dy * dy
    );

  targets[i].recentMovement = step;

  if (step < MIN_MOVEMENT_STEP) {
    // Хаотичність повільно згасає, але не зникає миттєво.
    if (targets[i].chaoticScore > CHAOTIC_SCORE_DECAY)
      targets[i].chaoticScore -= CHAOTIC_SCORE_DECAY;
    else
      targets[i].chaoticScore = 0;

    return false;
  }

  float nx = dx / step;
  float ny = dy / step;

  bool hadDirection =
    fabsf(targets[i].dirX) > 0.01 ||
    fabsf(targets[i].dirY) > 0.01;

  if (hadDirection) {

    float dot =
      nx * targets[i].dirX +
      ny * targets[i].dirY;

    // Послідовний напрямок.
    if (dot >= DIRECTION_COS_THRESHOLD) {

      if (targets[i].directionalFrames < 1000)
        targets[i].directionalFrames++;

      if (targets[i].chaoticScore > CHAOTIC_SCORE_DECAY)
        targets[i].chaoticScore -= CHAOTIC_SCORE_DECAY;
      else
        targets[i].chaoticScore = 0;
    }

    // Різка зміна напрямку.
    else {

      targets[i].directionChanges++;

      // Справжній реверс — рух у майже протилежному напрямку.
      // Це окремо від directionChanges: ортогональні/помірні
      // зміни напрямку не повинні рахуватися як reversal.
      if (dot <= -DIRECTION_COS_THRESHOLD) {
        if (targets[i].directionReversals < 255)
          targets[i].directionReversals++;
      }

      if (targets[i].directionalFrames > 2)
        targets[i].directionalFrames -= 2;
      else
        targets[i].directionalFrames = 0;

      if (targets[i].chaoticScore <=
          100 - CHAOTIC_SCORE_STEP)
        targets[i].chaoticScore += CHAOTIC_SCORE_STEP;
      else
        targets[i].chaoticScore = 100;
    }

    // Майже протилежний/ортогональний рух
    // особливо характерний для коливань.
    if (dot < CHAOTIC_COS_THRESHOLD) {

      if (targets[i].chaoticScore <=
          100 - CHAOTIC_SCORE_STEP)
        targets[i].chaoticScore += CHAOTIC_SCORE_STEP;
      else
        targets[i].chaoticScore = 100;
    }
  }
  else {

    targets[i].directionalFrames = 1;
  }

  targets[i].dirX = nx;
  targets[i].dirY = ny;

  return true;
}

// =========================================================
// ЛОКАЛЬНИЙ АНАЛІЗ МАПИ
// =========================================================

uint16_t getLocalMapScore(
  float x,
  float y
) {

  int gx = getGridX(x);
  int gy = getGridY(y);

  uint16_t sum = 0;
  uint8_t count = 0;

  for (
    int dx = -MAP_NEIGHBOR_RADIUS;
    dx <= MAP_NEIGHBOR_RADIUS;
    dx++
  ) {

    for (
      int dy = -MAP_NEIGHBOR_RADIUS;
      dy <= MAP_NEIGHBOR_RADIUS;
      dy++
    ) {

      int cx = gx + dx;
      int cy = gy + dy;

      if (
        cx >= 0 &&
        cx < GRID_COLS &&
        cy >= 0 &&
        cy < GRID_ROWS
      ) {

        sum += bgMap[cx][cy];
        count++;
      }
    }
  }

  if (count == 0)
    return 0;

  return sum / count;
}

uint16_t getLocalDynamicScore(
  float x,
  float y
) {

  int gx = getGridX(x);
  int gy = getGridY(y);

  uint16_t sum = 0;
  uint8_t count = 0;

  for (
    int dx = -MAP_NEIGHBOR_RADIUS;
    dx <= MAP_NEIGHBOR_RADIUS;
    dx++
  ) {

    for (
      int dy = -MAP_NEIGHBOR_RADIUS;
      dy <= MAP_NEIGHBOR_RADIUS;
      dy++
    ) {

      int cx = gx + dx;
      int cy = gy + dy;

      if (
        cx >= 0 &&
        cx < GRID_COLS &&
        cy >= 0 &&
        cy < GRID_ROWS
      ) {

        sum += dynamicMap[cx][cy];
        count++;
      }
    }
  }

  if (count == 0)
    return 0;

  return sum / count;
}

uint16_t getLocalOscillationScore(
  float x,
  float y
) {

  int gx = getGridX(x);
  int gy = getGridY(y);

  uint16_t sum = 0;
  uint8_t count = 0;

  for (
    int dx = -MAP_NEIGHBOR_RADIUS;
    dx <= MAP_NEIGHBOR_RADIUS;
    dx++
  ) {

    for (
      int dy = -MAP_NEIGHBOR_RADIUS;
      dy <= MAP_NEIGHBOR_RADIUS;
      dy++
    ) {

      int cx = gx + dx;
      int cy = gy + dy;

      if (
        cx >= 0 &&
        cx < GRID_COLS &&
        cy >= 0 &&
        cy < GRID_ROWS
      ) {

        sum += oscillationMap[cx][cy];
        count++;
      }
    }
  }

  if (count == 0)
    return 0;

  return sum / count;
}

// =========================================================
// ВІДОМИЙ СТАТИЧНИЙ ФОН
// =========================================================
bool isKnownStaticBackground(
  float x,
  float y
) {

  int gx = getGridX(x);
  int gy = getGridY(y);

  uint8_t maxCell = 0;
  uint16_t localScore = 0;
  uint8_t count = 0;

  for (
    int dx = -MAP_NEIGHBOR_RADIUS;
    dx <= MAP_NEIGHBOR_RADIUS;
    dx++
  ) {

    for (
      int dy = -MAP_NEIGHBOR_RADIUS;
      dy <= MAP_NEIGHBOR_RADIUS;
      dy++
    ) {

      int cx = gx + dx;
      int cy = gy + dy;

      if (
        cx >= 0 &&
        cx < GRID_COLS &&
        cy >= 0 &&
        cy < GRID_ROWS
      ) {

        if (bgMap[cx][cy] > maxCell)
          maxCell = bgMap[cx][cy];

        localScore += bgMap[cx][cy];
        count++;
      }
    }
  }

  if (count == 0)
    return false;

  localScore /= count;

  if (maxCell >= CLUTTER_THRESHOLD)
    return true;

  if (localScore >= LOCAL_CLUTTER_THRESHOLD)
    return true;

  return false;
}

// =========================================================
// АДАПТИВНА КАРТА РУХОМОГО ФОНУ
//
// bgMap = стартовий просторовий prior 60-секундної сцени.
// adaptiveBackgroundMap = накопичений після старту коридор руху
// рослинності/іншого non-HUMAN фону. Людина ніколи сюди не записується.
//
// Важливий принцип: карта не робить довільну сусідню клітинку clutter.
// Вона лише запам'ятовує реально побачений рух у вже відомому фоні.
// =========================================================
uint16_t getLocalAdaptiveBackgroundScore(float x, float y) {
  int gx = getGridX(x);
  int gy = getGridY(y);
  uint16_t sum = 0;
  uint8_t count = 0;
  for (int dx = -MAP_NEIGHBOR_RADIUS; dx <= MAP_NEIGHBOR_RADIUS; dx++) {
    for (int dy = -MAP_NEIGHBOR_RADIUS; dy <= MAP_NEIGHBOR_RADIUS; dy++) {
      int cx = gx + dx;
      int cy = gy + dy;
      if (cx >= 0 && cx < GRID_COLS && cy >= 0 && cy < GRID_ROWS) {
        sum += adaptiveBackgroundMap[cx][cy];
        count++;
      }
    }
  }
  return count ? sum / count : 0;
}

bool isKnownAdaptiveBackground(float x, float y) {
  return getLocalAdaptiveBackgroundScore(x, y) >= ADAPTIVE_BG_THRESHOLD;
}

void learnAdaptiveBackgroundPoint(float x, float y, uint8_t amount) {
  int gx = getGridX(x);
  int gy = getGridY(y);
  for (int dx = -MAP_NEIGHBOR_RADIUS; dx <= MAP_NEIGHBOR_RADIUS; dx++) {
    for (int dy = -MAP_NEIGHBOR_RADIUS; dy <= MAP_NEIGHBOR_RADIUS; dy++) {
      int cx = gx + dx;
      int cy = gy + dy;
      if (cx < 0 || cx >= GRID_COLS || cy < 0 || cy >= GRID_ROWS) continue;
      uint8_t add = (dx == 0 && dy == 0) ? amount : ADAPTIVE_LEARN_NEIGHBOR_ADD;
      uint16_t v = adaptiveBackgroundMap[cx][cy] + add;
      adaptiveBackgroundMap[cx][cy] = (uint8_t)min((uint16_t)MAP_MAX, v);
    }
  }
}

// Шукаємо продовження старого non-HUMAN треку після великого стрибка.
// Умови навмисно незалежні від slot ID: важливі напрямок, дальність та
// близькість до карти відомого рухомого фону.
bool tryAdaptiveBackgroundContinuation(int i, float rawX, float rawY, int16_t rawSpeed,
                                       unsigned long now) {
  RadarTarget &t = targets[i];
  if (!t.active || t.state == TRACK_HUMAN || t.humanLock) return false;
  if (!trainingFinished || t.historyCount < ADAPTIVE_CONTINUITY_MIN_HISTORY) return false;

  float oldRange = sqrtf(t.x * t.x + t.y * t.y);
  float newRange = sqrtf(rawX * rawX + rawY * rawY);
  float oldAngle = atan2f(t.x, t.y) * 57.2957795f;
  float newAngle = atan2f(rawX, rawY) * 57.2957795f;
  float angleDelta = fabsf(newAngle - oldAngle);
  if (angleDelta > 180.0f) angleDelta = 360.0f - angleDelta;

  if (fabsf(newRange - oldRange) > ADAPTIVE_CONTINUITY_MAX_RANGE_DELTA) return false;
  if (angleDelta > ADAPTIVE_CONTINUITY_MAX_ANGLE_DELTA) return false;
  if (distance2D(rawX, rawY, t.x, t.y) > ADAPTIVE_CONTINUITY_MAX_JUMP) return false;

  // Пряма перевірка нової позиції та її 3x3 околиці.
  bool mapEvidence = isKnownStaticBackground(rawX, rawY) ||
                     isKnownAdaptiveBackground(rawX, rawY) ||
                     getLocalDynamicScore(rawX, rawY) >= ADAPTIVE_BG_THRESHOLD;
  if (!mapEvidence) return false;

  // Додаткова перевірка: старий трек сам повинен бути схожим на non-HUMAN фон.
  bool oldBackgroundEvidence =
    t.state == TRACK_IGNORE ||
    isKnownStaticBackground(t.x, t.y) ||
    isKnownAdaptiveBackground(t.x, t.y) ||
    getLocalDynamicScore(t.x, t.y) >= ADAPTIVE_BG_THRESHOLD;
  if (!oldBackgroundEvidence) return false;

  float acceptedJump = distance2D(rawX, rawY, t.x, t.y);

  // Приймаємо позицію без startNewTarget(): історія, repeat/path/reversal
  // та всі накопичені ознаки залишаються на тому ж логічному треку.
  t.x = rawX;
  t.y = rawY;
  t.prevX = rawX;
  t.prevY = rawY;
  t.radarSpeed = rawSpeed;
  t.recentMovement = 0.0f;
  t.lastSeen = now;
  t.isHuman = false;
  t.isClutter = true;
  if (t.state != TRACK_IGNORE) t.state = TRACK_IGNORE;
  t.aiConfirmedHuman = false;
  t.aiLowSince = 0;
  t.switchPending = false;
  t.switchPendingFrames = 0;
  t.switchPendingSince = 0;

  // Нову межу руху запам'ятовуємо як фон лише тому, що вся перевірка
  // continuity вже довела її зв'язок з відомим non-HUMAN коридором.
  learnAdaptiveBackgroundPoint(rawX, rawY, ADAPTIVE_LEARN_ADD);

  datasetLogf("[TM] AD T%d jump=%.0f range=%.0f->%.0f angle=%.1f->%.1f map=%u\n",
                i + 1, acceptedJump,
                oldRange, newRange, oldAngle, newAngle,
                (unsigned)getLocalAdaptiveBackgroundScore(rawX, rawY));
  return true;
}

// =========================================================
// ДИНАМІЧНИЙ ФОН
//
// Важливо: сама наявність dynamicMap не достатня.
// Потрібно, щоб поточна ціль демонструвала хаотичний
// характер руху.
// =========================================================
bool isKnownDynamicBackground(
  float x,
  float y
) {

  uint16_t dynamicScore =
    getLocalDynamicScore(x, y);

  uint16_t oscillationScore =
    getLocalOscillationScore(x, y);

  return (
    dynamicScore >= DYNAMIC_CLUTTER_THRESHOLD &&
    oscillationScore >= OSCILLATION_THRESHOLD
  );
}

// =========================================================
// ДОДАТИ СТАТИЧНИЙ ФОН
// =========================================================
void learnMapPoint(
  float x,
  float y,
  uint8_t amount
) {

  int gx = getGridX(x);
  int gy = getGridY(y);

  for (
    int dx = -MAP_NEIGHBOR_RADIUS;
    dx <= MAP_NEIGHBOR_RADIUS;
    dx++
  ) {

    for (
      int dy = -MAP_NEIGHBOR_RADIUS;
      dy <= MAP_NEIGHBOR_RADIUS;
      dy++
    ) {

      int cx = gx + dx;
      int cy = gy + dy;

      if (
        cx < 0 ||
        cx >= GRID_COLS ||
        cy < 0 ||
        cy >= GRID_ROWS
      )
        continue;

      uint8_t add = amount;

      if (dx != 0 || dy != 0)
        add = amount / 2;

      uint16_t newValue =
        bgMap[cx][cy] + add;

      if (newValue > MAP_MAX)
        newValue = MAP_MAX;

      bgMap[cx][cy] =
        (uint8_t)newValue;
    }
  }
}

float vegetationPatternScore(int i);

// =========================================================
// ДОДАТИ ДИНАМІЧНИЙ ФОН
//
// Записуємо тільки невеликі/середні переміщення,
// які відбуваються в уже відомій статичній області
// і мають хаотичний характер.
// =========================================================
void learnDynamicPoint(
  float x,
  float y,
  float step,
  uint8_t evidenceScore
) {
  if (step < DYNAMIC_MIN_STEP || step > DYNAMIC_MAX_STEP) return;

  // v0.9: learning is driven by independent vegetation/background evidence,
  // not by chaoticScore. chaoticScore remains a neural-network feature and
  // diagnostic, but is no longer a manual prerequisite for map learning.
  if (evidenceScore < 30) return;

  int gx = getGridX(x);
  int gy = getGridY(y);
  uint8_t amount = 1 + evidenceScore / 40;
  if (amount > 4) amount = 4;

  for (int dx = -MAP_NEIGHBOR_RADIUS; dx <= MAP_NEIGHBOR_RADIUS; dx++) {
    for (int dy = -MAP_NEIGHBOR_RADIUS; dy <= MAP_NEIGHBOR_RADIUS; dy++) {
      int cx = gx + dx, cy = gy + dy;
      if (cx < 0 || cx >= GRID_COLS || cy < 0 || cy >= GRID_ROWS) continue;
      uint8_t add = (dx != 0 || dy != 0) ? amount / 2 : amount;
      uint16_t nv = dynamicMap[cx][cy] + add;
      if (nv > MAP_MAX) nv = MAP_MAX;
      dynamicMap[cx][cy] = (uint8_t)nv;
      uint8_t oscAdd = (uint8_t)(1 + evidenceScore / 50);
      uint16_t no = oscillationMap[cx][cy] + oscAdd;
      if (no > MAP_MAX) no = MAP_MAX;
      oscillationMap[cx][cy] = (uint8_t)no;
    }
  }
}

// =========================================================
// ПОКАЗ СТАРТУ
// =========================================================
static void drawCenteredText(const char* text, int16_t y, uint8_t size) {
  display.setTextSize(size);
  display.setTextColor(ST77XX_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - (int16_t)w) / 2;
  if (x < 0) x = 0;

  display.setCursor(x, y);
  display.print(text);
}

static void drawCenteredU8g2Text(const char* text, int16_t baselineY) {
  u8g2Fonts.setFont(u8g2_font_helvB14_tr);
  u8g2Fonts.setFontMode(1);
  u8g2Fonts.setFontDirection(0);
  u8g2Fonts.setForegroundColor(ST77XX_WHITE);

  int16_t width = u8g2Fonts.getUTF8Width(text);
  int16_t x = (SCREEN_WIDTH - width) / 2;
  if (x < 0) x = 0;

  u8g2Fonts.drawUTF8(x, baselineY, text);
}

static void drawCenteredUtf8(const char* text, int16_t baselineY) {
  // U8g2_for_Adafruit_GFX provides UTF-8/Unicode glyph rendering, including Ω.
  u8g2Fonts.setFont(u8g2_font_unifont_t_greek);
  u8g2Fonts.setFontMode(1);
  u8g2Fonts.setFontDirection(0);
  u8g2Fonts.setForegroundColor(ST77XX_WHITE);

  int16_t width = u8g2Fonts.getUTF8Width(text);
  int16_t x = (SCREEN_WIDTH - width) / 2;
  if (x < 0) x = 0;

  u8g2Fonts.drawUTF8(x, baselineY, text);
}

void showStartup() {

  // SCREEN 1 — SYSTEM IDENTITY
  display.fillScreen(ST77XX_BLACK);
  drawCenteredText("SRS", 78, 4);
  drawCenteredU8g2Text("Spartak Radar Systems", 138);
  delay(2200);

  // SCREEN 2 — MISSION / SYSTEM ROLE
  display.fillScreen(ST77XX_BLACK);
  drawCenteredText("TRENCH", 70, 3);
  drawCenteredText("SURVEILLANCE", 110, 2);
  drawCenteredText("PIDR DETECTION SYSTEM", 140, 1);
  delay(2600);

  // SCREEN 3 — INITIALIZATION + REAL UNICODE FIRMWARE ID
  display.fillScreen(ST77XX_BLACK);
  drawCenteredText("INITIALIZATION", 80, 1);
  drawCenteredUtf8(SRS_FIRMWARE_NAME, 104);
  drawCenteredU8g2Text(SRS_BUILD_DATE, 128);

  // Progress indicator.
  display.drawRect(15, 140, 210, 16, ST77XX_WHITE);

  for (int i = 0; i <= 18; i++) {
    int16_t fillWidth = 6 + (i * 11);
    if (fillWidth > 206) fillWidth = 206;
    display.fillRect(17, 142, fillWidth, 12, ST77XX_WHITE);
    delay(100);
  }

  delay(900);
}

// =========================================================
// БАТАРЕЯ
// =========================================================
float readBatteryVoltage() {

  long sum = 0;

  for (int i = 0; i < 64; i++) {

    sum += analogRead(BATTERY_PIN);

    delayMicroseconds(100);
  }

  return ((sum / 64.0) / 4095.0) * 6.06;
}

// =========================================================
// SIGN MAGNITUDE
// =========================================================
int16_t decodeSignMagnitude(
  uint8_t low,
  uint8_t high
) {

  uint16_t raw =
    low |
    ((uint16_t)high << 8);

  int16_t value =
    raw & 0x7FFF;

  if (!(raw & 0x8000))
    value = -value;

  return value;
}

// =========================================================
// RESET TARGET
// =========================================================
void resetTarget(int i) {

  targets[i].active = false;
  targets[i].isHuman = false;
  targets[i].isClutter = false;
  targets[i].state = TRACK_OBSERVE;

  targets[i].x = 0;
  targets[i].y = 0;

  targets[i].startX = 0;
  targets[i].startY = 0;

  targets[i].distance = 0;

  targets[i].angleOled = 90.0;
  targets[i].angleBle = 0.0;

  targets[i].prevX = 0;
  targets[i].prevY = 0;

  targets[i].dirX = 0;
  targets[i].dirY = 0;

  targets[i].movementAccum = 0;
  targets[i].recentMovement = 0;

  targets[i].directionalFrames = 0;

  targets[i].birthTime = 0;
  targets[i].lastSeen = 0;
  targets[i].lastMovementTime = 0;
  targets[i].lastHumanTime = 0;
  targets[i].aiLastPassTime = 0;
  targets[i].aiLowSince = 0;
  targets[i].aiConfirmedHuman = false;
  targets[i].humanLock = false;
  targets[i].humanDisplayed = false;
  targets[i].humanNotified = false;
  targets[i].humanGhost = false;
  targets[i].humanGhostSince = 0;
  targets[i].humanGhostEffectiveTime = 0;
  targets[i].humanGhostX = 0;
  targets[i].humanGhostY = 0;
  targets[i].humanGhostVX = 0;
  targets[i].humanGhostVY = 0;
  targets[i].ghostCandidateSlot = -1;
  targets[i].ghostCandidateFrames = 0;
  targets[i].ghostCandidateSince = 0;
  targets[i].ghostCandidateX = 0;
  targets[i].ghostCandidateY = 0;
  targets[i].switchPending = false;
  targets[i].switchPendingFrames = 0;
  targets[i].switchPendingSince = 0;
  targets[i].switchPendingX = 0;
  targets[i].switchPendingY = 0;
  targets[i].switchPendingSpeed = 0;
  targets[i].humanContinuityPending = false;
  targets[i].humanContinuityFrames = 0;
  targets[i].humanContinuitySince = 0;
  targets[i].humanContinuityX = 0;
  targets[i].humanContinuityY = 0;
  targets[i].humanContinuitySpeed = 0;
  targets[i].temporalScore = 0.0f;
  targets[i].temporalPositiveCount = 0;
  targets[i].temporalReady = false;
  memset(targets[i].temporalSampleFlags, 0, sizeof(targets[i].temporalSampleFlags));
  memset(targets[i].temporalSampleScores, 0, sizeof(targets[i].temporalSampleScores));
  targets[i].temporalSampleHead = 0;
  targets[i].temporalSampleCount = 0;
  targets[i].temporalLastSampleTime = 0;
  memset(targets[i].humanEvidenceWindow, 0, sizeof(targets[i].humanEvidenceWindow));
  targets[i].humanEvidenceSum = 0;
  targets[i].humanEvidenceHead = 0;
  targets[i].humanEvidenceCount = 0;
  targets[i].humanEvidenceLastSample = 0;

  targets[i].framesSeen = 0;

  targets[i].directionChanges = 0;
  targets[i].chaoticScore = 0;
  targets[i].radarSpeed = 0;
  targets[i].speedEvidenceFrames = 0;
  targets[i].historyCount = 0;
  targets[i].historyHead = 0;
  targets[i].pathLength = 0;
  targets[i].netDisplacement = 0;
  targets[i].pathRatio = 0;
  targets[i].entryFromEdge = false;
  targets[i].entryZone = 0;
  targets[i].entryFrames = 0;
  targets[i].entryInwardFrames = 0;
  targets[i].entryScore = 0.0f;
  targets[i].entryStartX = 0.0f;
  targets[i].entryStartY = 0.0f;
  targets[i].directionReversals = 0;
  targets[i].repeatHits = 0;
  targets[i].orbitScore = 0;
  targets[i].lastRepeatTime = 0;
  targets[i].observeSince = 0;
  for (int h=0; h<HISTORY_LEN; h++) {
    targets[i].history[h].x=0; targets[i].history[h].y=0;
    targets[i].history[h].speed=0; targets[i].history[h].t=0;
  }

  edgeLastX[i] = 0;
  edgeLastY[i] = 0;
  edgeStaticStart[i] = 0;
}

// =========================================================
// RESET 60-SECOND SCENE TRAINING
// =========================================================
// Called from Android over BLE. This is a true restart of the
// scene-learning window, not merely a visual timer reset:
// maps and software tracks are cleared so the next 60 seconds
// start from a clean scene.
void resetSceneTraining() {
  trainingCompleteStatusSent = false;
  lastTrainingBleUpdate = 0;

  unsigned long now = millis();

  // Stop any active alarm immediately.
  noTone(BUZZER_PIN);
  buzzerOn = false;
  anyTargetDetected = false;
  lastBeep = now;

  // Clear all scene/background maps.
  memset(bgMap, 0, sizeof(bgMap));
  memset(dynamicMap, 0, sizeof(dynamicMap));
  memset(oscillationMap, 0, sizeof(oscillationMap));
  memset(adaptiveBackgroundMap, 0, sizeof(adaptiveBackgroundMap));

  // Clear all software tracks so the new training starts cleanly.
  for (int i = 0; i < 3; i++) {
    resetTarget(i);
  }

  // Restart the exact same 60-second training window used at boot.
  bootTime = now;
  lastMapUpdate = 0;
  trainingFinished = false;

  datasetLog("[TM] TRAIN_RESET");
}

// =========================================================
// СТВОРЕННЯ НОВОЇ ЦІЛІ
// =========================================================
void startNewTarget(
  int i,
  float rawX,
  float rawY,
  int16_t rawSpeed,
  unsigned long now
) {

  targets[i].active = true;
  // Новий трек після SWITCH має починати класифікацію з нуля.
  targets[i].state = TRACK_OBSERVE;

  targets[i].isHuman = false;
  targets[i].isClutter = false;

  targets[i].x = rawX;
  targets[i].y = rawY;

  targets[i].startX = rawX;
  targets[i].startY = rawY;

  targets[i].prevX = rawX;
  targets[i].prevY = rawY;

  targets[i].movementAccum = 0;
  targets[i].recentMovement = 0;

  targets[i].dirX = 0;
  targets[i].dirY = 0;

  targets[i].directionalFrames = 0;

  targets[i].birthTime = now;
  targets[i].lastSeen = now;
  targets[i].lastMovementTime = now;
  targets[i].lastHumanTime = 0;
  targets[i].aiLastPassTime = 0;
  targets[i].aiLowSince = 0;
  targets[i].aiConfirmedHuman = false;
  targets[i].humanLock = false;
  targets[i].humanDisplayed = false;
  targets[i].humanNotified = false;
  targets[i].humanGhost = false;
  targets[i].humanGhostSince = 0;
  targets[i].humanGhostEffectiveTime = 0;
  targets[i].humanGhostX = 0;
  targets[i].humanGhostY = 0;
  targets[i].humanGhostVX = 0;
  targets[i].humanGhostVY = 0;
  targets[i].ghostCandidateSlot = -1;
  targets[i].ghostCandidateFrames = 0;
  targets[i].ghostCandidateSince = 0;
  targets[i].ghostCandidateX = 0;
  targets[i].ghostCandidateY = 0;
  targets[i].switchPending = false;
  targets[i].switchPendingFrames = 0;
  targets[i].switchPendingSince = 0;
  targets[i].switchPendingX = 0;
  targets[i].switchPendingY = 0;
  targets[i].switchPendingSpeed = 0;
  targets[i].humanContinuityPending = false;
  targets[i].humanContinuityFrames = 0;
  targets[i].humanContinuitySince = 0;
  targets[i].humanContinuityX = 0;
  targets[i].humanContinuityY = 0;
  targets[i].humanContinuitySpeed = 0;
  targets[i].temporalScore = 0.0f;
  targets[i].temporalPositiveCount = 0;
  targets[i].temporalReady = false;
  memset(targets[i].temporalSampleFlags, 0, sizeof(targets[i].temporalSampleFlags));
  memset(targets[i].temporalSampleScores, 0, sizeof(targets[i].temporalSampleScores));
  targets[i].temporalSampleHead = 0;
  targets[i].temporalSampleCount = 0;
  targets[i].temporalLastSampleTime = 0;
  memset(targets[i].humanEvidenceWindow, 0, sizeof(targets[i].humanEvidenceWindow));
  targets[i].humanEvidenceSum = 0;
  targets[i].humanEvidenceHead = 0;
  targets[i].humanEvidenceCount = 0;
  targets[i].humanEvidenceLastSample = 0;

  targets[i].framesSeen = 1;
  targets[i].directionChanges = 0;
  targets[i].chaoticScore = 0;
  targets[i].radarSpeed = rawSpeed;
  targets[i].speedEvidenceFrames = 0;
  targets[i].historyCount = 1;
  targets[i].historyHead = 0;
  targets[i].history[0] = {rawX, rawY, rawSpeed, now};
  targets[i].pathLength = 0;
  targets[i].netDisplacement = 0;
  targets[i].pathRatio = 0;
  initEntryVector(i, rawX, rawY);
  targets[i].directionReversals = 0;
  targets[i].repeatHits = 0;
  targets[i].orbitScore = 0;
  targets[i].lastRepeatTime = 0;
  targets[i].observeSince = now;
}

// =========================================================
// ENTRY VECTOR ANALYSIS
// =========================================================
uint8_t detectEntryZone(float x, float y) {
  float angle = fabsf(atan2f(x, y) * 57.2957795f);
  if (y >= ENTRY_FAR_Y) return 1;
  if (y <= ENTRY_NEAR_Y) return 4;
  if (x < 0 && angle >= ENTRY_EDGE_ANGLE) return 2;
  if (x >= 0 && angle >= ENTRY_EDGE_ANGLE) return 3;
  if (x <= -ENTRY_EDGE_X) return 2;
  if (x >= ENTRY_EDGE_X) return 3;
  return 0;
}

bool isEntryFromEdge(int i) {
  RadarTarget &t = targets[i];
  return t.entryFromEdge &&
         t.entryInwardFrames >= ENTRY_REQUIRED_INWARD_FRAMES &&
         t.entryScore >= ENTRY_MIN_SCORE;
}

void initEntryVector(int i, float x, float y) {
  RadarTarget &t = targets[i];
  t.entryZone = detectEntryZone(x, y);
  t.entryFromEdge = (t.entryZone != 0);
  t.entryFrames = 1;
  t.entryInwardFrames = 0;
  t.entryScore = t.entryFromEdge ? 0.35f : 0.0f;
  t.entryStartX = x;
  t.entryStartY = y;
}

void updateEntryVector(int i, float dx, float dy) {
  RadarTarget &t = targets[i];
  if (!t.entryFromEdge) return;
  if (t.entryFrames < 255) t.entryFrames++;
  float startLen = sqrtf(t.entryStartX*t.entryStartX + t.entryStartY*t.entryStartY);
  float moveLen = sqrtf(dx*dx + dy*dy);
  if (startLen < 100.0f || moveLen < MIN_MOVEMENT_STEP) return;
  float inwardX = -t.entryStartX / startLen;
  float inwardY = -t.entryStartY / startLen;
  float moveX = dx / moveLen;
  float moveY = dy / moveLen;
  float cosInward = inwardX*moveX + inwardY*moveY;
  if (cosInward >= ENTRY_INWARD_COS_MIN) {
    if (t.entryInwardFrames < 255) t.entryInwardFrames++;
    t.entryScore += 0.16f;
    if (t.entryScore > 1.0f) t.entryScore = 1.0f;
  } else if (cosInward < -0.25f) {
    if (t.entryInwardFrames > 0) t.entryInwardFrames--;
    t.entryScore -= 0.05f;
    if (t.entryScore < 0.0f) t.entryScore = 0.0f;
  }
}

// =========================================================
// V6 TRAJECTORY ANALYSIS
// Doppler is supporting evidence; spatial trajectory can also qualify.
// =========================================================
void pushHistory(int i, float x, float y, int16_t speed, unsigned long now) {
  RadarTarget &t=targets[i];
  if(t.historyCount>0) {
    TrackPoint &q=t.history[t.historyHead];
    float d=distance2D(x,y,q.x,q.y);
    if(d>=MIN_MOVEMENT_STEP) t.pathLength+=d;
  }
  t.historyHead=(t.historyHead+1)%HISTORY_LEN;
  t.history[t.historyHead]={x,y,speed,now};
  if(t.historyCount<HISTORY_LEN) t.historyCount++;
  t.netDisplacement=distance2D(x,y,t.startX,t.startY);
  t.pathRatio=t.pathLength/(t.netDisplacement>1.0?t.netDisplacement:1.0);
}

void analyzeTrajectory(int i,unsigned long now) {
  RadarTarget &t=targets[i];
  if(t.historyCount<6) return;
  uint8_t hits=0;
  for(int k=0;k<HISTORY_LEN;k++) {
    if(k==t.historyHead || t.history[k].t==0) continue;
    if(now-t.history[k].t<REPEAT_MIN_TIME) continue;
    if(distance2D(t.x,t.y,t.history[k].x,t.history[k].y)<=REPEAT_RADIUS) hits++;
  }
  if(hits) { if(t.repeatHits<255)t.repeatHits++; t.lastRepeatTime=now; }
  else if(t.repeatHits && now-t.lastRepeatTime>1800) t.repeatHits--;
  if(t.repeatHits>=ORBIT_REPEAT_REQUIRED) t.orbitScore=min(100,(int)t.orbitScore+12);
  else if(t.orbitScore>2)t.orbitScore-=2;
  if(t.pathRatio>=ORBIT_PATH_RATIO && t.netDisplacement<=ORBIT_MAX_NET_DISPLACEMENT)
    t.orbitScore=min(100,(int)t.orbitScore+18);
  if(t.directionReversals>=ORBIT_REVERSAL_REQUIRED)
    t.orbitScore=min(100,(int)t.orbitScore+14);
  if(abs(t.radarSpeed)<=ORBIT_MAX_RADAR_SPEED && t.repeatHits>=ORBIT_REPEAT_REQUIRED)
    t.orbitScore=min(100,(int)t.orbitScore+12);
}

// =========================================================
// RANGE SIGN CONSISTENCY (2 s window)
//
// Longest same-sign run of range change as a fraction of motion steps
// in the last ~2 seconds of track history.
//   High  (~0.6–1.0) = movement mostly one way  → human-like
//   Low   (~0.2–0.4) = oscillating +/−/+        → vegetation-like
// Stops (no range change) do not count as sign flips.
// =========================================================
float rangeMaxRunFrac(int i) {
  RadarTarget &t = targets[i];
  if (t.historyCount < 4) return 0.50f;  // neutral until enough history

  const unsigned long WINDOW_MS = 2000UL;
  const float STEP_MM = 25.0f;  // ignore sub-threshold range jitter

  unsigned long newestT = t.history[t.historyHead].t;
  // Collect chronological points inside the window (oldest → newest)
  float ranges[HISTORY_LEN];
  int n = 0;
  for (int k = t.historyCount - 1; k >= 0; k--) {
    int idx = (t.historyHead - k + HISTORY_LEN) % HISTORY_LEN;
    TrackPoint &q = t.history[idx];
    if (q.t == 0) continue;
    if (newestT - q.t > WINDOW_MS) continue;
    ranges[n++] = sqrtf(q.x * q.x + q.y * q.y);
  }
  if (n < 3) return 0.50f;

  // Build signs of consecutive range deltas that exceed STEP_MM
  int8_t signs[HISTORY_LEN];
  int ns = 0;
  for (int j = 1; j < n; j++) {
    float dr = ranges[j] - ranges[j - 1];
    if (fabsf(dr) < STEP_MM) continue;  // stop / noise — not a flip
    signs[ns++] = (dr > 0.0f) ? 1 : -1;
  }
  if (ns < 2) return 0.85f;  // little motion, or only one direction sample

  int best = 1, cur = 1;
  for (int j = 1; j < ns; j++) {
    if (signs[j] == signs[j - 1]) {
      cur++;
      if (cur > best) best = cur;
    } else {
      cur = 1;
    }
  }
  return (float)best / (float)ns;
}

bool isOrbitLike(int i) {
  RadarTarget &t=targets[i];
  bool slow=abs(t.radarSpeed)<=ORBIT_MAX_RADAR_SPEED;
  bool repeated=t.repeatHits>=ORBIT_REPEAT_REQUIRED;
  bool geometry=t.pathRatio>=ORBIT_PATH_RATIO && t.netDisplacement<=ORBIT_MAX_NET_DISPLACEMENT;
  bool reverse=t.directionReversals>=ORBIT_REVERSAL_REQUIRED;
  // orbitScore alone is no longer enough: require low range-run consistency
  // so a careful human (high run frac) is not marked orbit-like too early.
  bool scoreOrbit = t.orbitScore >= 55 && rangeMaxRunFrac(i) < 0.45f;
  return slow && ((repeated&&(geometry||reverse)) || scoreOrbit);
}

// =========================================================
// AI / TINYML EXPERIMENTAL LAYER
//
// Dataset reference selected for future supervised training:
// MVRHAR (TI IWR6843AOP), containing x/y/z, Doppler and SNR
// point-cloud sequences from front/side/back viewpoints.
//
// IMPORTANT FOR 2026-09-26 HARD-NEGATIVE REVIEW:
// The supplied new set contains one HUMAN recording and five independent
// tree/wind recordings. A true continuation of the existing 37-group
// Group-OOF model requires its original training matrix/weights; replacing
// those weights from this six-group set would be a methodological regression.
// Therefore ΩΞ 0.3 deliberately keeps the validated TinyML weights/feature
// contract and uses the new logs as hard-negative validation evidence.
// The zero-motion problem is handled by the existing sliding evidence/lock
// layer, not by a single-window hard veto.
//
// IMPORTANT:
// The RD-03D does NOT expose the same point-cloud feature set,
// therefore MVRHAR cannot be copied directly into this MCU model.
// This first AI firmware uses a tiny 10->6->3 neural scoring
// layer over the features that THIS radar actually provides.
// The weights below are deliberately conservative seed weights,
// not claimed to be trained on MVRHAR. They are intended for
// real-world A/B testing and later replacement by learned weights.
//
// Output classes:
//   0 = HUMAN
//   1 = VEGETATION / CHAOTIC
//   2 = UNKNOWN
// =========================================================
#define AI_ENABLED true
// FINAL OUTPUT GATE: a new human must reach at least 75% AI confidence.
#define AI_HUMAN_THRESHOLD 0.75f
#define AI_VEGETATION_THRESHOLD 0.62f
// Once confirmed, allow short AI dips while a real person is stationary.
// If AI stays clearly below this level for this long, revoke HUMAN.
#define AI_RELEASE_THRESHOLD 0.45f
const unsigned long AI_RELEASE_TIME = 2000;

float aiInput(int i, int k) {
  RadarTarget &t = targets[i];
  switch (k) {
    case 0: return constrain((float)t.speedEvidenceFrames / 8.0f, 0.0f, 1.0f);
    case 1: return constrain(t.netDisplacement / 2500.0f, 0.0f, 1.0f);
    case 2: return constrain(t.pathLength / 5000.0f, 0.0f, 1.0f);
    case 3: return constrain(t.pathRatio / 8.0f, 0.0f, 1.0f);
    case 4: return constrain((float)t.directionalFrames / 15.0f, 0.0f, 1.0f);
    case 5: return constrain((float)t.directionReversals / 6.0f, 0.0f, 1.0f);
    case 6: return constrain((float)t.repeatHits / 30.0f, 0.0f, 1.0f);
    case 7: return constrain((float)t.orbitScore / 100.0f, 0.0f, 1.0f);
    case 8: return constrain((float)t.chaoticScore / 100.0f, 0.0f, 1.0f);
    case 9: return constrain(t.recentMovement / 500.0f, 0.0f, 1.0f);
  }
  return 0.0f;
}

float aiRelu(float x) {
  return x > 0.0f ? x : 0.0f;
}

// Compact seed network. 6 hidden neurons, 3 output logits.
// Kept in flash as const so RAM impact is negligible.
const float AI_W1[6][10] = {
  { 1.60f, 1.00f, 0.90f, 0.20f, 1.20f, 0.00f, 0.00f, -0.60f, -1.20f, 0.30f },
  { 1.20f, 1.20f, 1.20f, 0.40f, 1.40f, 0.00f, 0.00f, -0.80f, -1.60f, 0.40f },
  { -0.30f, -0.20f, -0.10f, 0.00f, -0.20f, 0.80f, 0.80f, 0.90f, 2.20f, -0.20f },
  { -0.20f, -0.10f, -0.10f, 0.00f, -0.10f, 0.60f, 1.00f, 1.20f, 2.80f, -0.20f },
  { 0.80f, 0.60f, 0.50f, 0.20f, 0.90f, -0.20f, 0.00f, -0.40f, -0.80f, 0.20f },
  { 0.10f, 0.20f, 0.20f, 0.10f, 0.20f, 0.30f, 0.40f, 0.40f, 0.80f, 0.10f }
};

const float AI_B1[6] = {-1.60f, -1.80f, -0.70f, -0.90f, -0.90f, -0.60f};

const float AI_W2[3][6] = {
  { 1.60f, 1.80f, -0.80f, -0.80f, 0.90f, 0.20f },
  { -0.80f, -0.90f, 1.20f, 1.50f, -0.60f, 0.30f },
  { -0.20f, -0.30f, 0.20f, 0.20f, -0.10f, 0.10f }
};

const float AI_B2[3] = {0.30f, -0.20f, 0.10f};


void aiPredict(int i, float &human, float &vegetation, float &unknown) {
  float h[6];
  float logits[3];
  for (int j = 0; j < 6; j++) {
    float v = AI_B1[j];
    for (int k = 0; k < 10; k++) v += AI_W1[j][k] * aiInput(i, k);
    h[j] = aiRelu(v);
  }
  for (int o = 0; o < 3; o++) {
    float v = AI_B2[o];
    for (int j = 0; j < 6; j++) v += AI_W2[o][j] * h[j];
    logits[o] = v;
  }

  float mx = logits[0];
  if (logits[1] > mx) mx = logits[1];
  if (logits[2] > mx) mx = logits[2];
  float e0 = expf(logits[0] - mx);
  float e1 = expf(logits[1] - mx);
  float e2 = expf(logits[2] - mx);
  float sum = e0 + e1 + e2;
  human = e0 / sum;
  vegetation = e1 / sum;
  unknown = e2 / sum;
}


// =========================================================
// 2-SECOND TEMPORAL HUMAN/NON-HUMAN MODEL
//
// Trained from the collected tree + kitchen + sleep logs.
// This is deliberately a tiny decision tree: no dynamic allocation,
// no ML library, only the last 2 seconds of the existing track history.
//
// Features:
//   stepSum   = total local movement in the last 2 s (mm)
//   dRange    = range of target distance (mm)
//   aStd      = angle standard deviation (deg)
//   dispRange = range of displacement-from-track-start (mm)
//   moveMean  = mean local movement (mm)
//   speedMean = mean absolute radar radial speed (cm/s)
//
// The model is used as an ENTRY CONFIRMATION, not as a hard revocation
// mechanism. Once a target is confirmed HUMAN, short weak-motion periods
// must not make it disappear.
//
// Training result on blocked 5-minute groups from our current logs:
//   2 s window, 8 raw features: OOF AUC ~0.797
//   6 raw features (the ones available directly in the ESP32 history):
//   OOF AUC ~0.788
// This is therefore an experimental temporal gate, not a final guarantee.
// =========================================================
#define TEMPORAL_WINDOW_MS 2000UL
#define TEMPORAL_MIN_POINTS 6
#define TEMPORAL_HUMAN_THRESHOLD 0.50f
#define TEMPORAL_POSITIVE_WINDOWS_REQUIRED 2

float temporalHumanScore(int i, unsigned long now) {
  RadarTarget &t = targets[i];
  if (!t.active || t.historyCount < TEMPORAL_MIN_POINTS) {
    t.temporalReady = false;
    return 0.0f;
  }

  TrackPoint pts[HISTORY_LEN];
  int n = 0;

  // Collect newest -> oldest, then reverse to chronological order.
  int idx = t.historyHead;
  for (int k = 0; k < HISTORY_LEN && k < t.historyCount; k++) {
    TrackPoint q = t.history[idx];
    if (q.t != 0 && now >= q.t && now - q.t <= TEMPORAL_WINDOW_MS) {
      pts[n++] = q;
    }
    idx--;
    if (idx < 0) idx = HISTORY_LEN - 1;
  }

  if (n < TEMPORAL_MIN_POINTS) {
    t.temporalReady = false;
    return 0.0f;
  }

  for (int a = 0; a < n / 2; a++) {
    TrackPoint tmp = pts[a];
    pts[a] = pts[n - 1 - a];
    pts[n - 1 - a] = tmp;
  }

  float stepSum = 0.0f;
  float moveSum = 0.0f;
  float dMin = 1e9f, dMax = -1e9f;
  float dispMin = 1e9f, dispMax = -1e9f;
  float angleSum = 0.0f, angleSqSum = 0.0f;
  float speedSum = 0.0f;

  for (int k = 0; k < n; k++) {
    float d = sqrtf(pts[k].x * pts[k].x + pts[k].y * pts[k].y);
    float a = atan2f(pts[k].x, pts[k].y) * 57.2957795f;
    float disp = distance2D(pts[k].x, pts[k].y, t.startX, t.startY);

    dMin = min(dMin, d); dMax = max(dMax, d);
    dispMin = min(dispMin, disp); dispMax = max(dispMax, disp);
    angleSum += a;
    angleSqSum += a * a;
    speedSum += fabsf((float)pts[k].speed);

    if (k > 0) {
      float dx = pts[k].x - pts[k-1].x;
      float dy = pts[k].y - pts[k-1].y;
      float step = sqrtf(dx * dx + dy * dy);
      stepSum += step;
      moveSum += step;
    }
  }

  float countMove = (float)max(1, n - 1);
  float moveMean = moveSum / countMove;
  float speedMean = speedSum / (float)n;
  float angleMean = angleSum / (float)n;
  float angleVar = angleSqSum / (float)n - angleMean * angleMean;
  if (angleVar < 0.0f) angleVar = 0.0f;
  float aStd = sqrtf(angleVar);
  float dRange = dMax - dMin;
  float dispRange = dispMax - dispMin;

  // Exact compact tree fitted to the 2-second training set.
  float score;
  if (stepSum <= 119.43f) {
    if (dRange <= 38.50f) {
      if (aStd <= 0.05f) {
        score = (stepSum <= 4.42f) ? 0.7414f : 0.6039f;
      } else {
        score = (stepSum <= 79.84f) ? 0.8959f : 0.7195f;
      }
    } else {
      if (speedMean <= 0.44f) {
        score = (aStd <= 1.85f) ? 0.4469f : 0.8532f;
      } else {
        score = (aStd <= 0.47f) ? 0.8772f : 0.7231f;
      }
    }
  } else {
    if (speedMean <= 0.94f) {
      if (moveMean <= 7.87f) {
        score = (dRange <= 49.50f) ? 0.4718f : 0.3329f;
      } else {
        score = (dRange <= 65.50f) ? 0.2888f : 0.1691f;
      }
    } else {
      if (aStd <= 22.71f) {
        score = (speedMean <= 5.79f) ? 0.6658f : 0.8629f;
      } else {
        score = (stepSum <= 1883.18f) ? 0.3889f : 0.0099f;
      }
    }
  }

  // dispRange is intentionally calculated and retained as a feature
  // diagnostic, even though the final compact tree did not need it.
  (void)dispRange;

  t.temporalReady = true;
  t.temporalScore = score;
  return score;
}

const unsigned long HUMAN_EVIDENCE_SAMPLE_MS = 500UL;
const int8_t HUMAN_EVIDENCE_CONFIRM_SCORE = 5;
const int8_t HUMAN_EVIDENCE_RELEASE_SCORE = 0;

int8_t updateHumanEvidenceWindow(int i, unsigned long now,
                                 float human, float vegetation, float unknown,
                                 float veg, bool trajectory, bool entry) {
  RadarTarget &t = targets[i];
  if (t.humanEvidenceLastSample != 0 &&
      now - t.humanEvidenceLastSample < HUMAN_EVIDENCE_SAMPLE_MS)
    return t.humanEvidenceSum;
  t.humanEvidenceLastSample = now;

  // orbitScore>=90 required below already implies genuine repeated
  // in/out or circling behavior, so no separate entry exemption is
  // needed here (unlike the other two spots) -- an object that just
  // freshly entered the field cannot yet have orbitScore=90.
  bool persistentCorridor =
      t.repeatHits >= 30 && t.orbitScore >= 90 &&
      t.pathLength >= 1200.0f && t.netDisplacement < 1800.0f;

  // A real multi-metre translation cannot be a swaying branch. When net
  // displacement is large, do not let vegetation score (often inflated by
  // repeatHits along a long human path) push evidence to -1/-2 forever.
  bool strongDisplacement = t.netDisplacement >= 1500.0f && trajectory;
  float runFracEv = rangeMaxRunFrac(i);
  bool consistentDir = runFracEv >= 0.55f;

  int8_t evidence = 0;
  if (!entry && !strongDisplacement &&
      (veg >= 0.68f || vegetation >= 0.70f || persistentCorridor))
    evidence = -2;
  else if (!entry && !strongDisplacement &&
           (veg >= 0.55f || vegetation > human + 0.10f))
    evidence = -1;
  else if (trajectory && human >= 0.75f && human > vegetation && human > unknown)
    evidence = +2;
  else if (trajectory && human >= 0.55f && human > vegetation && human > unknown)
    evidence = +1;
  // AI-downgraded role: physics trajectory still counts without AI agreement.
  // strongDisplacement also unlocks +1 even if veg is elevated from REP.
  else if (trajectory && (veg < 0.55f || strongDisplacement))
    evidence = +1;

  // Extra recovery: multi-metre consistent motion must be able to climb out
  // of a previously poisoned evidence window (W=-8 case from 0.3.3 field log).
  if (strongDisplacement && consistentDir && evidence < 1)
    evidence = +1;
  if (strongDisplacement && consistentDir && human >= 0.55f && evidence < 2)
    evidence = +2;

  // Fast release (2026-09-26): field simulation on real strong-wind logs showed
  // strongDisplacement can fire on a swaying canopy too -- the reflection
  // centroid genuinely migrates several metres across a large moving mass,
  // not sensor noise. That is fine as long as release is quick once the
  // displacement stops being reinforced: a person keeps re-triggering
  // trajectory almost every sample, a gust does not. If neither
  // strongDisplacement nor trajectory holds THIS sample and the AI score does
  // not independently agree either, and the last couple of samples were
  // carrying positive evidence, pull the sum back down immediately instead of
  // waiting up to 4s for the 8-slot FIFO to forget the spike naturally.
  if (!strongDisplacement && !trajectory && human < 0.55f) {
    uint8_t lookback = (t.humanEvidenceCount < 2) ? t.humanEvidenceCount : 2;
    for (uint8_t k = 0; k < lookback; k++) {
      uint8_t idx = (uint8_t)((t.humanEvidenceHead + 8 - k) % 8);
      if (t.humanEvidenceWindow[idx] >= 1) { evidence = -2; break; }
    }
  }

  t.humanEvidenceHead = (uint8_t)((t.humanEvidenceHead + 1) % 8);
  if (t.humanEvidenceCount >= 8)
    t.humanEvidenceSum -= t.humanEvidenceWindow[t.humanEvidenceHead];
  else
    t.humanEvidenceCount++;
  t.humanEvidenceWindow[t.humanEvidenceHead] = evidence;
  t.humanEvidenceSum += evidence;
  return t.humanEvidenceSum;
}

bool weightedHumanCandidate(int i, unsigned long now,
                            float human, float vegetation, float unknown,
                            float veg, bool trajectory, bool entry) {
  RadarTarget &t = targets[i];
  int8_t score = updateHumanEvidenceWindow(i, now, human, vegetation, unknown,
                                           veg, trajectory, entry);
  bool recentStrongVegetation = false;
  uint8_t recent = (t.humanEvidenceCount < 3) ? t.humanEvidenceCount : 3;
  for (uint8_t k = 0; k < recent; k++) {
    uint8_t idx = (uint8_t)((t.humanEvidenceHead + 8 - k) % 8);
    if (t.humanEvidenceWindow[idx] <= -2) {
      recentStrongVegetation = true;
      break;
    }
  }
  bool confirmed = t.humanEvidenceCount >= 3 &&
                   score >= HUMAN_EVIDENCE_CONFIRM_SCORE &&
                   !recentStrongVegetation;
  if (score <= HUMAN_EVIDENCE_RELEASE_SCORE && t.humanEvidenceCount >= 4)
    return false;
  return confirmed;
}

bool aiHumanCandidate(int i) {
  if (!AI_ENABLED) return true;

  RadarTarget &t = targets[i];
  unsigned long now = millis();
  float human, vegetation, unknown;
  aiPredict(i, human, vegetation, unknown);

  float veg = vegetationPatternScore(i);
  bool entry = isEntryFromEdge(i);
  bool trajectory = hasHumanTrajectory(i);
  bool strongMap = getLocalAdaptiveBackgroundScore(t.x, t.y) >= ADAPTIVE_BG_STRONG_THRESHOLD;
  bool dynamicBg = isKnownDynamicBackground(t.x, t.y);

  // Hard vegetation veto requires independent behavior evidence. A learned
  // map alone is not enough, and the map is not allowed to create its own
  // evidence loop through this score.
  // Edge-entry exemption expires once orbitScore shows a genuine repeated
  // in/out or circling pattern at this spot (same threshold used by the
  // orbit detector itself) -- otherwise vegetation right at the boundary
  // of the scan range could sway in and out forever without ever being
  // vetoed, since every re-entry would look like a fresh "entry from edge".
  // Expire edge-entry protection only when short-window range motion
  // looks vegetation-like (low same-sign run). orbitScore>=55 alone was
  // too aggressive and demoted careful humans entering from outside FOV.
  float runFrac = rangeMaxRunFrac(i);
  bool entryExpired = entry &&
      runFrac < 0.40f &&
      (t.repeatHits >= 20 || t.pathRatio >= 4.0f);
  bool vegetationVeto = (!entry || entryExpired) && (
      veg >= 0.68f ||
      (strongMap && veg >= 0.50f) ||
      (dynamicBg && veg >= 0.55f)
    );

  if (vegetationVeto) {
    datasetLogf("[AI_VETO] T%d H=%.2f V=%.2f TEMP=%.2f VEG=%.2f ADAPT=%u DYN=%u\n",
      i + 1, human, vegetation, t.temporalScore, veg,
      (unsigned)getLocalAdaptiveBackgroundScore(t.x,t.y),
      (unsigned)getLocalDynamicScore(t.x,t.y));
    return false;
  }

  bool weightedPass = weightedHumanCandidate(i, now, human, vegetation, unknown,
                                              veg, trajectory, entry);
  return weightedPass && !vegetationVeto;
}

// =========================================================
// v0.9 VEGETATION PATTERN SCORE
// Cold-start evidence is deliberately independent of learned maps.
// Maps are reinforcement, not a prerequisite for vegetation recognition.
// =========================================================
float vegetationPatternScore(int i) {
  RadarTarget &t = targets[i];
  float repeat = constrain((float)t.repeatHits / 80.0f, 0.0f, 1.0f);
  float reversals = constrain((float)t.directionReversals / 8.0f, 0.0f, 1.0f);
  // range_max_run_frac: high = one-way motion (human), low = oscillation (tree).
  // Invert so it contributes to vegetation score like the old orbit term.
  float runFrac = rangeMaxRunFrac(i);
  float rangeOsc = constrain(1.0f - runFrac, 0.0f, 1.0f);
  float confinement = 0.0f;
  if (t.pathLength > 250.0f) {
    float ratio = t.pathLength / max(250.0f, t.netDisplacement + 250.0f);
    confinement = constrain((ratio - 1.0f) / 5.0f, 0.0f, 1.0f);
  }

  // Independent spatial/behavioral base. orbitScore replaced by rangeOsc
  // (1 - range_max_run_frac), the stronger short-window separator from logs.
  float base = 0.28f * repeat + 0.28f * reversals +
               0.24f * confinement + 0.20f * rangeOsc;

  float dynamic = constrain(getLocalDynamicScore(t.x,t.y) / 100.0f,0.0f,1.0f);
  float oscillation = constrain(getLocalOscillationScore(t.x,t.y) / 100.0f,0.0f,1.0f);
  float adaptive = constrain(getLocalAdaptiveBackgroundScore(t.x,t.y) / 100.0f,0.0f,1.0f);

  float mapBonus = 0.18f * dynamic + 0.12f * oscillation + 0.20f * adaptive;

  float progression = 0.0f;
  if (t.netDisplacement >= 1200.0f && t.directionalFrames >= 5) progression = 0.35f;
  if (t.netDisplacement >= 1500.0f && t.directionalFrames >= 5) progression = max(progression, 0.50f);
  if (t.netDisplacement >= 2200.0f && t.directionalFrames >= 8) progression = 0.70f;
  if (t.netDisplacement >= 2500.0f) progression = max(progression, 0.85f);
  if (isEntryFromEdge(i)) progression = min(1.0f, progression + 0.35f);

  // v0.9 cold-start fix: when the maps are empty, do not attenuate the
  // independent behavioral evidence by 0.75. A recurring tree can therefore
  // build meaningful vegetation confidence before the maps have learned it.
  float score = base + mapBonus * 0.25f - 0.28f * progression;

  // REP floor only for true stationary/oscillating clutter — not for a
  // person walking metres through the field (0.3.3 field-test failure mode).
  bool noMapEvidence = dynamic < 0.05f && oscillation < 0.05f && adaptive < 0.05f;
  if (noMapEvidence && t.repeatHits >= 50 &&
      t.netDisplacement < 1200.0f && rangeMaxRunFrac(i) < 0.45f)
    score = max(score, 0.55f);

  return constrain(score, 0.0f, 1.0f);
}

// =========================================================
// AI FINAL OUTPUT GATE / HOLD
//
// aiConfirmedHuman is used for user-facing HUMAN output. A new HUMAN
// must first pass the AI threshold and be displayed/notified. AFTER that
// output has occurred, humanLock is armed. From that moment the AI gate
// MUST NOT release HUMAN merely because the person stops moving or the
// Doppler evidence falls to zero. The lock remains valid until the
// physical track is actually lost and the target is reset/released by
// tracking logic.
// =========================================================
bool updateAIFinalGate(int i, unsigned long now) {
  if (!AI_ENABLED) return targets[i].state == TRACK_HUMAN && targets[i].isHuman;
  if (!targets[i].active || !trainingFinished) return false;

  float human, vegetation, unknown;
  aiPredict(i, human, vegetation, unknown);

  RadarTarget &t = targets[i];

  // HARD HUMAN LOCK:
  // Once HUMAN has been displayed on OLED and the BLE notification has
  // been issued, humanLock is armed in the post-output stage below.
  // From that moment AI is NO LONGER allowed to demote/release HUMAN.
  // A stationary person naturally produces weak/zero Doppler, so the
  // AI score may fall sharply; that must not cancel an already confirmed
  // human. The lock is cleared only by the physical tracking/loss logic.
  if (t.humanLock) {
    t.aiConfirmedHuman = true;
    t.aiLastPassTime = now;
    t.aiLowSince = 0;
    t.isHuman = true;
    t.isClutter = false;
    t.state = TRACK_HUMAN;
    return true;
  }

  bool pass = (human >= AI_HUMAN_THRESHOLD && human > vegetation && human > unknown);

  if (pass) {
    t.aiConfirmedHuman = true;
    t.aiLastPassTime = now;
    t.aiLowSince = 0;
    return true;
  }

  if (!t.aiConfirmedHuman) return false;

  // Confirmed HUMAN gets a short grace period for stationary/weak-Doppler
  // frames. A clearly non-human AI result must persist for 2 seconds
  // before the visible HUMAN is revoked.
  if (human < AI_RELEASE_THRESHOLD) {
    if (t.aiLowSince == 0) t.aiLowSince = now;
    if (now - t.aiLowSince >= AI_RELEASE_TIME) {
      // A locked HUMAN can never reach this release path because the
      // hard-lock check above returns first. Therefore reaching here
      // means the target was never output-locked.
      t.aiConfirmedHuman = false;
      t.humanDisplayed = false;
      t.humanNotified = false;
      t.isHuman = false;
      t.isClutter = true;
      t.state = TRACK_OBSERVE;
      t.observeSince = now;
      datasetLogf("[AI_GATE] T%d HUMAN RELEASE H=%.2f V=%.2f U=%.2f\n",
                    i + 1, human, vegetation, unknown);
      return false;
    }
  } else {
    t.aiLowSince = 0;
  }

  return true;
}



bool hasHumanTrajectory(int i) {
  RadarTarget &t=targets[i];
  bool doppler = t.speedEvidenceFrames >= HUMAN_SPEED_EVIDENCE_REQUIRED;
  bool displacementOK = t.netDisplacement >= HUMAN_MIN_DISPLACEMENT_V6;
  bool pathOK = t.pathLength >= HUMAN_MIN_PATH_V6;
  bool directionOK = t.directionalFrames >= HUMAN_DIRECTION_FRAMES;

  // Doppler is now supporting evidence, not an absolute gate.
  // This allows a target with a clearly accumulated spatial trajectory
  // to continue through the classifier even when the current radar
  // samples temporarily report zero radial speed.
  bool trajectoryShape = displacementOK || pathOK || directionOK;
  bool nonDopplerMotion =
    trajectoryShape &&
    t.framesSeen >= HUMAN_NO_DOPPLER_MIN_FRAMES &&
    t.recentMovement >= HUMAN_NO_DOPPLER_MIN_RECENT_STEP;

  return trajectoryShape && (doppler || nonDopplerMotion);
}

// =========================================================
// PRE-LOCK HUMAN TRACK CONTINUITY
//
// A large RD-03D position jump does not always mean that the physical
// target disappeared. Preserve a strong pre-lock human-like track while
// a short coherent cluster confirms the new position. HUMAN_LOCK is never
// created by this path.
// =========================================================
bool anotherActiveTrackNearCandidate(int currentIndex, float candidateX, float candidateY) {
  // RD-03D may report the same person in a different hardware slot.
  // Do not preserve the old pre-lock slot if another slot already owns
  // a nearby observation; otherwise both slots can remain visible.
  const float DUPLICATE_MERGE_DISTANCE = 650.0f;
  for (int j = 0; j < 3; j++) {
    if (j == currentIndex) continue;
    RadarTarget &o = targets[j];
    if (!o.active) continue;
    if (o.humanGhost) continue;
    float d = distance2D(candidateX, candidateY, o.x, o.y);
    if (d <= DUPLICATE_MERGE_DISTANCE) return true;
  }
  return false;
}

bool tryHumanTrackContinuity(int i, int16_t rawX, int16_t rawY,
                              int16_t rawSpeed, unsigned long now) {
  RadarTarget &t = targets[i];
  if (!t.active || t.humanLock || t.state == TRACK_HUMAN) return false;

  bool trajectoryEvidence = hasHumanTrajectory(i);
  bool accumulatedEvidence =
    t.netDisplacement >= HUMAN_MIN_DISPLACEMENT_V6 ||
    t.pathLength >= HUMAN_MIN_PATH_V6 ||
    t.directionalFrames >= HUMAN_DIRECTION_FRAMES;
  if (!trajectoryEvidence || !accumulatedEvidence) return false;

  float candidateX = (float)rawX;
  float candidateY = (float)rawY;
  float jump = distance2D(candidateX, candidateY, t.x, t.y);

  // If another slot already represents a nearby target, do not keep
  // this old pre-lock slot alive through continuity. This preserves the
  // stationary-person feature while preventing duplicate slot tracks.
  if (anotherActiveTrackNearCandidate(i, candidateX, candidateY)) {
    t.humanContinuityPending = false;
    t.humanContinuityFrames = 0;
    t.humanContinuitySince = 0;
    return false;
  }

  if (jump > HUMAN_CONTINUITY_MAX_JUMP) {
    t.humanContinuityPending = false;
    t.humanContinuityFrames = 0;
    t.humanContinuitySince = 0;
    return false;
  }

  if (!t.humanContinuityPending) {
    t.humanContinuityPending = true;
    t.humanContinuityFrames = 1;
    t.humanContinuitySince = now;
    t.humanContinuityX = candidateX;
    t.humanContinuityY = candidateY;
    t.humanContinuitySpeed = rawSpeed;
    datasetLogf("[TR] T%d jump=%.0f -> pending\n", i + 1, jump);
    return true;
  }

  float clusterMove = distance2D(candidateX, candidateY,
                                 t.humanContinuityX, t.humanContinuityY);
  if (now - t.humanContinuitySince > HUMAN_CONTINUITY_TIMEOUT ||
      clusterMove > HUMAN_CONTINUITY_CLUSTER_RADIUS) {
    t.humanContinuityFrames = 1;
    t.humanContinuitySince = now;
    t.humanContinuityX = candidateX;
    t.humanContinuityY = candidateY;
    t.humanContinuitySpeed = rawSpeed;
    datasetLogf("[TR] T%d restart cluster jump=%.0f\n", i + 1, jump);
    return true;
  }

  if (t.humanContinuityFrames < 255) t.humanContinuityFrames++;
  t.humanContinuityX = candidateX;
  t.humanContinuityY = candidateY;
  t.humanContinuitySpeed = rawSpeed;

  if (t.humanContinuityFrames < HUMAN_CONTINUITY_REQUIRED_FRAMES) return true;

  // Preserve lifetime trajectory evidence, but rebuild only the short
  // temporal history so the artificial slot jump is not counted as motion.
  t.x = candidateX;
  t.y = candidateY;
  t.prevX = candidateX;
  t.prevY = candidateY;
  t.recentMovement = 0.0f;
  t.radarSpeed = rawSpeed;

  for (int h = 0; h < HISTORY_LEN; h++) {
    t.history[h].x = 0;
    t.history[h].y = 0;
    t.history[h].speed = 0;
    t.history[h].t = 0;
  }
  t.historyCount = 1;
  t.historyHead = 0;
  t.history[0] = {candidateX, candidateY, rawSpeed, now};
  t.temporalScore = 0.0f;
  t.temporalPositiveCount = 0;
  t.temporalReady = false;
  initEntryVector(i, candidateX, candidateY);

  t.humanContinuityPending = false;
  t.humanContinuityFrames = 0;
  t.humanContinuitySince = 0;
  t.humanContinuityX = 0;
  t.humanContinuityY = 0;
  t.humanContinuitySpeed = 0;

  datasetLogf("[TR] T%d ACCEPTED x=%.0f y=%.0f preserved path=%.0f disp=%.0f dir=%d\n",
                i + 1, candidateX, candidateY,
                t.pathLength, t.netDisplacement, t.directionalFrames);
  return true;
}

// =========================================================
// RE-ASSOCIATION OF A LOCKED HUMAN TRACK
//
// Do not destroy a confirmed HUMAN immediately when one RD-03D frame
// jumps far away.  A radar target slot is not a guaranteed persistent
// person ID.  We therefore require a short, coherent cluster of new
// observations before moving the locked track to the new position.
// =========================================================
bool tryLockedHumanReassociation(int i, int16_t rawX, int16_t rawY,
                                  int16_t rawSpeed, unsigned long now) {
  RadarTarget &t = targets[i];

  if (!t.humanLock) return false;

  float candidateX = (float)rawX;
  float candidateY = (float)rawY;
  float jump = distance2D(candidateX, candidateY, t.x, t.y);

  // Too far away: this is not a safe re-association candidate.
  if (jump > HUMAN_REASSOC_MAX_JUMP) {
    if (t.switchPending) {
      datasetLogf("[TR] T%d REASSOC_FAIL j=%.0f", i + 1, jump);
    }
    t.switchPending = false;
    t.switchPendingFrames = 0;
    t.switchPendingSince = 0;
    return false;
  }

  // First frame of a candidate cluster.
  if (!t.switchPending) {
    t.switchPending = true;
    t.switchPendingFrames = 1;
    t.switchPendingSince = now;
    t.switchPendingX = candidateX;
    t.switchPendingY = candidateY;
    t.switchPendingSpeed = rawSpeed;
    datasetLogf("[TR] T%d locked jump=%.0f -> pending re-association\n",
                  i + 1, jump);
    return true;
  }

  // Candidate must remain spatially coherent from frame to frame.
  float clusterMove = distance2D(candidateX, candidateY,
                                 t.switchPendingX, t.switchPendingY);

  if (clusterMove <= HUMAN_REASSOC_CLUSTER_RADIUS &&
      now - t.switchPendingSince <= HUMAN_REASSOC_TIMEOUT) {
    if (t.switchPendingFrames < 255) t.switchPendingFrames++;
    t.switchPendingX = candidateX;
    t.switchPendingY = candidateY;
    t.switchPendingSpeed = rawSpeed;
  } else {
    // A new incoherent candidate starts a fresh confirmation window.
    t.switchPendingFrames = 1;
    t.switchPendingSince = now;
    t.switchPendingX = candidateX;
    t.switchPendingY = candidateY;
    t.switchPendingSpeed = rawSpeed;
    datasetLogf("[TR] T%d candidate restarted jump=%.0f\n",
                  i + 1, jump);
    return true;
  }

  if (t.switchPendingFrames < HUMAN_REASSOC_REQUIRED_FRAMES) {
    return true;
  }

  // Confirmed re-association. Preserve the already-established HUMAN
  // track state and stabilization lock, but restart the geometric
  // history so the large slot jump is not counted as human movement.
  // This path is reachable only for a lock that was armed after output.
  t.x = candidateX;
  t.y = candidateY;
  t.prevX = candidateX;
  t.prevY = candidateY;
  t.dirX = 0;
  t.dirY = 0;
  t.recentMovement = 0;
  t.movementAccum = 0;
  t.directionalFrames = 0;
  t.directionChanges = 0;
  t.pathLength = 0;
  t.netDisplacement = 0;
  t.pathRatio = 0;
  t.directionReversals = 0;
  t.repeatHits = 0;
  t.orbitScore = 0;
  t.chaoticScore = 0;
  t.speedEvidenceFrames = 0;
  t.framesSeen = 1;
  t.radarSpeed = rawSpeed;
  t.historyCount = 1;
  t.historyHead = 0;
  t.history[0] = {candidateX, candidateY, rawSpeed, now};

  t.state = TRACK_HUMAN;
  t.isHuman = true;
  t.isClutter = false;
  t.aiConfirmedHuman = true;
  t.lastHumanTime = now;
  t.aiLowSince = 0;

  t.switchPending = false;
  t.switchPendingFrames = 0;
  t.switchPendingSince = 0;

  datasetLogf("[TR] T%d ACCEPTED x=%.0f y=%.0f\n",
                i + 1, candidateX, candidateY);
  return true;
}

// =========================================================
// HUMAN GHOST / REACQUISITION
// =========================================================
//
// A confirmed HUMAN may disappear completely from the RD-03D target list
// for a short period while the physical person is still present.  Instead
// of resetting the lock immediately, keep the last known HUMAN position in
// a short GHOST state.  A new radar target can reclaim the lock only after
// several spatially coherent observations near that position.
// =========================================================
void enterHumanGhost(int i, unsigned long now) {
  RadarTarget &t = targets[i];
  if (!t.active || !t.humanLock) return;

  t.humanGhost = true;
  t.humanGhostSince = now;
  t.humanGhostX = t.x;
  t.humanGhostY = t.y;

  // Variant B: decide the effective ghost hold based on distance at the
  // moment of disappearance. This is independent of, and layered on top of,
  // the sole-candidate handoff logic further below -- that logic can still
  // reacquire immediately at any distance if the person reappears as the
  // only convincing candidate; this only shortens how long a STALE,
  // unmatched ghost slot lingers at the edges of radar range.
  float lastDist = sqrtf(t.humanGhostX * t.humanGhostX + t.humanGhostY * t.humanGhostY);
  if (lastDist < HUMAN_GHOST_FULL_HOLD_MIN_DIST_MM ||
      lastDist >= HUMAN_GHOST_FULL_HOLD_MAX_DIST_MM) {
    t.humanGhostEffectiveTime = HUMAN_GHOST_TIME_EDGE_DISTANCE;
  } else {
    t.humanGhostEffectiveTime = HUMAN_GHOST_TIME;
  }

  // Estimate the last Cartesian motion from the most recent history pair.
  // This is intentionally independent of RD-03D radial Doppler: a human can
  // move laterally while S/E remain zero.
  t.humanGhostVX = 0.0f;
  t.humanGhostVY = 0.0f;
  if (t.historyCount >= 2) {
    int last = t.historyHead;
    int prev = (last - 1 + HISTORY_LEN) % HISTORY_LEN;
    unsigned long dt = t.history[last].t - t.history[prev].t;
    if (dt >= 20 && dt <= 1000) {
      t.humanGhostVX = (t.history[last].x - t.history[prev].x) * 1000.0f / (float)dt;
      t.humanGhostVY = (t.history[last].y - t.history[prev].y) * 1000.0f / (float)dt;
    }
  }

  t.ghostCandidateSlot = -1;
  t.ghostCandidateFrames = 0;
  t.ghostCandidateSince = 0;
  t.ghostCandidateX = 0;
  t.ghostCandidateY = 0;

  // Keep the visible HUMAN state during the short grace period.
  t.state = TRACK_HUMAN;
  t.isHuman = true;
  t.isClutter = false;
  t.aiConfirmedHuman = true;
  t.lastHumanTime = now;

  datasetLogf("[HG] T%d ENTER x=%.0f y=%.0f vx=%.0f vy=%.0f\n",
                i + 1, t.humanGhostX, t.humanGhostY,
                t.humanGhostVX, t.humanGhostVY);
}

void clearGhostCandidate(int i) {
  targets[i].ghostCandidateSlot = -1;
  targets[i].ghostCandidateFrames = 0;
  targets[i].ghostCandidateSince = 0;
  targets[i].ghostCandidateX = 0;
  targets[i].ghostCandidateY = 0;
}

// Predict where the locked human should be after a dropout and compute a
// time/speed-dependent search gate.  The gate grows with elapsed time and
// with the last measured Cartesian velocity, but is always capped.
void getHumanGhostPrediction(const RadarTarget &g, unsigned long now,
                             float &predX, float &predY, float &gate) {
  float elapsed = (float)(now - g.humanGhostSince) / 1000.0f;
  if (elapsed < 0.0f) elapsed = 0.0f;

  predX = g.humanGhostX + g.humanGhostVX * elapsed;
  predY = g.humanGhostY + g.humanGhostVY * elapsed;

  float cartSpeed = sqrtf(g.humanGhostVX * g.humanGhostVX +
                          g.humanGhostVY * g.humanGhostVY);
  gate = HUMAN_GHOST_MIN_GATE +
         HUMAN_GHOST_GATE_SPEED_GAIN * cartSpeed * elapsed +
         HUMAN_GHOST_GATE_TIME_GROWTH * sqrtf(elapsed);
  if (gate > HUMAN_GHOST_MAX_REACQUIRE_DISTANCE)
    gate = HUMAN_GHOST_MAX_REACQUIRE_DISTANCE;
}

// Direction check is deliberately soft: it rejects candidates that appear
// strongly behind a moving human, but does not constrain a stationary human.
bool humanGhostDirectionCompatible(const RadarTarget &g, float cx, float cy,
                                   unsigned long now) {
  float vx = g.humanGhostVX;
  float vy = g.humanGhostVY;
  float speed = sqrtf(vx * vx + vy * vy);
  if (speed < 150.0f) return true;

  float dt = (float)(now - g.humanGhostSince) / 1000.0f;
  float dx = cx - g.humanGhostX;
  float dy = cy - g.humanGhostY;
  float d = sqrtf(dx * dx + dy * dy);
  if (d < 250.0f || dt < 0.15f) return true;

  float cosDir = (dx * vx + dy * vy) / (d * speed);
  return cosDir >= HUMAN_GHOST_DIRECTION_COS_MIN;
}

int soleActiveHumanGhostSlot() {
  int found = -1;
  for (int j = 0; j < 3; j++) {
    if (targets[j].active && targets[j].humanLock && targets[j].humanGhost) {
      if (found != -1) return -1;
      found = j;
    }
  }
  return found;
}

bool isSoleVegetationClearedCandidate(int candidateSlot) {
  if (candidateSlot < 0 || candidateSlot >= 3) return false;
  if (!vegetationClearedThisFrame[candidateSlot]) return false;

  for (int j = 0; j < 3; j++) {
    if (j == candidateSlot) continue;
    if (!targets[j].active) continue;

    // Any other strong candidate or independently active HUMAN makes the
    // alternative handoff ambiguous.
    if (targets[j].state == TRACK_HUMAN ||
        vegetationClearedThisFrame[j]) {
      return false;
    }
  }
  return true;
}

bool acceptHumanGhostReacquisition(int ghostIdx, int candidateSlot,
                                    int16_t rawX, int16_t rawY,
                                    int16_t rawSpeed, unsigned long now) {
  RadarTarget &g = targets[ghostIdx];
  if (!g.humanGhost || !g.humanLock) return false;
  if (candidateSlot < 0 || candidateSlot >= 3) return false;
  if (candidateSlot == ghostIdx) return false;
  // A candidate may already have entered TRACK_HUMAN during this frame.
  // That is allowed only when it is not independently locked/ghosted; the
  // uniqueness and multi-frame checks below still decide the handoff.
  if (targets[candidateSlot].humanLock ||
      targets[candidateSlot].humanGhost) return false;

  float cx = (float)rawX;
  float cy = (float)rawY;
  float predX, predY, gate;
  getHumanGhostPrediction(g, now, predX, predY, gate);
  float fromGhost = distance2D(cx, cy, predX, predY);
  bool spatiallyPlausible = (fromGhost <= gate) &&
                            humanGhostDirectionCompatible(g, cx, cy, now);

  bool soleConfidentAlternative = !spatiallyPlausible &&
                                   soleActiveHumanGhostSlot() == ghostIdx &&
                                   isSoleVegetationClearedCandidate(candidateSlot);

  if (!spatiallyPlausible && !soleConfidentAlternative) return false;

  if (g.ghostCandidateSlot != candidateSlot ||
      g.ghostCandidateFrames == 0) {
    g.ghostCandidateSlot = candidateSlot;
    g.ghostCandidateFrames = 1;
    g.ghostCandidateSince = now;
    g.ghostCandidateX = cx;
    g.ghostCandidateY = cy;
    datasetLogf("[HG] GHOST T%d candidate=T%d d=%.0f\n",
                  ghostIdx + 1, candidateSlot + 1, fromGhost);
    return false;
  }

  float clusterMove = distance2D(cx, cy, g.ghostCandidateX, g.ghostCandidateY);
  if (now - g.ghostCandidateSince > HUMAN_GHOST_CLUSTER_TIMEOUT ||
      clusterMove > HUMAN_GHOST_CLUSTER_RADIUS) {
    g.ghostCandidateFrames = 1;
    g.ghostCandidateSince = now;
    g.ghostCandidateX = cx;
    g.ghostCandidateY = cy;
    datasetLogf("[HG] GHOST T%d restart candidate=T%d d=%.0f\n",
                  ghostIdx + 1, candidateSlot + 1, fromGhost);
    return false;
  }

  if (g.ghostCandidateFrames < 255) g.ghostCandidateFrames++;
  g.ghostCandidateX = cx;
  g.ghostCandidateY = cy;

  if (g.ghostCandidateFrames < HUMAN_GHOST_REQUIRED_FRAMES) return false;

  // Move the HUMAN identity to the new radar slot.  Do not copy arbitrary
  // target state; only preserve confirmed HUMAN identity and start a fresh
  // short geometry history from the reacquired position.
  RadarTarget &r = targets[candidateSlot];
  r.active = true;
  r.state = TRACK_HUMAN;
  r.isHuman = true;
  r.isClutter = false;
  r.x = cx;
  r.y = cy;
  r.startX = cx;
  r.startY = cy;
  r.prevX = cx;
  r.prevY = cy;
  r.dirX = 0;
  r.dirY = 0;
  r.movementAccum = 0;
  r.recentMovement = 0;
  r.directionalFrames = 0;
  r.birthTime = now;
  r.lastSeen = now;
  r.lastMovementTime = now;
  r.lastHumanTime = now;
  r.aiLastPassTime = now;
  r.aiLowSince = 0;
  r.aiConfirmedHuman = true;
  r.humanLock = true;
  r.humanGhost = false;
  r.humanGhostSince = 0;
  r.humanGhostEffectiveTime = 0;
  r.radarSpeed = rawSpeed;
  r.framesSeen = 1;
  r.directionChanges = 0;
  r.chaoticScore = 0;
  r.speedEvidenceFrames = 0;
  r.historyCount = 1;
  r.historyHead = 0;
  r.history[0] = {cx, cy, rawSpeed, now};
  r.pathLength = 0;
  r.netDisplacement = 0;
  r.pathRatio = 0;
  r.directionReversals = 0;
  r.repeatHits = 0;
  r.orbitScore = 0;
  r.lastRepeatTime = 0;
  r.observeSince = now;
  r.switchPending = false;
  r.switchPendingFrames = 0;
  r.switchPendingSince = 0;
  r.switchPendingX = 0;
  r.switchPendingY = 0;
  r.switchPendingSpeed = 0;
  r.temporalScore = 0.0f;
  r.temporalPositiveCount = 0;
  r.temporalReady = false;

  datasetLogf("[HG] GHOST T%d -> T%d ACCEPTED x=%.0f y=%.0f\n",
                ghostIdx + 1, candidateSlot + 1, cx, cy);

  // The old ghost slot must be reset only after the new slot is fully armed.
  resetTarget(ghostIdx);
  return true;
}

// Check one valid radar observation against a same-slot HUMAN GHOST.
bool processHumanGhostSameSlot(int i, int16_t rawX, int16_t rawY,
                               int16_t rawSpeed, unsigned long now) {
  RadarTarget &g = targets[i];
  if (!g.humanGhost || !g.humanLock) return false;

  float cx = (float)rawX;
  float cy = (float)rawY;
  float predX, predY, gate;
  getHumanGhostPrediction(g, now, predX, predY, gate);
  float fromGhost = distance2D(cx, cy, predX, predY);
  if (fromGhost > gate) return false;
  if (!humanGhostDirectionCompatible(g, cx, cy, now)) return false;

  if (g.ghostCandidateSlot != i) {
    g.ghostCandidateSlot = i;
    g.ghostCandidateFrames = 1;
    g.ghostCandidateSince = now;
    g.ghostCandidateX = cx;
    g.ghostCandidateY = cy;
  } else {
    float clusterMove = distance2D(cx, cy, g.ghostCandidateX, g.ghostCandidateY);
    if (now - g.ghostCandidateSince > HUMAN_GHOST_CLUSTER_TIMEOUT ||
        clusterMove > HUMAN_GHOST_CLUSTER_RADIUS) {
      g.ghostCandidateFrames = 1;
      g.ghostCandidateSince = now;
    } else if (g.ghostCandidateFrames < 255) {
      g.ghostCandidateFrames++;
    }
    g.ghostCandidateX = cx;
    g.ghostCandidateY = cy;
  }

  if (g.ghostCandidateFrames < HUMAN_GHOST_REQUIRED_FRAMES) return true;

  // Same-slot reacquisition: re-arm the old track in place.
  g.humanGhost = false;
  g.humanGhostSince = 0;
  g.humanGhostEffectiveTime = 0;
  g.x = cx;
  g.y = cy;
  g.prevX = cx;
  g.prevY = cy;
  g.lastSeen = now;
  g.lastHumanTime = now;
  g.aiConfirmedHuman = true;
  g.humanLock = true;
  g.state = TRACK_HUMAN;
  g.isHuman = true;
  g.isClutter = false;
  g.radarSpeed = rawSpeed;
  g.framesSeen = 1;
  g.historyCount = 1;
  g.historyHead = 0;
  g.history[0] = {cx, cy, rawSpeed, now};
  g.pathLength = 0;
  g.netDisplacement = 0;
  g.pathRatio = 0;
  g.movementAccum = 0;
  g.recentMovement = 0;
  g.directionalFrames = 0;
  g.directionChanges = 0;
  g.directionReversals = 0;
  g.repeatHits = 0;
  g.orbitScore = 0;
  g.chaoticScore = 0;
  g.speedEvidenceFrames = 0;
  g.temporalScore = 0.0f;
  g.temporalPositiveCount = 0;
  g.temporalReady = false;
  clearGhostCandidate(i);

  datasetLogf("[HG] T%d SAME-SLOT ACCEPTED x=%.0f y=%.0f\n",
                i + 1, cx, cy);
  return true;
}

// Scan all active radar slots for a target that can reclaim a HUMAN GHOST.
void tryCrossSlotHumanReacquisition(int ghostIdx,
                                    TargetData* radarTargets[3],
                                    unsigned long now) {
  RadarTarget &g = targets[ghostIdx];
  if (!g.active || !g.humanGhost || !g.humanLock) return;

  if (now - g.humanGhostSince > HUMAN_GHOST_TIME) {
    datasetLogf("[RS] T%d HG_EXP", ghostIdx + 1);
    resetTarget(ghostIdx);
    return;
  }

  for (int j = 0; j < 3; j++) {
    if (j == ghostIdx) continue;
    TargetData* rt = radarTargets[j];
    if (rt == nullptr || !rt->isValid()) continue;

    float cx = (float)rt->x;
    float cy = (float)rt->y;
    float predX, predY, gate;
    getHumanGhostPrediction(g, now, predX, predY, gate);
    float d = distance2D(cx, cy, predX, predY);
    if (d > gate) continue;
    if (!humanGhostDirectionCompatible(g, cx, cy, now)) continue;

    // Do not let a known dynamic/oscillating background object immediately
    // inherit a HUMAN lock during ghost recovery unless it has real Doppler
    // evidence. This protects against the tree pattern seen in field tests.
    if (targets[j].speedEvidenceFrames < 3 &&
        vegetationPatternScore(j) >= 0.45f &&
        isKnownDynamicBackground(cx, cy)) {
      continue;
    }

    if (acceptHumanGhostReacquisition(ghostIdx, j, rt->x, rt->y, rt->speed, now)) {
      break;
    }
  }
}

// Return true when the current candidate position is close enough to the
// predicted position of another HUMAN GHOST to be treated as a possible
// cross-slot continuation. This is deliberately a pre-classification gate:
// it does not delete, merge, or alter any track. It only prevents the new
// slot from becoming HUMAN before tryCrossSlotHumanReacquisition() gets a
// chance to reclaim it.
bool humanGhostNearPreclassificationCandidate(int candidateSlot,
                                               float cx, float cy,
                                               unsigned long now) {
  for (int j = 0; j < 3; j++) {
    if (j == candidateSlot) continue;

    RadarTarget &g = targets[j];
    if (!g.active || !g.humanLock || !g.humanGhost) continue;

    float predX, predY, gate;
    getHumanGhostPrediction(g, now, predX, predY, gate);

    float d = distance2D(cx, cy, predX, predY);
    float effectiveGate = gate;
    if (effectiveGate > HUMAN_GHOST_PRECLASS_GATE) {
      effectiveGate = HUMAN_GHOST_PRECLASS_GATE;
    }

    if (d <= effectiveGate &&
        humanGhostDirectionCompatible(g, cx, cy, now)) {
      return true;
    }
  }

  return false;
}

// =========================================================
// ОБРОБКА ОДНІЄЇ ЦІЛІ
// =========================================================
void processTarget(
  int i, int16_t rawX, int16_t rawY, int16_t rawSpeed, unsigned long now
) {
  if(rawX==0 && rawY==0) return;
  RadarTarget &t=targets[i];

  if (t.humanGhost) {
    processHumanGhostSameSlot(i, rawX, rawY, rawSpeed, now);
    if (t.humanGhost) return;
    // Same-slot reacquisition accepted and the track has been re-armed.
    return;
  }

  if(!t.active) startNewTarget(i,rawX,rawY,rawSpeed,now);
  else {
    t.radarSpeed=rawSpeed;
    int16_t a=abs(rawSpeed);
    if(a>=RADAR_SPEED_MIN && a<=RADAR_SPEED_MAX) {
      if(t.speedEvidenceFrames<255)t.speedEvidenceFrames++;
    } else if(t.speedEvidenceFrames>0) t.speedEvidenceFrames--;

    float oldX=t.x,oldY=t.y;
    float jump=distance2D(rawX,rawY,oldX,oldY);
    if(jump>TARGET_SWITCH_DISTANCE) {
      // A confirmed HUMAN gets a short re-association window.  Do not
      // destroy the lock on the first large slot jump.
      if(t.humanLock) {
        if(!tryLockedHumanReassociation(i,rawX,rawY,rawSpeed,now)) {
          // Candidate is too far / unsafe: this really is a new track.
          startNewTarget(i,rawX,rawY,rawSpeed,now);
        }
      } else if(tryHumanTrackContinuity(i,rawX,rawY,rawSpeed,now)) {
        // Strong pre-lock human evidence survives a suspicious slot jump.
      } else if(tryAdaptiveBackgroundContinuation(i,rawX,rawY,rawSpeed,now)) {
        // Known vegetation moved outside the original 60 s envelope.
        // Keep the same logical track and its accumulated history.
      } else {
        startNewTarget(i,rawX,rawY,rawSpeed,now);
      }
    } else {
      // Any normal frame cancels an unfinished switch candidate.
      if(t.switchPending) {
        t.switchPending = false;
        t.switchPendingFrames = 0;
        t.switchPendingSince = 0;
      }
      if(t.humanContinuityPending) {
        t.humanContinuityPending = false;
        t.humanContinuityFrames = 0;
        t.humanContinuitySince = 0;
      }
      t.x=oldX*(1.0-TRACK_ALPHA)+rawX*TRACK_ALPHA;
      t.y=oldY*(1.0-TRACK_ALPHA)+rawY*TRACK_ALPHA;
      float dx=t.x-oldX,dy=t.y-oldY;
      float step=sqrtf(dx*dx+dy*dy);
      t.recentMovement=step;
      updateEntryVector(i, dx, dy);
      if(updateMovementDirection(i,dx,dy)) {
        t.movementAccum+=step;
        t.lastMovementTime=now;
      }
      pushHistory(i,t.x,t.y,rawSpeed,now);
      analyzeTrajectory(i,now);
      if(t.framesSeen<65535)t.framesSeen++;

      // =====================================================
      // v0.3.1 SCENE MAP + ENTRY VECTOR
      // Learned scene remains a strong anti-tree prior, but a NEW target
      // that clearly entered from the perimeter is allowed through.
      // This prevents the 60 s map from blocking a human approaching
      // from 5-7 m while still rejecting center-born background motion.
      // =====================================================
      bool learnedScene = trainingFinished && isKnownStaticBackground(t.x,t.y);
      bool edgeEntry = isEntryFromEdge(i);
      bool weakDoppler = t.speedEvidenceFrames < HUMAN_SPEED_EVIDENCE_REQUIRED;

      if(learnedScene && t.state!=TRACK_HUMAN && !edgeEntry && weakDoppler) {
        if(t.state!=TRACK_IGNORE)
          datasetLogf("[CL] map T%d learned scene + no edge entry -> IGNORE\n", i+1);
        t.state=TRACK_IGNORE;
        t.isHuman=false;
        t.isClutter=true;
      } else if(learnedScene && edgeEntry && t.entryFrames == ENTRY_REQUIRED_INWARD_FRAMES) {
        datasetLogf("[CL] E T%d edge=%u inward=%u score=%.2f -> scene veto bypass\n",
                      i+1, t.entryZone, t.entryInwardFrames, t.entryScore);
      }

      // Cache the already-computed strong-human predicate once per slot.
      // The cache is used by the post-process HUMAN GHOST handoff pass.
      vegetationClearedThisFrame[i] =
          trainingFinished && !antiTreePattern(i) &&
          hasHumanTrajectory(i) && !isOrbitLike(i) &&
          aiHumanCandidate(i);

      if(t.state==TRACK_OBSERVE) {
        unsigned long age=now-t.observeSince;
        // Same adaptive-background veto as the IGNORE -> HUMAN path below:
        // a cell the map already trusts as recurring non-human motion
        // requires a genuine edge entry before a fresh OBSERVE track is
        // allowed straight through to HUMAN.
        bool adaptiveSceneOKObserve = !isKnownAdaptiveBackground(t.x,t.y) || isEntryFromEdge(i);
        if(vegetationClearedThisFrame[i] && adaptiveSceneOKObserve &&
           !humanGhostNearPreclassificationCandidate(i, t.x, t.y, now)) {
          t.state=TRACK_HUMAN;t.isHuman=true;t.isClutter=false;t.lastHumanTime=now;
          datasetLogf("[CL] T%d H",i+1);
        } else if(trainingFinished && age>=IGNORE_CONFIRM_TIME &&
                  isOrbitLike(i) && t.speedEvidenceFrames<HUMAN_SPEED_EVIDENCE_REQUIRED) {
          t.state=TRACK_IGNORE;t.isHuman=false;t.isClutter=true;
          learnDynamicPoint(t.x,t.y,t.recentMovement,(uint8_t)max(30, (int)roundf(vegetationPatternScore(i)*100.0f)));
          datasetLogf("[CL] T%d I orb=%u/%u/%.1f",
                        i+1,t.repeatHits,t.directionReversals,t.pathRatio);
        } else if(trainingFinished && age>=OBSERVE_MAX_TIME &&
                  !hasHumanTrajectory(i)) {
          t.state=TRACK_IGNORE;t.isHuman=false;t.isClutter=true;
          datasetLogf("[CL] T%d I no-traj",i+1);
        }
      } else if(t.state==TRACK_HUMAN) {
        // Підтверджена HUMAN не демотується через швидкість 0.
        // Але фінальний AI-gate може відкликати її, якщо AI стабільно
        // бачить не-людину.
        t.isHuman=true;t.isClutter=false;t.lastHumanTime=now;
        updateAIFinalGate(i, now);
      } else {
        t.isHuman=false;t.isClutter=true;
        bool sceneOK = !isKnownStaticBackground(t.x,t.y) || isEntryFromEdge(i);
        // Adaptive-background veto: if this cell has already accumulated a
        // strong adaptive-background score (i.e. the map itself is
        // confident this is recurring non-human motion, e.g. wind-driven
        // foliage), require a genuine edge entry before allowing IGNORE ->
        // HUMAN here. This mirrors sceneOK's treatment of the static map,
        // but for the adaptive map, which previously fed learning only and
        // was never consulted at the point of HUMAN classification -- a gap
        // confirmed by a 265s false-positive episode where ADAPT_BG was
        // 76-92 (well above ADAPTIVE_BG_STRONG_THRESHOLD=55) throughout.
        bool adaptiveSceneOK = !isKnownAdaptiveBackground(t.x,t.y) || isEntryFromEdge(i);
        if(sceneOK && adaptiveSceneOK && vegetationClearedThisFrame[i] &&
           !humanGhostNearPreclassificationCandidate(i, t.x, t.y, now) &&
           (t.speedEvidenceFrames>=HUMAN_SPEED_EVIDENCE_REQUIRED || isEntryFromEdge(i))) {
          t.state=TRACK_HUMAN;t.isHuman=true;t.isClutter=false;t.lastHumanTime=now;
          datasetLogf("[CL] T%d I>H",i+1);
        }
      }

      if(trainingFinished && t.state==TRACK_IGNORE && !t.humanLock &&
         !isEntryFromEdge(i) &&
         t.recentMovement>=DYNAMIC_MIN_STEP && t.recentMovement<=DYNAMIC_MAX_STEP) {
        // v0.9: map learning is separate from the veto score. We only reinforce
        // a cell after the track is already classified non-human AND there is
        // independent recurrence evidence. This breaks score -> map -> score.
        float vegLearn = vegetationPatternScore(i);
        bool independentRepeat =
          t.repeatHits >= 20 &&
          t.directionReversals >= 1 &&
          t.netDisplacement < 2600.0f;
        bool knownBackground =
          isKnownStaticBackground(t.x,t.y) ||
          isKnownAdaptiveBackground(t.x,t.y) ||
          getLocalDynamicScore(t.x,t.y) >= ADAPTIVE_BG_THRESHOLD;
        bool nonHumanAI = true;
        if (AI_ENABLED) {
          float lh, lv, lu;
          aiPredict(i, lh, lv, lu);
          nonHumanAI = (lv >= lh || lu >= lh || lh < 0.50f);
        }

        if (independentRepeat && nonHumanAI && (knownBackground || vegLearn >= 0.45f)) {
          uint8_t add = (vegLearn >= 0.65f) ? (ADAPTIVE_LEARN_ADD + 1) : ADAPTIVE_LEARN_ADD;
          learnAdaptiveBackgroundPoint(t.x,t.y,add);
          learnDynamicPoint(t.x,t.y,t.recentMovement,(uint8_t)roundf(vegLearn*100.0f));
        }
      }

      if(t.state!=TRACK_HUMAN &&
         now-t.birthTime>STATIC_CONFIRM_TIMEOUT &&
         now-t.lastMovementTime>STATIC_CONFIRM_TIMEOUT &&
         t.netDisplacement<700.0 && isKnownStaticBackground(t.x,t.y)) {
        t.state=TRACK_IGNORE;t.isHuman=false;t.isClutter=true;
      }

      float angleAbs=fabsf(atan2f(t.x,t.y)*180.0/PI);
      if(angleAbs>70.0) {
        float em=distance2D(t.x,t.y,edgeLastX[i],edgeLastY[i]);
        if(em<40.0) {
          if(edgeStaticStart[i]==0)edgeStaticStart[i]=now;
          if(now-edgeStaticStart[i]>2200 && t.state!=TRACK_HUMAN) {
            t.state=TRACK_IGNORE;t.isHuman=false;t.isClutter=true;
          }
        } else { edgeStaticStart[i]=now;edgeLastX[i]=t.x;edgeLastY[i]=t.y; }
      } else { edgeStaticStart[i]=now;edgeLastX[i]=t.x;edgeLastY[i]=t.y; }
    }
  }

  t.lastSeen=now;
  t.distance=sqrtf(t.x*t.x+t.y*t.y);
  // v0.9.6: angleBle used to be sent as -rawAngle to compensate for an
  // assumed left/right mirror in the app. In the field the OLED (which
  // uses the un-inverted rawAngle) is correct and the app is mirrored, so
  // that compensation was backwards. Send the same convention as the OLED.
  float rawAngle=atan2f(t.x,t.y)*180.0/PI;
  t.angleOled=90.0+rawAngle;
  t.angleBle=rawAngle;
  if(t.state==TRACK_HUMAN){
    // isHuman remains true for internal tracking, while OLED/BLE/buzzer
    // are gated by aiConfirmedHuman.
    t.isHuman=true;t.isClutter=false;
  } else t.isHuman=false;
}

// =========================================================
// ANTI-TREE PRE-HUMAN VETO
// Strong repeated oscillation + repeated direction reversals
// is treated as a vegetation/wind signature. This veto runs BEFORE
// any OBSERVE/IGNORE -> HUMAN transition, so a false AI spike cannot
// create a visible HUMAN target. HUMAN_LOCK is armed only afterwards,
// strictly as a track-stabilization mechanism.
// =========================================================
const uint16_t ANTI_TREE_REPEAT_THRESHOLD = 250;
const uint8_t  ANTI_TREE_REVERSAL_THRESHOLD = 2;

// Field-test pattern found in v0.3.3 and v0.3.4 tree logs:
// a wind-driven tree can keep one radar track in the same spatial
// corridor for many seconds.  The track accumulates repeatHits and
// pathLength, while its netDisplacement stays comparatively small.
// chaoticScore is often 0, so the old dynamicMap/chaos gate could not
// recognize it.
const uint16_t ANTI_TREE_PERSIST_REPEAT = 30;
const float    ANTI_TREE_PERSIST_PATH_MM = 1500.0f;
const float    ANTI_TREE_PERSIST_DISP_MM = 2300.0f;
const uint16_t ANTI_TREE_STRONG_REPEAT = 150;

bool antiTreePattern(int i) {
  RadarTarget &t = targets[i];
  // Edge-entries used to get an unconditional bypass here, so vegetation
  // swaying in/out at the boundary of the scan range (a bush half in view)
  // could never be caught by this veto no matter how long it kept
  // re-entering. First-time entries still get the benefit of the doubt
  // (a genuinely approaching person starts with repeatHits==0), but once
  // the same spot keeps re-entering/re-leaving enough to accumulate real
  // repeat evidence, the edge-entry exemption expires and this veto
  // applies exactly like anywhere else in the field.
  if (isEntryFromEdge(i) && t.repeatHits < ANTI_TREE_PERSIST_REPEAT) return false;

  float veg = vegetationPatternScore(i);
  bool saturatedOrbit =
    t.repeatHits >= ANTI_TREE_REPEAT_THRESHOLD &&
    t.directionReversals >= ANTI_TREE_REVERSAL_THRESHOLD;
  bool strongPersistent =
    t.repeatHits >= ANTI_TREE_STRONG_REPEAT &&
    t.netDisplacement < ANTI_TREE_PERSIST_DISP_MM;
  bool persistentReturn =
    t.repeatHits >= ANTI_TREE_PERSIST_REPEAT &&
    t.pathLength >= ANTI_TREE_PERSIST_PATH_MM &&
    t.netDisplacement < ANTI_TREE_PERSIST_DISP_MM;
  bool compactPersistentReturn =
    t.repeatHits >= 50 && t.pathLength >= 1200.0f &&
    t.netDisplacement < 2000.0f;

  return veg >= 0.68f || saturatedOrbit || strongPersistent ||
         persistentReturn || compactPersistentReturn;
}

// =========================================================
// ДРУК РАДАРА
// =========================================================
void printRadarData() {
  static unsigned long lastRadarLog = 0;
  unsigned long now = millis();
  if (now - lastRadarLog < 1000UL) return;
  lastRadarLog = now;
  char line[180];
  int pos = snprintf(line, sizeof(line), "[R] %s V=%.2f B=%d",
                     trainingFinished ? "RUN" : "TRN", cachedVoltage,
                     buzzerEnabled ? 1 : 0);
  for (int i = 0; i < 3 && pos < (int)sizeof(line); i++) {
    RadarTarget &t = targets[i];
    if (t.active) {
      const char *st = (t.state == TRACK_HUMAN) ? "H" :
                       (t.state == TRACK_IGNORE) ? "I" : "O";
      pos += snprintf(line + pos, sizeof(line) - pos,
                      " T%d=%s,%.1f,%d", i + 1, st,
                      t.distance / 1000.0f, (int)t.radarSpeed);
    } else {
      pos += snprintf(line + pos, sizeof(line) - pos, " T%d=-", i + 1);
    }
  }
  datasetLog(line);
}

// =========================================================
// BUZZER INTERVAL
// =========================================================
unsigned long getBeepInterval() {

  float nearest = 99999.0;

  for (int i = 0; i < 3; i++) {

    // v5: тон/інтервал сигналу рахуємо тільки по підтверджених
    // людях (isHuman), а не по будь-якій не-clutter цілі —
    // інакше гілка, яка ще не встигла стати isClutter, теж
    // впливала б на гучність/частоту сигналу.
    if (
      targets[i].active &&
      targets[i].isHuman &&
      targets[i].aiConfirmedHuman &&
      targets[i].distance < nearest
    ) {

      nearest =
        targets[i].distance;
    }
  }

  if (nearest < 700)
    return 140;

  if (nearest < 1500)
    return 220;

  if (nearest < 2500)
    return 320;

  if (nearest < 4000)
    return 450;

  return 650;
}

// =========================================================
// BUZZER
// =========================================================
void updateBuzzer(
  unsigned long now
) {

  if (
    !buzzerEnabled ||
    !anyTargetDetected ||
    !trainingFinished
  ) {

    noTone(BUZZER_PIN);
    buzzerOn = false;

    return;
  }

  if (buzzerOn) {

    if (
      now - lastBeep >=
      BEEP_DURATION
    ) {

      noTone(BUZZER_PIN);
      buzzerOn = false;
    }

    return;
  }

  unsigned long interval =
    getBeepInterval();

  if (
    now - lastBeep >=
    interval
  ) {

    lastBeep = now;

    tone(
      BUZZER_PIN,
      1800
    );

    buzzerOn = true;
  }
}

// =========================================================
// OLED
// =========================================================
void updateRadar() {

  // Draw the complete frame into RAM first, then transfer one finished
  // frame to the ST7789. This removes the visible black-frame flicker.
  GFXcanvas16 &c = radarCanvas;
  c.fillScreen(ST77XX_BLACK);

  // -------------------------------------------------------
  // HEADER
  // -------------------------------------------------------
  c.setTextWrap(false);
  c.setTextSize(2);
  c.setTextColor(ST77XX_WHITE);

  c.setCursor(0, 0);
  c.print(cachedVoltage, 2);
  c.print("V");

  // BZ status is intentionally the same text size as the voltage.
  c.setCursor(0, 18);
  c.print(buzzerEnabled ? "BZ:ON" : "BZ:OFF");

  // v0.9.9: chip temperature, same size, directly under BZ.
  c.setCursor(0, 36);
  // v0.9.11: "TEMP:" -> "TP:", degree symbol dropped -- char(176) didn't
  // render correctly on this display's font, so just print the number + C.
  c.print("TP:");
  c.print((int)roundf(cachedTempC));
  c.print("C");

  // RUN / calibration status centered at the top.
  const char* statusText = trainingFinished ? "RUN" : "CAL";
  if (!trainingFinished) {
    unsigned long elapsed = millis() - bootTime;
    int secondsLeft =
      (elapsed < TRAINING_TIME)
        ? (TRAINING_TIME - elapsed + 999) / 1000
        : 0;

    char calText[12];
    snprintf(calText, sizeof(calText), "CAL:%d", secondsLeft);

    c.setTextSize(2);
    c.setTextColor(ST77XX_GREEN);
    int16_t x1, y1;
    uint16_t w, h;
    c.getTextBounds(calText, 0, 0, &x1, &y1, &w, &h);
    c.setCursor((SCREEN_WIDTH - (int16_t)w) / 2, 0);
    c.print(calText);
  }
  else {
    c.setTextSize(2);
    c.setTextColor(ST77XX_GREEN);
    int16_t x1, y1;
    uint16_t w, h;
    c.getTextBounds(statusText, 0, 0, &x1, &y1, &w, &h);
    c.setCursor((SCREEN_WIDTH - (int16_t)w) / 2, 0);
    c.print(statusText);
  }

  // -------------------------------------------------------
  // TARGET INFO — same font size as voltage.
  // Right-aligned so the larger text does not collide with voltage/BZ/status.
  // -------------------------------------------------------
  c.setTextSize(2);
  c.setTextColor(ST77XX_RED);
  int infoY = 0;

  for (int i = 0; i < 3; i++) {
    if (targets[i].active && targets[i].state == TRACK_HUMAN &&
        targets[i].isHuman && targets[i].aiConfirmedHuman && trainingFinished) {
      char targetText[16];
      snprintf(targetText, sizeof(targetText), "T%d:%.1fm", i + 1,
               targets[i].distance / 1000.0);
      int16_t tx1, ty1;
      uint16_t tw, th;
      c.getTextBounds(targetText, 0, infoY, &tx1, &ty1, &tw, &th);
      int16_t tx = SCREEN_WIDTH - (int16_t)tw - 2;
      if (tx < 0) tx = 0;
      c.setCursor(tx, infoY);
      c.print(targetText);
      infoY += 18;
    }
  }

  // -------------------------------------------------------
  // RADAR GRID — upper semicircle only, green.
  // The radar origin is at the bottom edge of the screen so no
  // lower/under-origin semicircles can appear.
  // -------------------------------------------------------
  const uint16_t gridColor = ST77XX_GREEN;

  // v0.9.7: reverted the v0.9.6 FOV-clipped arcs. Rings are full circles
  // again (drawCircle), on purpose sticking out past the +-60 deg spokes
  // at the bottom corners, even off-screen — that's the look that was
  // asked for, over the cleanly-closed sector shape.
  for (float ringMm = GRID_RING_MM; ringMm <= MAX_DISPLAY_DISTANCE + 1.0f; ringMm += GRID_RING_MM) {
    float f = ringMm / MAX_DISPLAY_DISTANCE;
    c.drawCircle(CX, CY, (int)roundf(f * RADAR_R), gridColor);
  }

  // v0.9.8: restored the two outer spokes (+-80 deg) that v0.9.5 dropped
  // when the spoke range was narrowed to the real +-60 deg FOV. 9 spokes
  // total, as in the original v0.6-0.9 grid: -80,-60,-40,-20,0,20,40,60,80.
  for (int angle = -80; angle <= 80; angle += 20) {
    float rad = angle * PI / 180.0;
    c.drawLine(
      CX,
      CY,
      CX + (int)roundf(sinf(rad) * RADAR_R),
      CY - (int)roundf(cosf(rad) * RADAR_R),
      gridColor
    );
  }

  c.fillCircle(CX, CY, 3, gridColor);

  // -------------------------------------------------------
  // HUMAN TARGETS — red marker, beam and pulse.
  // -------------------------------------------------------
  for (int i = 0; i < 3; i++) {
    if (
      !targets[i].active ||
      targets[i].state != TRACK_HUMAN ||
      !targets[i].isHuman ||
      !trainingFinished
    )
      continue;

    float distance = targets[i].distance;
    if (distance > MAX_DISPLAY_DISTANCE)
      distance = MAX_DISPLAY_DISTANCE;

    float rangeFrac = distance / MAX_DISPLAY_DISTANCE;

    float relativeAngle = targets[i].angleOled - 90.0;
    if (relativeAngle < -HALF_FOV) relativeAngle = -HALF_FOV;
    if (relativeAngle > HALF_FOV) relativeAngle = HALF_FOV;

    float rad = relativeAngle * PI / 180.0;

    // v0.9.1: bearing is measured from the forward (vertical) axis, the same
    // convention used by the grid spokes: x = sin, y = cos.
    // v0.9.10: the app and the OLED were fed the exact same angle
    // (angleBle == relativeAngle used here) yet rendered the target on
    // opposite sides -- confirmed with a matched app-screenshot/OLED-photo
    // pair. So the two disagree only in which screen-side a positive angle
    // maps to. The app was independently confirmed correct (matches the
    // real-world side the person was standing on), so the OLED's sign is
    // flipped here to match it, instead of touching angleBle again.
    // v0.9.5: same true-circle scale as the grid (RADAR_R for both axes),
    // so the marker sits on the ring that matches its real distance.
    int targetX = CX - (int)roundf(sinf(rad) * rangeFrac * RADAR_R);
    int targetY = CY - (int)roundf(cosf(rad) * rangeFrac * RADAR_R);

    c.drawLine(CX, CY, targetX, targetY, ST77XX_RED);
    c.fillCircle(targetX, targetY, 4, ST77XX_RED);

    float pulse = (sinf(millis() * 0.012f) + 1.0f) * 0.5f;
    int pulseRadius = 7 + (int)(pulse * 5.0f);
    c.drawCircle(targetX, targetY, pulseRadius, ST77XX_RED);

    if (targets[i].humanLock) {
      unsigned long phase = millis() % 900UL;
      int captureRadius = 5 + (int)((phase * 14UL) / 900UL);
      c.drawCircle(targetX, targetY, captureRadius, ST77XX_RED);
      if (captureRadius > 12) {
        c.drawCircle(targetX, targetY, captureRadius - 5, ST77XX_RED);
      }
    }

    targets[i].humanDisplayed = true;
  }

  // One completed frame -> one SPI transfer. No black frame is exposed.
  display.drawRGBBitmap(0, 0, c.getBuffer(), SCREEN_WIDTH, SCREEN_HEIGHT);
}

// =========================================================
// MAP TRAINING
// =========================================================
void updateTraining(
  unsigned long now
) {

  if (trainingFinished)
    return;

  unsigned long elapsed =
    now - bootTime;

  // -------------------------------------------------------
  // ПЕРШІ 60 СЕКУНД:
  // ВИВЧАЄМО РЕАЛЬНУ СЦЕНУ ЦІЛКОМ.
  //
  // Свідоме правило цієї версії:
  // під час стартового навчання в зоні радара НЕМАЄ ЛЮДЕЙ.
  // Тому кожна валідна radar-ціль вважається частиною
  // початкової сцени — у тому числі дерево, гілки та інша
  // рухома рослинність на вітрі.
  //
  // Сирі кадри не накопичуються. Ми лише підсилюємо
  // лічильник відповідної просторової клітинки.
  // -------------------------------------------------------
  if (elapsed < TRAINING_TIME) {

    for (int i = 0; i < 3; i++) {

      if (!targets[i].active)
        continue;

      // Будь-яке спостереження під час калібрування
      // є частиною базової сцени.
      learnMapPoint(
        targets[i].x,
        targets[i].y,
        3
      );
    }

    return;
  }

  // -------------------------------------------------------
  // ЗАВЕРШЕННЯ НАВЧАННЯ
  // -------------------------------------------------------
  startupBeep(2);

  trainingFinished = true;

  Serial.println();
  Serial.println(
    "================================================="
  );
  Serial.println(
    "TRAINING COMPLETE"
  );
  Serial.println(
    "Learned scene map is permanent until TRAINING_RESET or restart."
  );
  Serial.println(
    "Dynamic background learning is ACTIVE."
  );
  Serial.println(
    "48x32 map, 250mm cells, 3x3 local analysis."
  );
  Serial.println(
    "Movement direction analysis ACTIVE."
  );
  Serial.println(
    "3 simultaneous tracks ACTIVE."
  );
  Serial.println(
    "================================================="
  );
  Serial.println();
}

// =========================================================
// ΩΞ 0.1 COMPACT DIAGNOSTICS
// R=radar, TR=track, AI=decision evidence, CL=classification,
// HG=human ghost, TM=training/raw control, RS=reset, B=buzzer.
// States: H=HUMAN, O=OBSERVE, I=IGNORE. W=sliding HUMAN evidence score.
// Routine diagnostics run at 1 Hz; RAW is separate and OFF by default.
// =========================================================
unsigned long lastCompactDiagnostic = 0;
void printCompactDiagnostics(unsigned long now) {
  if (now - lastCompactDiagnostic < 1000UL) return;
  lastCompactDiagnostic = now;
  if (!trainingFinished) return;
  for (int i = 0; i < 3; i++) {
    RadarTarget &t = targets[i];
    if (!t.active) continue;
    float h,v,u; aiPredict(i,h,v,u);
    float veg=vegetationPatternScore(i);
    const char *st=(t.state==TRACK_HUMAN)?"H":(t.state==TRACK_IGNORE)?"I":"O";
    datasetLogf("[TR] T%d S=%s X=%.0f Y=%.0f R=%.0f SP=%d P=%.0f D=%.0f REP=%u REV=%u",
      i+1,st,t.x,t.y,t.distance,t.radarSpeed,t.pathLength,t.netDisplacement,
      (unsigned)t.repeatHits,(unsigned)t.directionReversals);
    datasetLogf("[AI] T%d H=%.2f V=%.2f U=%.2f W=%d G=%.2f A=%u D=%u F=%d",
      i+1,h,v,u,(int)t.humanEvidenceSum,veg,
      (unsigned)getLocalAdaptiveBackgroundScore(t.x,t.y),
      (unsigned)getLocalDynamicScore(t.x,t.y),t.aiConfirmedHuman?1:0);
  }
}

// =========================================================
// SETUP
// =========================================================
void startupBeep(uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    tone(BUZZER_PIN, STARTUP_BEEP_FREQ);
    delay(STARTUP_BEEP_DURATION);
    noTone(BUZZER_PIN);
    if (i + 1 < count) delay(STARTUP_BEEP_GAP);
  }
}

// =========================================================
// TTP223 TOUCH BUTTON HANDLER + ANTHEM MELODY
// Pure UX/cosmetic layer: reads the touch pin, debounces it, counts clicks,
// and dispatches one of three actions. None of this reads or writes any
// tracking/classification state (targets[], AI scores, scene maps, etc.),
// so it cannot affect detection behavior.
// =========================================================

// Short excerpt of the well-known "Imperial March" theme (opening motif,
// key of G minor, Maestoso tempo), single voice, passive-buzzer tone()
// playback. Roughly 10 seconds including inter-note gaps.
struct AnthemNote { uint16_t freqHz; uint16_t durMs; };
const AnthemNote ANTHEM_MELODY[] = {
  {392, 500}, // G4
  {392, 500}, // G4
  {392, 500}, // G4
  {311, 350}, // Eb4
  {466, 150}, // Bb4
  {392, 500}, // G4
  {311, 350}, // Eb4
  {466, 150}, // Bb4
  {392, 1000}, // G4 (long)
  {587, 500}, // D5
  {587, 500}, // D5
  {587, 500}, // D5
  {622, 350}, // Eb5
  {466, 150}, // Bb4
  {370, 500}, // F#4 (passing tone)
  {311, 350}, // Eb4
  {466, 150}, // Bb4
  {392, 1000}, // G4 (long, final)
};
const uint8_t ANTHEM_NOTE_COUNT = sizeof(ANTHEM_MELODY) / sizeof(ANTHEM_MELODY[0]);
const uint16_t ANTHEM_NOTE_GAP_MS = 110;

// Draws a simple pixel-block silhouette of the Ukrainian tryzub (coat of
// arms), anchored at (originX, originY) as its top-left corner. Pure
// Adafruit_GFX fillRect() calls -- no bitmap/PROGMEM data at all, 45
// rectangles, drawn in a 92x152px design grid.
void drawTryzub(int16_t originX, int16_t originY, uint16_t color) {
  display.fillRect(originX + 44, originY + 0, 4, 88, color);
  display.fillRect(originX + 0, originY + 12, 4, 112, color);
  display.fillRect(originX + 88, originY + 12, 4, 112, color);
  display.fillRect(originX + 4, originY + 20, 4, 8, color);
  display.fillRect(originX + 84, originY + 20, 4, 8, color);
  display.fillRect(originX + 8, originY + 24, 4, 4, color);
  display.fillRect(originX + 80, originY + 24, 4, 4, color);
  display.fillRect(originX + 12, originY + 28, 4, 12, color);
  display.fillRect(originX + 76, originY + 28, 4, 12, color);
  display.fillRect(originX + 16, originY + 32, 4, 32, color);
  display.fillRect(originX + 72, originY + 32, 4, 32, color);
  display.fillRect(originX + 20, originY + 52, 4, 28, color);
  display.fillRect(originX + 68, originY + 52, 4, 28, color);
  display.fillRect(originX + 24, originY + 56, 4, 24, color);
  display.fillRect(originX + 64, originY + 56, 4, 24, color);
  display.fillRect(originX + 16, originY + 80, 4, 8, color);
  display.fillRect(originX + 40, originY + 80, 4, 12, color);
  display.fillRect(originX + 48, originY + 80, 4, 12, color);
  display.fillRect(originX + 72, originY + 80, 4, 8, color);
  display.fillRect(originX + 4, originY + 84, 4, 8, color);
  display.fillRect(originX + 52, originY + 84, 4, 8, color);
  display.fillRect(originX + 8, originY + 88, 8, 4, color);
  display.fillRect(originX + 76, originY + 88, 12, 4, color);
  display.fillRect(originX + 12, originY + 92, 4, 4, color);
  display.fillRect(originX + 32, originY + 92, 4, 16, color);
  display.fillRect(originX + 56, originY + 92, 4, 16, color);
  display.fillRect(originX + 76, originY + 92, 4, 4, color);
  display.fillRect(originX + 16, originY + 96, 4, 8, color);
  display.fillRect(originX + 60, originY + 96, 4, 40, color);
  display.fillRect(originX + 72, originY + 96, 4, 8, color);
  display.fillRect(originX + 20, originY + 104, 12, 4, color);
  display.fillRect(originX + 64, originY + 104, 8, 4, color);
  display.fillRect(originX + 28, originY + 108, 4, 28, color);
  display.fillRect(originX + 40, originY + 108, 16, 4, color);
  display.fillRect(originX + 40, originY + 112, 12, 4, color);
  display.fillRect(originX + 44, originY + 116, 4, 36, color);
  display.fillRect(originX + 4, originY + 120, 24, 4, color);
  display.fillRect(originX + 32, originY + 120, 12, 4, color);
  display.fillRect(originX + 48, originY + 120, 12, 4, color);
  display.fillRect(originX + 64, originY + 120, 24, 4, color);
  display.fillRect(originX + 32, originY + 136, 4, 4, color);
  display.fillRect(originX + 56, originY + 136, 4, 4, color);
  display.fillRect(originX + 40, originY + 140, 4, 8, color);
  display.fillRect(originX + 48, originY + 140, 8, 4, color);
  display.fillRect(originX + 48, originY + 144, 4, 4, color);
}

// Displays the tryzub centered on the 240x240 screen against a blue
// background (Ukrainian national colors). The 92x152 design grid was
// chosen to read clearly at native size on this panel.
void showCoatOfArms() {
  display.fillScreen(ST77XX_BLUE);
  int16_t artW = 92;
  int16_t artH = 152;
  int16_t x = (SCREEN_WIDTH - artW) / 2;
  int16_t y = (SCREEN_HEIGHT - artH) / 2;
  drawTryzub(x, y, ST77XX_YELLOW);
}

// Blocking on purpose: this only runs on a deliberate 10-click gesture, not
// during normal radar operation, and radar frames are not lost by pausing
// for the ~9s the excerpt takes -- the RD-03D/UART buffer tolerates this
// the same way the existing startup beep sequence already does.
void playAnthemExcerpt() {
  if (!buzzerEnabled) return;
  anthemPlaying = true;
  showCoatOfArms();
  for (uint8_t i = 0; i < ANTHEM_NOTE_COUNT; i++) {
    tone(BUZZER_PIN, ANTHEM_MELODY[i].freqHz);
    delay(ANTHEM_MELODY[i].durMs);
    noTone(BUZZER_PIN);
    delay(ANTHEM_NOTE_GAP_MS);
  }
  anthemPlaying = false;
  // updateRadar() on the next loop() iteration repaints the normal radar
  // screen, the same way it already does after every frame.
}

void handleTouchButton() {
  bool rawState = digitalRead(TOUCH_BUTTON_PIN) == HIGH;
  unsigned long now = millis();

  // Debounce
  if (rawState != touchPinState) {
    touchPinState = rawState;
    touchLastChangeTime = now;
  }
  if (now - touchLastChangeTime >= TOUCH_DEBOUNCE_MS &&
      touchPinStableState != touchPinState) {
    touchPinStableState = touchPinState;

    if (touchPinStableState) {
      // Press started
      touchPressStartTime = now;
      touchHoldActionFired = false;
    } else {
      // Release: count a click only if the hold-action did not already fire
      unsigned long heldFor = now - touchPressStartTime;
      if (!touchHoldActionFired && heldFor < TOUCH_HOLD_MS) {
        if (now - touchLastClickTime > TOUCH_CLICK_GAP_MS) {
          touchClickCount = 0;
        }
        touchClickCount++;
        touchLastClickTime = now;

        if (touchClickCount >= TOUCH_ANTHEM_CLICK_COUNT) {
          touchClickCount = 0;
          playAnthemExcerpt();
        }
      }
    }
  }

  // Long-hold detection while the button is still pressed
  if (touchPinStableState && !touchHoldActionFired &&
      (now - touchPressStartTime) >= TOUCH_HOLD_MS) {
    touchHoldActionFired = true;
    touchClickCount = 0;
    resetSceneTraining();
    datasetLog("[TM] RESET");
  }

  // Resolve a completed single-click gesture once the click-train timeout
  // has elapsed without reaching the anthem threshold.
  if (touchClickCount == 1 &&
      !touchPinStableState &&
      (now - touchLastClickTime) > TOUCH_CLICK_GAP_MS) {
    touchClickCount = 0;
    buzzerEnabled = !buzzerEnabled;
    if (!buzzerEnabled) noTone(BUZZER_PIN);
    datasetLogf("[B] %d", buzzerEnabled ? 1 : 0);
  }
}

void setup() {

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.println("Spartak Radar System");
  Serial.print("Firmware: ");
  Serial.println(SRS_FIRMWARE_NAME);
  Serial.print("Build: ");
  Serial.println(SRS_BUILD_DATE);
  Serial.println("========================================");
  Serial.println();

  // -------------------------------------------------------
  // BUZZER
  // -------------------------------------------------------
  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  noTone(BUZZER_PIN);

  // -------------------------------------------------------
  // TTP223 TOUCH BUTTON
  // -------------------------------------------------------
  pinMode(TOUCH_BUTTON_PIN, INPUT);

  analogReadResolution(12);

  // -------------------------------------------------------
  // RD-03D RADAR
  // -------------------------------------------------------
  // The library configures the UART and switches the radar
  // to MULTI_TARGET mode.
  if (!radar.initialize(RD03D::MULTI_TARGET)) {
    Serial.println("RD03D initialization/ACK timeout!");
    Serial.println("Continuing anyway; check RX/TX wiring if no targets appear.");
  }
  else {
    Serial.println("RD03D initialized in MULTI_TARGET mode.");
  }

  // -------------------------------------------------------
  // ST7789 TFT (205x240 window on a 240x240 IPS panel)
  // -------------------------------------------------------
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH); // backlight on

  // The GMT130-V1.0 is ST7789, 240x240, 7-pin SPI without CS.
  // IMPORTANT: use hardware SPI so SPI_MODE3 is actually applied.
  // 60 MHz is used with a RAM backbuffer: the physical panel receives only
  // complete frames, so the old black-frame flicker is not visible.
  SPI.begin(TFT_SCK, -1, TFT_SDA, -1);
  display.init(240, 240, SPI_MODE3);
  display.setSPISpeed(60000000);
  display.setRotation(2);
  display.fillScreen(ST77XX_BLACK);

  // U8g2 font engine is attached to the existing Adafruit ST7789 display.
  // It is used only for UTF-8/Unicode startup text such as Ω.
  u8g2Fonts.begin(display);

  // -------------------------------------------------------
  // RESET TARGETS
  // -------------------------------------------------------
  for (int i = 0; i < 3; i++)
    resetTarget(i);

  // -------------------------------------------------------
  // STARTUP
  // -------------------------------------------------------
  showStartup();

  // -------------------------------------------------------
  // BATTERY
  // -------------------------------------------------------
  cachedVoltage =
    readBatteryVoltage();

  lastBatteryCheck =
    millis();

  cachedTempC =
    temperatureRead();

  lastTempCheck =
    millis();

  // -------------------------------------------------------
  // MAP
  // -------------------------------------------------------
  memset(
    bgMap,
    0,
    sizeof(bgMap)
  );

  memset(
    dynamicMap,
    0,
    sizeof(dynamicMap)
  );

  memset(
    oscillationMap,
    0,
    sizeof(oscillationMap)
  );

  // One short beep marks the exact start of the 60 s training countdown.
  startupBeep(1);

  // Навчання стартує після startup + beep.
  bootTime =
    millis();

  trainingFinished = false;

  // -------------------------------------------------------
  // BLE
  // -------------------------------------------------------
  BLEDevice::init(
    "AveRadar_SRS"
  );

  // Request a larger ATT MTU so diagnostic lines can be transported
  // without being artificially split into 20-byte packets.
  BLEDevice::setMTU(247);

  pServer =
    BLEDevice::createServer();

  pServer->setCallbacks(
    new MyServerCallbacks()
  );

  BLEService* pService =
    pServer->createService(
      SERVICE_UUID
    );

  pCharacteristic =
    pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_WRITE
    );

  pCharacteristic->setCallbacks(
    new MyCharacteristicCallbacks()
  );

  pCharacteristic->addDescriptor(
    new BLE2902()
  );

  // Log characteristic: NOTIFY-only mirror of every USB Serial log line.
  // Separate UUID from telemetry so the Android telemetry parser never has
  // to distinguish log text from T1/T2/T3:V packets.
  pLogCharacteristic =
    pService->createCharacteristic(
      LOG_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
    );

  pLogCharacteristic->addDescriptor(
    new BLE2902()
  );

  pService->start();

  BLEAdvertising* advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(
    SERVICE_UUID
  );

  advertising->setScanResponse(
    true
  );

  advertising->setMinPreferred(
    0x06
  );

  advertising->setMinPreferred(
    0x12
  );

  BLEDevice::startAdvertising();

  Serial.println(
    "System ready."
  );
  Serial.println("BLE log mirror: enable with BLE LOG in the app (GET_FW available)");

  Serial.println(
    "60-second scene background training started."
  );
  Serial.println(
    "Training can be restarted from Android via TRAINING_RESET."
  );
}

// =========================================================
// BLE LOG STREAM
// =========================================================
// Every line that goes to USB Serial via datasetLog()/datasetLogf() is
// mirrored 1:1 to a dedicated BLE log characteristic (pLogCharacteristic),
// completely separate from the T1/T2/T3:V telemetry characteristic so the
// Android telemetry parser never sees log text. Only active while
// bleLogStreamEnabled is set (RAW_ON/RAW_OFF from the Android app), so BLE
// traffic and the dataset queue stay idle the rest of the time. Queued and
// transmitted asynchronously; the radar loop never blocks on BLE. If BLE
// falls behind, the oldest queued chunk is dropped -- USB Serial remains
// the complete, authoritative log regardless of BLE state.
//
// Lines longer than one BLE packet's payload are split into multiple
// chunks, each prefixed with \x01<seq>/<total>\x01 (seq/total are 1-based
// decimal). The Android app reassembles chunks sharing the same "total"
// sequence before writing the line to the log file. A single-chunk line
// (the common case) carries no prefix, so short lines are unaffected.
void enqueueBleDataset(const char* line) {
  if (!deviceConnected || !bleLogStreamEnabled || pLogCharacteristic == nullptr || line == nullptr)
    return;

  size_t len = strlen(line);
  uint8_t totalChunks = (uint8_t)((len + BLE_CHUNK_PAYLOAD_SIZE - 1) / BLE_CHUNK_PAYLOAD_SIZE);
  if (totalChunks == 0) totalChunks = 1;  // empty line still gets queued as-is

  for (uint8_t seq = 1; seq <= totalChunks; seq++) {
    uint8_t next = (uint8_t)((bleDatasetHead + 1) % BLE_DATASET_QUEUE_SIZE);
    if (next == bleDatasetTail) {
      // Drop the oldest queued chunk instead of blocking the radar loop.
      bleDatasetTail = (uint8_t)((bleDatasetTail + 1) % BLE_DATASET_QUEUE_SIZE);
      bleDatasetDropped++;
    }

    char *dst = bleDatasetQueue[bleDatasetHead];
    size_t offset = (size_t)(seq - 1) * BLE_CHUNK_PAYLOAD_SIZE;
    size_t chunkLen = len - offset;
    if (chunkLen > BLE_CHUNK_PAYLOAD_SIZE) chunkLen = BLE_CHUNK_PAYLOAD_SIZE;

    if (totalChunks == 1) {
      snprintf(dst, BLE_DATASET_LINE_SIZE, "%s", line);
    } else {
      int hdrLen = snprintf(dst, BLE_DATASET_LINE_SIZE, "\x01%u/%u\x01", seq, totalChunks);
      if (hdrLen < 0) hdrLen = 0;
      if ((size_t)hdrLen < BLE_DATASET_LINE_SIZE) {
        size_t room = BLE_DATASET_LINE_SIZE - (size_t)hdrLen - 1;
        size_t copyLen = (chunkLen < room) ? chunkLen : room;
        memcpy(dst + hdrLen, line + offset, copyLen);
        dst[hdrLen + copyLen] = '\0';
      }
    }
    bleDatasetHead = next;
  }
}

void datasetLog(const char* line) {
  if (line == nullptr) return;

  // USB gets the exact line.
  Serial.println(line);

  // BLE gets the same line asynchronously, over the log characteristic.
  enqueueBleDataset(line);
}

void datasetLogf(const char* fmt, ...) {
  char msg[BLE_DATASET_LINE_SIZE];
  va_list args;
  va_start(args, fmt);
  vsnprintf(msg, sizeof(msg), fmt, args);
  va_end(args);

  // Match the previous Serial.printf() behavior: one physical line,
  // without an additional blank line from datasetLog().
  size_t n = strlen(msg);
  while (n > 0 && (msg[n - 1] == '\n' || msg[n - 1] == '\r')) {
    msg[--n] = '\0';
  }

  datasetLog(msg);
}

void serviceBleDatasetQueue() {
  if (!deviceConnected || !bleLogStreamEnabled || pLogCharacteristic == nullptr) return;
  if (bleDatasetHead == bleDatasetTail) return;  // queue empty

  unsigned long now = millis();
  if (now - lastBleDatasetTx < BLE_DATASET_TX_INTERVAL) return;
  lastBleDatasetTx = now;

  pLogCharacteristic->setValue((uint8_t*)bleDatasetQueue[bleDatasetTail],
                                strlen(bleDatasetQueue[bleDatasetTail]));
  pLogCharacteristic->notify();
  bleDatasetTail = (uint8_t)((bleDatasetTail + 1) % BLE_DATASET_QUEUE_SIZE);
}

// Reconstruct the RD-03D 30-byte multi-target frame from the public
// TargetData representation. The RD03D_Arduino library decodes each target
// from X/Y/speed/distanceRes and exposes those values publicly. The original
// UART bytes are not directly exposed by the library, so this is a
// reconstructed raw target-level frame. Zero/invalid targets are encoded as
// zero blocks; valid targets use the same sign-magnitude representation used
// by the library's parser.
void logRawRD03DFrame(uint32_t frameNo, TargetData* radarTargets[3]) {
  if (!rawCaptureEnabled) return;
  lastRawFrameNo = frameNo;
  uint8_t frame[30] = {
    0xAA, 0xFF, 0x03, 0x00,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,
    0x55, 0xCC
  };

  uint32_t commonTs = millis();
  uint8_t count = 0;

  for (int i = 0; i < 3; i++) {
    TargetData* t = radarTargets[i];
    if (t == nullptr || !t->isValid()) continue;

    count++;
    if (t->timestamp != 0) commonTs = t->timestamp;

    int base = 4 + i * 8;

    auto encodeSignMagnitude = [](int16_t value) -> uint16_t {
      if (value >= 0)
        return (uint16_t)value | 0x8000;
      return (uint16_t)(-value) & 0x7FFF;
    };

    uint16_t rawX = encodeSignMagnitude(t->x);
    uint16_t rawY = encodeSignMagnitude(t->y);
    uint16_t rawS = encodeSignMagnitude(t->speed);
    uint16_t rawD = t->distanceRes;

    frame[base + 0] = rawX & 0xFF;
    frame[base + 1] = rawX >> 8;
    frame[base + 2] = rawY & 0xFF;
    frame[base + 3] = rawY >> 8;
    frame[base + 4] = rawS & 0xFF;
    frame[base + 5] = rawS >> 8;
    frame[base + 6] = rawD & 0xFF;
    frame[base + 7] = rawD >> 8;
  }

  char hex[91];
  char* p = hex;
  size_t remaining = sizeof(hex);
  for (int i = 0; i < 30; i++) {
    int n = snprintf(p, remaining, "%02X%s", frame[i], (i == 29) ? "" : " ");
    if (n < 0 || (size_t)n >= remaining) break;
    p += n;
    remaining -= n;
  }

  char msg[220];
  snprintf(msg, sizeof(msg),
           "[RAW] F=%lu TS=%lu C=%u HEX=%s",
           (unsigned long)frameNo,
           (unsigned long)commonTs,
           count,
           hex);

  datasetLog(msg);
}

// =========================================================
// BLE TRAINING STATUS
// Android receives the authoritative ESP32 training countdown.
// Format: TRAINING:<seconds> and finally TRAINING:COMPLETE.
// =========================================================
void sendTrainingBleStatus(unsigned long now) {
  if (!deviceConnected || pCharacteristic == nullptr) return;

  if (!trainingFinished) {
    unsigned long elapsed = now - bootTime;
    int secondsLeft = (elapsed < TRAINING_TIME)
        ? (int)((TRAINING_TIME - elapsed + 999) / 1000)
        : 0;

    if (now - lastTrainingBleUpdate < 1000) return;
    lastTrainingBleUpdate = now;

    char msg[32];
    snprintf(msg, sizeof(msg), "TRAINING:%d", secondsLeft);
    pCharacteristic->setValue(msg);
    pCharacteristic->notify();
    return;
  }

  if (!trainingCompleteStatusSent) {
    trainingCompleteStatusSent = true;
    lastTrainingBleUpdate = now;
    pCharacteristic->setValue("TRAINING:COMPLETE");
    pCharacteristic->notify();
  }
}

// =========================================================
// LOOP
// =========================================================
void loop() {

  unsigned long now =
    millis();

  // =======================================================
  // TTP223 TOUCH BUTTON (cosmetic UX layer, see handleTouchButton())
  // =======================================================
  handleTouchButton();

  // =======================================================
  // BATTERY
  // =======================================================
  if (
    now - lastBatteryCheck >=
    BATTERY_UPDATE_INTERVAL
  ) {

    cachedVoltage =
      readBatteryVoltage();

    lastBatteryCheck =
      now;
  }

  // v0.9.9: chip temperature, refreshed independently of the battery voltage.
  if (
    now - lastTempCheck >=
    TEMP_UPDATE_INTERVAL
  ) {

    cachedTempC =
      temperatureRead();

    lastTempCheck =
      now;
  }

  // =======================================================
  // RADAR DATA — RD03D_Arduino 1.0.1
  // =======================================================
  // tasks() must be called frequently. It consumes the UART
  // stream and updates the library's three target slots.
  if (radar.tasks()) {

    lastRadarFrame = now;
    radarFrameCount++;

    TargetData* radarTargets[3] = {
      radar.getTarget(0),
      radar.getTarget(1),
      radar.getTarget(2)
    };

    // RAW RD-03D target-level capture. This happens immediately after the
    // library parsed a frame and before processTarget()/AI/tracking logic.
    // The line reconstructs the 30-byte RD-03D multi-target frame from the
    // public TargetData fields exposed by RD03D_Arduino. It is therefore
    // raw target-level data, not baseband/IQ/ADC data.
    logRawRD03DFrame(radarFrameCount, radarTargets);

    // -----------------------------------------------------
    // 3 TARGETS
    // -----------------------------------------------------
    for (int i = 0; i < 3; i++) vegetationClearedThisFrame[i] = false;

    for (int i = 0; i < 3; i++) {

      TargetData* rt = radarTargets[i];

      if (rt != nullptr && rt->isValid()) {

        // Library gives x/y in millimetres.
        // Keep the original SRS coordinate convention:
        // x = left/right, y = forward.
        processTarget(
          i,
          rt->x,
          rt->y,
          rt->speed,
          now
        );
      }
      else {
        // Do not immediately reset a target.
        // HUMAN tracks can enter GHOST, while other targets use timeout.
      }
    }

    // Reacquire a HUMAN GHOST even if RD-03D moved the person to another slot.
    for (int i = 0; i < 3; i++) {
      if (targets[i].active && targets[i].humanGhost) {
        tryCrossSlotHumanReacquisition(i, radarTargets, now);
      }
    }

    // -----------------------------------------------------
    // TRAINING MAP
    // -----------------------------------------------------
    if (
      !trainingFinished &&
      now - lastMapUpdate >= MAP_UPDATE_INTERVAL
    ) {

      lastMapUpdate = now;
      updateTraining(now);
    }

    printRadarData();
  }

  // =======================================================
  // TRAINING CHECK
  //
  // Навіть якщо не було нових кадрів,
  // 15 секунд все одно повинні завершитися.
  // =======================================================
  if (
    !trainingFinished &&
    now - bootTime >=
    TRAINING_TIME
  ) {

    updateTraining(now);
  }

  // =======================================================
  // V0.9 DIAGNOSTICS + DECISION LOG
  // =======================================================
  printCompactDiagnostics(now);

  // =======================================================
  // ОЧИЩЕННЯ ВТРАЧЕНИХ ТРЕКІВ
  // =======================================================
  anyTargetDetected = false;

  for (int i = 0; i < 3; i++) {

    if (targets[i].active) {
      RadarTarget &t = targets[i];
      bool outsideVirtualMap =
        (t.x < MAP_X_MIN) ||
        (t.x > (MAP_X_MIN + GRID_COLS * CELL_SIZE)) ||
        (t.y < MAP_Y_MIN) ||
        (t.y > (MAP_Y_MIN + GRID_ROWS * CELL_SIZE));

      if (outsideVirtualMap) {
        datasetLogf("[RS] T%d MAP", i + 1);
        resetTarget(i);
      }
      else if (t.state == TRACK_HUMAN) {
        // HUMAN LOCK survives temporary/weak observations. If the radar
        // completely loses the track, enter a short GHOST state instead of
        // immediately destroying the human identity.
        if (!t.humanGhost && now - t.lastSeen > HUMAN_HOLD_TIME) {
          enterHumanGhost(i, now);
        }
        if (t.humanGhost && now - t.humanGhostSince > t.humanGhostEffectiveTime) {
          datasetLogf("[RS] T%d HG_TO", i + 1);
          resetTarget(i);
        }
      }
      else if (now - t.lastSeen > TARGET_LOST_TIMEOUT) {
        datasetLogf("[RS] T%d LOST", i + 1);
        resetTarget(i);
      }
    }

    // v5: сигналізація (buzzer/BLE) реагує тільки на
    // targets[i].isHuman == true, тобто тільки після того, як
    // S/E Doppler-фільтр підтвердив реальний радіальний рух.
    // Раніше тут перевірявся лише !isClutter, тому гілки, які
    // радар ще (або взагалі ніколи) не встиг занести в карту
    // фону, продовжували викликати сигнал попри те, що
    // isHuman для них так і не ставав true.
    if (
      targets[i].active &&
      targets[i].state == TRACK_HUMAN &&
      targets[i].isHuman &&
      targets[i].aiConfirmedHuman &&
      trainingFinished
    ) {

      anyTargetDetected = true;
    }
  }

  // =======================================================
  // OLED + BUZZER
  // =======================================================
  if (
    now - lastDisplayUpdate >=
    DISPLAY_UPDATE_INTERVAL
  ) {

    lastDisplayUpdate =
      now;

    updateBuzzer(now);
    updateRadar();
  }

  // =======================================================
  // BLE
  // =======================================================
  // Authoritative scene-training status for Android.
  sendTrainingBleStatus(now);

  if (
    deviceConnected &&
    now - lastBleUpdate >=
    BLE_UPDATE_INTERVAL
  ) {

    lastBleUpdate =
      now;

    // v6 FIX: повернуто фіксований 3-слотовий формат
    // "T1:...|T2:...|T3:...:Vxx", як у попередніх прошивках
    // (speed_filter_v4, 48x32_dynamic_background_3targets),
    // під який написаний Android-додаток. Раніше цей цикл
    // повністю пропускав слоти цілей, що не пройшли isHuman-
    // класифікацію, через що кількість і позиція сегментів
    // "T1|T2|T3" у рядку "плавали" — додаток, розрахований на
    // завжди-3-слотовий рядок, губив другу/третю ціль. Прапорець
    // 0/1 у кожному слоті і далі рахується по новішій, точнішій
    // умові (isHuman && TRACK_HUMAN), сирі distance/angle
    // передаються завжди (додаток ігнорує їх при прапорці 0).
    char bleMsg[160];
    snprintf(
      bleMsg, sizeof(bleMsg),
      "T1:%.1f,%.1f,%d|T2:%.1f,%.1f,%d|T3:%.1f,%.1f,%d:V%.2f",
      targets[0].distance/1000.0, targets[0].angleBle,
      (targets[0].active && targets[0].state==TRACK_HUMAN &&
       targets[0].isHuman && targets[0].aiConfirmedHuman && trainingFinished) ? 1 : 0,
      targets[1].distance/1000.0, targets[1].angleBle,
      (targets[1].active && targets[1].state==TRACK_HUMAN &&
       targets[1].isHuman && targets[1].aiConfirmedHuman && trainingFinished) ? 1 : 0,
      targets[2].distance/1000.0, targets[2].angleBle,
      (targets[2].active && targets[2].state==TRACK_HUMAN &&
       targets[2].isHuman && targets[2].aiConfirmedHuman && trainingFinished) ? 1 : 0,
      cachedVoltage
    );

    Serial.printf("[BLE_TX] %s\n", bleMsg);

    pCharacteristic->setValue(
      bleMsg
    );

    pCharacteristic->notify();

    // BLE notification has now actually been issued. Mark only the
    // slots that were sent as visible HUMAN targets. The lock is armed
    // later, after both display and notification have happened.
    for (int bi = 0; bi < 3; bi++) {
      if (targets[bi].active &&
          targets[bi].state == TRACK_HUMAN &&
          targets[bi].isHuman &&
          targets[bi].aiConfirmedHuman &&
          trainingFinished) {
        targets[bi].humanNotified = true;
      }
    }

  }

  // -------------------------------------------------------
  // POST-OUTPUT HUMAN TRACK STABILIZATION
  // -------------------------------------------------------
  // HUMAN_LOCK is armed only after the target has already appeared on
  // the radar grid and, when BLE is connected, after the corresponding
  // notification has actually been sent. Once armed, it is a HARD HUMAN
  // lock: AI score, Doppler=0, stationary motion, scene-map evidence,
  // orbit/repeat scores, etc. cannot release HUMAN. Only genuine physical
  // track loss/reset may clear the lock.
  for (int li = 0; li < 3; li++) {
    RadarTarget &lt = targets[li];
    if (!lt.active || lt.state != TRACK_HUMAN || !lt.isHuman ||
        !lt.aiConfirmedHuman || !trainingFinished) {
      continue;
    }

    bool outputReady = lt.humanDisplayed &&
                       (!deviceConnected || lt.humanNotified);

    if (outputReady && !lt.humanLock) {
      lt.humanLock = true;
      datasetLogf("[TR] T%d LOCK", li + 1);
    }
  }
  // Non-blocking BLE dataset TX; never wait for BLE here.
  serviceBleDatasetQueue();

}
