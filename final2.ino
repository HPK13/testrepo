#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <MPU6050.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

//────────────────────────  CONSTANTS  ────────────────────────/
#define TCA_ADDR 0x70
#define SDA_PIN  21
#define SCL_PIN  22

#define MAX_WAYPOINTS      24

#define WAYPOINT_RADIUS    0.10f     // meters: consider waypoint reached if within this radius
#define MIN_WP_DISTANCE    0.15f     // meters: don't record waypoint if closer than this to previous
#define RETURN_RADIUS      0.10f     // meters: finish mapping if this close to start after lap 2
#define DESIRED_WP_COUNT   10        // for pruning/sampling
// #define MAX_SPEED          255
// #define MAPPING_SPEED      185
#define FAST_SPEED         240
#define AVOID_SPEED        185

const uint8_t LEFT_CH   = 2;
const uint8_t CENTRE_CH = 1;
const uint8_t RIGHT_CH  = 0;
const uint8_t COLOUR_CH = 3;

/*  obstacle-avoid gains  */
const int OBST_WARN_MM     = 180;
const int OBST_CRIT_MM     = 60;
const int SPEED_NORMAL     = 245;
const int SPEED_AVOID      = 185;
const int SPEED_REVERSE    = 140;


volatile float gz=0;  // Z-rate deg/s
volatile float headingZ_deg = 0.0f;  // integrated heading


/*  encoder pins  */
#define ENC_L_PIN 35
#define ENC_LB_PIN 34
#define ENC_R_PIN 39
#define ENC_RB_PIN 36

/*  motor pins  */
#define IN1 14
#define IN2 27
#define ENA 26
#define IN3 25
#define IN4 33
#define ENB 32

//────────────────────────  GLOBAL STATE  ─────────────────────/
// Odometry params
float x = 0, y = 0, theta = 0;
float wheel_radius = 0.032f/2.0f;  // meters, tune to your robot
float axle_length  = 0.0951f;  // meters, tune to your robot


constexpr int   TICKS_PER_REV  = 1450;
const float DIST_PER_TICK = (2.0f * PI * wheel_radius) / TICKS_PER_REV;


volatile uint16_t leftDist  = 8000;
volatile uint16_t centreDist= 8000;
volatile uint16_t rightDist = 8000;
volatile uint16_t colourR=0, colourG=0, colourB=0, colourC=0;
// volatile float    gz=0;                     /* Z-rate deg/s   */
volatile bool     newSensorData=false;

volatile long ticksL=0, ticksR=0;           /* encoders       */
volatile int  lastEncL=0, lastEncR=0;
long prevL = 0, prevR = 0;
/* ---------- GLOBALS ---------- */
volatile float gz_dps = 0.0f;     // gyro Z-rate in deg/s (written in SensorTask)


struct PID {
    float kp, ki, kd;
    float integral = 0, prevErr = 0;
    float outMin, outMax;

    PID(float p, float i, float d,
        float mn = -255, float mx = 255) :
        kp(p), ki(i), kd(d), outMin(mn), outMax(mx) {}

    /* dt in seconds, returns control effort */
    float update(float set, float meas, float dt) {
        float err = set - meas;
        /* wrap heading error to ±π  */
        while (err >  M_PI) err -= 2 * M_PI;
        while (err < -M_PI) err += 2 * M_PI;

        integral += err * dt;
        float deriv = (err - prevErr) / dt;
        prevErr = err;

        /* unconstrained output */
        float out =  kp*err + ki*integral + kd*deriv;

        /* anti-wind-up     */
        if (out > outMax) { out = outMax; integral -= err * dt; }
        if (out < outMin) { out = outMin; integral -= err * dt; }
        return out;
    }
};

/* ─ Heading controller gains (tune on track) ─ */
PID headingPID(4.0f, 0.02f, 0.15f, -100.0f, 100.0f);   // Kp, Ki, Kd
float desiredHeading = 0.0f;    // updated by the planner


/*  mapping / TSP variables (unchanged) …  */  // keep your earlier arrays
// Waypoint storage
struct Waypoint { float x, y; };
Waypoint rawPath[MAX_WAYPOINTS];
int rawWPCount = 0;
Waypoint wps[MAX_WAYPOINTS];
int wpCount = 0;

// Graph for TSP
float graph[MAX_WAYPOINTS][MAX_WAYPOINTS];
int tour[MAX_WAYPOINTS + 1];   // TSP tour
int tourLen = 0;

// Mapping state
bool mappingPhase = true;
bool onRedLine = false;
int lapCount = -1;
bool justCrossed = false;
float x_start = 0, y_start = 0;

static int currentIdx = 0;  

// At the top of your file, after the #includes
constexpr uint8_t  MAX_N = DESIRED_WP_COUNT;        // 10 or 11
static float       dp  [1 << MAX_N][MAX_N];
static uint8_t     prev[1 << MAX_N][MAX_N];         // uint8_t saves RAM
//────────────────────────  MUX HELPER  ───────────────────────/
inline void selectBus(uint8_t ch)
{
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

//──────────────────  SENSOR OBJECTS  ─────────────────────────/
Adafruit_VL53L0X tofL, tofC, tofR;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
MPU6050 mpu;
unsigned long lastTime = 0;       // For integration

//──────────────────  MOTOR DRIVERS  ─────────────────────────/
void drive(int l, int r)
{
  l = constrain(l,-255,255);  r = constrain(r,-255,255);
  /* left dir */
  digitalWrite(IN1, l<0);  digitalWrite(IN2, l>0);
  /* right dir*/
  digitalWrite(IN3, r<0);  digitalWrite(IN4, r>0);
  analogWrite(ENA, abs(l));
  analogWrite(ENB, abs(r));
}
void stopMotors(){ drive(0,0); }

//──────────────────  AVOID / REVERSE  ───────────────────────/
void avoidAndGo()
{
    // 1. CRITICAL: Extremely close → emergency reverse
    if (centreDist <= OBST_CRIT_MM ||
        leftDist   <= OBST_CRIT_MM ||
        rightDist  <= OBST_CRIT_MM)
    {
        Serial.println(" EMERGENCY CRITICAL OBSTACLE");

        // Always reverse first
        drive(-SPEED_REVERSE, -SPEED_REVERSE);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        stopMotors();

        return;
    }

    // 2. WARNING: All sensors detect something → block in front
    if (leftDist < OBST_WARN_MM && centreDist < OBST_WARN_MM && rightDist < OBST_WARN_MM)
    {
        Serial.println(" ALL DIRECTIONS BLOCKED - REVERSING & ESCAPING");

        // Step 1: reverse to create space
        drive(-SPEED_REVERSE, -SPEED_REVERSE);
        vTaskDelay(135 / portTICK_PERIOD_MS);
        stopMotors();

        // Step 2: Find direction with most free space
        if (leftDist >= rightDist && leftDist >= centreDist) {
            Serial.println("↩ TURNING LEFT (max clearance)");
            drive(-SPEED_REVERSE, SPEED_REVERSE); // pivot left
        }
        else if (rightDist >= leftDist && rightDist >= centreDist) {
            Serial.println("↪ TURNING RIGHT (max clearance)");
            drive(SPEED_REVERSE, -SPEED_REVERSE); // pivot right
        }
        else {
            Serial.println("⤴ ROTATING TO REPOSITION (center slightly clearer)");
            drive(SPEED_REVERSE, -SPEED_REVERSE); // fallback turn
        }

        vTaskDelay(150 / portTICK_PERIOD_MS); // allow pivot turn
        stopMotors();
        return;
    }

    // 3. Partial obstacles → steer away
    const int inner = SPEED_AVOID / 2;

    if (centreDist <= OBST_WARN_MM) {
        if (leftDist > rightDist) {
            Serial.println("↩ AVOIDING FRONT (veer LEFT)");
            drive(inner, SPEED_AVOID);
        } else {
            Serial.println("↪ AVOIDING FRONT (veer RIGHT)");
            drive(SPEED_AVOID, inner);
        }
    }
    else if (leftDist <= OBST_WARN_MM) {
        Serial.println("↪ AVOIDING LEFT (veer RIGHT)");
        drive(SPEED_AVOID, inner);
    }
    else if (rightDist <= OBST_WARN_MM) {
        Serial.println("↩ AVOIDING RIGHT (veer LEFT)");
        drive(inner, SPEED_AVOID);
    }
    else {
        // 4. All clear
        drive(SPEED_NORMAL, SPEED_NORMAL);
    }
}

//──────────────────  ENCODER ISRs  ──────────────────────────/
void IRAM_ATTR encL(){
  int a=digitalRead(ENC_L_PIN), b=digitalRead(ENC_LB_PIN);
  int code=(a<<1)|b, sum=(lastEncL<<2)|code;
  if(sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) ticksL--;
  if(sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) ticksL++;
  lastEncL=code;
}
void IRAM_ATTR encR(){
  int a=digitalRead(ENC_R_PIN), b=digitalRead(ENC_RB_PIN);
  int code=(a<<1)|b, sum=(lastEncR<<2)|code;
  if(sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) ticksR--;
  if(sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) ticksR++;
  lastEncR=code;
}

//──────────────────  SENSOR TASK  (core-0)  ─────────────────/
void SensorTask(void*)
{
    /* 20 ms budget (same as the working one-off test earlier) */
    selectBus(LEFT_CH);   tofL.setMeasurementTimingBudgetMicroSeconds(15000);
    selectBus(CENTRE_CH); tofC.setMeasurementTimingBudgetMicroSeconds(15000);
    selectBus(RIGHT_CH);  tofR.setMeasurementTimingBudgetMicroSeconds(15000);

    selectBus(LEFT_CH);   tofL.startRangeContinuous(0);
    selectBus(CENTRE_CH); tofC.startRangeContinuous(0);
    selectBus(RIGHT_CH);  tofR.startRangeContinuous(0);

    vTaskDelay(30 / portTICK_PERIOD_MS);              // first frame ready

    const uint8_t seq[3] = { LEFT_CH, CENTRE_CH, RIGHT_CH };
    uint8_t idx = 0;
    uint16_t lastL = 8191, lastC = 8191, lastR = 8191;
    unsigned long lastColour = 0;
    unsigned long tPrev = millis();


    while (true)
    {
        selectBus(seq[idx]);

        VL53L0X_RangingMeasurementData_t m;
        switch (seq[idx])
        {
            case LEFT_CH:
                tofL.rangingTest(&m, false);
                if (m.RangeStatus == 0) lastL = m.RangeMilliMeter;
                leftDist = lastL;
                break;

            case CENTRE_CH:
                tofC.rangingTest(&m, false);
                if (m.RangeStatus == 0) lastC = m.RangeMilliMeter;
                centreDist = lastC;
                break;

            case RIGHT_CH:
                tofR.rangingTest(&m, false);
                if (m.RangeStatus == 0) lastR = m.RangeMilliMeter;
                rightDist = lastR;
                break;
        }

#ifdef LOG_STATUS
        static uint32_t tDbg = 0;
        if (millis() - tDbg > 500) {
            Serial.printf("STS L:%d C:%d R:%d  D L:%u C:%u R:%u\n",
                          m.RangeStatus, m.RangeStatus, m.RangeStatus,
                          leftDist, centreDist, rightDist);
            tDbg = millis();
        }
#endif

        idx = (idx + 1) % 3;

        /* gyro */
        gz_dps = mpu.readNormalizeGyro().ZAxis;

        // Integrate heading
        unsigned long tNow = millis();
        float dt_sec = (tNow - tPrev) * 0.001f;
        tPrev = tNow;

        headingZ_deg += gz * dt_sec;
        if (headingZ_deg > 180.0f) headingZ_deg -= 360.0f;
        if (headingZ_deg < -180.0f) headingZ_deg += 360.0f;

       


        /* colour every 250 ms */
        if (millis() - lastColour > 250) {
            selectBus(COLOUR_CH);
            uint16_t r,g,b,c;
            tcs.getRawData(&r,&g,&b,&c);
            colourR=r; colourG=g; colourB=b; colourC=c;
            lastColour = millis();
        }

        newSensorData = true;
        vTaskDelay(10 / portTICK_PERIOD_MS);          // ≈100 Hz outer loop
    }
}



// ===================== ODOMETRY =====================
void updatePose()
{
    static uint32_t dbg = millis();
    /* -------- atomic tick copy -------- */
    long cl, cr;
    noInterrupts();           // stop ISRs for < 5 µs
    cl = ticksL;
    cr = ticksR;
    interrupts();

    /* -------- wheel-increment in metres -------- */
    float dl = (cl - prevL) * DIST_PER_TICK;
    float dr = (cr - prevR) * DIST_PER_TICK;
    prevL = cl;
    prevR = cr;

    float dc = 0.5f * (dl + dr);                 // chassis advance

    /* -------- heading update -------- */
    unsigned long now = millis();
    float dt = (now - lastTime) * 0.001f;        // s
    lastTime = now;

    /*  a)  gyro contribution */   // °/s
    float dtheta_gyro = gz_dps * DEG_TO_RAD * dt;

    /*  b)  encoder contribution  */
    float dtheta_enc  = (dr - dl) / axle_length;

    /*  c)  complementary fusion (98 % encoder, 2 % gyro) */
    float dtheta = dtheta_gyro;   // encoder contribution = 0
    theta += dtheta;

    if (millis() - dbg > 500) {
    Serial.println("updatePose() tick");   // <- should appear twice per second
    dbg = millis();
    }


    /*  normalise θ to [-π, π] */
    if (theta >  PI) theta -= 2.0f * PI;
    if (theta < -PI) theta += 2.0f * PI;

    /* -------- integrate chassis pose -------- */
    x += dc * cosf(theta);
    y += dc * sinf(theta);
}



// ===================== PATH RECORDING =====================
void recordWaypoint(float x, float y) {
  if (rawWPCount == 0 ||
      hypot(x - rawPath[rawWPCount-1].x, y - rawPath[rawWPCount-1].y) > MIN_WP_DISTANCE) {
    if (rawWPCount < MAX_WAYPOINTS) {
      rawPath[rawWPCount++] = {x, y};
    }
  }
}

// ===================== PRUNE/SAMPLE WAYPOINTS =====================
void pruneWaypoints() {
  // Simple sampling, more advanced pruning possible
  int step = max(1, rawWPCount / DESIRED_WP_COUNT);
  wpCount = 0;
  for (int i = 0; i < rawWPCount; i += step) {
    wps[wpCount++] = rawPath[i];
    if (wpCount >= DESIRED_WP_COUNT) break;
  }
}

// ===================== BUILD PATH GRAPH =====================
void buildGraph() {
  for (int i = 0; i < wpCount; ++i) {
    for (int j = 0; j < wpCount; ++j) {
      if (i == j) graph[i][j] = 0;
      else graph[i][j] = hypot(wps[i].x - wps[j].x, wps[i].y - wps[j].y);
    }
  }
}

// ===================== TSP SOLVER (DP, <16 NODES) =====================
void tsp(int N,
         const float cost[][MAX_WAYPOINTS],   // unchanged
         int*  path,
         float& best)
{
    const int ALL = 1 << N;

    /* ---- init ---- */
    for (int m = 0; m < ALL; ++m)
        for (int j = 0; j < N; ++j)
            dp[m][j] = 1e9f;

    dp[1][0] = 0.0f;

    /* ---- DP ---- */
    for (int mask = 1; mask < ALL; ++mask) {
        for (int u = 0; u < N; ++u) {
            if (!(mask & (1 << u))) continue;
            float base = dp[mask][u];
            if (base >= 1e9f) continue;           // unreachable
            for (int v = 0; v < N; ++v) {
                if (mask & (1 << v)) continue;
                int next = mask | (1 << v);
                float cand = base + cost[u][v];
                if (cand < dp[next][v]) {
                    dp[next][v]  = cand;
                    prev[next][v] = u;
                }
            }
        }
    }

    /* ---- find best tour ending city ---- */
    best = 1e9f;
    int last = 0;
    for (int i = 0; i < N; ++i) {
        float cand = dp[ALL - 1][i] + cost[i][0];
        if (cand < best) { best = cand; last = i; }
    }

    /* ---- reconstruct path ---- */
    int mask = ALL - 1;
    int idx  = N - 1;
    path[N]  = 0;                              // return to depot
    while (mask > 1) {
        path[idx--] = last;
        int tmp = last;
        last = prev[mask][last];
        mask ^= (1 << tmp);
    }
    path[0] = 0;
}


// ===================== PROPORTIONAL STEERING =====================
/**void steerToWaypoint(float tx, float ty, float th) {
  float desired = atan2(ty - y, tx - x);
  float err = desired - th;
  while (err > M_PI) err -= 2 * M_PI;
  while (err < -M_PI) err += 2 * M_PI;
  float turn = 4.0 * err;
  int vl = FAST_SPEED - int(turn * axle_length / 2 * 100);
  int vr = FAST_SPEED + int(turn * axle_length / 2 * 100);
  drive(vl, vr);
}**/

bool isRedLineDetected(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    if (c == 0) return false;                 // sensor fail-safe
    float invC = 1.0f / c;                    // ~3 µs faster than 3 divides
    float rC = r * invC;
    float gC = g * invC;
    float bC = b * invC;

    return (rC > 0.60f) &&                    // red dominant
           (gC < 0.35f) && (bC < 0.35f);      // green & blue suppressed
}
void pidSteerToWaypoint(float tx, float ty, float dt)
{
    /* 1.  Compute desired heading from current (x,y) to target */
    desiredHeading = atan2f(ty - y, tx - x);

    /* 2.  Run heading PID (gyro-only θ comes from updatePose) */
    float turnEffort = headingPID.update(desiredHeading, theta, dt); // −100…+100

    /* 3.  Convert effort to wheel PWM */
    int base = FAST_SPEED;                // ≈220 from your #define
    int vl   = constrain(base - (int)turnEffort, -255, 255);
    int vr   = constrain(base + (int)turnEffort, -255, 255);
    drive(vl, vr);
}




//──────────────────  SETUP  ───────────────────────────────/
void setup()
{
  Serial.begin(115200);

  /* pins */
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT);
  pinMode(ENC_L_PIN,INPUT_PULLUP); pinMode(ENC_LB_PIN,INPUT_PULLUP);
  pinMode(ENC_R_PIN,INPUT_PULLUP); pinMode(ENC_RB_PIN,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN),encL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LB_PIN),encL,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN),encR,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RB_PIN),encR,CHANGE);

  /* I²C */
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(400000);

  /* VL53L0X init (one-time) */
  selectBus(LEFT_CH);   if(!tofL.begin())   Serial.println("Left ToF FAIL");
  selectBus(CENTRE_CH); if(!tofC.begin())   Serial.println("Centre ToF FAIL");
  selectBus(RIGHT_CH);  if(!tofR.begin())   Serial.println("Right ToF FAIL");

  /* Colour sensor */
  selectBus(COLOUR_CH);
  if (!tcs.begin()) { Serial.println("TCS34725 FAIL"); while (1); }

  /* IMU */
  mpu.begin();  mpu.calibrateGyro(); mpu.setThreshold(3);

  /* start FreeRTOS task on core-0 */
  xTaskCreatePinnedToCore(SensorTask,"SensorTask",4096,NULL,1,NULL,1);

    // --- initialise pose / mapping state ---
  x = y = theta = 0;
  prevL = prevR = 0;
  recordWaypoint(x, y);           // starting point
  lapCount     = 0;
  mappingPhase = true;
  lastTime     = millis();

  /* rest of your mapping / TSP initialisation here … */
}

/* ---------- spin-lock used only for the newSensorData flag ---------- */
static portMUX_TYPE loopMux = portMUX_INITIALIZER_UNLOCKED;

void loop()
{
    /* 0. Wait until sensor task says a frame is ready */
    if (!newSensorData) { vTaskDelay(2 / portTICK_PERIOD_MS); return; }

    /* 1. Take and clear the flag atomically */
    portENTER_CRITICAL(&loopMux);
    newSensorData = false;
    portEXIT_CRITICAL(&loopMux);

    /* 2. Update pose immediately */
    updatePose();

    /* 3. Safety layer: obstacle avoidance first */
    if (centreDist <= OBST_WARN_MM ||
        leftDist   <= OBST_WARN_MM ||
        rightDist  <= OBST_WARN_MM)
    {
        avoidAndGo();                       // may reverse or veer
        /* nothing else should overwrite the wheels this frame */
    }
    else
    {
        /* 4. Lap / red-line detection */
        static bool onRedLine = false;
        bool redNow = isRedLineDetected(colourR, colourG, colourB, colourC);

        if (redNow && !onRedLine) {
            ++lapCount;
            onRedLine = true;
            Serial.printf("Lap %u completed!\n", lapCount);
            stopMotors();
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }
        if (!redNow) onRedLine = false;

        if (lapCount >= 5) {
            stopMotors();
            Serial.println("All laps complete - robot halted.");
            while (true) vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        /* 5. Mapping (laps 1-2) or waypoint follower (laps 3-5) */
        if (mappingPhase)
        {
            recordWaypoint(x, y);

            if (lapCount >= 2 &&
                hypotf(x - rawPath[0].x, y - rawPath[0].y) < RETURN_RADIUS)
            {
                mappingPhase = false;
                pruneWaypoints();
                buildGraph();
                float best = 1e9f;
                tsp(wpCount, graph, tour, best);
                tourLen = wpCount;
                currentIdx = 0;

                Serial.println("Mapping done → optimised laps ");
                stopMotors();
                vTaskDelay(500 / portTICK_PERIOD_MS);
            }
            else {
                drive(SPEED_NORMAL, SPEED_NORMAL);
            }
        }
        else    /* optimised-lap phase */
        {
            int   nextIdx = (currentIdx + 1) % tourLen;
            float tx = wps[tour[nextIdx]].x;
            float ty = wps[tour[nextIdx]].y;

            if (hypotf(x - tx, y - ty) < WAYPOINT_RADIUS) {
                currentIdx = nextIdx;
                stopMotors();
                vTaskDelay(40 / portTICK_PERIOD_MS);
            } else {
                /* compute dt for PID (same dt you used in updatePose) */
                float dt_sec = 0.002f;                     // 2 ms – matches vTaskDelay(2)
                pidSteerToWaypoint(tx, ty, dt_sec);

            }
        }
    }

    /* 6. Encoder / pose debug print every 100 ms */
    static uint32_t tPrint = 0;
    if (millis() - tPrint > 100) {
        long l, r;
        noInterrupts();          // snapshot encoders
        l = ticksL;  r = ticksR;
        interrupts();

        Serial.printf("L:%5u C:%5u R:%5u  |  "
                    "Ticks L:%ld R:%ld  |  "
                    "Pose x:%.2f y:%.2f θ:%.1f°\n",
                    leftDist, centreDist, rightDist,   // ←--- NEW
                    l, r, x, y, theta * 180.0f / PI);
        tPrint = millis();
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);     // cooperative yield
}