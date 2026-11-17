#include <Arduino.h>
#include <stdio.h>

// true runs the unit tests with the USB cable only.
// false runs the sensor and buzzer code on the MKR Zero.
constexpr bool kRunSimulationTests = true;

// Proposed D01 and D03 settings from Sections 3 and 4 of the report.
constexpr uint32_t kSamplePeriodMs = 10;
constexpr uint32_t kPnn50WindowMs = 60000;
constexpr uint32_t kPnn50UpdateMs = 5000;

// Provisional D03/D06 software and sensor settings. Confirm during integration.
constexpr uint16_t kMinimumPnn50Pairs = 20;
constexpr uint16_t kMinIntervalMs = 250;
constexpr uint16_t kMaxIntervalMs = 2000;
constexpr uint16_t kRiseCounts = 85;
constexpr uint16_t kFallCounts = 40;
constexpr uint32_t kMaxPulseWidthMs = 300;
constexpr uint16_t kAdcMax = 1023;
constexpr uint8_t kRailMarginCounts = 2;
constexpr uint16_t kDifferenceThresholdMs = 50;

struct Reading {
bool valid;
float value;
};

struct Sample {
uint32_t timeMs;
uint16_t counts;
uint32_t sequence;
};

struct Interval {
uint32_t endMs;
uint16_t durationMs;
};

enum class LimitState { Unknown, InRange, Uncertain, OutOfRange };
class PulseMonitor;

struct Limits {
bool enabled;
float low;
float high;
float uncertainty;
};

// D07: After the group agrees on a metric's limits, replace its zeros with
// low, high and uncertainty, and set enabled to true. Units are BPM and %.
constexpr Limits kLiveBpmLimits = {false, 0.0f, 0.0f, 0.0f};
constexpr Limits kLivePnn50Limits = {false, 0.0f, 0.0f, 0.0f};

// One-shot readiness check for a periodic task. Keep the original time grid
// and skip expired releases if the loop resumes late.
bool periodicDue(uint32_t nowMs, uint32_t periodMs, uint32_t &lastReleaseMs) {
const uint32_t elapsed = nowMs - lastReleaseMs;
if (elapsed < periodMs) {
return false;
}
lastReleaseMs += (elapsed / periodMs) * periodMs;
return true;
}

Reading calculateBpm(uint16_t intervalMs) {
if (intervalMs < kMinIntervalMs || intervalMs > kMaxIntervalMs) {
return {false, 0.0f};
}
return {true, 60000.0f / intervalMs};
}

bool countPnn50Comparison(const Interval &earlier, const Interval &later,
uint16_t &total, uint16_t &over50) {
// Adjacent intervals must share a pulse. Missing data breaks adjacency.
if (earlier.endMs != later.endMs - later.durationMs) {
return false;
}
const int difference = int(later.durationMs) - int(earlier.durationMs);
++total;
if (difference > kDifferenceThresholdMs ||
difference < -int(kDifferenceThresholdMs)) {
++over50;
}
return true;
}

class IntervalWindow {
public:
void push(const Interval &value) {
entries_[next_] = value;
next_ = (next_ + 1) % kCapacity;
if (length_ < kCapacity) {
++length_;
}
}

Reading pnn50(uint32_t nowMs, uint16_t minimumPairs) const {
uint16_t total = 0;
uint16_t over50 = 0;
bool havePrevious = false;
Interval previous = {};
const size_t oldest = (next_ + kCapacity - length_) % kCapacity;
for (size_t i = 0; i < length_; ++i) {
const Interval current = entries_[(oldest + i) % kCapacity];
// Both beats defining each interval must be inside the window.
if (nowMs - (current.endMs - current.durationMs) > kPnn50WindowMs) {
continue;
}
if (havePrevious) {
countPnn50Comparison(previous, current, total, over50);
}
previous = current;
havePrevious = true;
}
if (total == 0 || total < minimumPairs) {
return {false, 0.0f};
}
return {true, 100.0f * over50 / total};
}

private:
static constexpr size_t kCapacity = kPnn50WindowMs / kMinIntervalMs + 2;
Interval entries_[kCapacity] = {};
size_t next_ = 0;
size_t length_ = 0;
};

class PulseIntervals {
public:
bool pushPeak(uint32_t peakMs, Interval &interval) {
if (!havePeak_) {
lastPeakMs_ = peakMs;
havePeak_ = true;
return false;
}

const uint32_t gap = peakMs - lastPeakMs_;
lastPeakMs_ = peakMs;
if (gap < kMinIntervalMs || gap > kMaxIntervalMs) {
  bpm = {false, 0.0f};
  return false;
}
interval = {peakMs, static_cast<uint16_t>(gap)};
bpm = calculateBpm(interval.durationMs);
return true;

}

bool timedOut(uint32_t nowMs) const {
return havePeak_ && nowMs - lastPeakMs_ > kMaxIntervalMs;
}

void breakSequence() {
havePeak_ = false;
bpm = {false, 0.0f};
}

Reading bpm = {false, 0.0f};

private:
bool havePeak_ = false;
uint32_t lastPeakMs_ = 0;
};

class ThreePointAverage {
public:
bool push(const Sample &sample, Sample &filtered) {
const bool ready = seen_ >= 2;
if (ready) {
// Associate the average with its centre sample to remove the delay.
filtered = previous_;
filtered.counts = (older_ + previous_.counts + sample.counts) / 3;
} else {
++seen_;
}
older_ = previous_.counts;
previous_ = sample;
return ready;
}

private:
uint8_t seen_ = 0;
uint16_t older_ = 0;
Sample previous_ = {};
};

enum class PeakEvent { None, Peak, Invalid };

class PeakDetector {
public:
PeakEvent push(const Sample &sample, uint32_t &peakMs) {
const float value = sample.counts;
if (!haveBaseline_) {
baseline_ = value;
haveBaseline_ = true;
return PeakEvent::None;
}

const float height = value - baseline_;
if (!tracking_) {
  if (height >= kRiseCounts) {
    tracking_ = true;
    startedMs_ = sample.timeMs;
    maximum_ = sample.counts;
    maximumMs_ = sample.timeMs;
  } else {
    baseline_ += (value - baseline_) / 32.0f;
  }
  return PeakEvent::None;
}

if (sample.timeMs - startedMs_ > kMaxPulseWidthMs) {
  *this = PeakDetector{};
  return PeakEvent::Invalid;
}
if (sample.counts > maximum_) {
  maximum_ = sample.counts;
  maximumMs_ = sample.timeMs;
}
if (height <= kFallCounts) {
  tracking_ = false;
  peakMs = maximumMs_;
  return PeakEvent::Peak;
}
return PeakEvent::None;

}

private:
bool haveBaseline_ = false;
bool tracking_ = false;
float baseline_ = 0.0f;
uint16_t maximum_ = 0;
uint32_t startedMs_ = 0;
uint32_t maximumMs_ = 0;
};

LimitState classifyReading(const Reading &reading, const Limits &limits) {
if (!reading.valid) {
return LimitState::Unknown;
}
const float lower = reading.value - limits.uncertainty;
const float upper = reading.value + limits.uncertainty;
if (upper < limits.low || lower > limits.high) {
return LimitState::OutOfRange;
}
if (lower >= limits.low && upper <= limits.high) {
return LimitState::InRange;
}
return LimitState::Uncertain;
}

bool updateAlarm(bool wasOn, const Reading &bpm, const Reading &pnn50,
const Limits &bpmLimits, const Limits &pnn50Limits) {
const LimitState bpmState = bpmLimits.enabled
? classifyReading(bpm, bpmLimits) : LimitState::InRange;
const LimitState pnn50State = pnn50Limits.enabled
? classifyReading(pnn50, pnn50Limits) : LimitState::InRange;

if (bpmState == LimitState::OutOfRange ||
pnn50State == LimitState::OutOfRange) {
return true;
}
if (bpmState == LimitState::InRange && pnn50State == LimitState::InRange) {
return false;
}
// Proposed reset rule: hold the previous state for unknown/uncertain data.
return wasOn;
}

class PulseMonitor {
public:
PulseMonitor(uint32_t startedMs, bool fallingPulse,
const Limits &bpmLimits, const Limits &pnn50Limits)
: startedMs_(startedMs), pnn50ReleaseMs_(startedMs),
fallingPulse_(fallingPulse), bpmLimits_(bpmLimits),
pnn50Limits_(pnn50Limits) {}

void ingest(Sample sample) {
if (haveSample_ && (sample.sequence - lastSequence_ != 1 ||
sample.timeMs - lastSampleMs_ > 2 * kSamplePeriodMs)) {
invalidateSignal();
}
haveSample_ = true;
lastSequence_ = sample.sequence;
lastSampleMs_ = sample.timeMs;

if (sample.counts <= kRailMarginCounts ||
    sample.counts >= kAdcMax - kRailMarginCounts) {
  invalidateSignal();
  return;
}
if (fallingPulse_) {
  sample.counts = kAdcMax - sample.counts;
}

Sample filtered = {};
if (!filter_.push(sample, filtered)) {
  return;
}
uint32_t peakMs = 0;
const PeakEvent event = detector_.push(filtered, peakMs);
if (event == PeakEvent::Invalid) {
  invalidateSignal();
} else if (event == PeakEvent::Peak) {
  const bool previouslyValid = intervals_.bpm.valid;
  Interval interval = {};
  if (intervals_.pushPeak(peakMs, interval)) {
    window_.push(interval);
    evaluateAlarm();
  } else if (previouslyValid) {
    pnn50 = {false, 0.0f};
    evaluateAlarm();
  }
}

}

void serviceTiming(uint32_t nowMs) {
if (intervals_.timedOut(nowMs)) {
invalidateSignal();
}

windowReady_ = windowReady_ || nowMs - startedMs_ >= kPnn50WindowMs;
if (periodicDue(nowMs, kPnn50UpdateMs, pnn50ReleaseMs_)) {
  pnn50 = windowReady_ && intervals_.bpm.valid
      ? window_.pnn50(nowMs, kMinimumPnn50Pairs)
      : Reading{false, 0.0f};
  evaluateAlarm();
}

}

Reading bpm() const { return intervals_.bpm; }

Reading pnn50 = {false, 0.0f};
bool alarmOn = false;

private:
void invalidateSignal() {
filter_ = ThreePointAverage{};
detector_ = PeakDetector{};
intervals_.breakSequence();
pnn50 = {false, 0.0f};
evaluateAlarm();
}

void evaluateAlarm() {
alarmOn = updateAlarm(alarmOn, bpm(), pnn50, bpmLimits_, pnn50Limits_);
}

const uint32_t startedMs_;
uint32_t pnn50ReleaseMs_;
const bool fallingPulse_;
const Limits bpmLimits_;
const Limits pnn50Limits_;
bool windowReady_ = false;
bool haveSample_ = false;
uint32_t lastSequence_ = 0;
uint32_t lastSampleMs_ = 0;
ThreePointAverage filter_;
PeakDetector detector_;
PulseIntervals intervals_;
IntervalWindow window_;
};

constexpr uint32_t kSimulationPeaks[] = {200, 1000, 1850, 2800};
constexpr uint16_t kExpectedIntervals[] = {800, 850, 950};
constexpr float kExpectedBpm[] = {75.00f, 70.59f, 63.16f};
constexpr Limits kTestBpmLimits = {true, 60.0f, 80.0f, 2.0f};
constexpr Limits kTestPnn50Limits = {true, 20.0f, 70.0f, 5.0f};
constexpr Limits kDisabledLimits = {false, 0.0f, 0.0f, 0.0f};

uint16_t pulseShape(uint32_t timeMs, uint32_t peakMs) {
const uint32_t distance = timeMs >= peakMs ? timeMs - peakMs : peakMs - timeMs;
constexpr uint16_t shape[] = {700, 580, 460, 400};
return distance <= 30 ? shape[distance / 10] : 400;
}

Sample simulatedSample(uint32_t timeMs) {
uint16_t counts = 400;
for (const uint32_t peakMs : kSimulationPeaks) {
const uint16_t candidate = pulseShape(timeMs, peakMs);
if (candidate > counts) {
counts = candidate;
}
}
return {timeMs, counts, timeMs / kSamplePeriodMs};
}

bool nearValue(const Reading &reading, float expected) {
return reading.valid && reading.value >= expected - 0.01f &&
reading.value <= expected + 0.01f;
}

bool testSamples() {
const Sample before = simulatedSample(190);
const Sample peak = simulatedSample(200);
return peak.timeMs - before.timeMs == 10 &&
peak.sequence - before.sequence == 1 &&
before.counts == 580 && peak.counts == 700 &&
simulatedSample(230).counts == 400;
}

bool testFilter() {
ThreePointAverage filter;
Sample output = {};
const bool readyFirst = filter.push({0, 400, 0}, output);
const bool readySecond = filter.push({10, 700, 1}, output);
const bool readyThird = filter.push({20, 400, 2}, output);
return !readyFirst && !readySecond && readyThird &&
output.timeMs == 10 && output.counts == 500;
}

bool testPeakDetector() {
PeakDetector detector;
uint8_t count = 0;
for (uint32_t now = 0; now <= 2850; now += kSamplePeriodMs) {
uint32_t peakMs = 0;
if (detector.push(simulatedSample(now), peakMs) == PeakEvent::Peak) {
if (count >= 4 || peakMs != kSimulationPeaks[count]) {
return false;
}
++count;
}
}
return count == 4;
}

bool testPulseIntervals() {
PulseIntervals intervals;
Interval output = {};
if (intervals.pushPeak(kSimulationPeaks[0], output)) {
return false;
}
for (size_t i = 0; i < 3; ++i) {
if (!intervals.pushPeak(kSimulationPeaks[i + 1], output) ||
output.durationMs != kExpectedIntervals[i]) {
return false;
}
}
intervals.breakSequence();
return !intervals.pushPeak(6000, output) && !intervals.bpm.valid;
}

bool testBpm() {
for (size_t i = 0; i < 3; ++i) {
if (!nearValue(calculateBpm(kExpectedIntervals[i]), kExpectedBpm[i])) {
return false;
}
}
return !calculateBpm(0).valid;
}

bool testPnn50Boundary() {
IntervalWindow window;
window.push({1000, 800});
const bool insufficient = !window.pnn50(1000, 1).valid;
window.push({1850, 850});
const bool exactly50Excluded = nearValue(window.pnn50(1850, 1), 0.0f);
window.push({2800, 950});
const bool mixed = nearValue(window.pnn50(2800, 1), 50.0f);
return insufficient && exactly50Excluded && mixed &&
!window.pnn50(2800, 3).valid;
}

bool testPnn50Window() {
IntervalWindow window;
window.push({4800, 800});
window.push({5600, 800});
window.push({6400, 800});
window.push({10700, 700});
window.push({11600, 900});
// At 65 s the first two intervals start before the window. The 6400
// interval and the 10700 interval have no shared beat and form no pair.
return nearValue(window.pnn50(65000, 1), 100.0f) &&
!window.pnn50(65000, 2).valid;
}

bool testAlarm() {
const Reading invalid = {false, 0.0f};
const Reading inRange = {true, 75.0f};
const Reading uncertain = {true, 81.0f};
const Reading confirmed = {true, 83.0f};
return !updateAlarm(false, inRange, invalid, kTestBpmLimits, kDisabledLimits) &&
!updateAlarm(false, uncertain, invalid, kTestBpmLimits, kDisabledLimits) &&
!updateAlarm(false, invalid, invalid, kTestBpmLimits, kDisabledLimits) &&
updateAlarm(false, confirmed, invalid, kTestBpmLimits, kDisabledLimits) &&
updateAlarm(true, invalid, invalid, kTestBpmLimits, kDisabledLimits) &&
!updateAlarm(true, inRange, invalid, kTestBpmLimits, kDisabledLimits) &&
updateAlarm(false, inRange, {true, 80.0f}, kTestBpmLimits, kTestPnn50Limits);
}

bool testSchedule() {
uint32_t last = 0;
const bool onTime = !periodicDue(9, 10, last) && periodicDue(10, 10, last);
const bool missedReleasesSkipped = periodicDue(39, 10, last) && last == 30 &&
!periodicDue(39, 10, last);
uint32_t beforeWrap = UINT32_MAX - 4;
return onTime && missedReleasesSkipped && periodicDue(5, 10, beforeWrap);
}

bool testRollingUpdate() {
PulseMonitor monitor(0, false, kDisabledLimits, kDisabledLimits);
uint32_t peak = 200;
uint16_t nextGap = 800;
bool beforeWindow = false;
bool firstUpdate = false;
for (uint32_t now = 0; now <= 65000; now += kSamplePeriodMs) {
if (now > peak + 30) {
peak += nextGap;
nextGap = nextGap == 800 ? 900 : 800;
}
const uint16_t counts = pulseShape(now, peak);
monitor.ingest({now, counts, now / kSamplePeriodMs});
monitor.serviceTiming(now);
if (now == 59990) {
beforeWindow = !monitor.pnn50.valid;
}
if (now == 60000) {
firstUpdate = nearValue(monitor.pnn50, 100.0f);
}
}
return beforeWindow && firstUpdate && nearValue(monitor.pnn50, 100.0f);
}

bool testPipelineAndSignalLoss() {
PulseMonitor monitor(0, false, kTestBpmLimits, kDisabledLimits);
for (uint32_t now = 0; now <= 5000; now += kSamplePeriodMs) {
monitor.ingest(simulatedSample(now));
monitor.serviceTiming(now);
if (now == 1040 && !nearValue(monitor.bpm(), 75.0f)) {
return false;
}
if (now == 1890 && !nearValue(monitor.bpm(), 70.59f)) {
return false;
}
if (now == 2840 && !nearValue(monitor.bpm(), 63.16f)) {
return false;
}
}
return !monitor.bpm().valid && !monitor.pnn50.valid && !monitor.alarmOn;
}

bool testClippingAndMissingSample() {
PulseMonitor monitor(0, false, kDisabledLimits, kDisabledLimits);
for (uint32_t now = 0; now <= 1040; now += kSamplePeriodMs) {
monitor.ingest(simulatedSample(now));
}
if (!nearValue(monitor.bpm(), 75.0f)) {
return false;
}
monitor.ingest(simulatedSample(1060));
if (monitor.bpm().valid) {
return false;
}
for (uint32_t now = 1070; now <= 2840; now += kSamplePeriodMs) {
monitor.ingest(simulatedSample(now));
}
const bool recovered = nearValue(monitor.bpm(), 63.16f);
monitor.ingest({2850, 1023, 285});
return recovered && !monitor.bpm().valid;
}

void printResult(const char *name, bool passed, uint8_t &failures) {
Serial.print(passed ? "PASS  " : "FAIL  ");
Serial.println(name);
if (!passed) {
++failures;
}
}

void runSimulationTests() {
Serial.println("ECNG 3006 simulated unit tests");
uint8_t failures = 0;
printResult("Simulated sampling (10 ms)", testSamples(), failures);
printResult("Three-point moving average", testFilter(), failures);
printResult("Peak timestamps", testPeakDetector(), failures);
printResult("Pulse intervals", testPulseIntervals(), failures);
printResult("BPM calculation", testBpm(), failures);
printResult("pNN50 threshold and insufficient data", testPnn50Boundary(), failures);
printResult("pNN50 window and gap exclusion", testPnn50Window(), failures);
printResult("Alarm, uncertainty and reset", testAlarm(), failures);
printResult("Periodic scheduling", testSchedule(), failures);
printResult("60 s pNN50 window with 5 s refresh", testRollingUpdate(), failures);
printResult("Filtered samples to BPM and signal timeout", testPipelineAndSignalLoss(), failures);
printResult("Clipping and missing samples", testClippingAndMissingSample(), failures);

Serial.println("Changing-pulse results (simulated)");
IntervalWindow window;
for (size_t i = 1; i < 4; ++i) {
const uint16_t interval = kSimulationPeaks[i] - kSimulationPeaks[i - 1];
window.push({kSimulationPeaks[i], interval});
Serial.print("Interval ms = ");
Serial.print(interval);
Serial.print(", BPM = ");
Serial.println(calculateBpm(interval).value, 2);
}
Serial.print("Short-sequence pNN50 = ");
Serial.print(window.pnn50(kSimulationPeaks[3], 1).value, 2);
Serial.println("%");
Serial.print("Failures = ");
Serial.println(failures);
}

#if defined(ARDUINO_SAMD_MKRZERO)

constexpr uint8_t kSensorPin = A0;
constexpr uint8_t kBuzzerPin = 6;
constexpr bool kFallingSensorPulse = true;
constexpr bool kPassiveBuzzer = true;
constexpr uint16_t kBuzzerHz = 2500;  // Test setting for the proposed buzzer.
constexpr uint32_t kReportPeriodMs = 1000;

constexpr uint8_t kQueueSize = 64;
volatile Sample sampleQueue[kQueueSize] = {};
volatile uint8_t queueWrite = 0;
volatile uint8_t queueRead = 0;
volatile uint32_t sampleTimeMs = 0;
volatile uint32_t sampleSequence = 0;
volatile uint32_t adcSequence = 0;
volatile uint32_t droppedSamples = 0;
volatile bool adcBusy = false;

extern "C" void TC3_Handler() {
TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
++sampleSequence;
if (adcBusy) {
++droppedSamples;
return;
}
adcBusy = true;
sampleTimeMs = millis();
adcSequence = sampleSequence;
ADC->SWTRIG.reg = ADC_SWTRIG_START;
}

extern "C" void ADC_Handler() {
const uint16_t result = ADC->RESULT.reg;
ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

const uint8_t next = (queueWrite + 1) % kQueueSize;
if (next == queueRead) {
++droppedSamples;
} else {
sampleQueue[queueWrite].timeMs = sampleTimeMs;
sampleQueue[queueWrite].counts = result;
sampleQueue[queueWrite].sequence = adcSequence;
queueWrite = next;
}
adcBusy = false;
}

void beginAcquisition() {
analogReadResolution(10);
analogReference(AR_DEFAULT);
// The Arduino core selects A0, configures the ADC and performs dummy reads.
(void)analogRead(kSensorPin);
ADC->CTRLA.bit.ENABLE = 1;
while (ADC->STATUS.bit.SYNCBUSY) {}
ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY | ADC_INTFLAG_OVERRUN;
ADC->INTENSET.reg = ADC_INTENSET_RESRDY;
NVIC_ClearPendingIRQ(ADC_IRQn);
NVIC_SetPriority(ADC_IRQn, 1);
NVIC_EnableIRQ(ADC_IRQn);

// TC3 uses a 48 MHz generic clock. TC5 remains available for tone().
PM->APBCMASK.reg |= PM_APBCMASK_TC3;
GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 |
GCLK_CLKCTRL_ID(GCM_TCC2_TC3);
while (GCLK->STATUS.bit.SYNCBUSY) {}
TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
while (TC3->COUNT16.STATUS.bit.SYNCBUSY || TC3->COUNT16.CTRLA.bit.SWRST) {}
TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ |
TC_CTRLA_PRESCALER_DIV64;
while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {}
TC3->COUNT16.CC[0].reg = F_CPU / 64 / 1000 * kSamplePeriodMs - 1;
while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {}
TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
NVIC_ClearPendingIRQ(TC3_IRQn);
NVIC_SetPriority(TC3_IRQn, 1);
NVIC_EnableIRQ(TC3_IRQn);
TC3->COUNT16.CTRLA.bit.ENABLE = 1;
while (TC3->COUNT16.STATUS.bit.SYNCBUSY) {}
}

bool popSample(Sample &sample) {
const uint32_t interruptState = __get_PRIMASK();
__disable_irq();
const bool pending = queueRead != queueWrite;
if (pending) {
sample = {sampleQueue[queueRead].timeMs, sampleQueue[queueRead].counts,
sampleQueue[queueRead].sequence};
queueRead = (queueRead + 1) % kQueueSize;
}
__set_PRIMASK(interruptState);
return pending;
}

void driveBuzzer(bool on) {
if (kPassiveBuzzer) {
if (on) {
tone(kBuzzerPin, kBuzzerHz);
} else {
noTone(kBuzzerPin);
}
} else {
digitalWrite(kBuzzerPin, on ? HIGH : LOW);
}
}

void formatReading(char *buffer, size_t capacity, const Reading &reading) {
if (!reading.valid) {
snprintf(buffer, capacity, "NA");
return;
}
const unsigned int rounded = static_cast<unsigned int>(reading.value * 100 + 0.5f);
snprintf(buffer, capacity, "%u.%02u", rounded / 100, rounded % 100);
}

void reportLive(const PulseMonitor &monitor) {
char bpm[12];
char pnn50[12];
char line[64];
formatReading(bpm, sizeof bpm, monitor.bpm());
formatReading(pnn50, sizeof pnn50, monitor.pnn50);
const int length = snprintf(line, sizeof line,
"BPM=%s pNN50=%s alarm=%d dropped=%lu\n", bpm, pnn50, monitor.alarmOn,
static_cast<unsigned long>(droppedSamples));
if (Serial && length > 0 && length < int(sizeof line) &&
Serial.availableForWrite() >= length) {
Serial.write(reinterpret_cast<const uint8_t *>(line), length);
}
}

void runLiveLoop() {
static PulseMonitor monitor(millis(), kFallingSensorPulse,
kLiveBpmLimits, kLivePnn50Limits);
static uint32_t lastReportMs = millis();
static bool buzzerOn = false;
Sample sample = {};
const bool processedSample = popSample(sample);
const uint32_t now = millis();
if (processedSample) {
monitor.ingest(sample);
} else {
monitor.serviceTiming(now);
}
if (monitor.alarmOn != buzzerOn) {
buzzerOn = monitor.alarmOn;
driveBuzzer(buzzerOn);
}
if (!processedSample && periodicDue(now, kReportPeriodMs, lastReportMs)) {
reportLive(monitor);
}
}
#endif

void setup() {
Serial.begin(115200);
#if defined(ARDUINO_SAMD_MKRZERO)
if (!kRunSimulationTests) {
pinMode(kBuzzerPin, OUTPUT);
digitalWrite(kBuzzerPin, LOW);
beginAcquisition();
}
#endif
}

void loop() {
if (kRunSimulationTests) {
static bool printed = false;
if (Serial && !printed) {
printed = true;
runSimulationTests();
}
return;
}
#if defined(ARDUINO_SAMD_MKRZERO)
runLiveLoop();
#endif
}