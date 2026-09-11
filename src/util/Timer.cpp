#include "hololib/util/Timer.hpp"
#include "pros/rtos.hpp"

namespace hololib {

Timer::Timer(uint32_t time) : m_period(time) { m_lastTime = pros::millis(); }

// The clean, centralized update helper
void Timer::update() {
    const uint32_t time = pros::millis();
    if (!m_paused) {
        m_timeWaited += time - m_lastTime;
    }
    m_lastTime = time;
}

uint32_t Timer::getTimeSet() {
    this->update();
    return m_period;
}

uint32_t Timer::getTimeLeft() {
    this->update();
    // Compare before subtracting: both operands are unsigned, so an expired
    // timer would otherwise wrap to a huge value instead of going negative.
    return (m_timeWaited < m_period) ? (m_period - m_timeWaited) : 0;
}

uint32_t Timer::getTimePassed() {
    this->update();
    return m_timeWaited;
}

bool Timer::isDone() {
    this->update();
    return m_timeWaited >= m_period;
}

bool Timer::isPaused() {
    // Go through update() so m_lastTime advances with m_timeWaited. Accumulating
    // without moving m_lastTime made every call re-count the same elapsed span,
    // running the timer fast whenever isPaused() was polled.
    this->update();
    return m_paused;
}

void Timer::set(uint32_t time) {
    m_period = time;
    reset();
}

void Timer::reset() {
    m_timeWaited = 0;
    m_lastTime = pros::millis();
}

void Timer::pause() {
    if (!m_paused)
        m_lastTime = pros::millis();
    m_paused = true;
}

void Timer::resume() {
    if (m_paused)
        m_lastTime = pros::millis();
    m_paused = false;
}

void Timer::waitUntilDone() {
    do {
        pros::delay(5);
    } while (!this->isDone());
}
} // namespace hololib