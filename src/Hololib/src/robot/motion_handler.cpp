#include "motion_handler.h"

/**
 *@brief The task that runs the motion handler loop.
 *@param param The motion handler to run the task for.
 *@return void
 */
void motionHandlerTask(void *param) {
  static_cast<MotionHandler *>(param)->loop();
}

/**
 *@brief Constructor for the motion handler.
 *@return void
 */
MotionHandler::MotionHandler()
    : lastEnqueuedId(0), currentRunningId(0), onMotionStartCallback(nullptr),
      running(false) {
  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
}

/**
 *@brief Enqueues a motion to be executed.
 *@param motion The motion to enqueue.
 *@param async Whether the motion should be executed asynchronously.
 *@return void
 */
void MotionHandler::enqueue(std::function<void()> motion, bool async) {
  auto done = std::make_shared<bool>(false);

  mutex.take();
  lastEnqueuedId++;
  queue.push([this, motion, done]() {
    motion();
    mutex.take();
    if (running) {
      *done = true;
    }
    mutex.give();
  });
  mutex.give();

  if (!async) {
    while (!*done) {
      pros::delay(10);
    }
  }
}

/**
 *@brief Cancels the current motion.
 *@return void
 */
void MotionHandler::cancelMotion() {
  mutex.take();
  if (task != nullptr) {
    task->remove();
    delete task;
    task = nullptr;
  }

  if (!queue.empty()) {
    queue.pop();
  }

  currentRunningId++;
  running = false;

  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
  mutex.give();
}

/**
 *@brief Cancels all motions.
 *@return void
 */
void MotionHandler::cancelAll() {
  mutex.take();
  if (task != nullptr) {
    task->remove();
    delete task;
    task = nullptr;
  }

  while (!queue.empty()) {
    queue.pop();
  }

  currentRunningId = lastEnqueuedId;
  running = false;

  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
  mutex.give();
}

/**
 *@brief Waits until all motions are done.
 *@return void
 */
void MotionHandler::waitUntilDone() {
  while (true) {
    mutex.take();
    bool done = queue.empty() && !running;
    mutex.give();

    if (done)
      break;
    pros::delay(10);
  }
}

/**
 *@brief The main loop of the motion handler.
 *@return void
 */
void MotionHandler::loop() {
  while (true) {
    std::function<void()> currentMotion = nullptr;

    mutex.take();
    if (!queue.empty()) {
      currentMotion = queue.front();
    }
    mutex.give();

    if (currentMotion) {
      mutex.take();
      if (!queue.empty()) {
        queue.pop();
      }
      running = true;
      currentRunningId++;
      if (onMotionStartCallback) {
        onMotionStartCallback();
      }
      mutex.give();

      currentMotion();

      mutex.take();
      running = false;
      mutex.give();
    } else {
      pros::delay(10);
    }
  }
}

/**
 *@brief Checks if the motion handler is in motion.
 *@return bool True if the motion handler is in motion.
 */
bool MotionHandler::isInMotion() {
  mutex.take();
  bool inMotion = running || !queue.empty();
  mutex.give();
  return inMotion;
}