#include "StateMachine/Persistence.h"

bool Persistence::update(bool condition, uint64_t timeUs) {
  if (!condition) {
    running_ = false;
    held_ = false;
    lastUs_ = timeUs;
    return false;
  }

  // Start a run on the first true sample. If sample time went backwards the
  // elapsed time of the old run means nothing any more, so the run starts
  // again from this sample.
  if (!running_ || timeUs < lastUs_) {
    running_ = true;
    sinceUs_ = timeUs;
  }
  lastUs_ = timeUs;

  // timeUs >= sinceUs_ always holds here, so the subtraction cannot wrap.
  held_ = (timeUs - sinceUs_) >= holdUs_;
  return held_;
}

void Persistence::reset() {
  sinceUs_ = 0;
  lastUs_ = 0;
  running_ = false;
  held_ = false;
}
