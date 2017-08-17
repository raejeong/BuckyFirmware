#include "PID.h"
#include "Arduino.h"

PID::PID(float p, float i, float d, float max, float min)
    : p_(p)
    , i_(i)
    , d_(d)
    , lastErr_(0)
    , integralErr_(0)
    , max_(max)
    , min_(min)
{
    lastTime_ = millis();
}

PID::~PID()
{
}

float PID::getCmd(float ref, float meas)
{
    float err = ref - meas;
    float curTime = millis();
    float tDiff = (curTime - lastTime_) / 1000.0;
    float cmd = 0;
    // 
    // Prevent division by zero. 
    if (tDiff <= 0.0) {
        return 0;
    }
    // 
    // Integrate.
    integralErr_ += tDiff * i_ * err;
    //
    // Anti-windup.
    integralErr_ = min(max(integralErr_, min_), max_);
    // 
    // Calculate the PID and clamp.
    cmd = err * p_ + integralErr_ + d_ * (err - lastErr_) / tDiff;
    cmd = min(max(cmd, min_), max_);
    lastErr_ = err;
    lastTime_ = curTime;
    if(abs(ref) < 0.05)
    {
        cmd = 0.0;
        integralErr_ = 0.0;
    }
    return cmd;
}