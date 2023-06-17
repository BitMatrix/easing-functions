#include <cmath>
#include <map>

#include "easing.h"

float easeInSine(float t) {
  return sinf(M_PI_2 * t);
}

float easeOutSine(float t) {
  return 1.0f + sinf(M_PI_2 * (t - 1));
}

float easeInOutSine(float t) {
  return 0.5f * (1.0f + sinf(M_PI * (t - 0.5f)));
}

float easeInQuad(float t) {
  return t * t;
}

float easeOutQuad(float t) {
  return t * (2 - t);
}

float easeInOutQuad(float t) {
  return t < 0.5 ? 2 * t * t : t * (4 - 2 * t) - 1;
}

float easeInCubic(float t) {
  return t * t * t;
}

float easeOutCubic(float t) {
  float tMinusOne = t - 1;
  return 1 + tMinusOne * t * t;
}

float easeInOutCubic(float t) {
  float tVal = (t < 0.5f) ? t : t - 1.0f;
  float cubic = tVal * tVal * tVal;
  return (t < 0.5f) ? 4.0f * cubic : 0.5f * cubic + 1.0f;
}

float easeInQuart(float t) {
  t *= t;
  return t * t;
}

float easeOutQuart(float t) {
  float tMinusOne = t - 1;
  t = tMinusOne * t;
  return 1 - t * t;
}

float easeInOutQuart(float t) {
  float tVal = (t < 0.5f) ? t : t - 1.0f;
  float quartic = tVal * tVal * tVal * tVal;
  return (t < 0.5f) ? 8.0f * quartic : 0.5f * quartic + 1.0f;
}

float easeInQuint(float t) {
  float quintic = t * t * t * t * t;
  return quintic;
}

float easeOutQuint(float t) {
  t = t - 1.0f;
  float quintic = t * t * t * t * t;
  return quintic + 1.0f;
}

float easeInOutQuint(float t) {
  float tVal = (t < 0.5f) ? t : t - 1.0f;
  float quintic = tVal * tVal * tVal * tVal * tVal;
  return (t < 0.5f) ? 16.0f * quintic : 0.5f * quintic + 1.0f;
}

float easeInExpo(float t) {
  return powf(2.0f, 10.0f * (t - 1.0f));
}

float easeOutExpo(float t) {
  return 1.0f - powf(2.0f, -10.0f * t);
}

float easeInOutExpo(float t) {
  if (t == 0.0f) {
    return 0.0f;
  }
  if (t == 1.0f) {
    return 1.0f;
  }
  if (t < 0.5f) {
    return 0.5f * powf(2.0f, (20.0f * t) - 10.0f);
  } else {
    return -0.5f * powf(2.0f, (-20.0f * t) + 10.0f) + 1.0f;
  }
}

float easeInCirc(float t) {
  return -1.0f * (sqrtf(1.0f - t * t) - 1.0f);
}

float easeOutCirc(float t) {
  t = t - 1.0f;
  return sqrtf(1.0f - t * t);
}

float easeInOutCirc(float t) {
  t *= 2.0f;
  if (t < 1.0f) {
    return -0.5f * (sqrtf(1.0f - t * t) - 1.0f);
  } else {
    t -= 2.0f;
    return 0.5f * (sqrtf(1.0f - t * t) + 1.0f);
  }
}

float easeInBack(float t) {
  const float s = 1.70158f;
  return t * t * ((s + 1.0f) * t - s);
}

float easeOutBack(float t) {
  const float s = 1.70158f;
  t = t - 1.0f;
  return (t * t * ((s + 1.0f) * t + s)) + 1.0f;
}

float easeInOutBack(float t) {
  float s = 1.70158f;
  t *= 2.0f;
  if (t < 1.0f) {
    s *= 1.525f;
    return 0.5f * (t * t * ((s + 1.0f) * t - s));
  } else {
    t -= 2.0f;
    s *= 1.525f;
    return 0.5f * (t * t * ((s + 1.0f) * t + s) + 2.0f);
  }
}

float easeInElastic(float t) {
  float p = 0.3f;
  float a = 1.0f;
  float s = p / 4.0f;
  t = t - 1.0f;
  return -(a * powf(2.0f, 10.0f * t) * sinf((t - s) * (2.0f * M_PI) / p));
}

float easeOutElastic(float t) {
  float p = 0.3f;
  float a = 1.0f;
  float s = p / 4.0f;
  return (a * powf(2.0f, -10.0f * t) * sinf((t - s) * (2.0f * M_PI) / p) + 1.0f);
}

float easeInOutElastic(float t) {
  t = t * 2.0f;
  float p = 0.3f * 1.5f;
  float a = 1.0f;
  float s = p / 4.0f;
  if (t < 1.0f) {
    t = t - 1.0f;
    return -0.5f * (a * powf(2.0f, 10.0f * t) * sinf((t - s) * (2.0f * M_PI) / p));
  } else {
    t = t - 1.0f;
    return a * powf(2.0f, -10.0f * t) * sinf((t - s) * (2.0f * M_PI) / p) * 0.5f + 1.0f;
  }
}

float easeOutBounce(float t) {
  if (t < 4 / 11.0f) {
    return (121 * t * t) / 16.0f;
  } else if (t < 8 / 11.0f) {
    return (363 / 40.0f * t * t) - (99 / 10.0f * t) + 17 / 5.0f;
  } else if (t < 9 / 10.0f) {
    return (4356 / 361.0f * t * t) - (35442 / 1805.0f * t) + 16061 / 1805.0f;
  } else {
    return (54 / 5.0f * t * t) - (513 / 25.0f * t) + 268 / 25.0f;
  }
}

float easeInBounce(float t) {
  return 1.0f - easeOutBounce(1.0f - t);
}

float easeInOutBounce(float t) {
  if(t < 0.5f) {
    return 0.5f * easeInBounce(t*2.0f);
  } else {
    return 0.5f * easeOutBounce(t * 2.0f - 1.0f) + 0.5f;
  }
}

easingFunction getEasingFunction(easing_functions function) {
  static std::map<easing_functions, easingFunction> easingFunctions;
  if (easingFunctions.empty()) {
    easingFunctions.insert(std::make_pair(EaseInSine, easeInSine));
    easingFunctions.insert(std::make_pair(EaseOutSine, easeOutSine));
    easingFunctions.insert(std::make_pair(EaseInOutSine, easeInOutSine));
    easingFunctions.insert(std::make_pair(EaseInQuad, easeInQuad));
    easingFunctions.insert(std::make_pair(EaseOutQuad, easeOutQuad));
    easingFunctions.insert(std::make_pair(EaseInOutQuad, easeInOutQuad));
    easingFunctions.insert(std::make_pair(EaseInCubic, easeInCubic));
    easingFunctions.insert(std::make_pair(EaseOutCubic, easeOutCubic));
    easingFunctions.insert(std::make_pair(EaseInOutCubic, easeInOutCubic));
    easingFunctions.insert(std::make_pair(EaseInQuart, easeInQuart));
    easingFunctions.insert(std::make_pair(EaseOutQuart, easeOutQuart));
    easingFunctions.insert(std::make_pair(EaseInOutQuart, easeInOutQuart));
    easingFunctions.insert(std::make_pair(EaseInQuint, easeInQuint));
    easingFunctions.insert(std::make_pair(EaseOutQuint, easeOutQuint));
    easingFunctions.insert(std::make_pair(EaseInOutQuint, easeInOutQuint));
    easingFunctions.insert(std::make_pair(EaseInExpo, easeInExpo));
    easingFunctions.insert(std::make_pair(EaseOutExpo, easeOutExpo));
    easingFunctions.insert(std::make_pair(EaseInOutExpo, easeInOutExpo));
    easingFunctions.insert(std::make_pair(EaseInCirc, easeInCirc));
    easingFunctions.insert(std::make_pair(EaseOutCirc, easeOutCirc));
    easingFunctions.insert(std::make_pair(EaseInOutCirc, easeInOutCirc));
    easingFunctions.insert(std::make_pair(EaseInBack, easeInBack));
    easingFunctions.insert(std::make_pair(EaseOutBack, easeOutBack));
    easingFunctions.insert(std::make_pair(EaseInOutBack, easeInOutBack));
    easingFunctions.insert(std::make_pair(EaseInElastic, easeInElastic));
    easingFunctions.insert(std::make_pair(EaseOutElastic, easeOutElastic));
    easingFunctions.insert(std::make_pair(EaseInOutElastic, easeInOutElastic));
    easingFunctions.insert(std::make_pair(EaseInBounce, easeInBounce));
    easingFunctions.insert(std::make_pair(EaseOutBounce, easeOutBounce));
    easingFunctions.insert(std::make_pair(EaseInOutBounce, easeInOutBounce));

  }
  auto it = easingFunctions.find(function);
  return it == easingFunctions.end() ? nullptr : it->second;
}