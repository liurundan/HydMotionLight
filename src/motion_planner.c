#include "motion_planner.h"
#include "segment_limits.h"
#include "motion_time.h"
#include <math.h>
#include <string.h>

static HYD_REAL HYD_MinReal(HYD_REAL left, HYD_REAL right) {
    return (left < right) ? left : right;
}

static HYD_REAL HYD_MaxReal(HYD_REAL left, HYD_REAL right) {
    return (left > right) ? left : right;
}

static HYD_REAL HYD_CompareTolerance(HYD_REAL magnitude) {
    HYD_REAL tolerance = magnitude * 16.0 * HYD_REAL_EPSILON;
    if (tolerance < 16.0 * HYD_REAL_EPSILON) {
        tolerance = 16.0 * HYD_REAL_EPSILON;
    }
    return tolerance;
}

static HYD_REAL HYD_ResolveConfiguredBrakingAcceleration(const HYD_MotionSegment* segment) {
    if (segment == NULL) {
        return 0.0;
    }
    return (segment->maxDeceleration > 0.0)
        ? segment->maxVelocity / HYD_MotionTimeMsToSeconds(segment->maxDeceleration)
        : segment->maxVelocity / HYD_MotionTimeMsToSeconds(segment->maxAcceleration);
}

static HYD_REAL HYD_ResolveBrakingAcceleration(const HYD_MotionSegment* segment,
                                               HYD_REAL currentMagnitude,
                                               HYD_REAL targetMagnitude) {
    HYD_REAL deltaVelocity;

    if (segment == NULL) {
        return 0.0;
    }
    deltaVelocity = fabs(targetMagnitude - currentMagnitude);
    if (deltaVelocity <= HYD_REAL_EPSILON) {
        return HYD_ResolveConfiguredBrakingAcceleration(segment);
    }
    return (segment->maxDeceleration > 0.0)
        ? deltaVelocity / HYD_MotionTimeMsToSeconds(segment->maxDeceleration)
        : deltaVelocity / HYD_MotionTimeMsToSeconds(segment->maxAcceleration);
}

static HYD_REAL HYD_ResolveTransitionRate(const HYD_MotionSegment* segment,
                                          HYD_REAL startMagnitude,
                                          HYD_REAL targetMagnitude,
                                          HYD_BOOL accelerating) {
    HYD_TIME transitionTime;
    HYD_REAL delta;

    if (segment == NULL) {
        return 0.0;
    }
    transitionTime = HYD_MotionTimeMsToSeconds(
        accelerating ? segment->maxAcceleration : segment->maxDeceleration);
    delta = fabs(targetMagnitude - startMagnitude);
    if (delta <= HYD_REAL_EPSILON) {
        delta = fabs(segment->maxVelocity);
    }
    return (transitionTime > 0.0) ? delta / transitionTime : 0.0;
}

static HYD_REAL HYD_ResolveTransitionTimeMs(const HYD_MotionSegment* segment,
                                            HYD_REAL previousVelocity,
                                            HYD_REAL targetVelocity) {
    HYD_BOOL sameDirection =
        (previousVelocity >= 0.0 && targetVelocity >= 0.0) ||
        (previousVelocity <= 0.0 && targetVelocity <= 0.0);
    HYD_BOOL accelerating = fabs(targetVelocity) > fabs(previousVelocity) &&
        sameDirection;

    return (segment != NULL && accelerating)
        ? segment->maxAcceleration
        : (segment != NULL ? segment->maxDeceleration : 0.0);
}

static HYD_REAL HYD_AdvanceStatefulVelocity(HYD_MotionPlannerState* state,
                                           HYD_REAL targetVelocity,
                                           HYD_REAL transitionTimeMs,
                                           HYD_REAL deltaTime) {
    HYD_REAL normalizedTimeMs = HYD_NormalizeMotionTimeMs(transitionTimeMs);
    HYD_REAL nextVelocity;

    if (state->transitionTimeMs <= 0.0 ||
        fabs(state->transitionTargetVelocity - targetVelocity) >
            HYD_CompareTolerance(HYD_MaxReal(
                fabs(state->transitionTargetVelocity), fabs(targetVelocity))) ||
        fabs(state->transitionTimeMs - normalizedTimeMs) >
            HYD_CompareTolerance(HYD_MaxReal(
                fabs(state->transitionTimeMs), fabs(normalizedTimeMs)))) {
        state->transitionTargetVelocity = targetVelocity;
        state->transitionTimeMs = normalizedTimeMs;
        state->transitionRate =
            (targetVelocity - state->lastTargetVelocity) /
            HYD_MotionTimeMsToSeconds(normalizedTimeMs);
    }

    if (deltaTime <= 0.0) {
        return state->lastTargetVelocity;
    }

    nextVelocity = state->lastTargetVelocity + state->transitionRate * deltaTime;
    if ((state->transitionRate > 0.0 && nextVelocity > targetVelocity) ||
        (state->transitionRate < 0.0 && nextVelocity < targetVelocity)) {
        return targetVelocity;
    }
    return nextVelocity;
}

static HYD_REAL HYD_ApplyVelocityRateLimit(HYD_REAL previousVelocity,
                                           HYD_REAL desiredVelocity,
                                           HYD_REAL acceleration,
                                           HYD_REAL deceleration,
                                           HYD_REAL deltaTime) {
    HYD_REAL delta;
    HYD_REAL limit;

    if (deltaTime <= 0.0) {
        return previousVelocity;
    }

    delta = desiredVelocity - previousVelocity;
    if (delta >= 0.0) {
        limit = acceleration * deltaTime;
    } else {
        limit = deceleration * deltaTime;
    }

    if (limit <= 0.0) {
        return desiredVelocity;
    }

    if (delta > limit) {
        return previousVelocity + limit;
    }
    if (delta < -limit) {
        return previousVelocity - limit;
    }
    return desiredVelocity;
}

static HYD_REAL HYD_GetDirectionSign(HYD_MotionDirection direction) {
    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            return 1.0;
        case HYD_DIRECTION_RETRACT:
            return -1.0;
        default:
            return 0.0;
    }
}

static HYD_REAL HYD_ComputeRemainingDistance(const HYD_MotionSegment* segment,
                                             const HYD_AxisRef* axisRef,
                                             HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;

    if (segment == NULL || axisRef == NULL) {
        return 0.0;
    }

    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            remainingDistance = segment->targetPosition - axisRef->position;
            break;
        case HYD_DIRECTION_RETRACT:
            remainingDistance = axisRef->position - segment->targetPosition;
            break;
        default:
            remainingDistance = 0.0;
            break;
    }

    if (remainingDistance <= 0.0) {
        return 0.0;
    }
    return remainingDistance;
}

static HYD_BOOL HYD_IsBlendMode(HYD_BufferMode bufferMode) {
    return bufferMode >= HYD_BUFFER_MODE_BLENDING_LOW &&
           bufferMode <= HYD_BUFFER_MODE_BLENDING_HIGH;
}

static HYD_BOOL HYD_IsActiveBlendContext(const HYD_MotionPlannerInput* input,
                                         HYD_MotionDirection direction) {
    if (input == NULL || input->blend == NULL || input->segment == NULL ||
        input->axisRef == NULL) {
        return false;
    }
    if (!input->blend->active || !HYD_IsBlendMode(input->blend->bufferMode)) {
        return false;
    }
    if (input->segment->mode != HYD_MODE_POSITION ||
        input->segment->endCondition != HYD_END_POSITION) {
        return false;
    }
    return direction == HYD_DIRECTION_EXTEND ||
           direction == HYD_DIRECTION_RETRACT;
}

static HYD_REAL HYD_ComputeRemainingDistanceToPosition(HYD_REAL targetPosition,
                                                       const HYD_AxisRef* axisRef,
                                                       HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;

    if (axisRef == NULL) {
        return 0.0;
    }

    switch (direction) {
        case HYD_DIRECTION_EXTEND:
            remainingDistance = targetPosition - axisRef->position;
            break;
        case HYD_DIRECTION_RETRACT:
            remainingDistance = axisRef->position - targetPosition;
            break;
        default:
            remainingDistance = 0.0;
            break;
    }

    return (remainingDistance > 0.0) ? remainingDistance : 0.0;
}

static HYD_REAL HYD_ApplyTerminalVelocityRateLimit(HYD_REAL previousMagnitude,
                                                   HYD_REAL terminalMagnitude,
                                                   HYD_REAL acceleration,
                                                   HYD_REAL deceleration,
                                                   HYD_REAL deltaTime) {
    if (deltaTime <= 0.0) {
        return previousMagnitude;
    }
    return HYD_ApplyVelocityRateLimit(previousMagnitude,
                                      terminalMagnitude,
                                      acceleration,
                                      deceleration,
                                      deltaTime);
}

/* Reserve one control-cycle of travel before applying the configured braking
 * envelope.  The hydraulic response delay is represented by deltaTime here;
 * machine-specific delay must be covered by the calibrated deceleration
 * lower bound rather than by adding planner state or a trajectory buffer. */
static HYD_REAL HYD_ComputeDelayAwareBrakingVelocityMagnitude(
    HYD_REAL remainingDistance,
    HYD_REAL brakingAcceleration,
    HYD_REAL maxVelocity,
    HYD_REAL responseDelay,
    HYD_REAL terminalVelocity) {
    HYD_REAL discriminant;
    HYD_REAL velocityMagnitude;

    if (!isfinite(terminalVelocity) || terminalVelocity < 0.0) {
        terminalVelocity = 0.0;
    }

    if (maxVelocity <= 0.0) {
        return 0.0;
    }

    if (remainingDistance <= 0.0) {
        return HYD_ClampReal(terminalVelocity, 0.0, maxVelocity);
    }

    if (brakingAcceleration <= 0.0) {
        return 0.0;
    }

    if (!isfinite(responseDelay) || responseDelay < 0.0) {
        responseDelay = 0.0;
    }

    discriminant = (brakingAcceleration * responseDelay) *
                   (brakingAcceleration * responseDelay) +
                   (terminalVelocity * terminalVelocity) +
                   (2.0 * brakingAcceleration * remainingDistance);
    if (!isfinite(discriminant) || discriminant <= 0.0) {
        return 0.0;
    }

    velocityMagnitude = sqrt(discriminant) -
                        (brakingAcceleration * responseDelay);
    return HYD_ClampReal(velocityMagnitude, 0.0, maxVelocity);
}

static HYD_REAL HYD_ComputePositionBasedVelocityMagnitude(HYD_REAL remainingDistance,
                                                          HYD_REAL acceleration,
                                                          HYD_REAL maxVelocity,
                                                          HYD_REAL responseDelay) {
    return HYD_ComputeDelayAwareBrakingVelocityMagnitude(remainingDistance,
                                                          acceleration,
                                                          maxVelocity,
                                                          responseDelay,
                                                          0.0);
}

static HYD_REAL HYD_ComputeTimeBasedVelocityMagnitude(HYD_REAL elapsedTime,
                                                      HYD_REAL acceleration,
                                                      HYD_REAL maxVelocity) {
    HYD_REAL velocityMagnitude;

    if (elapsedTime <= 0.0 || acceleration <= 0.0 || maxVelocity <= 0.0) {
        return 0.0;
    }

    velocityMagnitude = acceleration * elapsedTime;
    return HYD_ClampReal(velocityMagnitude, 0.0, maxVelocity);
}

static HYD_REAL HYD_ComputeOnlineTrapezoidVelocityMagnitude(const HYD_MotionPlannerInput* input,
                                                            HYD_MotionDirection direction) {
    const HYD_MotionSegment* segment;
    HYD_REAL brakingAcceleration;
    HYD_REAL remainingDistance;
    HYD_REAL positionTolerance;
    HYD_REAL previousMagnitude;
    HYD_REAL velocityMagnitude;
    HYD_REAL safetyVelocityMagnitude;
    HYD_REAL brakeDistance;
    HYD_REAL brakeDecisionTolerance;
    HYD_REAL terminalVelocity;
    HYD_REAL measuredMagnitude;
    HYD_BOOL blendActive;
    HYD_BOOL braking;
    HYD_BOOL zeroEndBlendRampActive;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    segment = input->segment;
    if (segment->maxVelocity <= 0.0 ||
        segment->maxAcceleration < 0.0 ||
        segment->maxDeceleration < 0.0) {
        return 0.0;
    }

    blendActive = HYD_IsActiveBlendContext(input, direction);
    terminalVelocity = 0.0;
    remainingDistance = HYD_ComputeRemainingDistance(segment, input->axisRef, direction);
    positionTolerance = HYD_Segment_GetPositionTolerance(segment);

    if (blendActive) {
        terminalVelocity = HYD_ClampReal(input->blend->blendVelocity,
                                         0.0,
                                         segment->maxVelocity);
        remainingDistance = HYD_ComputeRemainingDistanceToPosition(input->blend->switchPosition,
                                                                   input->axisRef,
                                                                   direction);
        if (input->blend->switchTolerance > 0.0) {
            positionTolerance = input->blend->switchTolerance;
        }
    }

    previousMagnitude = 0.0;
    if (input->state != NULL && input->state->initialized) {
        previousMagnitude = fabs(input->state->lastTargetVelocity);
    }

    brakingAcceleration = HYD_ResolveConfiguredBrakingAcceleration(segment);
    zeroEndBlendRampActive = blendActive &&
        terminalVelocity <= HYD_THRESH_STOP_VEL_DONE_MAG &&
        input->state != NULL && input->state->initialized &&
        fabs(input->state->transitionTargetVelocity) <=
            HYD_THRESH_STOP_VEL_DONE_MAG &&
        input->state->transitionTimeMs > 0.0 &&
        input->state->transitionRate * HYD_GetDirectionSign(direction) < 0.0 &&
        previousMagnitude > HYD_THRESH_STOP_VEL_DONE_MAG;

    if (remainingDistance <= positionTolerance) {
        if (previousMagnitude <= HYD_THRESH_STOP_VEL_DONE_MAG &&
            terminalVelocity <= HYD_THRESH_STOP_VEL_DONE_MAG) {
            velocityMagnitude = 0.0;
        } else if (input->state != NULL) {
            HYD_REAL targetVelocity = terminalVelocity *
                HYD_GetDirectionSign(direction);
            velocityMagnitude = fabs(HYD_AdvanceStatefulVelocity(
                input->state,
                targetVelocity,
                HYD_ResolveTransitionTimeMs(segment,
                                            input->state->lastTargetVelocity,
                                            targetVelocity),
                input->deltaTime));
        } else {
            velocityMagnitude = HYD_ApplyTerminalVelocityRateLimit(
                previousMagnitude,
                terminalVelocity,
                HYD_ResolveTransitionRate(segment,
                                         previousMagnitude,
                                         terminalVelocity,
                                         true),
                brakingAcceleration,
                input->deltaTime);
        }
    } else {
        if (positionTolerance > 0.0 &&
            remainingDistance <= positionTolerance * 10.0 &&
            previousMagnitude <= 0.05) {
            velocityMagnitude = 0.0;
        } else {
            if (previousMagnitude <= terminalVelocity) {
                brakeDistance = 0.0;
            } else {
                HYD_BOOL fixedTimeBlendStop = blendActive &&
                    terminalVelocity <= HYD_THRESH_STOP_VEL_DONE_MAG;
                {
                    HYD_TIME decelSeconds = HYD_MotionTimeMsToSeconds(
                        (segment->maxDeceleration > 0.0)
                            ? segment->maxDeceleration
                            : segment->maxAcceleration);
                    /* Trigger from the previous planned velocity and the
                     * configured transition time. */
                    brakeDistance = 0.5 * (previousMagnitude + terminalVelocity) *
                                    decelSeconds;
                }
                if (input->deltaTime > 0.0 && isfinite(input->deltaTime)) {
                    if (fixedTimeBlendStop) {
                        /* The first ramped command is emitted this cycle, so
                         * the discrete sum omits half a cycle of delta-v. This
                         * makes an exactly reachable stop land at the switch
                         * point instead of stopping early and re-accelerating. */
                        brakeDistance -= 0.5 *
                            (previousMagnitude - terminalVelocity) *
                            input->deltaTime;
                        if (brakeDistance < 0.0) {
                            brakeDistance = 0.0;
                        }
                    } else {
                        brakeDistance += (previousMagnitude - terminalVelocity) *
                                         input->deltaTime;
                    }
                }
            }
            brakeDecisionTolerance = HYD_CompareTolerance(HYD_MaxReal(remainingDistance,
                                                                      brakeDistance));
            braking = previousMagnitude > terminalVelocity &&
                remainingDistance <= brakeDistance + brakeDecisionTolerance;

            /* D2：语义A 下的短行程可达峰值，只在加速阶段施加。
             * 已锚定峰值用精确运动学校验，仍可达则保持（不重锚，速率恒定）；
             * 不可达时跳过 vp cap 保持 maxV 目标（今日行为，被包络/落点
             * 下限削顶）。结果型爬行闸门：可达峰值低于
             * HYD_THRESH_SHORT_STROKE_MIN_PEAK 的亚死区行程同样跳过。 */
            {
                HYD_REAL cruiseTargetMagnitude = segment->maxVelocity;
                HYD_BOOL skipVpCap = false;

                if (!braking && input->state != NULL &&
                    previousMagnitude < segment->maxVelocity) {
                    HYD_TIME accelSeconds =
                        HYD_MotionTimeMsToSeconds(segment->maxAcceleration);
                    HYD_TIME decelSeconds = HYD_MotionTimeMsToSeconds(
                        (segment->maxDeceleration > 0.0)
                            ? segment->maxDeceleration
                            : segment->maxAcceleration);
                    HYD_REAL cycle = (input->deltaTime > 0.0 &&
                                      isfinite(input->deltaTime))
                        ? input->deltaTime : 0.0;
                    HYD_REAL reachablePeak =
                        (2.0 * remainingDistance -
                         accelSeconds * previousMagnitude -
                         decelSeconds * terminalVelocity +
                         2.0 * terminalVelocity * cycle) /
                        (accelSeconds + decelSeconds + 2.0 * cycle);
                    HYD_REAL anchoredTarget =
                        fabs(input->state->transitionTargetVelocity);
                    HYD_REAL anchoredRate =
                        fabs(input->state->transitionRate);

                    if (anchoredTarget > previousMagnitude &&
                        anchoredTarget <= segment->maxVelocity &&
                        reachablePeak < anchoredTarget &&
                        anchoredRate > 0.0) {
                        HYD_REAL neededDistance =
                            (anchoredTarget * anchoredTarget -
                             previousMagnitude * previousMagnitude) /
                                (2.0 * anchoredRate) +
                            0.5 * (anchoredTarget + terminalVelocity) *
                                decelSeconds +
                            (anchoredTarget - terminalVelocity) * cycle;
                        /* anchoredTarget*cycle：离散积分松弛量 */
                        if (remainingDistance + anchoredTarget * cycle >=
                            neededDistance) {
                            reachablePeak = anchoredTarget;
                        } else {
                            /* 不可达锚定加速斜坡补全（裁定链）：锚定斜坡
                             * 经精确运动学判定不可达时跳过 vp cap，保持
                             * maxV 目标再逼近（今日行为，被包络/落点下限
                             * 削顶）。否则目标被钳到 freshVp<prev 再被下限
                             * 钉在 prev，重锚速率归零，命令冻结成
                             * <1mm/s 的多周期爬行（泵死区滞止）。 */
                            skipVpCap = true;
                        }
                    }
                    if (!skipVpCap) {
                        HYD_REAL effectivePeak;

                        if (!isfinite(reachablePeak)) {
                            /* 非有限可达峰值：结果型闸门的数值防护 */
                            skipVpCap = true;
                        } else {
                            effectivePeak = HYD_MaxReal(
                                HYD_MaxReal(reachablePeak, previousMagnitude),
                                terminalVelocity);
                            /* 结果型爬行闸门（裁定链 Task 2/5）：钳位后的
                             * 有效巡航峰值低于 HYD_THRESH_SHORT_STROKE_MIN_PEAK
                             * 时，行程属亚死区噪声，跳过整个 vp cap——恒时间
                             * 迷你梯形会以亚 mm/s 爬行完成，造成泵死区滞止；
                             * 保持 maxV 目标的快速包络/落点下限削顶再逼近才是
                             * 正确行为。注意在钳位结果（含下限 max(prev, vt)）
                             * 上判定而非裸 freshVp：加速中 freshVp 单调下降且
                             * vp<prev 时下限本就钉回 prev（等待制动触发），
                             * 裸判会中途掐断仍在可达范围内的已锚定梯形。 */
                            if (effectivePeak <
                                HYD_THRESH_SHORT_STROKE_MIN_PEAK) {
                                skipVpCap = true;
                            } else {
                                cruiseTargetMagnitude =
                                    HYD_MinReal(cruiseTargetMagnitude,
                                                effectivePeak);
                                cruiseTargetMagnitude =
                                    HYD_MinReal(cruiseTargetMagnitude,
                                                segment->maxVelocity);
                            }
                        }
                    }
                }

                if (input->state != NULL) {
                    HYD_REAL targetMagnitude =
                        braking ? terminalVelocity : cruiseTargetMagnitude;
                    HYD_REAL targetVelocity =
                        targetMagnitude * HYD_GetDirectionSign(direction);
                    velocityMagnitude = fabs(HYD_AdvanceStatefulVelocity(
                        input->state,
                        targetVelocity,
                        HYD_ResolveTransitionTimeMs(segment,
                                                    input->state->lastTargetVelocity,
                                                    targetVelocity),
                        input->deltaTime));
                    if (braking && !blendActive) {
                        /* Landing floor: closes the loop on remaining distance at the anchored delta-v/t_dec
                         * rate, so an early trigger no longer leaves a non-blend stop up to ~1.5*v0*dt short
                         * (a final step of at most a_env*dt may remain). Never accelerates; one-cycle delay
                         * model. Non-blend only: blend terminal stops land on the zero-delay blend envelope
                         * via the quadratic trigger above; vt > 0 blends only need vt by the switch point. */
                        HYD_REAL anchoredRate = fabs(input->state->transitionRate);

                        if (anchoredRate > 0.0) {
                            HYD_REAL landingMagnitude =
                                HYD_ComputeDelayAwareBrakingVelocityMagnitude(
                                    remainingDistance,
                                    anchoredRate,
                                    HYD_MaxReal(segment->maxVelocity, previousMagnitude),
                                    input->deltaTime,
                                    terminalVelocity);
                            velocityMagnitude = HYD_MaxReal(
                                velocityMagnitude,
                                HYD_MinReal(landingMagnitude, previousMagnitude));
                        }
                    }
                } else if (input->deltaTime <= 0.0) {
                    velocityMagnitude = previousMagnitude;
                } else if (braking) {
                    velocityMagnitude =
                        HYD_ApplyTerminalVelocityRateLimit(
                            previousMagnitude,
                            terminalVelocity,
                            HYD_ResolveTransitionRate(segment,
                                                      previousMagnitude,
                                                      terminalVelocity,
                                                      true),
                            brakingAcceleration,
                            input->deltaTime);
                } else {
                    velocityMagnitude =
                        HYD_MinReal(segment->maxVelocity,
                                    previousMagnitude +
                                    HYD_ResolveTransitionRate(segment,
                                                              previousMagnitude,
                                                              segment->maxVelocity,
                                                              true) * input->deltaTime);
                }
            } /* 巡航目标（cruiseTargetMagnitude）计算块结束 */
        }
    }

    if (zeroEndBlendRampActive && remainingDistance <= 0.0) {
        /* The zero-end transition is already latched. Once measured position
         * has crossed the target, keep emitting the planned v->0 ramp instead
         * of collapsing the distance envelope to zero in one scan. */
        safetyVelocityMagnitude = velocityMagnitude;
    } else {
        safetyVelocityMagnitude = HYD_ComputeDelayAwareBrakingVelocityMagnitude(
            remainingDistance,
            brakingAcceleration,
            HYD_MaxReal(segment->maxVelocity, previousMagnitude),
            blendActive ? 0.0 : input->deltaTime,
            terminalVelocity);
    }
    measuredMagnitude = 0.0;
    if (isfinite(input->axisRef->velocity)) {
        measuredMagnitude = fabs(input->axisRef->velocity);
    }
    if (!blendActive && measuredMagnitude > safetyVelocityMagnitude +
                            HYD_CompareTolerance(HYD_MaxReal(measuredMagnitude,
                                                              safetyVelocityMagnitude))) {
        /* Measured overspeed: decelerate from the last planned value at the
         * envelope rate, but only ever tighten the planned ramp (under
         * fixed-time semantics the planned delta-v/t_dec ramp can be steeper
         * than maxVelocity/t_dec). */
        velocityMagnitude = HYD_MinReal(velocityMagnitude,
                                        HYD_ApplyVelocityRateLimit(previousMagnitude,
                                                                   0.0,
                                                                   HYD_ResolveTransitionRate(segment,
                                                                                              previousMagnitude,
                                                                                              0.0,
                                                                                              false),
                                                                   brakingAcceleration,
                                                                   input->deltaTime));
    }

    return HYD_MinReal(velocityMagnitude, safetyVelocityMagnitude);
}

HYD_BOOL HYD_PlanTrapezoid(HYD_TrapezoidProfile* profile,
                           HYD_REAL distance,
                           HYD_REAL vMax,
                           HYD_REAL acc) {
    HYD_REAL sBrake;

    if (profile == NULL || distance <= 0.0 || vMax <= 0.0 || acc <= 0.0) {
        if (profile != NULL) {
            memset(profile, 0, sizeof(*profile));
        }
        return false;
    }

    sBrake = (vMax * vMax) / (2.0 * acc);

    if (2.0 * sBrake >= distance) {
        HYD_REAL vPeak = sqrt(acc * distance);
        profile->tAcc   = vPeak / acc;
        profile->tConst = 0.0;
        profile->tDec   = profile->tAcc;
        profile->sAcc   = distance * 0.5;
        profile->sConst = 0.0;
        profile->sDec   = distance * 0.5;
        profile->vPeak  = vPeak;
    } else {
        profile->tAcc   = vMax / acc;
        profile->sAcc   = sBrake;
        profile->sDec   = sBrake;
        profile->sConst = distance - 2.0 * sBrake;
        profile->tConst = profile->sConst / vMax;
        profile->tDec   = profile->tAcc;
        profile->vPeak  = vMax;
    }
    return true;
}

HYD_REAL HYD_EvalTrapezoid(const HYD_TrapezoidProfile* profile,
                          HYD_REAL elapsed,
                          HYD_REAL acc,
                          HYD_REAL vMax) {
    HYD_REAL tDec;
    (void)vMax;

    if (profile == NULL || elapsed <= 0.0) {
        return 0.0;
    }

    if (elapsed < profile->tAcc) {
        return acc * elapsed;
    }

    if (elapsed < profile->tAcc + profile->tConst) {
        return profile->vPeak;
    }

    tDec = elapsed - profile->tAcc - profile->tConst;
    if (tDec >= profile->tDec) {
        return 0.0;
    }

    return HYD_ClampReal(profile->vPeak - acc * tDec, 0.0, profile->vPeak);
}

static HYD_REAL HYD_ConvertVelocityToFlowMagnitude(HYD_REAL velocityMagnitude,
                                                   const HYD_MotionSegment* segment) {
    HYD_REAL gain;
    HYD_REAL flowMagnitude;

    if (segment == NULL || velocityMagnitude <= 0.0) {
        return 0.0;
    }

    gain = segment->velocityToFlowGain;
    if (gain <= 0.0) {
        gain = 1.0;
    }

    flowMagnitude = velocityMagnitude * gain;
    return HYD_ClampReal(flowMagnitude, 0.0, segment->maxFlow);
}

static HYD_REAL HYD_ApplyModeFlowCap(const HYD_MotionSegment* segment,
                                     HYD_REAL flowMagnitude) {
    HYD_REAL flowLimit;

    if (segment == NULL) {
        return 0.0;
    }

    flowLimit = segment->maxFlow;
    if ((segment->mode == HYD_MODE_POSITION || segment->mode == HYD_MODE_SPEED_RAMP) &&
        (segment->targetFlow > 0.0) &&
        (segment->targetFlow < flowLimit)) {
        flowLimit = segment->targetFlow;
    }

    return HYD_ClampReal(flowMagnitude, 0.0, flowLimit);
}

static HYD_REAL HYD_ComputePositionModeVelocityMagnitude(const HYD_MotionPlannerInput* input,
                                                         HYD_MotionDirection direction) {
    HYD_REAL remainingDistance;
    HYD_REAL brakeVelocityMagnitude;
    HYD_REAL rampVelocityMagnitude;
    HYD_REAL brakingAcceleration;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    remainingDistance = HYD_ComputeRemainingDistance(input->segment, input->axisRef, direction);
    brakingAcceleration = HYD_ResolveConfiguredBrakingAcceleration(input->segment);
    brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       brakingAcceleration,
                                                                       input->segment->maxVelocity,
                                                                       input->deltaTime);

    if (input->segment->planner == HYD_PLANNER_POSITION_BASED) {
        return HYD_ComputeOnlineTrapezoidVelocityMagnitude(input, direction);
    }

    rampVelocityMagnitude = HYD_ComputeTimeBasedVelocityMagnitude(input->elapsedTime,
                                                                  HYD_ResolveTransitionRate(input->segment,
                                                                                            0.0,
                                                                                            input->segment->maxVelocity,
                                                                                            true),
                                                                  input->segment->maxVelocity);
    return HYD_MinReal(rampVelocityMagnitude, brakeVelocityMagnitude);
}

static HYD_REAL HYD_ComputeSpeedRampVelocityMagnitude(const HYD_MotionPlannerInput* input,
                                                      HYD_MotionDirection direction) {
    HYD_REAL velocityMagnitude;
    HYD_REAL remainingDistance;
    HYD_REAL brakeVelocityMagnitude;
    HYD_REAL decelVelocity;
    HYD_REAL brakingAcceleration;

    if (input == NULL || input->segment == NULL || input->axisRef == NULL) {
        return 0.0;
    }

    velocityMagnitude = HYD_ComputeTimeBasedVelocityMagnitude(input->elapsedTime,
                                                              HYD_ResolveTransitionRate(input->segment,
                                                                                        0.0,
                                                                                        input->segment->maxVelocity,
                                                                                        true),
                                                              input->segment->maxVelocity);

    if (input->decelElapsed > 0.0 && input->decelStartVel > 0.0) {
        brakingAcceleration =
            HYD_ResolveBrakingAcceleration(input->segment, input->decelStartVel, 0.0);
        decelVelocity = input->decelStartVel -
            brakingAcceleration * input->decelElapsed;
        if (decelVelocity < 0.0) {
            decelVelocity = 0.0;
        }
        velocityMagnitude = HYD_MinReal(velocityMagnitude, decelVelocity);
    }

    if (input->segment->endCondition != HYD_END_POSITION) {
        return velocityMagnitude;
    }

    remainingDistance = HYD_ComputeRemainingDistance(input->segment,
                                                     input->axisRef,
                                                     direction);
    brakingAcceleration = HYD_ResolveConfiguredBrakingAcceleration(input->segment);
    brakeVelocityMagnitude = HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                                       brakingAcceleration,
                                                                       input->segment->maxVelocity,
                                                                       0.0);
    return HYD_MinReal(velocityMagnitude, brakeVelocityMagnitude);
}

void HYD_MotionPlanner_Execute(const HYD_MotionPlannerInput* input, HYD_MotionPlannerOutput* output) {
    HYD_MotionDirection direction;
    HYD_REAL directionSign;
    HYD_REAL velocityMagnitude;
    HYD_REAL flowMagnitude;
    HYD_REAL previousVelocity;
    HYD_REAL previousDirectionSign;

    if (output == NULL) {
        return;
    }

    output->direction = HYD_DIRECTION_HOLD;
    output->targetVelocity = 0.0;
    output->targetFlow = 0.0;

    if (input == NULL || input->axisRef == NULL || input->segment == NULL) {
        return;
    }

    direction = HYD_Segment_ResolveDirection(input->segment, input->axisRef,
                                             input->lastActiveDirection);
    output->direction = direction;

    if (input->segment->mode == HYD_MODE_PRESSURE_CLOSED_LOOP) {
        return;
    }

    previousVelocity = (input->state != NULL && input->state->initialized)
        ? input->state->lastTargetVelocity
        : 0.0;
    previousDirectionSign = (previousVelocity > 0.0) ? 1.0 : ((previousVelocity < 0.0) ? -1.0 : 0.0);

    if (direction == HYD_DIRECTION_HOLD &&
        input->segment->mode == HYD_MODE_POSITION &&
        input->segment->planner == HYD_PLANNER_POSITION_BASED &&
        input->segment->direction == HYD_DIRECTION_SHORTEST_WAY &&
        previousDirectionSign != 0.0) {
        direction = (previousDirectionSign > 0.0) ? HYD_DIRECTION_POSITIVE : HYD_DIRECTION_NEGATIVE;
        output->direction = direction;
    }

    if (direction == HYD_DIRECTION_HOLD) {
        return;
    }

    directionSign = HYD_GetDirectionSign(direction);
    if (input->state != NULL && input->state->initialized &&
        previousDirectionSign != 0.0 && directionSign != 0.0 &&
        previousDirectionSign != directionSign) {
        HYD_REAL plannedVelocity = HYD_AdvanceStatefulVelocity(
            input->state,
            0.0,
            input->segment->maxDeceleration,
            input->deltaTime);

        output->direction = (previousDirectionSign > 0.0)
            ? HYD_DIRECTION_POSITIVE
            : HYD_DIRECTION_NEGATIVE;
        output->targetVelocity = plannedVelocity;
        output->targetFlow = HYD_ConvertVelocityToFlowMagnitude(
            fabs(plannedVelocity), input->segment);
        output->targetFlow = HYD_ApplyModeFlowCap(input->segment,
                                                 output->targetFlow);
        input->state->lastTargetVelocity = output->targetVelocity;
        input->state->lastTargetFlow = output->targetFlow;
        input->state->lastTimestamp = input->axisRef->timestamp;
        return;
    }

    if (input->segment->mode == HYD_MODE_POSITION &&
        input->segment->planner == HYD_PLANNER_TIME_BASED &&
        input->state != NULL) {
        HYD_REAL targetVelocity = input->segment->maxVelocity * directionSign;
        velocityMagnitude = fabs(HYD_AdvanceStatefulVelocity(
            input->state,
            targetVelocity,
            HYD_ResolveTransitionTimeMs(input->segment,
                                        input->state->lastTargetVelocity,
                                        targetVelocity),
            input->deltaTime));
    } else if (input->segment->mode == HYD_MODE_POSITION) {
        velocityMagnitude = HYD_ComputePositionModeVelocityMagnitude(input, direction);
    } else if (input->state != NULL) {
        HYD_REAL targetVelocity = (input->decelElapsed > 0.0)
            ? 0.0
            : input->segment->maxVelocity * directionSign;
        velocityMagnitude = fabs(HYD_AdvanceStatefulVelocity(
            input->state,
            targetVelocity,
            HYD_ResolveTransitionTimeMs(input->segment,
                                        input->state->lastTargetVelocity,
                                        targetVelocity),
            input->deltaTime));
        if (input->decelElapsed > 0.0 && input->decelStartVel > 0.0) {
            HYD_REAL brakingAcceleration =
                HYD_ResolveBrakingAcceleration(input->segment, input->decelStartVel, 0.0);
            HYD_REAL decelVelocity = input->decelStartVel -
                brakingAcceleration * input->decelElapsed;
            if (decelVelocity < 0.0) {
                decelVelocity = 0.0;
            }
            velocityMagnitude = HYD_MinReal(velocityMagnitude, decelVelocity);
        }
        if (input->segment->endCondition == HYD_END_POSITION) {
            HYD_REAL remainingDistance = HYD_ComputeRemainingDistance(
                input->segment, input->axisRef, direction);
            HYD_REAL brakingAcceleration =
                HYD_ResolveConfiguredBrakingAcceleration(input->segment);
            HYD_REAL safetyVelocityMagnitude =
                HYD_ComputePositionBasedVelocityMagnitude(
                    remainingDistance, brakingAcceleration,
                    input->segment->maxVelocity, 0.0);
            velocityMagnitude = HYD_MinReal(velocityMagnitude,
                                            safetyVelocityMagnitude);
        }
    } else {
        velocityMagnitude = HYD_ComputeSpeedRampVelocityMagnitude(input, direction);
    }

    flowMagnitude = HYD_ConvertVelocityToFlowMagnitude(velocityMagnitude, input->segment);
    flowMagnitude = HYD_ApplyModeFlowCap(input->segment, flowMagnitude);

    /* The state slew limit above protects command continuity, but it must not
     * raise a TIME_BASED position command above the distance-based safety
     * envelope.  This final cap is intentionally limited to position mode so
     * SPEED_RAMP and pressure-control behavior remain unchanged. */
    if (input->segment->mode == HYD_MODE_POSITION &&
        input->segment->planner == HYD_PLANNER_TIME_BASED) {
        HYD_REAL remainingDistance = HYD_ComputeRemainingDistance(input->segment,
                                                                   input->axisRef,
                                                                   direction);
        HYD_REAL brakingAcceleration = HYD_ResolveBrakingAcceleration(
            input->segment, fabs(previousVelocity), 0.0);
        HYD_REAL safetyVelocityMagnitude =
            HYD_ComputePositionBasedVelocityMagnitude(remainingDistance,
                                                       brakingAcceleration,
                                                       input->segment->maxVelocity,
                                                       input->deltaTime);
        HYD_REAL measuredMagnitude = isfinite(input->axisRef->velocity)
            ? fabs(input->axisRef->velocity) : 0.0;

        if (measuredMagnitude > safetyVelocityMagnitude +
                                HYD_CompareTolerance(HYD_MaxReal(measuredMagnitude,
                                                                  safetyVelocityMagnitude))) {
            velocityMagnitude = HYD_ApplyVelocityRateLimit(fabs(previousVelocity),
                                                           0.0,
                                                           HYD_ResolveTransitionRate(input->segment,
                                                                                      fabs(previousVelocity),
                                                                                      0.0,
                                                                                      false),
                                                           brakingAcceleration,
                                                           input->deltaTime);
        }
        velocityMagnitude = HYD_MinReal(velocityMagnitude,
                                        safetyVelocityMagnitude);
        flowMagnitude = HYD_ConvertVelocityToFlowMagnitude(velocityMagnitude,
                                                           input->segment);
        flowMagnitude = HYD_ApplyModeFlowCap(input->segment, flowMagnitude);
    }

    output->targetFlow = flowMagnitude;
    output->targetVelocity = velocityMagnitude * directionSign;

    if (input->state != NULL) {
        input->state->initialized = true;
        input->state->lastTargetVelocity = output->targetVelocity;
        input->state->lastTargetFlow = output->targetFlow;
        input->state->lastTimestamp = input->axisRef->timestamp;
    }
}
