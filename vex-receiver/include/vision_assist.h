#ifndef BALL_VISION_ASSIST_H
#define BALL_VISION_ASSIST_H
// Camera reports only identification. No steering or speed is carried here.
struct VisionInput {
    char command; unsigned age_ms; bool stop_requested; unsigned observation_id;
    constexpr VisionInput(char c, unsigned age, bool stop, unsigned id = 0)
        : command(c), age_ms(age), stop_requested(stop), observation_id(id) {}
};
typedef VisionInput (*VisionReader)(void*);
// Identification happens with the wheels stopped, not in a camera steering loop.
// Include capture/inference time without expiring normal CPU video between frames.
constexpr unsigned CV_FRAME_MAX_AGE_MS = 750;
constexpr bool visionLive(VisionInput v) {
    return !v.stop_requested && v.age_ms <= CV_FRAME_MAX_AGE_MS && (v.command == 'Y' || v.command == 'N');
}
struct MissionStartGate { bool touch_was_down, press_armed, ready; };
constexpr MissionStartGate missionStartStep(MissionStartGate old, bool camera_ready, bool touch_down) {
    return {touch_down, camera_ready && (old.press_armed || (touch_down && !old.touch_was_down)),
            camera_ready && old.press_armed && !touch_down};
}
struct BallCheck {
    unsigned started_ms, last_id, hits;
};
constexpr BallCheck beginBallCheck(VisionInput v, unsigned now) {
    return {now, v.observation_id, 0};
}
// Discard frames captured before the robot has stopped facing this candidate.
// Count distinct observations, never repeated reads of one serial packet.
constexpr BallCheck ballCheckStep(BallCheck old, VisionInput v, unsigned now) {
    return !visionLive(v) ? BallCheck{old.started_ms, v.observation_id, 0} :
        (now - old.started_ms < 150 || v.age_ms > now - old.started_ms - 150) ?
            BallCheck{old.started_ms, v.observation_id, 0} :
        v.observation_id == old.last_id ? old :
        BallCheck{old.started_ms, v.observation_id, v.command == 'Y' ? old.hits + 1 : 0};
}
constexpr bool ballApproved(BallCheck check) { return check.hits >= 2; }
#endif
