#include "cv_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
using namespace cvbot;
struct Sim {
    Controller bot; Input in; Output out;
    bool physics = true, camera = true;
    Sim() { bot.configured = true; bot.cfg.length = 6000; bot.cfg.width = 5000; }
    void tick(bool newFrame = true) {
        double vl = out.left * 8, vr = out.right * 8;
        if (physics) {
            in.left += vl * .02 / 1.111; in.right += vr * .02 / 1.111;
            in.heading -= (vr - vl) / 225 * .02 * 180 / PI;
        }
        in.now += 20;
        if (camera && newFrame) {
            in.frame.sequence++; in.frame.received = in.now; in.frame.valid = true; in.frame.age = 100;
        }
        out = bot.step(in);
        assert(fabs(out.left) <= 50 && fabs(out.right) <= 50);
    }
    void run(int ms) { for (int n=0; n<ms; n+=20) tick(n%100==0); }
    void arm() { in.touch=false; tick(); in.touch=true; tick(); in.touch=false; tick(); assert(bot.state==SCAN); }
    void ball(int x, int y, int id=1) {
        in.frame.visible=1; in.frame.target=id; in.frame.x=x; in.frame.y=y;
        in.frame.w=in.frame.h=80; in.frame.confidence=800; tick();
    }
    void lost() {
        in.frame.visible=0; in.frame.target=0;
        in.frame.x=in.frame.y=in.frame.w=in.frame.h=in.frame.confidence=0; tick();
    }
    void finish() { arm(); ball(500,800); ball(500,810); assert(bot.state==FINISH); }
    void stopped(const char* why) {
        if (bot.state!=STOPPED || strcmp(bot.reason,why)) printf("Expected %s, got %s %s\n",why,stateName(bot.state),bot.reason);
        assert(bot.state==STOPPED && strcmp(bot.reason,why)==0);
        assert(out.left==0 && out.right==0 && out.intake==0);
    }
};
int main() {
    { Sim s; s.run(1000); assert(!s.bot.active()); }
    { Sim s; s.bot.configured=false; s.in.touch=false; s.tick(); s.in.touch=true; s.tick(); s.in.touch=false; s.tick(); assert(!s.bot.active()); }
    { Sim s; s.in.touch=true; s.run(200); s.in.touch=false; s.tick(); assert(!s.bot.active()); }
    { Sim s; s.camera=false; s.in.touch=false; s.tick(); s.in.touch=true; s.tick(); s.camera=true; s.tick(); s.in.touch=false; s.tick(); assert(!s.bot.active()); s.arm(); }
    { Sim s; s.arm(); s.ball(800,500); assert(s.bot.state==TRACK && s.out.left>0 && s.out.right<0); }
    { Sim s; s.arm(); s.ball(200,500); assert(s.out.left<0 && s.out.right>0); }
    { Sim s; s.arm(); s.ball(540,500); assert(s.out.left>s.out.right && s.out.right>0); }
    { Sim s; s.bot.cfg.steer_sign=-1; s.arm(); s.ball(800,500); assert(s.out.left<s.out.right); }
    { Sim s; s.arm(); s.ball(500,500); assert(s.out.left>0 && s.out.left==s.out.right); }
    { Sim s; s.arm(); s.ball(500,800); for(int i=0;i<10;++i)s.tick(false); assert(s.bot.state==TRACK); s.tick(); assert(s.bot.state==FINISH); }
    { Sim s; s.arm(); s.ball(700,850); s.ball(700,850); assert(s.bot.state==TRACK); }
    { Sim s; s.finish(); s.lost(); assert(s.bot.state==FINISH && s.out.left>0); s.run(2500); assert(s.bot.attempts==1 && s.bot.state==SCAN); }
    { Sim s; s.finish(); s.in.touch=true; s.tick(); s.stopped("TOUCH_STOP"); }
    { Sim s; s.finish(); s.in.check=true; s.tick(); s.stopped("CHECK_STOP"); }
    { Sim s; s.finish(); s.in.stop=true; s.tick(); s.stopped("PC_STOP"); }
    { Sim s; s.finish(); s.camera=false; s.run(800); s.stopped("CAMERA_TIMEOUT"); s.camera=true; s.run(300); s.stopped("CAMERA_TIMEOUT"); }
    { Sim s; s.arm(); s.in.frame.age=900; s.camera=false; s.tick(false); s.stopped("CAMERA_TIMEOUT"); }
    { Sim s; s.arm(); s.ball(600,500); s.lost(); assert(s.bot.state==REACQUIRE && s.out.left+s.out.right==0); s.ball(580,510); s.tick(); assert(s.bot.state==TRACK); }
    { Sim s; s.arm(); s.ball(600,500); s.lost(); s.ball(500,500,2); assert(s.bot.state==REACQUIRE); }
    { Sim s; s.arm(); s.ball(600,500); s.lost(); s.run(1700); assert(s.bot.state==SCAN); }
    { Sim s; s.finish(); s.in.bumper=true; s.tick(); assert(s.bot.state==BACKOFF && s.out.left<0); s.in.bumper=false; s.run(1500); assert(s.bot.state==AVOID_TURN && s.bot.attempts==0); }
    { Sim s; s.finish(); s.in.bumper=true; s.tick(); s.run(1500); s.stopped("BUMPER_HELD"); }
    { Sim s; s.arm(); s.physics=false; s.run(800); s.stopped("DRIVE_STALLED"); }
    { Sim s; s.arm(); s.in.now+=300000; s.tick(); s.stopped("TIME_LIMIT"); assert(s.bot.secondsLeft(s.in.now)==0); }
    { Sim s; s.finish(); s.bot.attempts=9; s.lost(); s.run(2500); s.stopped("PICKUP_LIMIT"); assert(s.bot.attempts==10); }
    { Sim s; s.arm(); s.bot.x=5900; s.tick(); s.stopped("AREA_EDGE"); }
    { Sim s; s.arm(); s.run(11500); assert(s.bot.state==TRAVEL); s.ball(500,500); assert(s.bot.state==TRACK); }
    { Sim s; s.bot.cfg.length=s.bot.cfg.width=2000; s.arm(); for(int i=0;i<15000 && s.bot.active();++i)s.tick(i%5==0); s.stopped("AREA_COVERED"); }
    Settings c;
    assert(parseConfig("500 780 200 2000 2000 1 240 120",c));
    assert(!parseConfig("500 780 200 2000 2000 0 240 120",c));
    assert(!parseConfig("500 780 99999 2000 2000 1 240 120",c));
    assert(!parseConfig("500 780 200 2000 2000 1 240 120 extra",c));
    Frame f;
    assert(parseFrame("1 2 1 500 700 100 100 800 150",f,0,100));
    assert(!parseFrame("1 2 1 500 700 100 100 800 150",f,1,100));
    assert(!parseFrame("2 2 1 500 700 100 100 800 150 extra",f,1,100));
    assert(!parseFrame("2 2 1 500 700 100 100 800 -1",f,1,100));
    assert(!parseFrame("2 2 1 500 700 100 100 800",f,1,100));
    assert(!parseFrame("2 0 1 500 700 100 100 800 150",f,1,100));
    assert(!parseFrame("2 2 0 500 700 100 100 800 150",f,1,100));
    assert(!parseFrame("2 2 1 990 700 100 100 800 150",f,1,100));
    assert(parseFrame("2 2 0 0 0 0 0 0 150",f,1,100));
    puts("CV motion scenarios and protocol validation passed.");
}
