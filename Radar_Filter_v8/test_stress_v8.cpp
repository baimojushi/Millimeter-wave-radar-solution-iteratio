#include "radar_resp_filter_v8.h"
#include <cstdio>
#include <cmath>
#include <cstdint>
using namespace radar_v8;
static uint32_t rstate=123456789;
static float rnd() {
    rstate=1664525u*rstate+1013904223u;
    return ((rstate>>8)&0xFFFFFFu)/16777216.0f-0.5f;
}
static void run(const char* name, int mode, int dt=20) {
    Filter f; f.on_human(true);
    float brown=0.0f;
    int n_ok=0, n_total=0;
    float mean_est=0;
    for(int i=0;i<6500;++i) {
        float t=i/50.0f, signal=0;
        switch(mode) {
            case 0: brown += 0.09f*rnd(); signal=brown; break;
            case 1: signal=0.4f*sinf(K_TAU*0.12f*t)+0.8f*sinf(K_TAU*1.5f*t); break;
            case 2: signal=0.7f*sinf(K_TAU*16/60*t)+0.8f*sinf(K_TAU*1.2f*t); break;
            case 3: signal=0.08f*sinf(K_TAU*16/60*t)+0.12f*rnd(); break;
            case 4: signal=0.7f*sinf(K_TAU*16/60*t)+0.55f*rnd(); break;
            case 5: signal=0.7f*sinf(K_TAU*15/60*t); break; // indistinguishable periodic motion
            case 6: signal=0.7f*sinf(K_TAU*16/60*t); if (i%25==0) signal+=0.6f; break;
            case 7: signal=0.8f*sinf(K_TAU*16/60*t); break;
        }
        f.add(signal, (uint32_t)(0xFFFFD000u+i*(uint32_t)dt));
        if (i>=5000 && i%100==0) {
            auto r=f.result();
            if(r.status==OK) {n_ok++;mean_est+=r.br;}
            n_total++;
        }
    }
    const auto r=f.result();
    std::printf("%-29s | valid %2d/%2d | mean %5.2f | last %6.2f %-14s | fs %.2f\n",
          name,n_ok,n_total,n_ok?mean_est/n_ok:0.0f,r.br,status_name(r.status),r.fs);
}
int main() {
    run("random walk (no respiration)",0);
    run("0.12Hz sway plus 1.5Hz",1);
    run("16bpm plus 1.2Hz motion",2);
    run("low amplitude 16 bpm",3);
    run("16bpm high white noise",4);
    run("pure 15bpm periodic motion",5);
    run("16bpm periodic mild spikes",6);
    run("millis uint32 wrap-around",7);
    run("incorrect 55.6Hz",7,18);
    return 0;
}
