// Additional adversarial sweeps: out-of-range periodic signals,
// stale recovery, short pulses, and unsigned [0, 2pi] phase wrap.
#include "radar_resp_filter_v8.h"
#include <cstdio>
#include <cmath>
#include <cstdint>
#include <initializer_list>
using namespace radar_v8;
static float wrap_0_tau(float x) {
    while (x < 0.0f) x += K_TAU;
    while (x >= K_TAU) x -= K_TAU;
    return x;
}
static int failures=0;
static void check(bool ok, const char *label, const Result& r) {
    printf("[%s] %-27s br=%.2f q=%s\n",
           ok?"PASS":"FAIL",label,r.br,status_name(r.status));
    if(!ok)failures++;
}
static void sample(Filter& f, float bpm, int start, int stop, bool wrapped=false) {
    for(int i=start; i<stop; ++i) {
        float x=0.7f*sinf(K_TAU*bpm/60.0f*(i/FS));
        if(wrapped) x=wrap_0_tau(x);
        f.add(x, (uint32_t)(i*20u));
    }
}
int main() {
    for(float freq: {6.0f, 7.0f, 35.0f, 45.0f, 55.0f}) {
        Filter f; f.on_human(true);
        sample(f,freq,0,6000);
        char label[128];
        snprintf(label,sizeof(label),"out of range %.0fbpm",freq);
        check(f.result().br==0.0f && f.result().status!=OK,label,f.result());
    }
    {
        Filter f; f.on_human(true);
        sample(f,16,0,3500,true);
        check(f.result().status==OK && fabsf(f.result().br-16)<1,
              "unsigned 0..2pi phase",f.result());
    }
    {
        Filter f; f.on_human(true);
        sample(f,16,0,2000);
        f.tick(42000);
        check(f.result().br==0 && f.result().status==STALE,
              "gap invalidates",f.result());
        sample(f,16,2300,3700);
        check(f.result().status==OK && fabsf(f.result().br-16)<1,
              "recover after UART gap",f.result());
    }
    {
        Filter f; f.on_human(true);
        sample(f,16,0,2000);
        f.add(NAN, 40020);
        sample(f,16,2002,3402);
        check(f.result().status==OK && fabsf(f.result().br-16)<1,
              "recover after NaN",f.result());
    }
    printf("EDGE RESULT: %d failures\n",failures);
    return failures?1:0;
}
