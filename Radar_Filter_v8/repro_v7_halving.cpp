// Reproduction of v7's exact ACF lag-selection and "*2 if >40%" fragment.
// This is NOT a full v7 firmware simulation (Hilbert/CV gating omitted).
#include <cmath>
#include <cstdio>
using namespace std;
static constexpr int N=700, MINLAG=100, MAXLAG=375;
static constexpr float FS=50.0f, TAU=6.283185307179586f;
static float ring[N], diff[N-1], acf[MAXLAG+1];
static float v7_lag(float bpm) {
    float lp=0, alpha=0.2f;
    for (int i=0; i<N; ++i) {
        float raw=sinf(TAU*bpm/60.0f * i/FS);
        lp = i==0 ? raw : alpha*raw+(1-alpha)*lp;
        ring[i] = lp;
    }
    float r0=0;
    for (int i=0;i<N-1;++i) {
        diff[i]=ring[i+1]-ring[i];
        r0+=diff[i]*diff[i];
    }
    float best=-1;
    int bestk=MINLAG;
    for (int k=MINLAG;k<=MAXLAG;++k) {
        float r=0;
        for (int i=0;i<N-1-k;++i) r+=diff[i]*diff[i+k];
        r/=r0;
        acf[k]=r;
        if (r>best) {best=r;bestk=k;}
    }
    int dbl=bestk*2;
    if (dbl<=MAXLAG && acf[dbl]>acf[bestk]*0.4f) bestk=dbl;
    return 60.0f*FS/bestk;
}
int main() {
  const float truth[]={12,16,20,24,30};
  for(float bpm:truth) {
     float predicted=v7_lag(bpm);
     printf("v7 ACF fragment: truth=%2.0f bpm, candidate=%5.2f bpm, error=%+.2f\n",
         bpm,predicted,predicted-bpm);
  }
  return 0;
}
