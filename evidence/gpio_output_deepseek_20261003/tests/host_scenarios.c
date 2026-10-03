#include "gpio_output.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
static unsigned writes;
static uint8_t channels[2048];
static bool values[2048];
static unsigned failures;
#define CHECK(e) do { if (!(e)) { failures++; fprintf(stderr,"line %d: %s\n",__LINE__,#e); } } while(0)
void board_gpio_write(uint8_t channel, bool on) {
    if (writes < 2048) { channels[writes]=channel; values[writes]=on; }
    writes++;
}
static void state(bool a, bool b) {
    bool x=!a, y=!b;
    CHECK(gpio_output_get(0,&x)); CHECK(gpio_output_get(1,&y));
    CHECK(x==a); CHECK(y==b);
}
static void event(unsigned n,uint8_t ch,bool on) {
    CHECK(writes==n+1); CHECK(channels[n]==ch); CHECK(values[n]==on);
}
static void scenario(int s) {
    bool x=true; unsigned n;
    if(s==1) {
        CHECK(!gpio_output_set(0,true)); CHECK(!gpio_output_toggle(1));
        CHECK(!gpio_output_get(0,&x)); CHECK(x); CHECK(writes==0); return;
    }
    gpio_output_init();
    switch(s) {
    case 2:
        CHECK(writes==2); CHECK(channels[0]==0); CHECK(!values[0]);
        CHECK(channels[1]==1); CHECK(!values[1]); state(false,false); CHECK(writes==2); break;
    case 3:
        n=writes; CHECK(gpio_output_set(0,true)); event(n,0,true); state(true,false);
        n=writes; CHECK(gpio_output_set(0,false)); event(n,0,false); state(false,false); break;
    case 4:
        n=writes; CHECK(gpio_output_set(1,true)); event(n,1,true); state(false,true);
        n=writes; CHECK(gpio_output_set(1,false)); event(n,1,false); state(false,false); break;
    case 5:
        n=writes; CHECK(gpio_output_set(0,false)); event(n,0,false);
        n=writes; CHECK(gpio_output_set(0,false)); event(n,0,false);
        n=writes; CHECK(gpio_output_set(1,true)); event(n,1,true);
        n=writes; CHECK(gpio_output_set(1,true)); event(n,1,true); state(false,true); break;
    case 6:
        n=writes; CHECK(gpio_output_toggle(0)); event(n,0,true); state(true,false);
        n=writes; CHECK(gpio_output_toggle(1)); event(n,1,true); state(true,true);
        n=writes; CHECK(gpio_output_toggle(0)); event(n,0,false); state(false,true);
        n=writes; CHECK(gpio_output_toggle(1)); event(n,1,false); state(false,false); break;
    case 7:
        CHECK(gpio_output_set(0,true)); n=writes;
        for(unsigned i=2;i<=255;i++) {
            x=true; CHECK(!gpio_output_set((uint8_t)i,false));
            CHECK(!gpio_output_toggle((uint8_t)i)); CHECK(!gpio_output_get((uint8_t)i,&x)); CHECK(x);
        }
        state(true,false); CHECK(writes==n); break;
    case 8:
        CHECK(gpio_output_set(1,true)); n=writes;
        for(int i=0;i<5;i++) state(false,true);
        CHECK(writes==n); break;
    case 9:
        CHECK(gpio_output_set(0,true)); n=writes;
        CHECK(!gpio_output_get(0,NULL)); CHECK(!gpio_output_get(1,NULL)); CHECK(!gpio_output_get(255,NULL));
        state(true,false); CHECK(writes==n); break;
    case 10:
        CHECK(gpio_output_set(0,true)); CHECK(gpio_output_set(1,true)); n=writes;
        gpio_output_init(); CHECK(writes==n+2);
        CHECK(channels[n]==0); CHECK(!values[n]); CHECK(channels[n+1]==1); CHECK(!values[n+1]);
        state(false,false); CHECK(writes==n+2); break;
    default: failures++;
    }
}
int main(int argc,char **argv) {
    if(argc!=2) return 2;
    int s=atoi(argv[1]); if(s<1||s>10) return 2;
    scenario(s); printf("{\"scenario\":\"GO-S%02d\",\"passed\":%s,\"failed_assertions\":%u}\n",s,failures?"false":"true",failures);
    return failures ? 1 : 0;
}
