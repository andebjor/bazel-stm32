#include "cpp_test.h"

namespace {

struct non_trivial
{
    non_trivial() {
        volatile int l = 2;

        led_ = l;
    }

    int led_;
};

non_trivial led;

} // anonymous namespace

extern "C"
{

auto cpp_led_on() -> int
{
    return led.led_;
//     return 1;
}

}
