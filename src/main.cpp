#include "app_roles.h"

extern "C" void app_main()
{
#if defined(ROLE_BTN_LED)
  btn_led_main();
#elif defined(ROLE_LED_LORA)
  lora_led_main();
#elif defined(ROLE_MUTEX_DEADLOCK)
  mutex_deadlock();
#else
#error "Build with -DROLE_BTN_LED, -DROLE_LED_LORA or DROLE_MUTEX_DEADLOCK"
#endif
}