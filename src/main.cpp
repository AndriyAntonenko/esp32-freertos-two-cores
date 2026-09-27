#include "app_roles.h"

extern "C" void app_main()
{
#if defined(ROLE_BTN_LED)
  btn_led_main();
#elif defined(ROLE_LED_LORA)
  lora_led_main();
#else
#error "Build with -DROLE_BTN_LED or -DROLE_LED_LORA (use env tx or rx)"
#endif
}