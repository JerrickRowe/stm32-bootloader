
#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef enum main_sta {
	MAIN_STA_USB_STORAGE  = 0xF0,
	MAIN_STA_BL_NOAPP	  = 0xF1,
	MAIN_STA_BL_UPGRADING = 0xF2,
} main_sta_t;

void RC_SimpleConnection(main_sta_t);
bool RC_Presented(void);

#ifdef __cplusplus
}
#endif
