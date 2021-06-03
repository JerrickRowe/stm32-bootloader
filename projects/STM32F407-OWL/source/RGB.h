#ifndef __RGB_DEF_H__
#define __RGB_DEF_H__

#define RGB(r, g, b)                                                                       \
	((uint32_t)((((uint32_t)(r & 0x000000FF)) << 16) | (((uint32_t)(g & 0x000000FF)) << 8) \
				| (((uint32_t)(b & 0x000000FF)) << 0)))

#define RGB_OFF (RGB(0, 0, 0))
#define RGB_R	(RGB(255, 0, 0))
#define RGB_Y	(RGB(255, 255, 0))
#define RGB_G	(RGB(0, 255, 0))
#define RGB_C	(RGB(0, 255, 255))
#define RGB_B	(RGB(0, 0, 255))
#define RGB_P	(RGB(255, 0, 255))
#define RGB_W	(RGB(255, 255, 255))

#endif
