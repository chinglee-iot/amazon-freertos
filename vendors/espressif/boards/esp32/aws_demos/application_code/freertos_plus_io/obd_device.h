#ifndef __OBD_DEVICE_H__
#define __OBD_DEVICE_H__

/* Device read/write related ioctl starts from 0x10000000. */
#define ioctlOBD_READ_TIMEOUT    0x10000000

/* OBD device link reset. */
#define ioctlOBD_RESET           0x20000000

#endif
