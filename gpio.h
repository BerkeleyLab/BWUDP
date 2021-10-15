/* Stand-in only, not (yet?) intended to map to real hardware */
#define GPIO_IDX_NET_RX_DATA     0xfeed0001
#define GPIO_IDX_NET_CONFIG_CSR  0xfeed0002
#define GPIO_IDX_NET_TX_CSR      0xfeed0003
#define GPIO_IDX_NET_RX_CSR      0xfeed0004

static unsigned int GPIO_READ(unsigned long a) {return *(unsigned int *) a;}
static void GPIO_WRITE(unsigned long a, unsigned int d) {*(unsigned int *)a = d;}