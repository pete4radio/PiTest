#  Radio Packet

typedef struct {
  uint8_t src;
  uint8_t dst;
  uint8_t flags;
  uint8_t seq;
  uint8_t len;
  uint8_t data[252];
} packet_t;